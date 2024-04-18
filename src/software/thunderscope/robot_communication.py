import threading
import time
import os
import software.python_bindings as tbots_cpp

from typing import Type
from google.protobuf.message import Message
from pyqtgraph.Qt import QtCore
from proto.import_all_protos import *
from software.logger.logger import create_logger
from software.thunderscope.robot_diagnostics.diagnostics_input_widget import ControlMode
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import *
from software.thunderscope.constants import (
    ROBOT_COMMUNICATIONS_TIMEOUT_S,
    IndividualRobotMode,
    EstopMode,
)

NUM_ROBOTS = MAX_ROBOT_IDS_PER_SIDE


class RobotCommunication(object):
    """ Communicate with the robots """

    def __init__(
        self,
        current_proto_unix_io: ProtoUnixIO,
        multicast_channel: str,
        interface: str,
        estop_mode: EstopMode,
        estop_path: os.PathLike = None,
        estop_baudrate: int = 115200,
        enable_radio: bool = False,
    ):
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_mode: what estop mode we are running right now, of type EstopMode
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop
        :param enable_radio: Whether to use radio to send primitives to robots

        """
        self.receive_ssl_referee_proto = None
        self.receive_ssl_wrapper = None
        self.sequence_number = 0
        self.last_time = time.time()
        self.current_proto_unix_io = current_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.running = False
        self.enable_radio = enable_radio

        self.logger = create_logger("RobotCommunication")

        self.fullsystem_primitive_set_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.diagnostics_primitive_buffer = ThreadSafeBuffer(1, Primitive)

        self.current_proto_unix_io.register_observer(
            PrimitiveSet, self.fullsystem_primitive_set_buffer
        )

        self.current_proto_unix_io.register_observer(
            Primitive, self.diagnostics_primitive_buffer
        )

        self.send_estop_state_thread = threading.Thread(
            target=self.__send_estop_state, daemon=True
        )
        self.run_primitive_set_thread = threading.Thread(
            target=self.__run_primitive_set, daemon=True
        )

        # dynamic map of robot id to the individual control mode
        self.robot_control_mode_map: dict[int, IndividualRobotMode] = {}

        # static map of robot id to stop primitive
        self.robot_stop_primitives_map: dict[int, StopPrimitive] = {}

        # dynamic map of robot id to the number of times to send a stop primitive
        self.robot_stop_primitive_send_count_map: dict[int, int] = {}

        # load control mode and stop primitive maps with default values
        for robot_id in range(NUM_ROBOTS):
            self.robot_control_mode_map[robot_id] = IndividualRobotMode.NONE
            self.robot_stop_primitives_map[robot_id] = Primitive(stop=StopPrimitive())
            self.robot_stop_primitive_send_count_map[robot_id] = 0

        # TODO: (#3174): move estop state management out of robot_communication
        self.estop_mode = estop_mode
        self.estop_path = estop_path
        self.estop_buadrate = estop_baudrate

        # initialising the estop
        # tries to access a plugged in estop. if not found, throws an exception
        # if using keyboard estop, skips this step
        self.estop_reader = None
        self.estop_is_playing = False
        # when the estop has just been stopped,
        # we want to send a stop primitive once to all currently connected robots
        self.should_send_stop = False

        # only checks for estop if we are in physical estop mode
        if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            try:
                self.estop_reader = tbots_cpp.ThreadedEstopReader(
                    self.estop_path, self.estop_buadrate
                )
            except Exception:
                raise Exception(f"Invalid Estop found at location {self.estop_path}")

    def setup_for_fullsystem(self) -> None:
        """
        Sets up a listener for SSL vision and referee data, and connects all robots to fullsystem as default
        """
        self.robot_control_mode_map.update(
            (robot_id, IndividualRobotMode.AI)
            for robot_id in self.robot_control_mode_map.keys()
        )

        self.receive_ssl_wrapper = tbots_cpp.SSLWrapperPacketProtoListener(
            SSL_VISION_ADDRESS,
            SSL_VISION_PORT,
            lambda data: self.__forward_to_proto_unix_io(SSL_WrapperPacket, data),
            True,
        )

        self.receive_ssl_referee_proto = tbots_cpp.SSLRefereeProtoListener(
            SSL_REFEREE_ADDRESS,
            SSL_REFEREE_PORT,
            lambda data: self.current_proto_unix_io.send_proto(Referee, data),
            True,
        )

    def close_for_fullsystem(self) -> None:
        if self.receive_ssl_wrapper:
            self.receive_ssl_wrapper.close()

        if self.receive_ssl_referee_proto:
            self.receive_ssl_referee_proto.close()

    def toggle_keyboard_estop(self) -> None:
        """
        If keyboard estop is being used, toggles the estop state
        And sends a message to the console
        """
        if self.estop_mode == EstopMode.KEYBOARD_ESTOP:
            self.estop_is_playing = not self.estop_is_playing
            self.logger.debug(
                "Keyboard Estop changed to "
                + (
                    f"\x1b[32mPLAY \x1b[0m"
                    if self.estop_is_playing
                    else f"\x1b[31;20mSTOP \x1b[0m"
                )
            )

    def toggle_individual_robot_control_mode(
        self, robot_id: int, mode: IndividualRobotMode
    ):
        """
        Changes the input mode for a robot between NONE, MANUAL, or AI
        If changing from MANUAL OR AI to NONE, add robot id to stop primitive
        map so that multiple stop primitives are sent - safety number one priority

        :param mode: the mode of input for this robot's primitives
        :param robot_id: the id of the robot whose mode we're changing
        """

        self.robot_control_mode_map[robot_id] = mode
        self.robot_stop_primitive_send_count_map[robot_id] = (
            NUM_TIMES_SEND_STOP if mode == IndividualRobotMode.NONE else 0
        )

    def __send_estop_state(self) -> None:
        """
        Constant loop which sends the current estop status proto if estop is not disabled
        Uses the keyboard estop value for keyboard estop mode
        If we're in physical estop mode, uses the physical estop value
        If estop has just changed from playing to stop, set flag to send stop primitive once to connected robots
        """
        previous_estop_is_playing = True
        if self.estop_mode != EstopMode.DISABLE_ESTOP:
            while True:
                if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
                    self.estop_is_playing = self.estop_reader.isEstopPlay()

                # Send stop primitive once when estop is paused
                if previous_estop_is_playing and not self.estop_is_playing:
                    self.should_send_stop = True
                else:
                    self.should_send_stop = False

                previous_estop_is_playing = self.estop_is_playing

                self.current_proto_unix_io.send_proto(
                    EstopState, EstopState(is_playing=self.estop_is_playing)
                )
                time.sleep(0.1)

    def __should_send_packet(self) -> bool:
        """
        Returns True if the proto sending threads should send a proto
        :return: boolean
        """
        return (
            self.estop_mode != EstopMode.DISABLE_ESTOP
            and self.estop_is_playing
            and (
                IndividualRobotMode.AI
                or IndividualRobotMode.MANUAL
                in self.robot_id_individual_mode_dict.values()
            )
        )

    def __run_primitive_set(self) -> None:
        """Forward PrimitiveSet protos from fullsystem to the robots.

        For AI protos, blocks for 10ms if no proto is available, and then returns a cached proto

        For Diagnostics protos, does not block and returns cached message if none available
        Sleeps for 10ms for diagnostics

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.

        NOTE: If disconnect_fullsystem_from_robots is called, then the packets
        will not be forwarded to the robots.

        send_override_primitive_set can be used to send a primitive set, which
        is useful to dip in and out of robot diagnostics.

        """
        while self.running:
            # map of robot id to diagnostics/fullsystem primitive map
            robot_primitives_map = {}

            # get the most recent diagnostics primitive
            diagnostics_primitive = self.diagnostics_primitive_buffer.get(block=False)

            # filter for diagnostics controlled robots
            diagnostics_robots = list(
                robot_id
                for robot_id, mode in self.robot_control_mode_map.items()
                if mode == IndividualRobotMode.MANUAL
            )

            # set diagnostics primitives for diagnostics robots
            for robot_id in diagnostics_robots:
                robot_primitives_map[robot_id] = diagnostics_primitive

            # get the most recent fullsystem primitives
            fullsystem_primitive_set = self.fullsystem_primitive_set_buffer.get(
                block=True, timeout=ROBOT_COMMUNICATIONS_TIMEOUT_S
            )

            # filter for fullsystem controlled robots
            fullsystem_robots = list(
                robot_id
                for robot_id, mode in self.robot_control_mode_map.items()
                if mode == IndividualRobotMode.AI
            )

            # set fullsystem primitives for fullsystem robots
            for robot_id in fullsystem_robots:
                robot_primitives_map[
                    robot_id
                ] = fullsystem_primitive_set.robot_primitives[robot_id]

            # sends a final stop primitive to all disconnected robots and removes them from list
            # in order to prevent robots acting on cached old primitives
            for (
                robot_id,
                num_times_to_stop,
            ) in self.robot_stop_primitive_send_count_map.items():
                if num_times_to_stop > 0:
                    robot_primitives_map[robot_id] = Primitive(stop=StopPrimitive())
                    self.robot_stop_primitive_send_count_map[robot_id] = (
                        num_times_to_stop - 1
                    )

            # initialize total primitive set and send it
            primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=self.robot_stop_primitives_map
                if self.should_send_stop
                else robot_primitives_map,
                sequence_number=self.sequence_number,
            )

            self.sequence_number += 1

            if self.__should_send_packet() or self.should_send_stop:
                self.send_primitive_set.send_proto(primitive_set)
                self.should_send_stop = False

            # sleep if not running fullsystem
            if IndividualRobotMode.AI not in self.robot_control_mode_map.values():
                time.sleep(ROBOT_COMMUNICATIONS_TIMEOUT_S)

    def __forward_to_proto_unix_io(self, type: Type[Message], data: Message) -> None:
        """
        Forwards to proto unix IO iff running is true
        :param data: the data to be passed through
        :param type: the proto type
        """
        if self.running:
            self.current_proto_unix_io.send_proto(type, data)

    def __enter__(self) -> "self":
        """Enter RobotCommunication context manager. Setup multicast listeners
        for RobotStatus, RobotLogs, and RobotCrash msgs, and multicast sender for PrimitiveSet

        """
        # Create the multicast listeners
        # TODO (#3172): `--disable_communication` still requires an interface to be configured
        # error thrown on string concatenation of `self.interface`,
        # since it is NoneType object if not passed as argument to tscope...
        self.receive_robot_status = tbots_cpp.RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            lambda data: self.__forward_to_proto_unix_io(RobotStatus, data),
            True,
        )

        self.receive_robot_log = tbots_cpp.RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            lambda data: self.__forward_to_proto_unix_io(RobotLog, data),
            True,
        )

        self.receive_robot_crash = tbots_cpp.RobotCrashProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_CRASH_PORT,
            lambda data: self.current_proto_unix_io.send_proto(RobotCrash, data),
            True,
        )

        # Create multicast senders
        if self.enable_radio:
            self.send_primitive_set = tbots_cpp.PrimitiveSetProtoRadioSender()
        else:
            self.send_primitive_set = tbots_cpp.PrimitiveSetProtoUdpSender(
                self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
            )

        self.running = True

        self.send_estop_state_thread.start()
        self.run_primitive_set_thread.start()

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotCommunication context manager

        Ends all currently running loops and joins all currently active threads

        """
        self.running = False

        self.close_for_fullsystem()

        # TODO (#3172): if `--disable_communication` is set, this throws a runtime error on exit
        self.receive_robot_log.close()
        self.receive_robot_status.close()
        self.run_primitive_set_thread.join()
