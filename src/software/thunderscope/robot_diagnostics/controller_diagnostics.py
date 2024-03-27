import logging
import numpy
from evdev import InputDevice, categorize, ecodes, list_devices
from threading import Event, Thread

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.thunderscope.constants import *
from software.thunderscope.constants import ControllerConstants
from software.thunderscope.proto_unix_io import ProtoUnixIO


# TODO: change all logging to DEBUG level or remove entirely...


class MoveEventType(Enum):
    LINEAR = 1
    ROTATIONAL = 2


class ControllerInputHandler(object):
    """
    This class is responsible for reading from a handheld controller device and
    interpreting the inputs into usable inputs for the robot.
    """

    # TODO: remove proto_unix_io, and set Motor/Power control as class fields
    # TODO: add class init wrapper for easier handling of controller connection
    def __init__(
        self, proto_unix_io: ProtoUnixIO,
    ):
        self.proto_unix_io = proto_unix_io
        self.enabled = False
        self.controller = None

        self.__setup_controller()

        if self.controller is not None:
            logging.debug(
                "Initializing controller "
                + self.controller.info.__str__()
                + " and device path location: "
                + self.controller.path
            )

            self.__stop_event_thread = Event()
            self.__event_thread = Thread(target=self.__event_loop, daemon=True)
            self.__event_thread.start()

            self.constants = tbots_cpp.create2021RobotConstants()

            # Fields for holding the diagnostics types that get sent
            self.motor_control = MotorControl()
            self.power_control = PowerControl()

        else:
            logging.debug(
                "Tried to initialize a handheld controller from list available devices:"
            )
            logging.debug(list_devices())
            logging.debug(
                "Could not initialize a handheld controller device - check USB connections"
            )

    def get_latest_primitive_command(self):
        if self.controller_initialized():
            return DirectControlPrimitive(
                motor_control=self.motor_control, power_control=self.power_control,
            )
        else:
            return None

    def controller_initialized(self):
        return self.controller is not None

    def __setup_controller(self):
        for device in list_devices():
            controller = InputDevice(device)
            if (
                controller is not None
                and controller.name in ControllerConstants.VALID_CONTROLLER_NAMES
            ):
                self.controller = controller
                break

    def process_move_event_value(self, event_type, event_value) -> None:
        if event_type == "ABS_X":
            self.motor_control.direct_velocity_control.velocity.x_component_meters = self.__parse_move_event_value(
                MoveEventType.LINEAR, event_value
            )

        elif event_type == "ABS_Y":
            self.motor_control.direct_velocity_control.velocity.y_component_meters = self.__parse_move_event_value(
                MoveEventType.LINEAR, event_value
            )

        elif event_type == "ABS_RX":
            self.motor_control.direct_velocity_control.velocity.radians_per_second = self.__parse_move_event_value(
                MoveEventType.ROTATIONAL, event_value
            )

    def __process_event(self, event):
        kick_power = 0.0
        dribbler_speed = 0.0

        abs_event = categorize(event)
        event_type = ecodes.bytype[abs_event.event.type][abs_event.event.code]

        logging.debug(
            "Processing controller event with type "
            + str(event_type)
            + " and with value "
            + abs_event.event.value
        )

        # TODO: bump python version so we can use pattern matching for this
        if event.type == ecodes.EV_ABS:
            if event_type in ["ABS_X", "ABS_Y", "ABS_RX"]:
                self.process_move_event_value(event_type, abs_event.event.value)

        # TODO: can enable smooth scrolling for dribbler by chekcing for "ABS_HAT0Y" event
        if event_type == "ABS_HAT0X":
            dribbler_speed = self.__parse_kick_event_value(abs_event.event.value)

        if event_type == "ABS_HAT0Y":
            kick_power = self.__parse_dribble_event_value(abs_event.event.value)

        if event_type == "ABS_RZ" or "ABS_Z":
            if self.__parse_dribbler_enabled_event_value(abs_event.event.value):
                self.motor_control.dribbler_speed_rpm = dribbler_speed

        # TODO: possible to use `event_type` instead of `event.type`
        if event.type == ecodes.EV_KEY:
            if event.code == ecodes.ecodes["BTN_A"] and event.value == 1:
                self.power_control.geneva_slot = 1
                self.power_control.chicker.kick_speed_m_per_s = kick_power

            elif event.code == ecodes.ecodes["BTN_Y"] and event.value == 1:
                self.power_control.geneva_slot = 1
                self.power_control.chicker.chip_distance_meters = kick_power

    @staticmethod
    def __parse_move_event_value(
        event_type: MoveEventType, event_value: float
    ) -> float:
        factor = (
            ControllerConstants.MAX_ANGULAR_SPEED_RAD_PER_S
            if event_type == MoveEventType.ROTATIONAL
            else ControllerConstants.MAX_LINEAR_SPEED_METER_PER_S
            if event_type == MoveEventType.LINEAR
            else 1.0
        )

        if abs(event_value) < (ControllerConstants.DEADZONE_PERCENTAGE * factor):
            return 0
        else:
            return numpy.clip(
                event_value, 0, ControllerConstants.XBOX_MAX_RANGE * factor
            )

    @staticmethod
    def __parse_dribbler_enabled_event_value(value: float) -> bool:
        return value > (ControllerConstants.XBOX_BUTTON_MAX_RANGE / 2.0)

    @staticmethod
    def __parse_dribble_event_value(value: float) -> float:
        return numpy.clip(
            value * ControllerConstants.DRIBBLER_STEPPER,
            0,
            ControllerConstants.DRIBBLER_INDEFINITE_SPEED,
        )

    @staticmethod
    def __parse_kick_event_value(value: float) -> float:
        return numpy.clip(
            value * ControllerConstants.POWER_STEPPER,
            ControllerConstants.MIN_POWER,
            ControllerConstants.MAX_POWER,
        )

    def __event_loop(self):
        logging.debug("Starting handheld controller event handling loop")
        if self.enabled:
            for event in self.controller.read_loop():
                if self.__stop_event_thread.isSet():
                    return
                else:
                    self.__process_event(event)

    def close(self):
        logging.debug("Closing handheld controller event handling thread")
        self.__stop_event_thread.set()
        self.__event_thread.join()

    def set_controller_enabled(self, enabled: bool):
        """
        Changes the diagnostics input mode for all robots between Xbox and Diagnostics.

        :param enabled: to which state to set controller enabled.
        """
        self.enabled = enabled


# TODO: remove thee after field testing...
# {
#   ('EV_SYN', 0): [('SYN_REPORT', 0), ('SYN_CONFIG', 1), ('SYN_DROPPED', 3), ('?', 21)],
#   ('EV_KEY', 1): [
#     ('KEY_RECORD', 167),
#     (['BTN_A', 'BTN_GAMEPAD', 'BTN_SOUTH'], 304),
#     (['BTN_B', 'BTN_EAST'], 305),
#     (['BTN_NORTH', 'BTN_X'], 307),
#     (['BTN_WEST', 'BTN_Y'], 308),
#     ('BTN_TL', 310),
#     ('BTN_TR', 311),
#     ('BTN_SELECT', 314),
#     ('BTN_START', 315),
#     ('BTN_MODE', 316),
#     ('BTN_THUMBL', 317),
#     ('BTN_THUMBR', 318)
#   ],
#   ('EV_ABS', 3): [
#     (('ABS_X', 0), AbsInfo(value=1242, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_Y', 1), AbsInfo(value=425, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_Z', 2), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)),
#     (('ABS_RX', 3), AbsInfo(value=-418, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_RY', 4), AbsInfo(value=-485, min=-32768, max=32767, fuzz=16, flat=128, resolution=0)),
#     (('ABS_RZ', 5), AbsInfo(value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0)),
#     (('ABS_HAT0X', 16), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0)),
#     (('ABS_HAT0Y', 17), AbsInfo(value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0))
#   ],
#   ('EV_FF', 21): [
#     (['FF_EFFECT_MIN', 'FF_RUMBLE'], 80),
#     ('FF_PERIODIC', 81),
#     (['FF_SQUARE', 'FF_WAVEFORM_MIN'], 88),
#     ('FF_TRIANGLE', 89),
#     ('FF_SINE', 90),
#     (['FF_GAIN', 'FF_MAX_EFFECTS'], 96)
#   ]
# }
