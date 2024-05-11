import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from software.simulated_tests.excessive_dribbling import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team

@pytest.mark.parametrize(
    "ball_initial_position,robot_initial_position,kick_location,kick_direction",
    [
        (tbots_cpp.Point(1, 0), tbots_cpp.Point(0, 0), tbots_cpp.Point(1, 0), tbots_cpp.Angle.fromDegrees(0),),
        (tbots_cpp.Point(1, 0), tbots_cpp.Point(2, 0), tbots_cpp.Point(1, 0), tbots_cpp.Angle.fromDegrees(0),),
        (tbots_cpp.Point(1, 0), tbots_cpp.Point(1, 1), tbots_cpp.Point(1.6, 0), tbots_cpp.Angle.fromDegrees(0),),
        (tbots_cpp.Point(1, 0), tbots_cpp.Point(1, 1), tbots_cpp.Point(1.0, -0.50), tbots_cpp.Angle.fromDegrees(0),),
    ]
)
def test_pivot_kick_tactic(ball_initial_position, robot_initial_position,
                           kick_location, kick_direction, simulated_test_runner):

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[robot_initial_position],
            ball_location=ball_initial_position,
            ball_velocity=tbots_cpp.Vector(0, 0)
        ),
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].pivot_kick.CopyFrom(
        PivotKickTactic(
            kick_origin=tbots_cpp.createPointProto(kick_location),
            kick_direction=tbots_cpp.createAngleProto(kick_direction),
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=5.0),
        )
    )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    always_validation_sequence_set = [
        [
            NeverExcessivelyDribbles(),
        ]
    ]

    simulated_test_runner.run_test(
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=7,
    )


if __name__ == "__main__":
    pytest_main(__file__)
