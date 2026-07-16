"""Unit tests for JointTrajectory to Board1 CAN conversion."""

import math

from arm_can_bridge.can_protocol import rad_to_angle_raw
from arm_can_bridge.trajectory_converter import (
    ArmTrajectoryConverter,
    TrajectoryConversionError,
)

import pytest
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

JOINT_NAMES = (
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
)


def make_converter(**overrides) -> ArmTrajectoryConverter:
    """Create a converter with broad test joint limits."""
    kwargs = {
        'joint_names': JOINT_NAMES,
        'motor_ids': (0, 1, 2, 3),
        'min_positions_rad': (-math.pi,) * 4,
        'max_positions_rad': (math.pi,) * 4,
        'speed_raw': 0,
        'start_position_tolerance_rad': 0.02,
    }
    kwargs.update(overrides)
    return ArmTrajectoryConverter(
        **kwargs,
    )


def make_point(
    positions,
    time_from_start_ns,
) -> JointTrajectoryPoint:
    """Create a trajectory point with a nanosecond timestamp."""
    point = JointTrajectoryPoint()
    point.positions = list(positions)
    point.time_from_start.sec = (
        time_from_start_ns // 1_000_000_000
    )
    point.time_from_start.nanosec = (
        time_from_start_ns % 1_000_000_000
    )
    return point


def frame_target(frame) -> int:
    """Decode the signed target field from one 0x101 frame."""
    return int.from_bytes(
        frame.data[1:5],
        byteorder='little',
        signed=True,
    )


def frame_aux_raw(frame) -> int:
    """Decode Byte 5~6 as unsigned aux field."""
    return int.from_bytes(
        frame.data[5:7],
        byteorder='little',
        signed=False,
    )


def test_reorders_joint_names_and_builds_four_frames():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'arm_joint_3',
        'arm_joint_1',
        'arm_joint_4',
        'arm_joint_2',
    ]

    trajectory.points = [
        make_point(
            (
                math.radians(30.0),
                math.radians(10.0),
                math.radians(40.0),
                math.radians(20.0),
            ),
            50_000_000,
        )
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert len(batches) == 1
    assert batches[0].duration_ticks == 10
    assert batches[0].queue_slots == 4

    assert [frame.data[0] for frame in batches[0].frames] == [
        0x80,
        0x81,
        0x82,
        0x83,
    ]

    assert [frame_target(frame) for frame in batches[0].frames] == [
        1000,
        2000,
        3000,
        4000,
    ]


def test_builds_board1_and_board2_frames_for_five_axis_arm():
    converter = ArmTrajectoryConverter(
        joint_names=(
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'base_joint',
            'arm_joint_4',
        ),
        board_ids=(1, 1, 1, 1, 2),
        motor_ids=(0, 1, 2, 3, 0),
        min_positions_rad=(-math.pi,) * 5,
        max_positions_rad=(math.pi,) * 5,
        speed_raw=0,
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'base_joint',
        'arm_joint_1',
        'arm_joint_2',
        'arm_joint_3',
        'arm_joint_4',
    ]
    trajectory.points = [
        make_point(
            (
                math.radians(-60.0),
                math.radians(-50.0),
                math.radians(-40.0),
                math.radians(-30.0),
                math.radians(-20.0),
            ),
            50_000_000,
        )
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0,) * 5,
    )

    assert len(batches) == 1
    assert [frame.can_id for frame in batches[0].frames] == [
        0x101,
        0x101,
        0x101,
        0x101,
        0x102,
    ]
    assert [frame.data[0] for frame in batches[0].frames] == [
        0x80,
        0x81,
        0x82,
        0x83,
        0x80,
    ]
    assert [frame_target(frame) for frame in batches[0].frames] == [
        -5000,
        -4000,
        -3000,
        -6000,
        -2000,
    ]
    assert batches[0].queue_slots_by_board == {1: 4, 2: 1}


def test_can_frames_are_sorted_by_board_and_motor_id():
    converter = ArmTrajectoryConverter(
        joint_names=(
            'arm_joint_3',
            'base_joint',
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_4',
        ),
        board_ids=(1, 1, 1, 1, 2),
        motor_ids=(2, 3, 0, 1, 0),
        min_positions_rad=(-math.pi,) * 5,
        max_positions_rad=(math.pi,) * 5,
        speed_raw=0,
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'base_joint',
        'arm_joint_1',
        'arm_joint_2',
        'arm_joint_3',
        'arm_joint_4',
    ]
    trajectory.points = [
        make_point(
            (
                math.radians(-60.0),
                math.radians(-50.0),
                math.radians(-40.0),
                math.radians(-30.0),
                math.radians(-20.0),
            ),
            50_000_000,
        )
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0,) * 5,
    )

    assert [frame.can_id for frame in batches[0].frames] == [
        0x101,
        0x101,
        0x101,
        0x101,
        0x102,
    ]
    assert [frame.data[0] for frame in batches[0].frames] == [
        0x80,
        0x81,
        0x82,
        0x83,
        0x80,
    ]
    assert [frame_target(frame) for frame in batches[0].frames] == [
        -5000,
        -4000,
        -3000,
        -6000,
        -2000,
    ]


def test_builds_board1_frames_for_base_and_three_arm_joints():
    converter = ArmTrajectoryConverter(
        joint_names=(
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'base_joint',
        ),
        board_ids=(1, 1, 1, 1),
        motor_ids=(0, 1, 2, 3),
        min_positions_rad=(-math.pi,) * 4,
        max_positions_rad=(math.pi,) * 4,
        speed_raw=0,
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'base_joint',
        'arm_joint_1',
        'arm_joint_2',
        'arm_joint_3',
    ]
    trajectory.points = [
        make_point(
            (
                math.radians(-60.0),
                math.radians(-50.0),
                math.radians(-40.0),
                math.radians(-30.0),
            ),
            50_000_000,
        )
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0,) * 4,
    )

    assert len(batches) == 1
    assert [frame.can_id for frame in batches[0].frames] == [
        0x101,
        0x101,
        0x101,
        0x101,
    ]
    assert [frame.data[0] for frame in batches[0].frames] == [
        0x80,
        0x81,
        0x82,
        0x83,
    ]
    assert [frame_target(frame) for frame in batches[0].frames] == [
        -5000,
        -4000,
        -3000,
        -6000,
    ]
    assert batches[0].queue_slots_by_board == {1: 4}


def test_raw_position_signs_invert_motion_around_home_pose():
    converter = ArmTrajectoryConverter(
        joint_names=JOINT_NAMES,
        motor_ids=(0, 1, 2, 3),
        min_positions_rad=(-math.pi,) * 4,
        max_positions_rad=(math.pi,) * 4,
        raw_position_signs=(-1, -1, -1, -1),
        raw_position_offsets_rad=(
            math.radians(-180.0),
            math.radians(-180.0),
            math.radians(-180.0),
            math.radians(-180.0),
        ),
        speed_raw=0,
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point(
            (
                math.radians(-80.0),
                math.radians(-90.0),
                math.radians(-90.0),
                math.radians(-90.0),
            ),
            50_000_000,
        )
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(math.radians(-90.0),) * 4,
    )

    assert frame_target(batches[0].frames[0]) == rad_to_angle_raw(
        math.radians(-100.0)
    )


def test_builds_board3_frames_for_integrated_gripper():
    joint_names = (
        'base_joint',
        'arm_joint_1',
        'arm_joint_2',
        'arm_joint_3',
        'arm_joint_4',
        'finger_1_base_joint',
        'finger_1_middle_joint',
        'finger_1_tip_joint',
        'finger_2_base_joint',
        'finger_2_middle_joint',
        'finger_2_tip_joint',
        'finger_3_base_joint',
        'finger_3_middle_joint',
        'finger_3_tip_joint',
    )
    converter = ArmTrajectoryConverter(
        joint_names=joint_names,
        board_ids=(1, 1, 1, 1, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3),
        motor_ids=(0, 1, 2, 3, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8),
        min_positions_rad=(-math.pi,) * len(joint_names),
        max_positions_rad=(math.pi,) * len(joint_names),
        speed_raw=0,
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    trajectory.points = [
        make_point(
            (
                math.radians(-60.0),
                math.radians(-60.0),
                math.radians(-40.0),
                math.radians(-60.0),
                math.radians(-120.0),
                math.radians(-20.0),
                math.radians(-45.0),
                math.radians(-30.0),
                math.radians(-20.0),
                math.radians(-45.0),
                math.radians(-30.0),
                math.radians(-20.0),
                math.radians(-45.0),
                math.radians(-30.0),
            ),
            50_000_000,
        )
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0,) * len(joint_names),
    )

    assert len(batches) == 1
    assert [frame.can_id for frame in batches[0].frames] == [
        0x101,
        0x101,
        0x101,
        0x101,
        0x102,
        0x103,
        0x103,
        0x103,
        0x103,
        0x103,
        0x103,
        0x103,
        0x103,
        0x103,
    ]
    assert [frame.data[0] for frame in batches[0].frames[-9:]] == [
        0x80,
        0x81,
        0x82,
        0x83,
        0x84,
        0x85,
        0x86,
        0x87,
        0x88,
    ]
    assert batches[0].queue_slots_by_board == {1: 4, 2: 1, 3: 9}


def test_board3_frames_use_configured_default_target_load():
    joint_names = (
        'finger_1_base_joint',
        'finger_1_middle_joint',
        'finger_1_tip_joint',
        'finger_2_base_joint',
        'finger_2_middle_joint',
        'finger_2_tip_joint',
        'finger_3_base_joint',
        'finger_3_middle_joint',
        'finger_3_tip_joint',
    )
    converter = ArmTrajectoryConverter(
        joint_names=joint_names,
        board_ids=(3,) * len(joint_names),
        motor_ids=tuple(range(len(joint_names))),
        min_positions_rad=(-math.pi,) * len(joint_names),
        max_positions_rad=(math.pi,) * len(joint_names),
        aux_raw_by_board={3: 500},
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    trajectory.points = [
        make_point((0.1,) * len(joint_names), 50_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0,) * len(joint_names),
    )

    assert [frame_aux_raw(frame) for frame in batches[0].frames] == [500] * 9


def test_board3_frames_can_override_target_load_with_effort():
    joint_names = (
        'finger_1_base_joint',
        'finger_1_middle_joint',
        'finger_1_tip_joint',
    )
    converter = ArmTrajectoryConverter(
        joint_names=joint_names,
        board_ids=(3, 3, 3),
        motor_ids=(0, 1, 2),
        min_positions_rad=(-math.pi,) * len(joint_names),
        max_positions_rad=(math.pi,) * len(joint_names),
        aux_raw_by_board={3: 500},
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(joint_names)
    point = make_point((0.1, 0.2, 0.3), 50_000_000)
    point.effort = [300.0, 450.0, 600.0]
    trajectory.points = [point]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0,) * len(joint_names),
    )

    assert [frame_aux_raw(frame) for frame in batches[0].frames] == [
        300,
        450,
        600,
    ]


def test_matching_zero_time_start_point_is_skipped():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((0.0, 0.0, 0.0, 0.0), 0),
        make_point(
            (0.1, 0.2, 0.3, 0.4),
            50_000_000,
        ),
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert len(batches) == 1
    assert batches[0].source_point_index == 1
    assert batches[0].duration_ticks == 10


def test_mismatching_zero_time_start_point_is_rejected():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((0.5, 0.0, 0.0, 0.0), 0)
    ]

    with pytest.raises(
        TrajectoryConversionError,
        match='zero-time',
    ):
        converter.convert(
            trajectory,
            current_positions_rad=(0.0, 0.0, 0.0, 0.0),
        )


def test_two_second_segment_is_split_for_uint8_duration():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((1.0, 1.0, 1.0, 1.0), 2_000_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert [batch.duration_ticks for batch in batches] == [
        255,
        145,
    ]

    assert frame_target(batches[-1].frames[0]) == (
        rad_to_angle_raw(1.0)
    )

    first_expected = rad_to_angle_raw(1.0 * 255.0 / 400.0)
    assert frame_target(batches[0].frames[0]) == first_expected


def test_arm_segment_can_be_split_into_20_ms_absolute_points():
    converter = make_converter(max_segment_duration_ticks=4)

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((1.0, 1.0, 1.0, 1.0), 50_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert [batch.duration_ticks for batch in batches] == [4, 4, 2]
    assert all(
        frame.data[-1] <= 4
        for batch in batches
        for frame in batch.frames
    )
    assert [frame.data[0] for frame in batches[-1].frames] == [
        0x80,
        0x81,
        0x82,
        0x83,
    ]
    assert frame_target(batches[-1].frames[0]) == rad_to_angle_raw(1.0)


def test_seven_ms_arm_stream_retimes_points_to_at_least_40_ms():
    converter = make_converter(
        max_segment_duration_ticks=8,
        min_segment_duration_ticks=8,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((0.2, 0.2, 0.2, 0.2), 50_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert [batch.duration_ticks for batch in batches] == [10]
    assert frame_target(batches[-1].frames[0]) == rad_to_angle_raw(0.2)


def test_short_moveit_segment_is_stretched_to_40_ms():
    converter = make_converter(
        max_segment_duration_ticks=8,
        min_segment_duration_ticks=8,
    )
    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((0.1, 0.1, 0.1, 0.1), 20_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert [batch.duration_ticks for batch in batches] == [8]


def test_40_ms_partition_never_leaves_a_short_tail():
    converter = make_converter(
        max_segment_duration_ticks=8,
        min_segment_duration_ticks=8,
    )
    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((0.2, 0.2, 0.2, 0.2), 85_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(0.0, 0.0, 0.0, 0.0),
    )

    assert [batch.duration_ticks for batch in batches] == [9, 8]


def test_raw_command_limits_reject_generated_target_before_send():
    converter = ArmTrajectoryConverter(
        joint_names=('arm_joint_1',),
        board_ids=(2,),
        motor_ids=(0,),
        min_positions_rad=(math.radians(-90.0),),
        max_positions_rad=(math.radians(90.0),),
        command_min_angle_raw=(-8500,),
        command_max_angle_raw=(9000,),
    )
    trajectory = JointTrajectory()
    trajectory.joint_names = ['arm_joint_1']
    trajectory.points = [
        make_point((math.radians(-86.0),), 50_000_000)
    ]

    with pytest.raises(
        TrajectoryConversionError,
        match='firmware command limit',
    ):
        converter.convert(
            trajectory,
            current_positions_rad=(math.radians(-86.5),),
        )


def test_post_home_escape_is_one_board1_batch_at_exact_raw_boundary():
    converter = ArmTrajectoryConverter(
        joint_names=(
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'base_joint',
            'arm_joint_4',
        ),
        board_ids=(1, 1, 1, 1, 2),
        motor_ids=(0, 1, 2, 3, 0),
        min_positions_rad=(math.radians(-90.0),) * 5,
        max_positions_rad=(math.radians(180.0),) * 5,
        command_min_angle_raw=(-8500, -7810, -9150, -9000, -9000),
        command_max_angle_raw=(9000, 8000, 9000, 18000, 9000),
    )

    batch = converter.build_command_limit_entry_batch(
        (
            math.radians(-86.5),
            math.radians(-78.1),
            math.radians(-91.5),
            math.radians(-90.0),
            math.radians(-90.0),
        ),
        duration_ticks=60,
    )

    assert batch is not None
    assert batch.duration_ticks == 60
    assert [frame.can_id for frame in batch.frames] == [0x101] * 4
    assert [frame.data[0] for frame in batch.frames] == [
        0x80, 0x81, 0x82, 0x83,
    ]
    assert [frame_target(frame) for frame in batch.frames] == [
        -8500, -7810, -9150, -9000,
    ]
    assert all(frame.data[7] == 60 for frame in batch.frames)
    assert rad_to_angle_raw(batch.target_positions_rad[0]) == -8500


def test_post_home_escape_is_skipped_when_home_is_inside_raw_limits():
    converter = ArmTrajectoryConverter(
        joint_names=('arm_joint_4',),
        board_ids=(2,),
        motor_ids=(0,),
        min_positions_rad=(math.radians(-90.0),),
        max_positions_rad=(math.radians(90.0),),
        command_min_angle_raw=(-9000,),
        command_max_angle_raw=(9000,),
    )

    assert converter.build_command_limit_entry_batch(
        (math.radians(-90.0),),
        duration_ticks=60,
    ) is None


def test_clamps_out_of_limit_current_position_before_interpolation():
    converter = ArmTrajectoryConverter(
        joint_names=('arm_joint_3',),
        board_ids=(2,),
        motor_ids=(0,),
        min_positions_rad=(math.radians(-90.0),),
        max_positions_rad=(math.radians(90.0),),
        speed_raw=0,
        start_position_tolerance_rad=0.02,
        max_segment_duration_ticks=4,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = ['arm_joint_3']
    trajectory.points = [
        make_point((0.0,), 15_000_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(math.radians(-91.5),),
    )

    assert batches[0].target_positions_rad[0] >= math.radians(-90.0)
    assert frame_target(batches[0].frames[0]) >= -9000


def test_rejects_missing_or_unexpected_joint_names():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = [
        'arm_joint_1',
        'arm_joint_2',
        'arm_joint_3',
        'wrong_joint',
    ]
    trajectory.points = [
        make_point((0.0, 0.0, 0.0, 0.0), 50_000_000)
    ]

    with pytest.raises(
        TrajectoryConversionError,
        match='Joint name mismatch',
    ):
        converter.convert(
            trajectory,
            current_positions_rad=(0.0, 0.0, 0.0, 0.0),
        )


def test_rejects_non_increasing_time():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((0.1, 0.1, 0.1, 0.1), 50_000_000),
        make_point((0.2, 0.2, 0.2, 0.2), 50_000_000),
    ]

    with pytest.raises(
        TrajectoryConversionError,
        match='strictly increase',
    ):
        converter.convert(
            trajectory,
            current_positions_rad=(0.0, 0.0, 0.0, 0.0),
        )


def test_rejects_joint_limit_violation():
    converter = make_converter()

    trajectory = JointTrajectory()
    trajectory.joint_names = list(JOINT_NAMES)
    trajectory.points = [
        make_point((4.0, 0.0, 0.0, 0.0), 50_000_000)
    ]

    with pytest.raises(
        TrajectoryConversionError,
        match='exceeds the limit',
    ):
        converter.convert(
            trajectory,
            current_positions_rad=(0.0, 0.0, 0.0, 0.0),
        )


def test_accepts_tiny_rounding_error_at_joint_limit():
    converter = ArmTrajectoryConverter(
        joint_names=('arm_joint_2',),
        board_ids=(2,),
        motor_ids=(0,),
        min_positions_rad=(-1.39626340,),
        max_positions_rad=(1.39626340,),
        speed_raw=0,
        start_position_tolerance_rad=0.02,
    )

    trajectory = JointTrajectory()
    trajectory.joint_names = ['arm_joint_2']
    trajectory.points = [
        make_point((-1.3962634015954636,), 50_000_000)
    ]

    batches = converter.convert(
        trajectory,
        current_positions_rad=(-1.39626340,),
    )

    assert batches[0].target_positions_rad == (-1.39626340,)
