"""Send one gripper FollowJointTrajectory target pose from the command line."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Sequence

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .can_protocol import BOARD3_TARGET_LOAD_MAX


GRIPPER_ACTION_NAME = '/gripper_controller/follow_joint_trajectory'
JOINT_STATES_TOPIC = '/joint_states'
CURRENT_POSITION_TIMEOUT_SEC = 3.0

GRIPPER_JOINT_NAMES = [
    'finger_1_base_joint',
    'finger_1_middle_joint',
    'finger_1_tip_joint',
    'finger_2_base_joint',
    'finger_2_middle_joint',
    'finger_2_tip_joint',
    'finger_3_base_joint',
    'finger_3_middle_joint',
    'finger_3_tip_joint',
]

GRIPPER_MIN_POSITIONS_RAD = [
    math.radians(-70.3),
    math.radians(-137.7),
    math.radians(-111.3),
    math.radians(-70.3),
    math.radians(-137.7),
    math.radians(-111.3),
    math.radians(-70.3),
    math.radians(-137.7),
    math.radians(-111.3),
]

GRIPPER_MAX_POSITIONS_RAD = [
    math.radians(70.3),
    math.radians(52.7),
    math.radians(111.3),
    math.radians(70.3),
    math.radians(52.7),
    math.radians(111.3),
    math.radians(70.3),
    math.radians(52.7),
    math.radians(111.3),
]


class GripperPoseClient(Node):
    """CLI client that sends one gripper target pose."""

    def __init__(
        self,
        *,
        action_name: str,
        joint_states_topic: str,
    ) -> None:
        super().__init__('send_gripper_pose')
        self._action_name = str(action_name)
        self._joint_states_topic = str(joint_states_topic)
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            self._action_name,
        )

    def send(
        self,
        *,
        target_positions_rad: Sequence[float] | None,
        relative_offsets_rad: Sequence[float] | None,
        close_step_rad: float | None,
        open_step_rad: float | None,
        duration_s: float,
        target_load_raw: int | None,
    ) -> int:
        """Send one absolute, relative, close, or open gripper pose goal."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not available: {self._action_name}'
            )
            return 1

        try:
            current = self._wait_for_current_positions()
            if close_step_rad is not None:
                target = self._offset_and_clamp(current, -abs(close_step_rad))
            elif open_step_rad is not None:
                target = self._offset_and_clamp(current, abs(open_step_rad))
            elif relative_offsets_rad is not None:
                target = [
                    current_value + offset
                    for current_value, offset in zip(
                        current,
                        relative_offsets_rad,
                    )
                ]
            elif target_positions_rad is not None:
                target = [float(value) for value in target_positions_rad]
            else:
                raise RuntimeError('A gripper target is required')

            self._validate_target(target)
            return self._send_goal(
                current,
                target,
                duration_s,
                target_load_raw,
            )
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return 1

    def _wait_for_current_positions(self) -> list[float]:
        positions: list[float] | None = None

        def callback(msg: JointState) -> None:
            nonlocal positions

            if positions is not None:
                return

            by_name = {
                str(name): float(msg.position[index])
                for index, name in enumerate(msg.name)
                if index < len(msg.position)
            }

            if all(name in by_name for name in GRIPPER_JOINT_NAMES):
                positions = [by_name[name] for name in GRIPPER_JOINT_NAMES]

        subscription = self.create_subscription(
            JointState,
            self._joint_states_topic,
            callback,
            10,
        )
        deadline = time.monotonic() + CURRENT_POSITION_TIMEOUT_SEC

        try:
            while (
                positions is None
                and rclpy.ok()
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(subscription)

        if positions is None:
            raise RuntimeError(
                f'Timeout waiting for {self._joint_states_topic} '
                f'with joints {GRIPPER_JOINT_NAMES}'
            )

        self.get_logger().info(
            'Current gripper degrees: '
            f'{[round(math.degrees(value), 3) for value in positions]}'
        )
        return positions

    @staticmethod
    def _offset_and_clamp(
        current: Sequence[float],
        offset_rad: float,
    ) -> list[float]:
        target = []
        for value, minimum, maximum in zip(
            current,
            GRIPPER_MIN_POSITIONS_RAD,
            GRIPPER_MAX_POSITIONS_RAD,
        ):
            target.append(
                min(max(float(value) + offset_rad, minimum), maximum)
            )
        return target

    def _validate_target(self, target: Sequence[float]) -> None:
        if len(target) != len(GRIPPER_JOINT_NAMES):
            raise RuntimeError(
                f'Expected {len(GRIPPER_JOINT_NAMES)} target values, '
                f'got {len(target)}'
            )

        for name, value, minimum, maximum in zip(
            GRIPPER_JOINT_NAMES,
            target,
            GRIPPER_MIN_POSITIONS_RAD,
            GRIPPER_MAX_POSITIONS_RAD,
        ):
            if not math.isfinite(float(value)):
                raise RuntimeError(f'{name} target is not finite')
            if value < minimum or value > maximum:
                raise RuntimeError(
                    f'{name} target {math.degrees(value):.3f} deg '
                    f'is outside [{math.degrees(minimum):.3f}, '
                    f'{math.degrees(maximum):.3f}] deg'
                )

    def _send_goal(
        self,
        current: Sequence[float],
        target: Sequence[float],
        duration_s: float,
        target_load_raw: int | None,
    ) -> int:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(GRIPPER_JOINT_NAMES)
        goal.trajectory.points = [
            self._point(current, 0.0),
            self._point(target, duration_s, target_load_raw),
        ]

        self.get_logger().info(
            'Target gripper degrees: '
            f'{[round(math.degrees(value), 3) for value in target]}'
        )
        if target_load_raw is not None:
            self.get_logger().info(
                f'Target gripper load raw: {target_load_raw}'
            )

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('gripper goal rejected')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f'gripper result error_code={result.error_code}, '
            f'error_string={result.error_string}'
        )
        return 0 if result.error_code == 0 else 2

    @staticmethod
    def _point(
        positions: Sequence[float],
        time_from_start_s: float,
        target_load_raw: int | None = None,
    ) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        if target_load_raw is not None:
            point.effort = [float(target_load_raw) for _ in positions]
        whole_seconds = int(time_from_start_s)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (time_from_start_s - whole_seconds) * 1_000_000_000
        )
        return point

    def _feedback_callback(self, msg) -> None:
        feedback = msg.feedback
        self.get_logger().info(
            'gripper feedback actual degrees='
            f'{[round(math.degrees(value), 3) for value in feedback.actual.positions]}'
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Send one gripper target pose to arm_can_bridge.'
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        '--degrees',
        nargs=9,
        type=float,
        metavar=(
            'F1B',
            'F1M',
            'F1T',
            'F2B',
            'F2M',
            'F2T',
            'F3B',
            'F3M',
            'F3T',
        ),
        help='Absolute target gripper joint angles in degrees.',
    )
    target_group.add_argument(
        '--relative-degrees',
        nargs=9,
        type=float,
        metavar=(
            'F1B',
            'F1M',
            'F1T',
            'F2B',
            'F2M',
            'F2T',
            'F3B',
            'F3M',
            'F3T',
        ),
        help='Relative gripper joint angle offsets in degrees.',
    )
    target_group.add_argument(
        '--close',
        action='store_true',
        help='Move every gripper joint in the negative, closing direction.',
    )
    target_group.add_argument(
        '--open',
        action='store_true',
        help='Move every gripper joint in the positive, opening direction.',
    )
    parser.add_argument(
        '--step',
        type=float,
        default=10.0,
        help='Step in degrees for --close or --open. Default: 10.0',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=1.5,
        help='Trajectory duration in seconds. Default: 1.5',
    )
    parser.add_argument(
        '--target-load',
        type=int,
        default=None,
        help=(
            'Board3 target load raw value 0..1023. If omitted, '
            'arm_can_bridge uses gripper_target_load_raw.'
        ),
    )
    parser.add_argument(
        '--action-name',
        default=GRIPPER_ACTION_NAME,
        help=f'FollowJointTrajectory action name. Default: {GRIPPER_ACTION_NAME}',
    )
    parser.add_argument(
        '--joint-states-topic',
        default=JOINT_STATES_TOPIC,
        help=f'JointState topic. Default: {JOINT_STATES_TOPIC}',
    )
    return parser


def _degrees_to_radians(values: Sequence[float] | None) -> list[float] | None:
    if values is None:
        return None
    return [math.radians(float(value)) for value in values]


def main(args=None):
    """Run the command-line client."""
    parser = _build_parser()
    parsed_args = parser.parse_args(args=args)

    if parsed_args.duration <= 0.0:
        parser.error('--duration must be greater than 0')
    if parsed_args.step <= 0.0:
        parser.error('--step must be greater than 0')
    if (
        parsed_args.target_load is not None
        and not 0 <= parsed_args.target_load <= BOARD3_TARGET_LOAD_MAX
    ):
        parser.error(
            f'--target-load must be in 0..{BOARD3_TARGET_LOAD_MAX}'
        )

    close_step_rad = None
    open_step_rad = None
    if parsed_args.close:
        close_step_rad = math.radians(float(parsed_args.step))
    if parsed_args.open:
        open_step_rad = math.radians(float(parsed_args.step))

    rclpy.init(args=None)
    node = GripperPoseClient(
        action_name=parsed_args.action_name,
        joint_states_topic=parsed_args.joint_states_topic,
    )

    try:
        code = node.send(
            target_positions_rad=_degrees_to_radians(parsed_args.degrees),
            relative_offsets_rad=_degrees_to_radians(
                parsed_args.relative_degrees
            ),
            close_step_rad=close_step_rad,
            open_step_rad=open_step_rad,
            duration_s=float(parsed_args.duration),
            target_load_raw=parsed_args.target_load,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
