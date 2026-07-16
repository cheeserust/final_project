"""Send one direct Board1 V3 + Board2 legacy target from the CLI."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from vicpinky_interfaces.action import ExecuteArmGoal


ARM_ACTION_NAME = '/arm_controller/execute_joint_goal'
JOINT_STATES_TOPIC = '/joint_states'
CURRENT_POSITION_TIMEOUT_SEC = 3.0

ARM_JOINT_NAMES = [
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
]

ARM_MIN_POSITIONS_RAD = [
    math.radians(-90.0),
    math.radians(-86.5),
    math.radians(-78.1),
    math.radians(-91.5),
    math.radians(-90.0),
]

ARM_MAX_POSITIONS_RAD = [
    math.radians(180.0),
    math.radians(90.0),
    math.radians(80.0),
    math.radians(90.0),
    math.radians(90.0),
]


class ArmPoseClient(Node):
    """CLI client that sends one arm target pose."""

    def __init__(
        self,
        *,
        action_name: str,
        joint_states_topic: str,
    ) -> None:
        super().__init__('send_arm_pose')
        self._action_name = str(action_name)
        self._joint_states_topic = str(joint_states_topic)
        self._client = ActionClient(
            self,
            ExecuteArmGoal,
            self._action_name,
        )

    def send(
        self,
        *,
        target_positions_rad: Sequence[float] | None,
        relative_offsets_rad: Sequence[float] | None,
        duration_s: float,
    ) -> int:
        """Send one absolute or relative arm pose goal."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not available: {self._action_name}'
            )
            return 1

        try:
            current = self._wait_for_current_positions()
            if relative_offsets_rad is not None:
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
                raise RuntimeError('target_positions_rad is required')

            self._validate_target(target)
            return self._send_goal(target, duration_s)
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

            if all(name in by_name for name in ARM_JOINT_NAMES):
                positions = [by_name[name] for name in ARM_JOINT_NAMES]

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
                f'with joints {ARM_JOINT_NAMES}'
            )

        self.get_logger().info(
            'Current arm degrees: '
            f'{[round(math.degrees(value), 3) for value in positions]}'
        )
        return positions

    def _validate_target(self, target: Sequence[float]) -> None:
        if len(target) != len(ARM_JOINT_NAMES):
            raise RuntimeError(
                f'Expected {len(ARM_JOINT_NAMES)} target values, '
                f'got {len(target)}'
            )

        for name, value, minimum, maximum in zip(
            ARM_JOINT_NAMES,
            target,
            ARM_MIN_POSITIONS_RAD,
            ARM_MAX_POSITIONS_RAD,
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
        target: Sequence[float],
        duration_s: float,
    ) -> int:
        duration_ms = round(float(duration_s) * 1000.0)
        if not 1 <= duration_ms <= 0xFFFF:
            raise RuntimeError('duration must be 1..65535 ms')
        goal = ExecuteArmGoal.Goal()
        goal.joint_names = list(ARM_JOINT_NAMES)
        goal.positions = [float(value) for value in target]
        goal.duration_ms = int(duration_ms)

        self.get_logger().info(
            'Target arm degrees: '
            f'{[round(math.degrees(value), 3) for value in target]}'
        )

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('arm goal rejected')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(
            f'arm result success={result.success}, message={result.message}'
        )
        return 0 if result.success else 2

    def _feedback_callback(self, msg) -> None:
        feedback = msg.feedback
        self.get_logger().info(
            f'arm feedback goal={feedback.goal_id} '
            f'phase={feedback.phase} detail={feedback.detail}'
        )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Send one arm target pose to arm_can_bridge.'
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        '--degrees',
        nargs=5,
        type=float,
        metavar=('BASE', 'J1', 'J2', 'J3', 'J4'),
        help='Absolute target joint angles in degrees.',
    )
    target_group.add_argument(
        '--relative-degrees',
        nargs=5,
        type=float,
        metavar=('BASE', 'J1', 'J2', 'J3', 'J4'),
        help='Relative joint angle offsets in degrees from current position.',
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=2.0,
        help='Trajectory duration in seconds. Default: 2.0',
    )
    parser.add_argument(
        '--action-name',
        default=ARM_ACTION_NAME,
        help=f'FollowJointTrajectory action name. Default: {ARM_ACTION_NAME}',
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

    rclpy.init(args=None)
    node = ArmPoseClient(
        action_name=parsed_args.action_name,
        joint_states_topic=parsed_args.joint_states_topic,
    )

    try:
        code = node.send(
            target_positions_rad=_degrees_to_radians(parsed_args.degrees),
            relative_offsets_rad=_degrees_to_radians(
                parsed_args.relative_degrees
            ),
            duration_s=float(parsed_args.duration),
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
