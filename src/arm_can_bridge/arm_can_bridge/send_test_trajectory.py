"""Send two independent direct arm goals and one Board3 trajectory."""

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
from vicpinky_interfaces.action import ExecuteArmGoal


ARM_ACTION_NAME = '/arm_controller/execute_joint_goal'
GRIPPER_ACTION_NAME = '/gripper_controller/follow_joint_trajectory'
JOINT_STATES_TOPIC = '/joint_states'
CURRENT_POSITION_TIMEOUT_SEC = 3.0
TEST_DELTA_RAD = math.radians(10.0)

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


class TestTrajectoryClient(Node):
    """Small CLI client for testing the arm and gripper trajectory actions."""

    def __init__(self):
        super().__init__('send_test_trajectory')
        self._arm_client = ActionClient(
            self,
            ExecuteArmGoal,
            ARM_ACTION_NAME,
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            GRIPPER_ACTION_NAME,
        )

    def send(self) -> int:
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not available: {ARM_ACTION_NAME}'
            )
            return 1

        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Action server not available: {GRIPPER_ACTION_NAME}'
            )
            return 1

        try:
            arm_result = self._send_arm_goal()
            if arm_result != 0:
                return arm_result

            return self._send_gripper_goal()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return 1

    def _send_arm_goal(self) -> int:
        current = self._wait_for_current_positions(ARM_JOINT_NAMES)
        target = self._offset_positions(
            current,
            ARM_MIN_POSITIONS_RAD,
            ARM_MAX_POSITIONS_RAD,
        )

        result = self._send_arm_direct_goal('arm target', target, 1000)
        if result != 0:
            return result
        return self._send_arm_direct_goal('arm return', current, 1000)

    def _send_arm_direct_goal(
        self,
        label: str,
        positions: Sequence[float],
        duration_ms: int,
    ) -> int:
        goal = ExecuteArmGoal.Goal()
        goal.joint_names = list(ARM_JOINT_NAMES)
        goal.positions = [float(value) for value in positions]
        goal.duration_ms = int(duration_ms)
        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label} goal rejected')
            return 1
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f'{label} success={result.success}, message={result.message}'
        )
        return 0 if result.success else 2

    def _send_gripper_goal(self) -> int:
        current = self._wait_for_current_positions(GRIPPER_JOINT_NAMES)
        target = self._offset_positions(
            current,
            GRIPPER_MIN_POSITIONS_RAD,
            GRIPPER_MAX_POSITIONS_RAD,
            preferred_direction=-1,
        )

        points = [
            self._point(current, 0.0),
            self._point(target, 1.0),
            self._point(current, 2.0),
        ]

        return self._send_goal(
            label='gripper',
            client=self._gripper_client,
            joint_names=GRIPPER_JOINT_NAMES,
            points=points,
        )

    def _wait_for_current_positions(
        self,
        joint_names: Sequence[str],
    ) -> list[float]:
        target_names = tuple(str(name) for name in joint_names)
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

            if all(name in by_name for name in target_names):
                positions = [by_name[name] for name in target_names]

        subscription = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
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
                f'Timeout waiting for {JOINT_STATES_TOPIC} '
                f'with joints {list(target_names)}'
            )

        self.get_logger().info(
            f'Current positions for {list(target_names)}: {positions}'
        )
        return positions

    @staticmethod
    def _offset_positions(
        current_positions: Sequence[float],
        min_positions: Sequence[float],
        max_positions: Sequence[float],
        *,
        preferred_direction: int = 1,
    ) -> list[float]:
        targets = []
        primary_delta = TEST_DELTA_RAD
        if preferred_direction < 0:
            primary_delta = -TEST_DELTA_RAD
        secondary_delta = -primary_delta

        for current, minimum, maximum in zip(
            current_positions,
            min_positions,
            max_positions,
        ):
            current_value = float(current)
            minimum_value = float(minimum)
            maximum_value = float(maximum)

            if minimum_value <= current_value + primary_delta <= maximum_value:
                targets.append(current_value + primary_delta)
            elif minimum_value <= current_value + secondary_delta <= maximum_value:
                targets.append(current_value + secondary_delta)
            else:
                targets.append(current_value)

        return targets

    def _send_goal(
        self,
        *,
        label: str,
        client: ActionClient,
        joint_names: list[str],
        points: list[JointTrajectoryPoint],
    ) -> int:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)
        goal.trajectory.points = points

        future = client.send_goal_async(
            goal,
            feedback_callback=(
                lambda msg, active_label=label:
                self._feedback_callback(active_label, msg)
            ),
        )

        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f'{label} goal rejected')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        self.get_logger().info(
            f'{label} result error_code={result.error_code}, '
            f'error_string={result.error_string}'
        )

        return 0 if result.error_code == 0 else 2

    @staticmethod
    def _point(
        positions: list[float],
        time_from_start_s: float,
    ) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        whole_seconds = int(time_from_start_s)
        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = int(
            (time_from_start_s - whole_seconds) * 1_000_000_000
        )
        return point

    def _feedback_callback(self, label: str, msg) -> None:
        feedback = msg.feedback
        self.get_logger().info(
            f'{label} feedback actual={list(feedback.actual.positions)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestTrajectoryClient()

    try:
        code = node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return code


if __name__ == '__main__':
    sys.exit(main())
