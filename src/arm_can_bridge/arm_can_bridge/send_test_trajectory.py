"""Send small FollowJointTrajectory goals to arm_can_bridge."""

import math
import sys

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_ACTION_NAME = '/arm_controller/follow_joint_trajectory'
GRIPPER_ACTION_NAME = '/gripper_controller/follow_joint_trajectory'

ARM_JOINT_NAMES = [
    'base_joint',
    'arm_joint_1',
    'arm_joint_2',
    'arm_joint_3',
    'arm_joint_4',
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


class TestTrajectoryClient(Node):
    """Small CLI client for testing the arm and gripper trajectory actions."""

    def __init__(self):
        super().__init__('send_test_trajectory')
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
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

        arm_result = self._send_arm_goal()
        if arm_result != 0:
            return arm_result

        return self._send_gripper_goal()

    def _send_arm_goal(self) -> int:
        home = [
            math.radians(-90.0),
            math.radians(-90.0),
            math.radians(-80.0),
            math.radians(-90.0),
            math.radians(-170.0),
        ]

        points = [
            self._point(home, 0.0),
            self._point([
                math.radians(-60.0),
                math.radians(-60.0),
                math.radians(-40.0),
                math.radians(-60.0),
                math.radians(-120.0),
            ], 1.0),
            self._point([
                math.radians(-30.0),
                math.radians(-30.0),
                math.radians(0.0),
                math.radians(-30.0),
                math.radians(-60.0),
            ], 2.0),
            self._point(home, 3.0),
        ]

        return self._send_goal(
            label='arm',
            client=self._arm_client,
            joint_names=ARM_JOINT_NAMES,
            points=points,
        )

    def _send_gripper_goal(self) -> int:
        home = [0.0 for _ in GRIPPER_JOINT_NAMES]

        inner = [
            math.radians(-20.0),
            math.radians(-45.0),
            math.radians(-30.0),
            math.radians(-20.0),
            math.radians(-45.0),
            math.radians(-30.0),
            math.radians(-20.0),
            math.radians(-45.0),
            math.radians(-30.0),
        ]
        outer = [
            math.radians(20.0),
            math.radians(20.0),
            math.radians(30.0),
            math.radians(20.0),
            math.radians(20.0),
            math.radians(30.0),
            math.radians(20.0),
            math.radians(20.0),
            math.radians(30.0),
        ]

        points = [
            self._point(home, 0.0),
            self._point(inner, 1.0),
            self._point(outer, 2.0),
            self._point(home, 3.0),
        ]

        return self._send_goal(
            label='gripper',
            client=self._gripper_client,
            joint_names=GRIPPER_JOINT_NAMES,
            points=points,
        )

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
