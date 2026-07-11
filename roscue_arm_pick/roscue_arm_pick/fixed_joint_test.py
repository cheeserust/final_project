#!/usr/bin/env python3

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class FixedJointTest(Node):

    def __init__(self):
        super().__init__('fixed_joint_test')

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.joint_names = [
            'arm_joint_1',
            'arm_joint_2',
            'arm_joint_3',
            'base_joint',
            'arm_joint_4',
        ]

    def send_arm_goal(self, positions, duration_sec=3.0):
        self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory...')

        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm action server not available.')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Joint names: {self.joint_names}')
        self.get_logger().info(f'Positions: {positions}')

        future = self.arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('Goal handle is None.')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by arm_can_bridge.')
            return False

        self.get_logger().info('Goal accepted.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result error_code: {result.error_code}')
        self.get_logger().info(f'Result error_string: {result.error_string}')

        return True


def main():
    rclpy.init()
    node = FixedJointTest()

    try:
        # bridge 기준 home pose
        home = [
            -1.50098316,
            -1.38055544,
            -1.57079633,
            -1.57079633,
            -1.57079633,
        ]

        # limit 안쪽의 아주 작은 테스트 자세
        # 너무 크게 움직이지 않게 home 근처로 잡음
        test_pose_1 = [
            -1.30,
            -1.20,
            -1.35,
            -1.30,
            -1.30,
        ]

        test_pose_2 = [
            -1.10,
            -1.00,
            -1.10,
            -1.00,
            -1.00,
        ]

        node.send_arm_goal(home, 3.0)
        node.send_arm_goal(test_pose_1, 3.0)
        node.send_arm_goal(test_pose_2, 3.0)
        node.send_arm_goal(home, 3.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
