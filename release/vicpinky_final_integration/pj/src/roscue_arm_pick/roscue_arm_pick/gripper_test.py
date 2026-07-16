#!/usr/bin/env python3

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperTest(Node):

    def __init__(self):
        super().__init__('gripper_test')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        self.joint_names = [
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

    def send_gripper_goal(self, positions, duration_sec=2.0, load=500.0):
        self.get_logger().info('Waiting for /gripper_controller/follow_joint_trajectory...')

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available.')
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.effort = [load] * len(self.joint_names)
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)

        goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Positions: {positions}')
        self.get_logger().info(f'Effort/load: {load}')

        future = self.client.send_goal_async(goal_msg)
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
    node = GripperTest()

    try:
        open_pose = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]

        # 임시 close 값. 실제 물체 잡으면서 조정해야 함.
        # 모든 값이 min/max 안쪽에 있음.
        close_pose = [
            0.5, -0.8, 0.8,
            0.5, -0.8, 0.8,
            0.5, -0.8, 0.8,
        ]

        node.send_gripper_goal(open_pose, 2.0, load=300.0)
        node.send_gripper_goal(close_pose, 2.0, load=500.0)
        node.send_gripper_goal(open_pose, 2.0, load=300.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
