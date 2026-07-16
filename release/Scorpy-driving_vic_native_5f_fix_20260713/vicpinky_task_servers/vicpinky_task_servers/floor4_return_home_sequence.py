#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from vicpinky_interfaces.action import RunTask


class Floor4ReturnHomeSequence(Node):
    def __init__(self):
        super().__init__('floor4_return_home_sequence')

        self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.rotate_client = ActionClient(self, RunTask, '/base/rotate')
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        self.mid_x = 1.40
        self.mid_y = -6.20
        self.mid_yaw = -1.363

        self.goal_x = 2.998
        self.goal_y = -12.433
        self.goal_yaw = -1.363

    def publish_status(self, text):
        self.get_logger().info(f'[STATE] {text}')
        self.status_pub.publish(String(data=text))

    def yaw_to_quat(self, yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def nav_to_pose(self, x, y, yaw, name):
        self.publish_status(f'NAV_TO_{name}')

        if not self.nav2_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/navigate_to_pose server not available')
            return False

        qz, qw = self.yaw_to_quat(yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav2_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Nav2 goal rejected: {name}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        self.get_logger().info(f'Nav2 {name} finished. status={status}')
        return status == 4

    def rotate_180(self):
        self.publish_status('ROTATE_180_TO_FACE_ELEVATOR')

        if not self.rotate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/base/rotate server not available')
            return False

        goal = RunTask.Goal()
        goal.task_id = 'rotate'
        goal.target_name = 'turn_180_at_402'
        goal.target_floor = 4
        goal.marker_id = -1
        goal.extra_json = json.dumps({'angle_deg': 180})

        send_future = self.rotate_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Rotate goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Rotate result: success={result.success}, message={result.message}')
        return bool(result.success)

    def run(self):
        self.publish_status('FLOOR4_RETURN_HOME_START')

        if not self.nav_to_pose(self.mid_x, self.mid_y, self.mid_yaw, 'RETURN_MID'):
            self.publish_status('FAILED_NAV_RETURN_MID')
            return

        if not self.nav_to_pose(self.goal_x, self.goal_y, self.goal_yaw, '402'):
            self.publish_status('FAILED_NAV_402')
            return

        if not self.rotate_180():
            self.publish_status('FAILED_ROTATE_180')
            return

        self.publish_status('FLOOR4_RETURN_HOME_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Floor4ReturnHomeSequence()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()