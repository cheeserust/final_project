#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from vicpinky_interfaces.action import RunTask


class BaseRotateServer(Node):
    def __init__(self):
        super().__init__('base_rotate_server')

        self.declare_parameter('server_name', '/base/rotate')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('angle_tolerance_deg', 3.0)
        self.declare_parameter('rotate_timeout_sec', 15.0)

        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.angle_tolerance_rad = math.radians(float(self.get_parameter('angle_tolerance_deg').value))
        self.rotate_timeout_sec = float(self.get_parameter('rotate_timeout_sec').value)

        self.latest_yaw = None

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.action_server = ActionServer(
            self,
            RunTask,
            self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Base Rotate Action Server Started.')
        self.get_logger().info(f'server_name: {self.server_name}')
        self.get_logger().info(f'mock_mode  : {self.mock_mode}')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.latest_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def goal_callback(self, goal_request):
        if goal_request.task_id not in ('rotate', 'turn', 'base_rotate'):
            self.get_logger().warn('Rejected: invalid task_id')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()

        angle_deg = 90.0
        try:
            if goal.extra_json:
                extra = json.loads(goal.extra_json)
                angle_deg = float(extra.get('angle_deg', angle_deg))
        except Exception as e:
            result.success = False
            result.message = f'invalid extra_json: {e}'
            goal_handle.abort()
            return result

        if goal.target_name in ('right_90', 'right', 'cw'):
            angle_deg = -abs(angle_deg)
        elif goal.target_name in ('left_90', 'left', 'ccw'):
            angle_deg = abs(angle_deg)

        if self.mock_mode:
            return self.run_mock(goal_handle, result, angle_deg)

        if self.latest_yaw is None:
            wait_start = time.time()
            while rclpy.ok() and self.latest_yaw is None:
                if time.time() - wait_start > 3.0:
                    result.success = False
                    result.message = 'odom not received'
                    goal_handle.abort()
                    return result
                self.publish_feedback(goal_handle, 'WAIT_ODOM', 0.05, 'waiting odom')
                time.sleep(0.1)

        start_yaw = self.latest_yaw
        target_delta = math.radians(angle_deg)
        target_yaw = self.normalize_angle(start_yaw + target_delta)
        direction = 1.0 if angle_deg > 0.0 else -1.0
        start_time = time.time()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                result.success = False
                result.message = 'base rotate canceled'
                goal_handle.canceled()
                return result

            if time.time() - start_time > self.rotate_timeout_sec:
                self.stop_robot()
                result.success = False
                result.message = 'base rotate timeout'
                goal_handle.abort()
                return result

            current_yaw = self.latest_yaw
            remaining = self.normalize_angle(target_yaw - current_yaw)

            if abs(remaining) <= self.angle_tolerance_rad:
                self.stop_robot()
                self.publish_feedback(goal_handle, 'ROTATED', 1.0, f'rotated {angle_deg:.1f} deg')
                result.success = True
                result.message = f'rotated {angle_deg:.1f} deg'
                goal_handle.succeed()
                return result

            cmd = Twist()
            cmd.angular.z = direction * self.angular_speed
            self.cmd_vel_pub.publish(cmd)

            turned = abs(self.normalize_angle(current_yaw - start_yaw))
            progress = min(0.95, turned / max(abs(target_delta), 0.001))
            self.publish_feedback(goal_handle, 'ROTATING', float(progress), f'remaining={math.degrees(remaining):.1f} deg')
            time.sleep(0.05)

        self.stop_robot()
        result.success = False
        result.message = 'rclpy shutdown'
        goal_handle.abort()
        return result

    def run_mock(self, goal_handle, result, angle_deg):
        for phase, progress, detail in [
            ('START_ROTATE', 0.25, f'mock rotate start {angle_deg:.1f} deg'),
            ('ROTATING', 0.60, 'mock rotating'),
            ('ROTATED', 1.00, 'mock rotated'),
        ]:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                result.success = False
                result.message = 'base rotate canceled'
                goal_handle.canceled()
                return result
            self.publish_feedback(goal_handle, phase, progress, detail)
            time.sleep(0.5)

        self.stop_robot()
        result.success = True
        result.message = f'mock rotated {angle_deg:.1f} deg'
        goal_handle.succeed()
        return result

    def publish_feedback(self, goal_handle, phase, progress, detail):
        feedback = RunTask.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = BaseRotateServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
