#!/usr/bin/env python3

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vicpinky_interfaces.action import RunTask


def parse_drive_options(extra_json, default_speed):
    """Parse and validate a straight-drive action's numeric options."""
    extra = json.loads(extra_json) if extra_json else {}
    distance_m = float(extra.get('distance_m', 0.60))
    speed_mps = float(extra.get('speed_mps', default_speed))
    start_delay_sec = float(extra.get('start_delay_sec', 0.0))

    if not all(math.isfinite(value) for value in (
        distance_m,
        speed_mps,
        start_delay_sec,
    )):
        raise ValueError('drive options must be finite')

    return distance_m, speed_mps, max(0.0, start_delay_sec)


class BaseDriveStraightServer(Node):
    def __init__(self):
        super().__init__('base_drive_straight_server')

        self.declare_parameter('server_name', '/base/drive_straight')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('distance_tolerance_m', 0.03)
        self.declare_parameter('drive_timeout_sec', 20.0)
        self.declare_parameter('odom_stale_timeout_sec', 0.5)

        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.distance_tolerance_m = float(
            self.get_parameter('distance_tolerance_m').value
        )
        self.drive_timeout_sec = float(
            self.get_parameter('drive_timeout_sec').value
        )
        self.odom_stale_timeout_sec = float(
            self.get_parameter('odom_stale_timeout_sec').value
        )

        self.latest_xy = None
        self.last_odom_monotonic = 0.0
        self.cb_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.action_server = ActionServer(
            self,
            RunTask,
            self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Base Drive Straight Action Server Started.')
        self.get_logger().info(f'server_name: {self.server_name}')
        self.get_logger().info(f'mock_mode  : {self.mock_mode}')

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        self.latest_xy = (float(pose.x), float(pose.y))
        self.last_odom_monotonic = time.monotonic()

    def odom_is_fresh(self):
        return (
            self.latest_xy is not None
            and time.monotonic() - self.last_odom_monotonic
            <= self.odom_stale_timeout_sec
        )

    def goal_callback(self, goal_request):
        if goal_request.task_id not in (
            'drive_straight',
            'drive_forward',
            'base_drive',
        ):
            self.get_logger().warn('Rejected: invalid task_id')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.stop_robot()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()

        try:
            distance_m, speed_mps, start_delay_sec = parse_drive_options(
                goal.extra_json,
                self.linear_speed,
            )
        except Exception as exc:
            result.success = False
            result.message = f'invalid extra_json: {exc}'
            goal_handle.abort()
            return result

        if goal.target_name in ('backward', 'reverse') or distance_m < 0.0:
            direction = -1.0
        else:
            direction = 1.0

        distance_m = abs(distance_m)
        speed_mps = max(abs(speed_mps), 0.01)

        if start_delay_sec > 0.0:
            delay_started = time.monotonic()
            self.stop_robot()

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    result.success = False
                    result.message = 'base drive canceled during start delay'
                    goal_handle.canceled()
                    return result

                elapsed = time.monotonic() - delay_started
                remaining = start_delay_sec - elapsed
                if remaining <= 0.0:
                    break

                self.publish_feedback(
                    goal_handle,
                    'WAIT_START_DELAY',
                    0.0,
                    f'drive starts in {remaining:.1f} s',
                )
                time.sleep(min(0.05, remaining))

            if not rclpy.ok():
                self.stop_robot()
                result.success = False
                result.message = 'rclpy shutdown during start delay'
                goal_handle.abort()
                return result

        if self.mock_mode:
            return self.run_mock(goal_handle, result, distance_m, direction)

        if not self.odom_is_fresh():
            wait_start = time.time()
            while rclpy.ok() and not self.odom_is_fresh():
                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    result.success = False
                    result.message = 'base drive canceled while waiting odom'
                    goal_handle.canceled()
                    return result
                if time.time() - wait_start > 3.0:
                    self.stop_robot()
                    result.success = False
                    result.message = 'fresh odom not received'
                    goal_handle.abort()
                    return result
                self.publish_feedback(goal_handle, 'WAIT_ODOM', 0.05, 'waiting odom')
                time.sleep(0.1)

        start_xy = self.latest_xy
        start_time = time.time()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                result.success = False
                result.message = 'base drive canceled'
                goal_handle.canceled()
                return result

            elapsed = time.time() - start_time
            if elapsed > self.drive_timeout_sec:
                self.stop_robot()
                result.success = False
                result.message = 'base drive timeout'
                goal_handle.abort()
                return result

            if not self.odom_is_fresh():
                self.stop_robot()
                result.success = False
                result.message = 'odom became stale during base drive'
                goal_handle.abort()
                return result

            traveled = self.distance_from_start(start_xy)
            remaining = distance_m - traveled
            if remaining <= self.distance_tolerance_m:
                self.stop_robot()
                self.publish_feedback(
                    goal_handle,
                    'DRIVE_DONE',
                    1.0,
                    f'drove {traveled:.2f} m',
                )
                result.success = True
                result.message = f'drove {traveled:.2f} m'
                goal_handle.succeed()
                return result

            cmd = Twist()
            cmd.linear.x = direction * speed_mps
            self.cmd_vel_pub.publish(cmd)

            progress = min(0.95, traveled / max(distance_m, 0.001))
            self.publish_feedback(
                goal_handle,
                'DRIVING',
                float(progress),
                f'remaining={max(remaining, 0.0):.2f} m',
            )
            time.sleep(0.05)

        self.stop_robot()
        result.success = False
        result.message = 'rclpy shutdown'
        goal_handle.abort()
        return result

    def run_mock(self, goal_handle, result, distance_m, direction):
        label = 'forward' if direction > 0.0 else 'backward'
        for phase, progress, detail in [
            ('START_DRIVE', 0.25, f'mock {label} start {distance_m:.2f} m'),
            ('DRIVING', 0.65, f'mock {label} driving'),
            ('DRIVE_DONE', 1.00, f'mock drove {distance_m:.2f} m'),
        ]:
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                result.success = False
                result.message = 'base drive canceled'
                goal_handle.canceled()
                return result
            self.publish_feedback(goal_handle, phase, progress, detail)
            time.sleep(0.3)

        self.stop_robot()
        result.success = True
        result.message = f'mock drove {distance_m:.2f} m'
        goal_handle.succeed()
        return result

    def distance_from_start(self, start_xy):
        if self.latest_xy is None:
            return 0.0
        dx = self.latest_xy[0] - start_xy[0]
        dy = self.latest_xy[1] - start_xy[1]
        return math.hypot(dx, dy)

    def publish_feedback(self, goal_handle, phase, progress, detail):
        feedback = RunTask.Feedback()
        feedback.phase = phase
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)

    def stop_robot(self):
        stop = Twist()
        for _ in range(6):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriveStraightServer()
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
