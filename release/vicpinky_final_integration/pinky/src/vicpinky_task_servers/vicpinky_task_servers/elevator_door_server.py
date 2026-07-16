#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from vicpinky_interfaces.action import RunTask


class ElevatorDoorServer(Node):
    def __init__(self):
        super().__init__('elevator_door_server')

        self.declare_parameter('server_name', '/elevator/wait_door_open')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('mock_delay_sec', 3.0)
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.declare_parameter('distance_threshold', 1.2)
        self.declare_parameter('min_open_count', 15)
        self.declare_parameter('angle_min', -0.35)
        self.declare_parameter('angle_max', 0.35)
        self.declare_parameter('timeout_sec', 30.0)

        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.mock_delay_sec = float(self.get_parameter('mock_delay_sec').value)
        self.scan_topic = self.get_parameter('scan_topic').value
        self.distance_threshold = float(self.get_parameter('distance_threshold').value)
        self.min_open_count = int(self.get_parameter('min_open_count').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.cb_group = ReentrantCallbackGroup()

        self.latest_scan = None
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10,
                                 callback_group=self.cb_group)

        self.action_server = ActionServer(
            self,
            RunTask,
            self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Elevator Door Action Server Started.')
        self.get_logger().info(f'server_name: {self.server_name}')
        self.get_logger().info(f'mock_mode: {self.mock_mode}')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def goal_callback(self, goal_request):
        if goal_request.task_id not in ('wait_door_open', 'door_open'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = RunTask.Result()
        start_time = time.time()

        if self.mock_mode:
            phases = [
                ('WAITING', 0.2, 'waiting elevator door'),
                ('CHECKING_LIDAR', 0.6, 'checking door gap'),
                ('DOOR_OPEN', 1.0, 'door opened'),
            ]
            for phase, progress, detail in phases:
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = 'wait door canceled'
                    goal_handle.canceled()
                    return result

                feedback = RunTask.Feedback()
                feedback.phase = phase
                feedback.progress = progress
                feedback.detail = detail
                goal_handle.publish_feedback(feedback)
                time.sleep(self.mock_delay_sec / len(phases))

            result.success = True
            result.message = '[MOCK] elevator door opened'
            goal_handle.succeed()
            return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'wait door canceled'
                goal_handle.canceled()
                return result

            elapsed = time.time() - start_time
            if elapsed > self.timeout_sec:
                result.success = False
                result.message = 'door open timeout'
                goal_handle.abort()
                return result

            door_open, open_count = self.is_door_open()

            feedback = RunTask.Feedback()
            feedback.phase = 'WAITING_DOOR'
            feedback.progress = min(elapsed / self.timeout_sec, 0.99)
            feedback.detail = f'open_count={open_count}'
            goal_handle.publish_feedback(feedback)

            if door_open:
                result.success = True
                result.message = 'elevator door opened'
                goal_handle.succeed()
                return result

            time.sleep(0.2)

    def is_door_open(self):
        if self.latest_scan is None:
            return False, 0

        open_count = 0
        msg = self.latest_scan

        for i, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            if self.angle_min <= angle <= self.angle_max:
                if distance >= self.distance_threshold:
                    open_count += 1

        return open_count >= self.min_open_count, open_count


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorDoorServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
