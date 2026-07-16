#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32

from vicpinky_interfaces.action import RunTask


class ElevatorFrontSequenceTest(Node):
    def __init__(self):
        super().__init__('elevator_front_sequence_test')

        self.nav_client = ActionClient(self, RunTask, '/nav/go_to')
        self.tag_align_client = ActionClient(self, RunTask, '/tag/align')
        self.rotate_client = ActionClient(self, RunTask, '/base/rotate')

        self.target_marker_id = 20

        # 마커가 너무 멀리서 보여도 Nav를 끊지 않도록 하는 조건
        self.nav_cancel_marker_distance_m = 1.5

        # 최종 정렬 목표
        self.align_target_distance_m = 1.45
        self.align_target_lateral_m = 0.0

        self.latest_marker_id = None
        self.latest_marker_time = 0.0
        self.latest_marker_distance = None
        self.latest_marker_distance_time = 0.0

        self.marker_sub = self.create_subscription(
            Int32,
            '/tag/marker_id',
            self.marker_callback,
            10
        )

        self.distance_sub = self.create_subscription(
            Float32,
            '/tag/target_distance',
            self.distance_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

    def marker_callback(self, msg):
        self.latest_marker_id = int(msg.data)
        self.latest_marker_time = time.time()

    def distance_callback(self, msg):
        self.latest_marker_distance = float(msg.data)
        self.latest_marker_distance_time = time.time()

    def marker_visible(self, marker_id=20, timeout_sec=0.8):
        return (
            self.latest_marker_id == marker_id
            and time.time() - self.latest_marker_time <= timeout_sec
        )

    def marker_distance_valid(self, timeout_sec=0.8):
        return (
            self.latest_marker_distance is not None
            and time.time() - self.latest_marker_distance_time <= timeout_sec
        )

    def marker_close_enough_for_nav_cancel(self):
        if not self.marker_visible(self.target_marker_id):
            return False

        if not self.marker_distance_valid():
            return False

        return self.latest_marker_distance <= self.nav_cancel_marker_distance_m

    def stop_robot(self):
        msg = Twist()
        for _ in range(8):
            self.cmd_vel_pub.publish(msg)
            self.cmd_vel_nav_pub.publish(msg)
            time.sleep(0.05)

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_nav_pub.publish(msg)

    def drive_forward_60cm(self):
        self.get_logger().info('Move forward 60cm before rotate.')

        speed = 0.05
        duration = 12.0  # 0.05m/s * 12s = 0.60m

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publish_cmd(linear_x=speed, angular_z=0.0)
            time.sleep(0.05)

        self.stop_robot()
        time.sleep(0.5)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'  feedback: {fb.phase} {fb.progress:.2f} {fb.detail}')

    def call_task(self, client, task_id, target_name, target_floor=4, marker_id=-1, extra=None):
        if extra is None:
            extra = {}

        self.get_logger().info(f'Waiting action server: {client._action_name}')
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server not available: {client._action_name}')
            return False

        goal = RunTask.Goal()
        goal.task_id = task_id
        goal.target_name = target_name
        goal.target_floor = int(target_floor)
        goal.marker_id = int(marker_id)
        goal.extra_json = json.dumps(extra)

        self.get_logger().info(f'Send goal: {client._action_name} / {task_id} / {target_name}')

        send_future = client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {client._action_name}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: success={result.success}, message={result.message}')
        return bool(result.success)

    def call_nav_until_marker_close(self):
        self.get_logger().info('Waiting action server: /nav/go_to')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available: /nav/go_to')
            return False

        goal = RunTask.Goal()
        goal.task_id = 'go_to'
        goal.target_name = 'elevator_front'
        goal.target_floor = 4
        goal.marker_id = -1
        goal.extra_json = '{}'

        self.get_logger().info('Send nav goal: go_to / elevator_front')
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav goal rejected')
            return False

        result_future = goal_handle.get_result_async()

        self.get_logger().info(
            f'Nav running... marker watch enabled. '
            f'Nav will cancel only when marker {self.target_marker_id} distance <= '
            f'{self.nav_cancel_marker_distance_m:.2f} m'
        )

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.marker_visible(self.target_marker_id) and self.marker_distance_valid():
                self.get_logger().info(
                    f'Marker {self.target_marker_id} visible. '
                    f'distance={self.latest_marker_distance:.3f} m'
                )

            if self.marker_close_enough_for_nav_cancel():
                self.get_logger().warn(
                    f'Marker {self.target_marker_id} close enough '
                    f'({self.latest_marker_distance:.3f} m). Cancel nav and switch to tag align.'
                )

                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)

                self.stop_robot()
                time.sleep(0.5)
                return True

        if result_future.done():
            result = result_future.result().result
            self.get_logger().info(f'Nav result: success={result.success}, message={result.message}')
            self.stop_robot()
            return bool(result.success)

        self.stop_robot()
        return False

    def search_marker(self, marker_id=20, timeout_sec=12.0):
        self.get_logger().info('Search marker after nav. Slow rotate until marker is visible.')

        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.marker_visible(marker_id=marker_id):
                if self.marker_distance_valid():
                    self.get_logger().info(
                        f'Marker {marker_id} found during search. '
                        f'distance={self.latest_marker_distance:.3f} m'
                    )
                else:
                    self.get_logger().info(f'Marker {marker_id} found during search.')

                self.stop_robot()
                time.sleep(0.5)
                return True

            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                self.stop_robot()
                self.get_logger().error('Marker search timeout.')
                return False

            # 천천히 왼쪽으로 탐색 회전
            self.publish_cmd(0.0, 0.12)
            time.sleep(0.05)

    def run(self):
        self.get_logger().info('========== ELEVATOR FRONT SEQUENCE TEST START ==========')

        ok = self.call_nav_until_marker_close()
        if not ok:
            self.get_logger().error('STOP: nav failed')
            return

        # Nav 중 가까운 거리에서 마커를 봤으면 바로 정렬.
        # Nav 도착 후에도 마커가 안 보이면 제자리 탐색.
        if not self.marker_visible(marker_id=self.target_marker_id):
            self.get_logger().warn(
                'Marker is not visible after nav, but continue to /tag/align. '
                'DockAlignServer will handle marker search/following.'
            )

        self.get_logger().info('Start tag align.')

        ok = self.call_task(
            self.tag_align_client,
            task_id='tag_align',
            target_name='elevator_front_marker',
            target_floor=4,
            marker_id=self.target_marker_id,
            extra={
                'target_distance_m': self.align_target_distance_m,
                'target_lateral_m': self.align_target_lateral_m
            }
        )
        if not ok:
            self.get_logger().error('STOP: tag align failed')
            return

        self.get_logger().info('MOCK: robot arm press elevator button')
        time.sleep(2.0)

        self.get_logger().info('MOCK: button brightness check success')
        time.sleep(1.0)

        self.drive_forward_60cm()

        ok = self.call_task(
            self.rotate_client,
            task_id='rotate',
            target_name='left_80',
            target_floor=4,
            marker_id=-1,
            extra={
                'angle_deg': 80
            }
        )

        if not ok:
            self.get_logger().error('STOP: rotate failed')
            return

        self.get_logger().info('========== ELEVATOR FRONT SEQUENCE TEST DONE ==========')


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorFrontSequenceTest()

    try:
        node.run()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()