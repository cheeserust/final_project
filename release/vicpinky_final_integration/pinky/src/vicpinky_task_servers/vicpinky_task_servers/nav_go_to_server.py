#!/usr/bin/env python3

import json
import math
import os
import time
import yaml

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from vicpinky_interfaces.action import RunTask


class NavGoToServer(Node):
    def __init__(self):
        super().__init__('nav_go_to_server')

        self.declare_parameter('nav_points_file', '')
        self.declare_parameter('navigate_action_name', '/navigate_to_pose')
        self.declare_parameter('server_name', '/nav/go_to')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('mock_delay_sec', 2.0)

        self.nav_points_file = self.get_parameter('nav_points_file').value
        self.navigate_action_name = self.get_parameter('navigate_action_name').value
        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.mock_delay_sec = float(self.get_parameter('mock_delay_sec').value)

        if not self.nav_points_file:
            pkg_share = get_package_share_directory('vicpinky_task_servers')
            self.nav_points_file = os.path.join(pkg_share, 'config', 'nav_points.yaml')

        self.points = self.load_points(self.nav_points_file)
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action_name)

        self.action_server = ActionServer(
            self,
            RunTask,
            self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('========================================')
        self.get_logger().info(' Nav GoTo Action Server Started')
        self.get_logger().info(f' server_name          : {self.server_name}')
        self.get_logger().info(f' navigate_action_name : {self.navigate_action_name}')
        self.get_logger().info(f' nav_points_file      : {self.nav_points_file}')
        self.get_logger().info(f' mock_mode            : {self.mock_mode}')
        self.get_logger().info('========================================')

    def load_points(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f'nav_points.yaml not found: {path}')

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        return data.get('points', {})

    def goal_callback(self, goal_request):
        self.get_logger().info('')
        self.get_logger().info('========== /nav/go_to GOAL ==========')
        self.get_logger().info(f'task_id      : {goal_request.task_id}')
        self.get_logger().info(f'target_name  : {goal_request.target_name}')
        self.get_logger().info(f'target_floor : {goal_request.target_floor}')
        self.get_logger().info(f'marker_id    : {goal_request.marker_id}')
        self.get_logger().info(f'extra_json   : {goal_request.extra_json}')
        self.get_logger().info('=====================================')

        if goal_request.task_id not in ('go_to', 'navigate', 'nav_go_to'):
            self.get_logger().warn(f'Reject goal: invalid task_id={goal_request.task_id}')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('[NAV] Cancel requested.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = RunTask.Result()
        try:
            start_delay_sec = self.parse_start_delay_sec(
                goal_handle.request.extra_json
            )
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            result.success = False
            result.message = f'invalid navigation extra_json: {exc}'
            goal_handle.abort()
            return result

        if start_delay_sec > 0.0:
            feedback = RunTask.Feedback()
            feedback.phase = 'WAIT_MAP_SETTLE'
            feedback.progress = 0.05
            feedback.detail = (
                f'waiting {start_delay_sec:.1f}s before Nav2 goal'
            )
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'Waiting {start_delay_sec:.1f}s before Nav2 goal: '
                f'{goal_handle.request.target_name}'
            )
            if not self.wait_for_start_delay(goal_handle, start_delay_sec):
                result.success = False
                result.message = 'nav task canceled during start delay'
                goal_handle.canceled()
                return result

        if self.mock_mode:
            return await self.execute_mock(goal_handle)

        return await self.execute_real_nav2(goal_handle)

    @staticmethod
    def parse_start_delay_sec(extra_json):
        """Return the optional per-goal delay before sending a Nav2 goal."""
        if not extra_json:
            return 0.0
        extra = json.loads(extra_json)
        if not isinstance(extra, dict):
            raise ValueError('extra_json root must be a mapping')
        delay_sec = float(extra.get('start_delay_sec', 0.0))
        if not math.isfinite(delay_sec) or delay_sec < 0.0:
            raise ValueError(
                'start_delay_sec must be a finite, non-negative number'
            )
        return delay_sec

    @staticmethod
    def wait_for_start_delay(goal_handle, delay_sec):
        """Wait for map settling while remaining responsive to cancellation."""
        deadline = time.monotonic() + delay_sec
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(0.05, remaining))
        return False

    async def execute_mock(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()

        phases = [
            ('LOAD_TARGET', 0.2, 'loading target pose'),
            ('PLANNING', 0.4, 'planning path'),
            ('NAVIGATING', 0.7, 'moving to target'),
            ('ARRIVED', 1.0, 'arrived'),
        ]

        start_time = time.time()

        for phase, progress, detail in phases:
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'mock nav canceled'
                goal_handle.canceled()
                return result

            feedback = RunTask.Feedback()
            feedback.phase = phase
            feedback.progress = float(progress)
            feedback.detail = f'{detail}: {goal.target_name}'
            goal_handle.publish_feedback(feedback)

            time.sleep(self.mock_delay_sec / len(phases))

        elapsed = time.time() - start_time

        result.success = True
        result.message = f'[MOCK] arrived {goal.target_name} on floor {goal.target_floor} in {elapsed:.1f}s'
        goal_handle.succeed()
        return result

    async def execute_real_nav2(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()

        pose = self.make_pose(goal.target_floor, goal.target_name)
        if pose is None:
            result.success = False
            result.message = f'unknown nav target: floor={goal.target_floor}, target={goal.target_name}'
            goal_handle.abort()
            return result

        feedback = RunTask.Feedback()
        feedback.phase = 'WAIT_NAV2_SERVER'
        feedback.progress = 0.05
        feedback.detail = 'waiting /navigate_to_pose server'
        goal_handle.publish_feedback(feedback)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = '/navigate_to_pose action server not ready'
            goal_handle.abort()
            return result

        feedback.phase = 'SEND_NAV2_GOAL'
        feedback.progress = 0.15
        feedback.detail = f'sending goal: {goal.target_name}'
        goal_handle.publish_feedback(feedback)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose

        send_future = self.nav_client.send_goal_async(nav_goal)
        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            result.success = False
            result.message = 'Nav2 goal rejected'
            goal_handle.abort()
            return result

        feedback.phase = 'NAVIGATING'
        feedback.progress = 0.30
        feedback.detail = 'Nav2 goal accepted'
        goal_handle.publish_feedback(feedback)

        nav_result_future = nav_goal_handle.get_result_async()
        start_time = time.time()

        while not nav_result_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = nav_goal_handle.cancel_goal_async()
                await cancel_future
                result.success = False
                result.message = 'nav task canceled'
                goal_handle.canceled()
                return result

            elapsed = time.time() - start_time
            feedback = RunTask.Feedback()
            feedback.phase = 'NAVIGATING'
            feedback.progress = min(0.30 + elapsed / 60.0, 0.95)
            feedback.detail = f'navigating to {goal.target_name}, elapsed={elapsed:.1f}s'
            goal_handle.publish_feedback(feedback)

            time.sleep(1.0)

        nav_result = nav_result_future.result()

        feedback = RunTask.Feedback()
        feedback.phase = 'FINISH'
        feedback.progress = 1.0
        feedback.detail = f'Nav2 finished. status={nav_result.status}'
        goal_handle.publish_feedback(feedback)

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            result.success = True
            result.message = f'arrived: {goal.target_name}'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f'Nav2 failed. status={nav_result.status}'
            goal_handle.abort()

        return result

    def make_pose(self, floor, target_name):
        floor_str = str(int(floor))

        if floor_str not in self.points:
            self.get_logger().error(f'floor not found in nav_points.yaml: {floor_str}')
            return None

        floor_points = self.points[floor_str]

        if target_name not in floor_points:
            self.get_logger().error(f'target not found: floor={floor_str}, target={target_name}')
            return None

        p = floor_points[target_name]

        x = float(p['x'])
        y = float(p['y'])
        yaw = float(p['yaw'])
        frame_id = p.get('frame_id', 'map')

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = NavGoToServer()

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
