#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32
from vicpinky_interfaces.action import RunTask


class FloorCheckServer(Node):
    def __init__(self):
        super().__init__('floor_check_server')

        self.declare_parameter('server_name', '/floor/check')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('mock_delay_sec', 2.0)
        self.declare_parameter('tag_topic', '/tag/floor_id')
        self.declare_parameter('timeout_sec', 120.0)

        self.server_name = self.get_parameter('server_name').value
        self.mock_mode = bool(self.get_parameter('mock_mode').value)
        self.mock_delay_sec = float(self.get_parameter('mock_delay_sec').value)
        self.tag_topic = self.get_parameter('tag_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.cb_group = ReentrantCallbackGroup()

        self.current_floor = None
        self.create_subscription(Int32, self.tag_topic, self.tag_callback, 10,
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

        self.get_logger().info('Floor Check Action Server Started.')
        self.get_logger().info(f'server_name: {self.server_name}')
        self.get_logger().info(f'mock_mode: {self.mock_mode}')

    def tag_callback(self, msg):
        self.current_floor = int(msg.data)

    def goal_callback(self, goal_request):
        if goal_request.task_id not in ('check_floor', 'floor_check'):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()
        start_time = time.time()

        if self.mock_mode:
            phases = [
                ('WAIT_TAG', 0.3, 'waiting floor tag'),
                ('READ_TAG', 0.7, f'read floor {goal.target_floor}'),
                ('MATCHED', 1.0, 'target floor matched'),
            ]

            for phase, progress, detail in phases:
                if goal_handle.is_cancel_requested:
                    result.success = False
                    result.message = 'floor check canceled'
                    goal_handle.canceled()
                    return result

                feedback = RunTask.Feedback()
                feedback.phase = phase
                feedback.progress = progress
                feedback.detail = detail
                goal_handle.publish_feedback(feedback)
                time.sleep(self.mock_delay_sec / len(phases))

            result.success = True
            result.message = f'[MOCK] current floor is {goal.target_floor}'
            goal_handle.succeed()
            return result

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'floor check canceled'
                goal_handle.canceled()
                return result

            elapsed = time.time() - start_time
            if elapsed > self.timeout_sec:
                result.success = False
                result.message = 'floor check timeout'
                goal_handle.abort()
                return result

            feedback = RunTask.Feedback()
            feedback.phase = 'CHECKING_FLOOR'
            feedback.progress = min(elapsed / self.timeout_sec, 0.99)
            feedback.detail = f'current={self.current_floor}, target={goal.target_floor}'
            goal_handle.publish_feedback(feedback)

            if self.current_floor == goal.target_floor:
                result.success = True
                result.message = f'floor matched: {goal.target_floor}'
                goal_handle.succeed()
                return result

            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = FloorCheckServer()
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
