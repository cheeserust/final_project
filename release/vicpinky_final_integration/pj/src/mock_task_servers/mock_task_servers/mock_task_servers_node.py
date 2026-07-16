from functools import partial
import os
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vicpinky_interfaces.action import RunTask
import yaml


class MockTaskServers(Node):

    def __init__(self):
        super().__init__('mock_task_servers')

        self.callback_group = ReentrantCallbackGroup()
        self.action_servers = []

        default_config_path = os.path.join(
            get_package_share_directory('mock_task_servers'),
            'config',
            'mock_tasks.yaml'
        )

        self.declare_parameter('config_file', default_config_path)

        self.config_file = self.get_parameter(
            'config_file'
        ).get_parameter_value().string_value

        self.server_configs = self.load_config(self.config_file)
        self.create_action_servers()

        self.get_logger().info('Mock task servers are ready')

    def load_config(self, path):
        """
        mock_tasks.yaml 파일을 읽는다.

        이 파일에는 어떤 action server를 만들지,
        각 server가 몇 초 뒤 성공/실패할지 적혀 있다.
        """
        if not os.path.exists(path):
            raise FileNotFoundError(f'Mock task config not found: {path}')

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        if data is None:
            raise ValueError('mock_tasks.yaml is empty')

        if 'servers' not in data:
            raise ValueError('mock_tasks.yaml must contain "servers" key')

        self.get_logger().info(f'Loaded mock task config: {path}')
        return data['servers']

    def create_action_servers(self):
        """
        YAML에 적힌 action 이름대로 ActionServer를 만든다.

        예:
          /nav/go_to
          /dock/align
          /arm/pick
        """
        for action_name, config in self.server_configs.items():
            action_server = ActionServer(
                self,
                RunTask,
                action_name,
                execute_callback=partial(
                    self.execute_callback,
                    action_name=action_name
                ),
                goal_callback=partial(
                    self.goal_callback,
                    action_name=action_name
                ),
                cancel_callback=partial(
                    self.cancel_callback,
                    action_name=action_name
                ),
                callback_group=self.callback_group
            )

            self.action_servers.append(action_server)

            delay_sec = float(config.get('delay_sec', 1.0))
            success = bool(config.get('success', True))

            self.get_logger().info(
                f'Action server ready: {action_name}, '
                f'delay={delay_sec}s, success={success}'
            )

    def goal_callback(self, goal_request, action_name):
        """
        goal을 받을지 말지 결정하는 함수다.

        지금 mock server는 모든 goal을 받는다.
        나중에 실제 서버에서는 marker_id가 이상하거나 target_name이 없으면 reject할 수도 있다.
        """
        self.get_logger().info(
            f'Goal received on {action_name}: '
            f'task_id={goal_request.task_id}, '
            f'target_name={goal_request.target_name}, '
            f'target_floor={goal_request.target_floor}, '
            f'marker_id={goal_request.marker_id}'
        )

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle, action_name):
        """
        action은 service와 달리 실행 중 취소가 가능하다.

        중앙서버가 timeout을 감지하면 나중에 cancel을 보낼 수 있다.
        그래서 mock server도 cancel 요청을 받을 수 있게 만들어둔다.
        """
        self.get_logger().warn(f'Cancel requested on {action_name}')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle, action_name):
        """
        실제 작업을 수행하는 부분이다.

        mock server이기 때문에 실제 로봇을 움직이지 않고,
        feedback을 보내면서 delay_sec만큼 기다린 뒤 result를 반환한다.
        """
        config = self.server_configs[action_name]

        delay_sec = float(config.get('delay_sec', 1.0))
        success = bool(config.get('success', True))
        phases = config.get('phases', ['running'])

        if not phases:
            phases = ['running']

        goal = goal_handle.request

        self.get_logger().info(
            f'[{action_name}] Start mock task: '
            f'task_id={goal.task_id}, '
            f'target_name={goal.target_name}'
        )

        feedback_msg = RunTask.Feedback()

        phase_count = len(phases)
        sleep_per_phase = delay_sec / phase_count if phase_count > 0 else delay_sec

        for index, phase in enumerate(phases):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = RunTask.Result()
                result.success = False
                result.message = f'Canceled: {action_name}'

                self.get_logger().warn(result.message)
                return result

            progress = float(index + 1) / float(phase_count)

            feedback_msg.phase = str(phase)
            feedback_msg.progress = progress
            feedback_msg.detail = (
                f'Mock running {action_name}: '
                f'phase={phase}, '
                f'progress={progress:.2f}'
            )

            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'[{action_name}] feedback: '
                f'phase={feedback_msg.phase}, '
                f'progress={feedback_msg.progress:.2f}'
            )

            time.sleep(sleep_per_phase)

        result = RunTask.Result()

        if success:
            goal_handle.succeed()
            result.success = True
            result.message = f'Mock task succeeded: {action_name}'
            self.get_logger().info(result.message)
        else:
            goal_handle.abort()
            result.success = False
            result.message = f'Mock task failed: {action_name}'
            self.get_logger().error(result.message)

        return result


def main(args=None):
    rclpy.init(args=args)

    node = MockTaskServers()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
