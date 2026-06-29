import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from vicpinky_interfaces.action import ExecuteMission


class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client')

        # /mission/execute action server에 goal을 보내기 위한 client다.
        self.client = ActionClient(
            self,
            ExecuteMission,
            '/mission/execute'
        )

    def feedback_callback(self, feedback_msg):
        """
        mission_manager가 보내는 중간 진행 상황을 출력한다.

        예:
          state=PICK_OBJECT
          task=/arm/pick
          progress=0.23
        """
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'[feedback] '
            f'state={feedback.current_state}, '
            f'task={feedback.current_task}, '
            f'progress={feedback.progress:.2f}, '
            f'detail="{feedback.detail}"'
        )

    def send_goal(
        self,
        mission_id: str,
        pickup_location: str,
        delivery_location: str,
        target_floor: int,
        object_label: str,
        wait_server_timeout_sec: float
    ) -> int:
        """
        /mission/execute action server에 ExecuteMission goal을 보낸다.

        반환값:
          0 = 성공
          1 = 서버 없음 또는 goal 거절
          2 = 미션 실패
        """
        self.get_logger().info('Waiting for /mission/execute action server...')

        if not self.client.wait_for_server(timeout_sec=wait_server_timeout_sec):
            self.get_logger().error(
                f'/mission/execute action server not available '
                f'within {wait_server_timeout_sec} sec'
            )
            return 1

        goal_msg = ExecuteMission.Goal()

        goal_msg.mission_id = mission_id
        goal_msg.pickup_location = pickup_location
        goal_msg.delivery_location = delivery_location
        goal_msg.target_floor = int(target_floor)
        goal_msg.object_label = object_label

        self.get_logger().info(
            'Sending mission goal: '
            f'mission_id={goal_msg.mission_id}, '
            f'pickup={goal_msg.pickup_location}, '
            f'delivery={goal_msg.delivery_location}, '
            f'target_floor={goal_msg.target_floor}, '
            f'object={goal_msg.object_label}'
        )

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error('Failed to receive goal handle')
            return 1

        if not goal_handle.accepted:
            self.get_logger().error('Mission goal was rejected')
            return 1

        self.get_logger().info('Mission goal accepted')

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        result_response = result_future.result()

        if result_response is None:
            self.get_logger().error('Failed to receive mission result')
            return 2

        result = result_response.result

        if result.success:
            self.get_logger().info(
                f'Mission succeeded: '
                f'final_state={result.final_state}, '
                f'message="{result.message}"'
            )
            return 0

        self.get_logger().error(
            f'Mission failed: '
            f'final_state={result.final_state}, '
            f'message="{result.message}"'
        )
        return 2


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description='Send a delivery mission goal to /mission/execute'
    )

    parser.add_argument(
        '--mission-id',
        default='demo_001',
        help='Mission ID used for logs and status messages'
    )

    parser.add_argument(
        '--pickup-location',
        default='pickup_zone',
        help='Pickup location name used by mission_flow.yaml'
    )

    parser.add_argument(
        '--delivery-location',
        default='delivery_zone',
        help='Delivery location name used by mission_flow.yaml'
    )

    parser.add_argument(
        '--target-floor',
        type=int,
        default=2,
        help='Target floor number'
    )

    parser.add_argument(
        '--object',
        dest='object_label',
        default='box',
        help='Object label to pick'
    )

    parser.add_argument(
        '--wait-server-timeout-sec',
        type=float,
        default=5.0,
        help='Timeout for waiting /mission/execute action server'
    )

    return parser


def main(argv=None):
    parser = build_argument_parser()

    # parse_known_args를 쓰는 이유:
    # ROS2 실행 시 --ros-args 같은 ROS 전용 인자가 붙을 수 있기 때문이다.
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args if ros_args else None)

    node = MissionClient()

    try:
        exit_code = node.send_goal(
            mission_id=args.mission_id,
            pickup_location=args.pickup_location,
            delivery_location=args.delivery_location,
            target_floor=args.target_floor,
            object_label=args.object_label,
            wait_server_timeout_sec=args.wait_server_timeout_sec
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())