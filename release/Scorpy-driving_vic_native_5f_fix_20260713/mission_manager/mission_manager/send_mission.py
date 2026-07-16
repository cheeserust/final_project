import argparse
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from vicpinky_interfaces.action import ExecuteMission
import yaml


DEFAULT_FLOOR = 4
DEFAULT_PICKUP_LOCATION = 'object'
DEFAULT_DELIVERY_LOCATION = 'object_place'

FALLBACK_LOCATION_FLOORS = {
    'dock': 4,
    'dock_4f': 4,
    'home': 4,
    'home_4f': 4,
    'object': 4,
    'room_401': 4,
    'room_402': 4,
    'elevator_front_4f': 4,
    'floor_4_marker': 4,
    'map_4f': 4,
    'pickup_zone': 4,
    'dock_5f': 5,
    'object_place': 5,
    'object_place_5f': 5,
    'room_501': 5,
    'elevator_front_5f': 5,
    'floor_5_marker': 5,
    'map_5f': 5,
}


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
          state=PLACE_OBJECT_AT_DELIVERY
          task=/arm/place
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


def _locations_file_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        package_share = get_package_share_directory('mission_manager')
        return os.path.join(package_share, 'config', 'locations.yaml')
    except Exception:
        package_root = os.path.dirname(os.path.dirname(__file__))
        return os.path.join(package_root, 'config', 'locations.yaml')


def _load_location_floors() -> dict[str, int]:
    floors = dict(FALLBACK_LOCATION_FLOORS)
    path = _locations_file_path()

    try:
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except Exception:
        return floors

    locations = data.get('locations', {}) if isinstance(data, dict) else {}
    if not isinstance(locations, dict):
        return floors

    for name, location in locations.items():
        if not isinstance(location, dict):
            continue

        if 'pose' not in location:
            continue

        floor = location.get('floor')
        if isinstance(floor, int):
            floors[str(name)] = floor
        elif isinstance(floor, str) and floor.isdigit():
            floors[str(name)] = int(floor)

    return floors


def _infer_target_floor(
    delivery_location: str,
    target_floor: int | None,
    location_floors: dict[str, int],
) -> int:
    if target_floor is not None:
        return int(target_floor)

    if delivery_location in location_floors:
        return location_floors[delivery_location]

    return DEFAULT_FLOOR


def _print_locations(location_floors: dict[str, int]) -> None:
    print('Available mission locations:')
    for name in sorted(location_floors):
        print(f'  {name}: floor {location_floors[name]}')


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
        default=DEFAULT_PICKUP_LOCATION,
        help='Pickup location name used by mission_flow.yaml'
    )

    parser.add_argument(
        '--delivery-location',
        default=DEFAULT_DELIVERY_LOCATION,
        help='Delivery location name used by mission_flow.yaml'
    )

    parser.add_argument(
        '--target-floor',
        type=int,
        default=None,
        help=(
            'Target floor number. If omitted, it is inferred from '
            'the delivery location; unknown locations default to floor 4.'
        )
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

    parser.add_argument(
        '--list-locations',
        action='store_true',
        help='Print configured location names and exit'
    )

    return parser


def main(argv=None):
    parser = build_argument_parser()

    # parse_known_args를 쓰는 이유:
    # ROS2 실행 시 --ros-args 같은 ROS 전용 인자가 붙을 수 있기 때문이다.
    args, ros_args = parser.parse_known_args(argv)

    location_floors = _load_location_floors()

    if args.list_locations:
        _print_locations(location_floors)
        return 0

    target_floor = _infer_target_floor(
        delivery_location=args.delivery_location,
        target_floor=args.target_floor,
        location_floors=location_floors,
    )

    rclpy.init(args=ros_args if ros_args else None)

    node = MissionClient()

    try:
        exit_code = node.send_goal(
            mission_id=args.mission_id,
            pickup_location=args.pickup_location,
            delivery_location=args.delivery_location,
            target_floor=target_floor,
            object_label=args.object_label,
            wait_server_timeout_sec=args.wait_server_timeout_sec
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
