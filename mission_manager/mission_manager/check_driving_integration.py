import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from vicpinky_interfaces.action import ExecuteMission, RunTask


DRIVING_ACTIONS = [
    ('/mission/execute', ExecuteMission),
    ('/nav/go_to', RunTask),
    ('/dock/align', RunTask),
    ('/base/drive_straight', RunTask),
    ('/base/rotate', RunTask),
    ('/elevator/wait_door_open', RunTask),
    ('/elevator/board', RunTask),
    ('/elevator/exit', RunTask),
    ('/floor/check', RunTask),
    ('/map/switch', RunTask),
]

ARM_ACTIONS = [
    ('/arm/pick', RunTask),
    ('/arm/place', RunTask),
    ('/arm/press_button', RunTask),
]

CORE_TOPICS = [
    '/odom',
    '/scan_filtered',
    '/tf',
    '/cmd_vel',
    '/tag/target_offset_x',
    '/tag/target_distance',
    '/tag/marker_id',
    '/tag/floor_id',
]

CAMERA_TOPICS = [
    '/front_camera/image_raw',
    '/rear_camera/image_raw',
]


class IntegrationCheck(Node):
    def __init__(self):
        super().__init__('check_driving_integration')
        self._action_clients = []

    def wait_for_action(self, name, action_type, timeout_sec):
        client = ActionClient(self, action_type, name)
        self._action_clients.append(client)
        return client.wait_for_server(timeout_sec=timeout_sec)

    def wait_for_topic(self, name, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            topic_names = {
                topic_name
                for topic_name, _ in self.get_topic_names_and_types()
            }
            if name in topic_names:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Check mission_manager to driving-server ROS graph wiring.',
    )
    parser.add_argument(
        '--timeout-sec',
        type=float,
        default=2.0,
        help='Wait time per action/topic.',
    )
    parser.add_argument(
        '--include-arm',
        action='store_true',
        help='Also require arm task action servers.',
    )
    parser.add_argument(
        '--skip-cameras',
        action='store_true',
        help='Do not require front/rear camera topics.',
    )
    return parser.parse_args(argv)


def print_row(ok, kind, name):
    status = 'PASS' if ok else 'FAIL'
    print(f'[{status}] {kind:<6} {name}')


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = IntegrationCheck()

    required_actions = list(DRIVING_ACTIONS)
    if args.include_arm:
        required_actions.extend(ARM_ACTIONS)

    required_topics = list(CORE_TOPICS)
    if not args.skip_cameras:
        required_topics.extend(CAMERA_TOPICS)

    failed = []

    print('Checking Action servers...')
    for name, action_type in required_actions:
        ok = node.wait_for_action(name, action_type, args.timeout_sec)
        print_row(ok, 'action', name)
        if not ok:
            failed.append(f'action:{name}')

    print('\nChecking topics...')
    for name in required_topics:
        ok = node.wait_for_topic(name, args.timeout_sec)
        print_row(ok, 'topic', name)
        if not ok:
            failed.append(f'topic:{name}')

    node.destroy_node()
    rclpy.shutdown()

    if failed:
        print('\nIntegration check failed:')
        for item in failed:
            print(f'  - {item}')
        return 1

    print('\nIntegration check passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
