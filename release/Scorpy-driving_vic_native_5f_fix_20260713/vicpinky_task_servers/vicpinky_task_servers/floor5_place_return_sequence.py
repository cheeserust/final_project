#!/usr/bin/env python3
# floor5_place_return_sequence

import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String

from vicpinky_interfaces.action import RunTask


class Floor5DeliverySequence(Node):
    def __init__(self):
        super().__init__('floor5_delivery_sequence')

        self.nav_client = ActionClient(self, RunTask, '/nav/go_to')
        self.tag_align_client = ActionClient(self, RunTask, '/tag/align')
        self.rotate_client = ActionClient(self, RunTask, '/base/rotate')

        self.target_floor = 5
        self.elevator_marker_id = 20

        # 엘베-마커 정렬 거리는 4층과 거의 동일
        self.nav_cancel_marker_distance_m = 1.50
        self.elevator_align_distance_m = 1.45
        self.elevator_align_lateral_m = 0.0

        # 팀원이 object_place 방향으로 회전 완료한 뒤, object_place까지 직진할 거리
        self.object_place_forward_distance_m = 3.50
        self.object_place_forward_speed = 0.05

        self.latest_marker_id = None
        self.latest_marker_time = 0.0
        self.latest_marker_distance = None
        self.latest_marker_distance_time = 0.0

        self.create_subscription(Int32, '/tag/marker_id', self.marker_callback, 10)
        self.create_subscription(Float32, '/tag/target_distance', self.distance_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

    def publish_status(self, text):
        self.get_logger().info(f'[STATE] {text}')
        self.status_pub.publish(String(data=text))

    def marker_callback(self, msg):
        self.latest_marker_id = int(msg.data)
        self.latest_marker_time = time.time()

    def distance_callback(self, msg):
        self.latest_marker_distance = float(msg.data)
        self.latest_marker_distance_time = time.time()

    def marker_visible(self, marker_id, timeout_sec=0.8):
        return (
            self.latest_marker_id == marker_id
            and time.time() - self.latest_marker_time <= timeout_sec
        )

    def marker_distance_valid(self, timeout_sec=0.8):
        return (
            self.latest_marker_distance is not None
            and time.time() - self.latest_marker_distance_time <= timeout_sec
        )

    def marker_close_enough_for_nav_cancel(self, marker_id):
        return (
            self.marker_visible(marker_id)
            and self.marker_distance_valid()
            and self.latest_marker_distance <= self.nav_cancel_marker_distance_m
        )

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

    def drive_forward_by_distance(self, distance_m, speed=0.05, state_name='DRIVE_FORWARD'):
        self.publish_status(state_name)

        duration = float(distance_m) / float(speed)
        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < duration:
            self.publish_cmd(linear_x=speed, angular_z=0.0)
            time.sleep(0.05)

        self.stop_robot()
        time.sleep(0.5)

    def drive_forward_60cm(self):
        self.drive_forward_by_distance(
            distance_m=0.60,
            speed=0.05,
            state_name='DRIVE_FORWARD_60CM'
        )

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'  feedback: {fb.phase} {fb.progress:.2f} {fb.detail}')

    def call_task(self, client, task_id, target_name, marker_id=-1, extra=None):
        if extra is None:
            extra = {}

        self.get_logger().info(f'Waiting action server: {client._action_name}')
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Action server not available: {client._action_name}')
            return False

        goal = RunTask.Goal()
        goal.task_id = task_id
        goal.target_name = target_name
        goal.target_floor = int(self.target_floor)
        goal.marker_id = int(marker_id)
        goal.extra_json = json.dumps(extra)

        self.get_logger().info(
            f'Send goal: {client._action_name} / {task_id} / {target_name}'
        )

        send_future = client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error(f'Goal send failed: {client._action_name}')
            return False

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected: {client._action_name}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: success={result.success}, message={result.message}')
        return bool(result.success)

    def rotate(self, angle_deg, target_name):
        self.publish_status(f'ROTATE_{angle_deg}')
        return self.call_task(
            self.rotate_client,
            task_id='rotate',
            target_name=target_name,
            marker_id=-1,
            extra={'angle_deg': angle_deg}
        )

    def nav_until_elevator_marker_close(self):
        self.publish_status('NAV_TO_ELEVATOR_FRONT')

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available: /nav/go_to')
            return False

        goal = RunTask.Goal()
        goal.task_id = 'go_to'
        goal.target_name = 'elevator_front'
        goal.target_floor = int(self.target_floor)
        goal.marker_id = -1
        goal.extra_json = '{}'

        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error('Nav goal send failed')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Nav goal rejected')
            return False

        result_future = goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.marker_visible(self.elevator_marker_id) and self.marker_distance_valid():
                self.get_logger().info(
                    f'Marker {self.elevator_marker_id} visible. '
                    f'distance={self.latest_marker_distance:.3f} m'
                )

            if self.marker_close_enough_for_nav_cancel(self.elevator_marker_id):
                self.get_logger().warn(
                    f'Marker {self.elevator_marker_id} close enough. Cancel nav.'
                )

                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)

                self.stop_robot()
                time.sleep(0.5)
                return True

        if result_future.done():
            result = result_future.result().result
            self.get_logger().info(
                f'Nav result: success={result.success}, message={result.message}'
            )
            self.stop_robot()
            return bool(result.success)

        self.stop_robot()
        return False

    def search_elevator_marker(self, timeout_sec=10.0):
        self.publish_status('SEARCH_ELEVATOR_MARKER_RIGHT_FIRST')

        # 1차: 오른쪽으로 조금씩 탐색
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout_sec * 0.7:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.marker_visible(self.elevator_marker_id):
                self.stop_robot()
                time.sleep(0.5)
                self.get_logger().info('Elevator marker found during right-first search.')
                return True

            # 오른쪽으로 천천히 조금씩 회전
            self.publish_cmd(linear_x=0.0, angular_z=-0.08)
            time.sleep(0.05)

        self.stop_robot()
        time.sleep(0.3)

        # 2차: 혹시 반대쪽이면 왼쪽으로 보정 탐색
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout_sec * 0.3:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.marker_visible(self.elevator_marker_id):
                self.stop_robot()
                time.sleep(0.5)
                self.get_logger().info('Elevator marker found during left fallback search.')
                return True

            self.publish_cmd(linear_x=0.0, angular_z=0.08)
            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().error('Elevator marker search timeout.')
        return False

    def tag_align_elevator(self):
        self.publish_status('TAG_ALIGN_ELEVATOR_FRONT')

        return self.call_task(
            self.tag_align_client,
            task_id='tag_align',
            target_name='elevator_front_marker',
            marker_id=self.elevator_marker_id,
            extra={
                'target_distance_m': self.elevator_align_distance_m,
                'target_lateral_m': self.elevator_align_lateral_m
            }
        )

    def align_to_elevator_front_short_path(self):
        self.publish_status('ALIGN_TO_ELEVATOR_FRONT_SHORT_PATH')

        nav_ok = self.nav_until_elevator_marker_close()

        # 짧은 경로라 Nav가 끝났는데 마커가 안 보일 수 있음
        if not self.marker_visible(self.elevator_marker_id):
            self.get_logger().warn(
                'Elevator marker is not visible after short nav. Start search rotation.'
            )

            search_ok = self.search_elevator_marker(timeout_sec=10.0)
            if not search_ok:
                self.publish_status('FAILED_SEARCH_ELEVATOR_MARKER')
                return False

        # 마커가 보이면 tag_align으로 최종 정렬
        if not self.tag_align_elevator():
            self.publish_status('FAILED_ELEVATOR_TAG_ALIGN')
            return False

        return True

    def run(self):
        self.publish_status('FLOOR5_DELIVERY_START')

        # 시작 조건:
        # 팀원 코드가 엘베에서 내린 뒤 object_place 방향으로 회전 완료한 상태
        self.publish_status('START_FROM_OBJECT_PLACE_DIRECTION')

        # 1. object_place까지 정해진 거리 직진
        self.drive_forward_by_distance(
            distance_m=self.object_place_forward_distance_m,
            speed=self.object_place_forward_speed,
            state_name='DRIVE_TO_OBJECT_PLACE_BY_DISTANCE'
        )

        # 2. 로봇팔 물건 내려놓기 mock
        self.publish_status('ARM_PLACE_OBJECT_MOCK')
        time.sleep(3.0)

        self.publish_status('ARM_RETURN_HOME_MOCK')
        time.sleep(1.0)

        # 3. 180도 회전해서 엘베 방향 바라보기
        if not self.rotate(180, 'turn_back_to_elevator'):
            self.publish_status('FAILED_ROTATE_180')
            return

        # 4. 짧은 경로용 엘베 앞 정렬
        # Nav가 방향을 완전히 못 잡아도 search_marker로 보완
        if not self.align_to_elevator_front_short_path():
            self.publish_status('FAILED_ALIGN_TO_ELEVATOR_FRONT')
            return

        # 4-1. 로봇팔이 내려가는 버튼 누르기 mock
        self.publish_status('ARM_PRESS_DOWN_BUTTON_MOCK')
        time.sleep(2.0)

        self.publish_status('BUTTON_BRIGHTNESS_CHECK_MOCK')
        time.sleep(1.0)

        # 60cm 전진
        self.drive_forward_60cm()

        # 왼쪽 85도 회전
        if not self.rotate(85, 'left_85_after_button'):
            self.publish_status('FAILED_LEFT_85_ROTATE')
            return

        self.publish_status('READY_TO_BOARD_ELEVATOR')
        self.publish_status('FLOOR5_DELIVERY_DONE')


def main(args=None):
    rclpy.init(args=args)
    node = Floor5DeliverySequence()

    try:
        node.run()
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()