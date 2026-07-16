#!/usr/bin/env python3
# 테스트 시나리오: 4F 401호 → 엘리베이터 → 5F object_place → 엘리베이터 → 4F 401호
# 로봇팔 미완성이라 버튼 조작 단계는 사람이 수행 (Enter로 진행)
#
# 사전 조건:
#   - Nav2가 4f.yaml 맵으로 실행 중
#   - task_servers.launch.py 실행 중 (실동작 모드)
#   - 시작 위치: 4층 401호 앞 (스크립트가 /initialpose 발행)
#
# 실행: ros2 run vicpinky_task_servers full_mission_test
#       ros2 run vicpinky_task_servers full_mission_test --ros-args -p use_dock_align:=true

import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from vicpinky_interfaces.action import RunTask

START_FLOOR = 4
TARGET_FLOOR = 5
START_NAME = '401'
TARGET_NAME = 'object_place'

# nav_points.yaml "4"/"401" 좌표와 동일하게 유지
START_POSE = (-0.535, -10.839, 0.0)  # x, y, yaw


class FullMissionTest(Node):
    def __init__(self):
        super().__init__('full_mission_test')
        self.declare_parameter('use_dock_align', False)
        self.declare_parameter('set_initial_pose', True)
        self.use_dock_align = bool(self.get_parameter('use_dock_align').value)
        self.set_initial_pose = bool(self.get_parameter('set_initial_pose').value)

        self.nav = ActionClient(self, RunTask, '/nav/go_to')
        self.dock = ActionClient(self, RunTask, '/dock/align')
        self.board = ActionClient(self, RunTask, '/elevator/board')
        self.exit_ = ActionClient(self, RunTask, '/elevator/exit')
        self.floor = ActionClient(self, RunTask, '/floor/check')
        self.map_ = ActionClient(self, RunTask, '/map/switch')

        self.rotate = ActionClient(self, RunTask, '/base/rotate')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    # ── helpers ──────────────────────────────────────────────
    def fb(self, msg):
        f = msg.feedback
        self.get_logger().info(f'  {f.phase} {f.progress:.2f} {f.detail}')

    def call(self, client, task_id, target_name='', target_floor=0, marker_id=-1, extra=None):
        name = client._action_name
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'server unavailable: {name}')
            return False
        goal = RunTask.Goal()
        goal.task_id = task_id
        goal.target_name = target_name
        goal.target_floor = int(target_floor)
        goal.marker_id = int(marker_id)
        goal.extra_json = json.dumps(extra or {})
        self.get_logger().info(f'>>> {name} task_id={task_id} '
                               f'target={target_name}/{target_floor}')
        send = client.send_goal_async(goal, feedback_callback=self.fb)
        rclpy.spin_until_future_complete(self, send)
        gh = send.result()
        if not gh.accepted:
            self.get_logger().error(f'goal rejected: {name}')
            return False
        res_f = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_f)
        res = res_f.result().result
        self.get_logger().info(f'<<< {name}: success={res.success} "{res.message}"')
        return bool(res.success)

    def pause(self, msg):
        print(f'\n[수동] {msg}')
        input('완료 후 Enter... ')

    def publish_start_pose(self):
        x, y, yaw = START_POSE
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        for _ in range(3):
            self.pose_pub.publish(m)
            time.sleep(0.3)
        self.get_logger().info(f'initialpose set: {START_NAME} {START_POSE}')

    # ── 편도 1회: 엘리베이터로 층 이동 ─────────────────────────
    def take_elevator(self, from_floor, to_floor):
        ok = self.call(self.nav, 'go_to', 'elevator_front', from_floor)
        if not ok:
            return False
        if self.use_dock_align:
            if not self.call(self.dock, 'dock_to_marker', 'elevator_front',
                             from_floor, marker_id=20):
                return False
        self.pause(f'{from_floor}층 엘리베이터 호출 버튼을 눌러주세요')
        self.drive_forward(0.60)
        if not self.call(self.rotate, 'rotate', 'left_80', from_floor,
                         extra={'angle_deg': 80}):
            return False
        if not self.call(self.board, 'board_elevator'):
            return False
        self.pause(f'{to_floor}층 버튼을 눌러주세요')
        if not self.call(self.floor, 'check_floor', target_floor=to_floor):
            return False
        if not self.call(self.exit_, 'exit_elevator', target_floor=to_floor):
            return False
        # EXIT → SWITCH 순서 (하차 완료 후 맵/initialpose 세팅)
        if not self.call(self.map_, 'map_switch', target_floor=to_floor):
            return False
        return True

    def drive_forward(self, dist_m=0.60, speed=0.15):
        self.get_logger().info(f'forward {dist_m*100:.0f}cm')
        t = Twist()
        t.linear.x = speed
        end = time.time() + dist_m / speed
        while time.time() < end:
            self.cmd_pub.publish(t)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)

    def run(self):
        if self.set_initial_pose:
            self.publish_start_pose()

        if not self.take_elevator(START_FLOOR, TARGET_FLOOR):
            return self.fail('4F→5F 이동 실패')
        if not self.call(self.nav, 'go_to', TARGET_NAME, TARGET_FLOOR):
            return self.fail('object_place 이동 실패')

        self.pause('물건을 로봇에 실어주세요 (팔 대행)')

        if not self.take_elevator(TARGET_FLOOR, START_FLOOR):
            return self.fail('5F→4F 이동 실패')
        if not self.call(self.nav, 'go_to', START_NAME, START_FLOOR):
            return self.fail('401 복귀 실패')

        self.get_logger().info('===== 미션 완료: 401 복귀 =====')
        return True

    def fail(self, msg):
        self.get_logger().error(f'===== 미션 중단: {msg} =====')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = FullMissionTest()
    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
