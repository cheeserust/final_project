#!/usr/bin/env python3
# /map/switch RunTask Action Server
# goal.target_floor(4/5) 또는 goal.marker_id(예: 10=엘리베이터 내부)로
# floor_markers.yaml에서 맵/초기 pose를 찾아 Nav2 load_map 호출 + /initialpose 발행

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from std_msgs.msg import Int32

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from vicpinky_interfaces.action import RunTask

PKG = 'vicpinky_task_servers'


class MapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')

        self.declare_parameter('server_name', '/map/switch')
        self.declare_parameter('load_map_timeout_sec', 10.0)
        # 단독 테스트용: /tag/floor_id 수신 시 자동 전환 (미션 매니저 사용 시 False 유지)
        self.declare_parameter('enable_topic_trigger', False)

        self.declare_parameter('amcl_converge_timeout_sec', 10.0)
        self.declare_parameter('converge_tol_xy', 0.5)
        self.declare_parameter('costmap_settle_sec', 1.5)

        self.server_name = self.get_parameter('server_name').value
        self.load_map_timeout_sec = float(self.get_parameter('load_map_timeout_sec').value)
        self.enable_topic_trigger = bool(self.get_parameter('enable_topic_trigger').value)

        self.amcl_converge_timeout_sec = float(self.get_parameter('amcl_converge_timeout_sec').value)
        self.converge_tol_xy = float(self.get_parameter('converge_tol_xy').value)
        self.costmap_settle_sec = float(self.get_parameter('costmap_settle_sec').value)

        self.cb_group = ReentrantCallbackGroup()

        self.load_map_cli = self.create_client(
            LoadMap, '/map_server/load_map', callback_group=self.cb_group)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.latest_amcl = None  # (x, y, stamp_time)
        amcl_qos = QoSProfile(depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose,
            amcl_qos, callback_group=self.cb_group)
        
        share_dir = get_package_share_directory(PKG)
        config_path = os.path.join(share_dir, 'config', 'floor_markers.yaml')
        self.marker_table = self.load_marker_table(config_path, share_dir)
        self.last_key = None  # debounce (topic trigger용)

        self.action_server = ActionServer(
            self, RunTask, self.server_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group)

        if self.enable_topic_trigger:
            self.create_subscription(
                Int32, '/tag/floor_id', self.on_floor_topic, 10,
                callback_group=self.cb_group)

        self.get_logger().info(f'Map Switch Server Started: {self.server_name}')
        self.get_logger().info(f'known keys: {sorted(self.marker_table.keys())}')

    # ── config 로드: {key: (abs_map_yaml, (x,y,yaw))} ────────
    def load_marker_table(self, config_path, share_dir):
        with open(config_path, 'r') as f:
            raw = yaml.safe_load(f)
        table = {}
        for marker_id, entry in raw['markers'].items():
            pose = entry['initial_pose']
            map_path = os.path.join(share_dir, entry['map_yaml'])
            table[int(marker_id)] = (map_path, (float(pose['x']), float(pose['y']),
                                                float(pose['yaw'])))
        return table

    # ── action ───────────────────────────────────────────────
    def goal_callback(self, goal_request):
        if goal_request.task_id not in ('map_switch', 'switch_map'):
            return GoalResponse.REJECT
        key = goal_request.marker_id if goal_request.marker_id > 0 else goal_request.target_floor
        if key not in self.marker_table:
            self.get_logger().warn(f'unknown floor/marker key: {key}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = RunTask.Result()
        key = goal.marker_id if goal.marker_id > 0 else goal.target_floor
        map_yaml, (x, y, yaw) = self.marker_table[key]

        fb = RunTask.Feedback()
        fb.phase = 'LOAD_MAP'
        fb.progress = 0.2
        fb.detail = os.path.basename(map_yaml)
        goal_handle.publish_feedback(fb)

        if not self.load_map_cli.wait_for_service(timeout_sec=2.0):
            result.success = False
            result.message = 'load_map service not available'
            goal_handle.abort()
            return result

        req = LoadMap.Request()
        req.map_url = map_yaml
        future = self.load_map_cli.call_async(req)

        start = time.time()
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'map switch canceled'
                goal_handle.canceled()
                return result
            if time.time() - start > self.load_map_timeout_sec:
                result.success = False
                result.message = 'load_map timeout'
                goal_handle.abort()
                return result
            time.sleep(0.1)

        res = future.result()
        if res is None or res.result != 0:
            result.success = False
            result.message = f'load_map failed: {res}'
            goal_handle.abort()
            return result

        fb.phase = 'SET_INITIAL_POSE'
        fb.progress = 0.8
        fb.detail = f'x={x:.2f} y={y:.2f} yaw={yaw:.2f}'
        goal_handle.publish_feedback(fb)
        # AMCL 수렴 대기: /amcl_pose가 seed 근처로 올 때까지 1초마다 재발행
        fb.phase = 'WAIT_AMCL'
        fb.progress = 0.85
        self.latest_amcl = None
        gate_start = time.time()
        last_pub = 0.0
        converged = False
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = 'map switch canceled'
                goal_handle.canceled()
                return result
            if time.time() - gate_start > self.amcl_converge_timeout_sec:
                break
            if time.time() - last_pub >= 1.0:
                self.publish_initialpose(x, y, yaw)
                last_pub = time.time()
            if self.latest_amcl is not None:
                ax, ay, at = self.latest_amcl
                if at >= gate_start and math.hypot(ax - x, ay - y) <= self.converge_tol_xy:
                    converged = True
                    break
            fb.detail = 'waiting amcl convergence'
            goal_handle.publish_feedback(fb)
            time.sleep(0.3)

        if not converged:
            result.success = False
            result.message = f'amcl did not converge to ({x:.2f},{y:.2f}) in {self.amcl_converge_timeout_sec}s'
            goal_handle.abort()
            self.get_logger().error(result.message)
            return result

        # global costmap이 새 /map을 반영할 시간 확보
        fb.phase = 'COSTMAP_SETTLE'
        fb.progress = 0.95
        fb.detail = f'{self.costmap_settle_sec}s'
        goal_handle.publish_feedback(fb)
        time.sleep(self.costmap_settle_sec)

        self.last_key = key
        result.success = True

        result.message = f'map switched (key {key})'
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result

    # ── 단독 테스트용 topic trigger ───────────────────────────
    def on_floor_topic(self, msg):
        key = int(msg.data)
        if key == self.last_key or key not in self.marker_table:
            return
        self.last_key = key
        map_yaml, (x, y, yaw) = self.marker_table[key]
        if not self.load_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('load_map service not available')
            return
        req = LoadMap.Request()
        req.map_url = map_yaml
        future = self.load_map_cli.call_async(req)
        future.add_done_callback(lambda f: self._topic_map_loaded(f, x, y, yaw))

    def _topic_map_loaded(self, future, x, y, yaw):
        res = future.result()
        if res is None or res.result != 0:
            self.get_logger().error(f'load_map failed: {res}')
            return
        self.publish_initialpose(x, y, yaw)
        self.get_logger().info(f'map loaded + initialpose (key {self.last_key})')

    def on_amcl_pose(self, msg):
        self.latest_amcl = (msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            time.time())

    def publish_initialpose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MapSwitcher()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
