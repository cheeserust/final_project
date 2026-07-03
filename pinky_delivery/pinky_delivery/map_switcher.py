#!/usr/bin/env python3
# 
"""
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

  이 코드는 vicpinky 안에서 실행되는 코드입니다  

        나중에 디버깅용 로그 코드 지우기

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
"""

import yaml, os, math
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory


class MapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')
        self.load_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        config_path = os.path.join(
            get_package_share_directory('pinky_delivery'), 'config', 'floor_markers.yaml'
        )
        self.marker_table = self.load_marker_table(config_path)
        self.last_floor = None  # debounce

        self.create_subscription(Int32, '/floor/arrived', self.on_floor_arrived, 10)

    def on_floor_arrived(self, msg):
        floor = msg.data
        if floor == self.last_floor:
            return
        if floor not in self.marker_table:
            self.get_logger().warn(f'unknown floor {floor}')
            return
        self.last_floor = floor
        map_yaml, (x, y, yaw) = self.marker_table[floor]
        self.switch_map(map_yaml, x, y, yaw)

    def switch_map(self, map_yaml, x, y, yaw):
        if not self.load_map_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('load_map service not available')
            return
        req = LoadMap.Request()
        req.map_url = map_yaml
        future = self.load_map_cli.call_async(req)
        future.add_done_callback(lambda f: self.on_map_loaded(f, x, y, yaw))

    def on_map_loaded(self, future, x, y, yaw):
        result = future.result()
        if result is None or result.result != 0:
            self.get_logger().error(f'load_map failed: {result}')
            return
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(m)
        self.get_logger().info(f'map loaded + initialpose (floor {self.last_floor})')

    def load_marker_table(self, config_path):
        flooring_dir = os.path.dirname(os.path.dirname(config_path))
        with open(config_path, 'r') as f:
            raw = yaml.safe_load(f)
        table = {}
        for marker_id, entry in raw['markers'].items():
            pose = entry['initial_pose']
            map_path = os.path.join(flooring_dir, entry['map_yaml'])
            table[int(marker_id)] = (map_path, (pose['x'], pose['y'], pose['yaw']))
        return table


def main():
    rclpy.init()
    node = MapSwitcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()