#!/usr/bin/env python3
import yaml
import os

import math
import cv2
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap
from geometry_msgs.msg import PoseWithCovarianceStamped


class MapSwitcher(Node):
    def __init__(self):
        super().__init__('map_switcher')

        self.load_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'floor_markers.yaml'
        )
        self.marker_table = self.load_marker_table(config_path)

        self.last_marker_id = None  # debounce: only act on a NEW marker

        # --- ArUco / camera setup ---
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.cap = cv2.VideoCapture('/dev/video2')
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open /dev/video2')

        # poll camera at 10 Hz — plenty for marker detection, light on CPU
        self.timer = self.create_timer(0.1, self.on_camera_frame)

    def on_camera_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera frame read failed')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is None or len(ids) == 0:
            return  # no marker this frame

        marker_id = int(ids[0][0])  # take first detected marker

        if marker_id == self.last_marker_id:
            return  # debounce: same marker as last trigger, ignore

        if marker_id not in self.marker_table:
            return  # unknown marker (e.g. button_panel type) — ignore for map switching

        self.last_marker_id = marker_id
        map_yaml, (x, y, yaw) = self.marker_table[marker_id]
        self.get_logger().info(f'Marker {marker_id} -> loading {map_yaml}')
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

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose_msg)
        self.get_logger().info('Map loaded, AMCL re-initialized')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

    def load_marker_table(self, config_path):
        flooring_dir = os.path.dirname(os.path.dirname(config_path))
        with open(config_path, 'r') as f:
            raw = yaml.safe_load(f)

        table = {}
        for marker_id, entry in raw['markers'].items():
            pose = entry['initial_pose']
            map_path = os.path.join(flooring_dir, entry['map_yaml'])  # relative -> absolute
            table[int(marker_id)] = (
                map_path,
                (pose['x'], pose['y'], pose['yaw'])
            )
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