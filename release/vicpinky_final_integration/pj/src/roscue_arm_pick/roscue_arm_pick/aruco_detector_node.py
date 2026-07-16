#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from vicpinky_interfaces.msg import DetectedMarker


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_marker_ids', [50, 51, 52, 53, 54, 55])
        self.declare_parameter('min_stable_frames', 3)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.target_marker_ids = {
            int(marker_id) for marker_id in self.get_parameter('target_marker_ids').value
        }
        self.min_stable_frames = max(
            1,
            int(self.get_parameter('min_stable_frames').value),
        )
        self.stable_counts = {}

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.pose_pub = self.create_publisher(PoseStamped, '/detected_marker_pose', 10)
        self.id_pub = self.create_publisher(Int32, '/detected_marker_id', 10)
        self.detection_pub = self.create_publisher(
            DetectedMarker,
            '/detected_marker',
            10,
        )

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = (
            cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            if hasattr(cv2.aruco, 'ArucoDetector')
            else None
        )

        self.get_logger().info('Aruco detector started')
        self.get_logger().info(f'image_topic={self.image_topic}')
        self.get_logger().info(f'camera_info_topic={self.camera_info_topic}')
        self.get_logger().info(f'marker_size_m={self.marker_size}')
        self.get_logger().info(f'target_marker_ids={sorted(self.target_marker_ids)}')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('CameraInfo received')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            # Ubuntu 24.04/Jazzy ships OpenCV 4.6, before ArucoDetector.
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )

        if ids is None:
            self.stable_counts.clear()
            return

        visible_ids = {int(value) for value in ids.flatten()}
        for marker_id in set(self.stable_counts) | visible_ids:
            if marker_id in visible_ids:
                self.stable_counts[marker_id] = min(
                    self.min_stable_frames,
                    self.stable_counts.get(marker_id, 0) + 1,
                )
            else:
                self.stable_counts.pop(marker_id, None)

        s = self.marker_size / 2.0
        object_points = np.array([
            [-s,  s, 0.0],
            [s,  s, 0.0],
            [s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

        for marker_corners, marker_id_raw in zip(corners, ids.flatten()):
            marker_id = int(marker_id_raw)
            if marker_id not in self.target_marker_ids:
                continue
            if self.stable_counts.get(marker_id, 0) < self.min_stable_frames:
                continue

            image_points = marker_corners[0].astype(np.float64)

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                self.get_logger().warn(f'solvePnP failed for marker id={marker_id}')
                continue

            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = msg.header.frame_id

            pose_msg.pose.position.x = float(tvec[0][0])
            pose_msg.pose.position.y = float(tvec[1][0])
            pose_msg.pose.position.z = float(tvec[2][0])

            rot_mat, _ = cv2.Rodrigues(rvec)
            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rot_mat)

            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            id_msg = Int32()
            id_msg.data = marker_id

            detection_msg = DetectedMarker()
            detection_msg.header = pose_msg.header
            detection_msg.marker_id = marker_id
            detection_msg.pose = pose_msg.pose

            self.detection_pub.publish(detection_msg)
            self.id_pub.publish(id_msg)
            self.pose_pub.publish(pose_msg)

            self.get_logger().info(
                f'ID={marker_id}, '
                f'x={tvec[0][0]:.3f}, y={tvec[1][0]:.3f}, z={tvec[2][0]:.3f}, '
                f'frame={pose_msg.header.frame_id}'
            )

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = np.empty((4,), dtype=np.float64)
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s

        return q[0], q[1], q[2], q[3]


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
