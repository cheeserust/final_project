#!/usr/bin/env python3

import math
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('marker_size_m', 0.10)
        self.declare_parameter('target_marker_id', 20)

        self.declare_parameter('camera_fx', 554.0)
        self.declare_parameter('camera_fy', 554.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.target_marker_id = int(self.get_parameter('target_marker_id').value)

        fx = float(self.get_parameter('camera_fx').value)
        fy = float(self.get_parameter('camera_fy').value)
        cx = float(self.get_parameter('camera_cx').value)
        cy = float(self.get_parameter('camera_cy').value)

        self.camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        self.bridge = CvBridge()

        self.offset_pub = self.create_publisher(Float32, '/tag/target_offset_x', 10)
        self.distance_pub = self.create_publisher(Float32, '/tag/target_distance', 10)
        self.yaw_pub = self.create_publisher(Float32, '/tag/target_yaw_error', 10)
        self.id_pub = self.create_publisher(Int32, '/tag/marker_id', 10)

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                aruco_dict,
                cv2.aruco.DetectorParameters()
            )
            self.use_new_api = True
        else:
            self.aruco_dict = aruco_dict
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False

        self.get_logger().info('Aruco Pose Publisher TVEC + YAW Started.')
        self.get_logger().info(f'image_topic      : {self.image_topic}')
        self.get_logger().info(f'marker_size_m    : {self.marker_size_m}')
        self.get_logger().info(f'target_marker_id : {self.target_marker_id}')
        self.get_logger().info(f'camera_matrix    : {self.camera_matrix.tolist()}')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.aruco_params,
            )

        if ids is None:
            return

        ids = ids.flatten()

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size_m,
            self.camera_matrix,
            self.dist_coeffs,
        )

        for i, marker_id in enumerate(ids):
            marker_id = int(marker_id)

            if marker_id != self.target_marker_id:
                continue

            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            offset_x_m = float(tvec[0])
            distance_m = float(tvec[2])

            rot_mat, _ = cv2.Rodrigues(rvec)

            # marker z-axis normal in camera frame
            marker_normal = rot_mat[:, 2]

            # yaw error: 0이면 카메라가 마커를 정면으로 봄
            yaw_error_rad = math.atan2(
                float(marker_normal[0]),
                float(marker_normal[2])
            )

            self.offset_pub.publish(Float32(data=offset_x_m))
            self.distance_pub.publish(Float32(data=distance_m))
            self.yaw_pub.publish(Float32(data=yaw_error_rad))
            self.id_pub.publish(Int32(data=marker_id))

            self.get_logger().info(
                f'id={marker_id}, '
                f'tvec_x={offset_x_m:.3f} m, '
                f'tvec_z={distance_m:.3f} m, '
                f'yaw={math.degrees(yaw_error_rad):.1f} deg',
                throttle_duration_sec=0.5,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()