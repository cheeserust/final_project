#!/usr/bin/env python3
# 로봇측 task server 일괄 실행 (실동작 모드)
# 제공 서버: /nav/go_to /dock/align /elevator/wait_door_open /floor/check
#           /map/switch /elevator/board /elevator/exit /base/rotate
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'vicpinky_task_servers'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    nav_points = os.path.join(share, 'config', 'nav_points.yaml')

    return LaunchDescription([
        # 엘리베이터 앞 정렬용 ArUco pose (전방 카메라, 마커 20)
        Node(package=PKG, executable='aruco_pose_publisher', output='screen',
             parameters=[{
                 'image_topic': '/front_camera/image_raw',
                 'target_marker_id': 20,
                 'marker_size_m': 0.10,
                 'camera_fx': 708.85065781,
                 'camera_fy': 707.92630029,
                 'camera_cx': 308.289349,
                 'camera_cy': 244.0512732,
             }]),
        Node(package=PKG, executable='nav_go_to_server', output='screen',
             parameters=[{'mock_mode': False, 'nav_points_file': nav_points}]),
        Node(package=PKG, executable='dock_align_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='elevator_door_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='floor_check_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='base_rotate_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='elevator_board_off', output='screen'),
        Node(package=PKG, executable='map_switcher', output='screen'),
    ])
