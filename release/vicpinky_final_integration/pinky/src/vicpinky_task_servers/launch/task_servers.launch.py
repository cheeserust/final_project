#!/usr/bin/env python3
# 로봇측 task server 일괄 실행 (실동작 모드)
# 제공 서버: /dock/align /elevator/wait_door_open /floor/check
#           /map/switch /elevator/board /elevator/exit
#           /base/drive_straight /base/rotate
from launch import LaunchDescription
from launch_ros.actions import Node

PKG = 'vicpinky_task_servers'


def generate_launch_description():
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
        Node(package=PKG, executable='dock_align_server', output='screen',
             parameters=[{
                 'mock_mode': False,
                 'target_distance_m': 1.27,
                 'aligned_hold_sec': 3.0,
             }]),
        Node(package=PKG, executable='elevator_door_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='floor_check_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='base_drive_straight_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='base_rotate_server', output='screen',
             parameters=[{'mock_mode': False}]),
        Node(package=PKG, executable='elevator_board_off', output='screen',
             parameters=[{
                 'boarding_target_distance_cm': 50.0,
                 'camera_stale_timeout_sec': 0.75,
             }]),
        Node(package=PKG, executable='map_switcher', output='screen'),
    ])
