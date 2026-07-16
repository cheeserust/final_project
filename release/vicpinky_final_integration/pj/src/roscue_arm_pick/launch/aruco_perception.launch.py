"""Launch the wrist RealSense and ArUco detection on the central PC."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create the central-PC camera and marker perception graph."""
    launch_camera = LaunchConfiguration('launch_camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        condition=IfCondition(launch_camera),
    )
    detector = Node(
        package='roscue_arm_pick',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'marker_size_m': 0.05,
            'target_marker_ids': [50, 51, 52, 53, 54, 55],
            'min_stable_frames': 3,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Start realsense2_camera before the detector.',
        ),
        camera_launch,
        detector,
    ])
