from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vicpinky_task_servers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'aruco_markers'), glob('aruco_markers/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='hari',
    maintainer_email='hari@example.com',
    description='VicPinky task action servers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
	    'console_scripts': [
		'nav_go_to_server = vicpinky_task_servers.nav_go_to_server:main',
		'dock_align_server = vicpinky_task_servers.dock_align_server:main',
		'elevator_door_server = vicpinky_task_servers.elevator_door_server:main',
        'elevator_front_sequence_test = vicpinky_task_servers.elevator_front_sequence_test:main',
		'floor_check_server = vicpinky_task_servers.floor_check_server:main',
		'elevator_board_off = vicpinky_task_servers.elevator_board_off:main',
		'map_switcher = vicpinky_task_servers.map_switcher:main',
        'full_mission_test = vicpinky_task_servers.full_mission_test:main',
		'aruco_pose_publisher = vicpinky_task_servers.aruco_pose_publisher:main',
		'base_rotate_server = vicpinky_task_servers.base_rotate_server:main',
        'floor5_delivery_sequence = vicpinky_task_servers.floor5_delivery_sequence:main',
        'floor4_return_home_sequence = vicpinky_task_servers.floor4_return_home_sequence:main',
	    ],
	},
)
