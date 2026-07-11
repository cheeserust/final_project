from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'arm_can_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sj',
    maintainer_email='seongjun000313@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_can_bridge_node = arm_can_bridge.arm_can_bridge_node:main',
            'send_test_trajectory = arm_can_bridge.send_test_trajectory:main',
            'send_arm_pose = arm_can_bridge.send_arm_pose:main',
            'send_gripper_pose = arm_can_bridge.send_gripper_pose:main',
            'board3_can_smoke_test = arm_can_bridge.board3_can_smoke_test:main',
            'board3_uart_debug = arm_can_bridge.board3_uart_debug:main',
        ],
    },
)
