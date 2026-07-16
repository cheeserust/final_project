from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'roscue_arm_pick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='hari',
    maintainer_email='hari@example.com',
    description='ROScue ArUco marker based arm pick and press task package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = roscue_arm_pick.aruco_detector_node:main',
            'task_executor_node = roscue_arm_pick.task_executor_node:main',
            'fixed_joint_test = roscue_arm_pick.fixed_joint_test:main',
            'gripper_test = roscue_arm_pick.gripper_test:main',
            'fake_joint_state_node = roscue_arm_pick.fake_joint_state_node:main',
            'marker_to_base_debug_node = roscue_arm_pick.marker_to_base_debug_node:main',
        ],
    },
)
