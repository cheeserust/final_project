from glob import glob

import os

from setuptools import find_packages, setup

package_name = 'mission_manager'

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
    maintainer='user',
    maintainer_email='user@example.com',
    description='Mission manager for VicPinky central server',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = mission_manager.mission_manager_node:main',
            'send_mission = mission_manager.send_mission:main',
            'send_demo_mission = mission_manager.send_demo_mission:main',
        ],
    },
)
