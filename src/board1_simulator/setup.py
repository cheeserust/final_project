import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'board1_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='SocketCAN simulator for STM32 Board1 arm controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'board1_simulator_node = board1_simulator.board1_simulator_node:main',
            
        ],
    },
)
