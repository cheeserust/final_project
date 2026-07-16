from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'arm_task_server'

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
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sj',
    maintainer_email='seongjun000313@gmail.com',
    description='Deprecated arm task servers kept for compatibility tests',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_task_server_node = arm_task_server.arm_task_server_node:main',
        ],
    },
)
