import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pinky_delivery'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kj',
    maintainer_email='jaewi96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'elevator_board_off = pinky_delivery.elevator_board_off:main',
            'map_switcher = pinky_delivery.map_switcher:main',
        ],
    },
)
