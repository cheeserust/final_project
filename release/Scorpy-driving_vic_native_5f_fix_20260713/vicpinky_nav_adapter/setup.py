from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'vicpinky_nav_adapter'

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
    maintainer='user',
    maintainer_email='user@example.com',
    description='RunTask to Nav2 NavigateToPose adapter for VicPinky',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'nav_adapter_node = '
                'vicpinky_nav_adapter.nav_adapter_node:main'
            ),
        ],
    },
)
