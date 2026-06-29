from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'vicpinky_gui'

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
        (
            os.path.join('share', package_name, 'static'),
            glob('static/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sj',
    maintainer_email='seongjun000313@gmail.com',
    description='Browser-based control dashboard for VicPinky central server',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vicpinky_gui_node = vicpinky_gui.gui_node:main',
        ],
    },
)
