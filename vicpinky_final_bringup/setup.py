from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'vicpinky_final_bringup'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VicPinky Team',
    maintainer_email='team@example.com',
    description=(
        'Final mission wrapper that preserves the team Pinky base launch.'
    ),
    license='MIT',
    tests_require=['pytest'],
)
