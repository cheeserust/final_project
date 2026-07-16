from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'final_project_presentation2'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.json'),
        ),
        (
            os.path.join('share', package_name, 'static'),
            glob('static/*.html')
            + glob('static/*.css')
            + glob('static/*.js'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sj',
    maintainer_email='sj@example.com',
    description=(
        'Standalone two-marker straight route, arm workflow, and web UI.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'final_project_presentation2 = '
            'final_project_presentation2.main_node:main',
            'final_project_presentation2_watchdog = '
            'final_project_presentation2.watchdog_node:main',
        ],
    },
)
