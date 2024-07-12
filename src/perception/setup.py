#!/usr/bin/env python3

import glob
import os

from setuptools import setup

package_name = 'perception'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yong-Jun Shin',
    maintainer_email='yjshin@etri.re.kr',
    description='perception package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_camera_perception = perception.my_camera_perception:main',
            'my_lidar_perception = perception.my_lidar_perception:main',
            'my_object_detection_service = perception.my_object_detection_service:main',
        ],
    },
)
