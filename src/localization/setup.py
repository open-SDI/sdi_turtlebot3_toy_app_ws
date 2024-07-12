#!/usr/bin/env python3

import glob
import os

from setuptools import setup

package_name = 'localization'
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
    description='localization package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_lidar_localization = localization.my_lidar_localization:main',
        ],
    },
)
