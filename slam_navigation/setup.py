from setuptools import setup
import os
from glob import glob

package_name = 'slam_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='SLAM integration package combining slam_toolbox and nvblox for 4-wheel differential drive robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_manager = slam_navigation.slam_manager:main',
            'cmd_vel_transformer = slam_navigation.cmd_vel_transformer:main',
            'map_fusion = slam_navigation.map_fusion:main',
        ],
    },
)