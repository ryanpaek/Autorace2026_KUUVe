from glob import glob
from setuptools import setup

package_name = 'limo_obstacle_avoid'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/paths', glob('paths/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunseo',
    maintainer_email='yunxeo072@gmail.com',
    description='3-lane static LIMO avoidance (LiDAR + odom/IMU local frame)',
    license='MIT',
    entry_points={
        'console_scripts': [
            's0_fake_world = limo_obstacle_avoid.s0_fake_world:main',
            's1_scan_check = limo_obstacle_avoid.s1_scan_check:main',
            's2_lane_view = limo_obstacle_avoid.s2_lane_view:main',
            's3_local_pose = limo_obstacle_avoid.s3_local_pose:main',
            's4_lane_change_test = limo_obstacle_avoid.s4_lane_change:main',
            'avoid_node = limo_obstacle_avoid.s5_avoid_node:main',
            'cmd_mux = limo_obstacle_avoid.cmd_mux:main',
            'm7_replay_node = limo_obstacle_avoid.m7_replay_node:main',
            'm7_waypoint_node = limo_obstacle_avoid.m7_waypoint_node:main',
        ],
    },
)
