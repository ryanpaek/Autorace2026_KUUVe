"""전체 스택 실행.

LIMO 드라이버·카메라·라이다는 LIMO 기본 bringup으로 먼저 띄운 뒤 실행한다.
  ros2 launch autorace_bringup autorace.launch.py
  ros2 launch autorace_bringup autorace.launch.py start_state:=M3_PEDESTRIAN use_detector:=false
출발: ros2 topic pub --once /mission/start std_msgs/msg/Empty
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    params = LaunchConfiguration('params')
    bev = LaunchConfiguration('bev_config')
    start_state = LaunchConfiguration('start_state')
    use_detector = LaunchConfiguration('use_detector')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('bev_config',
                              default_value=os.path.join(share, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('start_state', default_value='WAIT_START'),
        DeclareLaunchArgument('use_detector', default_value='true'),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev}], output='screen'),
        Node(package='autorace_mission', executable='mission_node', name='mission_node',
             parameters=[params, {'start_state': start_state}], output='screen'),
        Node(package='autorace_mission', executable='parking_node', name='parking_node',
             parameters=[params], output='screen'),
        Node(package='autorace_obstacle', executable='obstacle_node', name='obstacle_node',
             parameters=[params], output='screen'),
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=[params], output='screen', condition=IfCondition(use_detector)),
    ])
