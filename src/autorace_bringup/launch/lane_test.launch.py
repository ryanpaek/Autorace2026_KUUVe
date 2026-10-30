"""차선 주행만 시험 (1주차 마일스톤용).

lane_node + mission_node(LANE_ONLY)만 띄운다. mission_node가 /lane/cmd_vel을 /cmd_vel로 넘긴다.
  ros2 launch autorace_bringup lane_test.launch.py
  ros2 launch autorace_bringup lane_test.launch.py speed:=0.2
비상 정지: ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    params = LaunchConfiguration('params')
    bev = LaunchConfiguration('bev_config')
    speed = ParameterValue(LaunchConfiguration('speed'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('bev_config',
                              default_value=os.path.join(share, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('speed', default_value='0.25'),
        # 이 LIMO의 애커만 드라이버는 /limo/ack_cmd(AckermannDrive)만 받는다
        DeclareLaunchArgument('use_ackermann_bridge', default_value='true'),
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             output='screen', condition=IfCondition(LaunchConfiguration('use_ackermann_bridge'))),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev}], output='screen'),
        Node(package='autorace_mission', executable='mission_node', name='mission_node',
             parameters=[params, {'start_state': 'LANE_ONLY', 'base_speed': speed}],
             output='screen'),
    ])
