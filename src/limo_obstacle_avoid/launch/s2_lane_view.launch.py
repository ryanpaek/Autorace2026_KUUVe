from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

PKG = 'limo_obstacle_avoid'
CFG = os.path.join(get_package_share_directory(PKG), 'config', 'avoid_params.yaml')
RVIZ = os.path.join(get_package_share_directory(PKG), 'rviz', 'avoid.rviz')

# [2단계] 차선별 장애물 거리 + 회피 판단 보기 (로봇 정지)
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package=PKG, executable='s2_lane_view', output='screen', parameters=[CFG]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', RVIZ, '-f', 'base_link'],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
