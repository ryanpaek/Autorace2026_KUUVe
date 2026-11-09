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

# [1단계] 라이다 방향 확인 (로봇 정지)
def generate_launch_description():
    return LaunchDescription([
        Node(package=PKG, executable='s1_scan_check', output='screen', parameters=[CFG]),
    ])
