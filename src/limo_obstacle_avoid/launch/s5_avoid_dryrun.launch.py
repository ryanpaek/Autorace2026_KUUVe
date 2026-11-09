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

# [5단계] 회피 판단만 구경 (바퀴 안 움직임) — 손으로 밀거나 차선추종으로 달리면서
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        Node(package=PKG, executable='avoid_node', output='screen',
             parameters=[CFG, {'dry_run': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', RVIZ],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
