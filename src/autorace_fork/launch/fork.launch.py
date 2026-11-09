"""갈림길 판단 노드.  ros2 launch autorace_fork fork.launch.py
   문자열 토픽으로 시험:  ros2 launch autorace_fork fork.launch.py input_mode:=text"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CFG = os.path.join(get_package_share_directory('autorace_fork'), 'config', 'fork_params.yaml')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_mode', default_value='detections'),
        Node(package='autorace_fork', executable='fork_node', name='fork_node', output='screen',
             parameters=[CFG, {'input_mode': LaunchConfiguration('input_mode')}]),
    ])
