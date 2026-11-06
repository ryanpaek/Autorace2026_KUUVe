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

# [3단계] 로컬 위치(오도메트리+IMU) 확인 — teleop 으로 움직이며 줄자와 비교
def generate_launch_description():
    return LaunchDescription([
        Node(package=PKG, executable='s3_local_pose', output='screen', parameters=[CFG]),
    ])
