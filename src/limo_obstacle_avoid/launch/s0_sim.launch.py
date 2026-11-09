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

# [0단계] LIMO 없이 집에서: 가짜 트랙 + 회피 노드 + cmd_mux + RViz
#   ros2 launch limo_obstacle_avoid s0_sim.launch.py layout:=zigzag6 exit:=B
def generate_launch_description():
    layout = LaunchConfiguration('layout')
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='mixed7'),
        DeclareLaunchArgument('exit', default_value='auto'),
        DeclareLaunchArgument('odom_scale', default_value='1.0'),
        DeclareLaunchArgument('imu_drift_deg', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package=PKG, executable='s0_fake_world', output='screen', parameters=[CFG, {
            'layout': layout,
            'odom_scale': ParameterValue(LaunchConfiguration('odom_scale'), value_type=float),
            'imu_drift_deg': ParameterValue(LaunchConfiguration('imu_drift_deg'), value_type=float),
        }]),
        Node(package=PKG, executable='avoid_node', output='screen', parameters=[CFG, {
            'dry_run': False, 'enable_on_start': True,
            'exit_lane': LaunchConfiguration('exit'),
        }]),
        Node(package=PKG, executable='cmd_mux', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', RVIZ],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
