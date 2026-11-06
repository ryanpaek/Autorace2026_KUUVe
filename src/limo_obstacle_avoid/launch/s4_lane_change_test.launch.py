from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

PKG = 'limo_obstacle_avoid'
CFG = os.path.join(get_package_share_directory(PKG), 'config', 'avoid_params.yaml')


# [4단계] 장애물 없이 차선 변경만 (★ 바퀴 움직임)
#   out_mode:=twist (기본) → /cmd_vel, 팀 브리지 켜기  /  out_mode:=ack → /limo/ack_cmd 직접, 브리지 끄기
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('change_lanes', default_value='1'),
        DeclareLaunchArgument('total_dist', default_value='3.0'),
        DeclareLaunchArgument('out_mode', default_value='twist'),
        Node(package=PKG, executable='s4_lane_change_test', output='screen', parameters=[CFG, {
            'change_lanes': ParameterValue(LaunchConfiguration('change_lanes'), value_type=int),
            'total_dist': ParameterValue(LaunchConfiguration('total_dist'), value_type=float),
            'out_mode': ParameterValue(LaunchConfiguration('out_mode'), value_type=str),
        }]),
    ])
