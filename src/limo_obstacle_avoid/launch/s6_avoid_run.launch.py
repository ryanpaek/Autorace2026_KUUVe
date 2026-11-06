from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

PKG = 'limo_obstacle_avoid'
CFG = os.path.join(get_package_share_directory(PKG), 'config', 'avoid_params.yaml')


# [6단계] 진짜 회피 + 명령 교통정리(cmd_mux)
#   차선추종 노드는 /lane_follow/cmd_vel 로 내보내도록 바꿔둘 것
#   out_mode:=twist (기본) → /cmd_vel, 팀 브리지 켜기  /  out_mode:=ack → /limo/ack_cmd 직접, 브리지 끄기
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_on_start', default_value='false'),
        DeclareLaunchArgument('out_mode', default_value='twist'),
        Node(package=PKG, executable='avoid_node', output='screen', parameters=[CFG, {
            'dry_run': False,
            'enable_on_start': ParameterValue(LaunchConfiguration('enable_on_start'), value_type=bool),
        }]),
        Node(package=PKG, executable='cmd_mux', output='screen', parameters=[CFG, {
            'out_mode': ParameterValue(LaunchConfiguration('out_mode'), value_type=str),
        }]),
    ])
