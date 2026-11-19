"""M7 점선 기준 회피 단독 실행 (11/19 실험). LIMO 기본·라이다·카메라는 따로 켜 둔 상태에서.

  ros2 launch limo_obstacle_avoid m7_dash.launch.py                 # 브리지 + m7_dash_node
  ros2 launch limo_obstacle_avoid m7_dash.launch.py lane:=true      # + lane_node (노란선 보조, /lane/cmd_vel은 아무도 안 받음)
  ros2 launch limo_obstacle_avoid m7_dash.launch.py dry_run:=true   # 바퀴 안 움직임 (/cmd_vel_preview)
  시작: ros2 topic pub --once /m7dash/enable std_msgs/Bool "{data: true}"
  정지: ros2 topic pub --once /m7dash/enable std_msgs/Bool "{data: false}"
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
    bring = get_package_share_directory('autorace_bringup')
    params = os.path.join(bring, 'config', 'params.yaml')
    dash = os.path.join(get_package_share_directory('limo_obstacle_avoid'), 'config', 'm7_dash_params.yaml')
    bev = LaunchConfiguration('bev_config')
    return LaunchDescription([
        DeclareLaunchArgument('bev_config', default_value=os.path.join(bring, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('lane', default_value='false'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             parameters=[params], output='screen', condition=IfCondition(LaunchConfiguration('bridge'))),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev}], output='screen', condition=IfCondition(LaunchConfiguration('lane'))),
        Node(package='limo_obstacle_avoid', executable='m7_dash_node', name='m7_dash_node', output='screen',
             parameters=[dash, {'bev_config': bev,
                                'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
