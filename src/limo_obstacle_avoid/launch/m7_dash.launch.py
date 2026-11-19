"""M7 점선 회피 + M2 갈림길 단독 실행 (11/19). LIMO 기본·라이다·카메라는 따로 켜 둔 상태에서.
팀 주행 전체는 autorace_bringup full.launch.py / autorace.launch.py (m7_impl:=dash 기본)로.

  ros2 launch limo_obstacle_avoid m7_dash.launch.py                 # 브리지 + 표지 인식 + 점선 노드 (/cmd_vel 직접)
  ros2 launch limo_obstacle_avoid m7_dash.launch.py lane:=true      # + lane_node (노란선 보조, /lane/cmd_vel은 아무도 안 받음)
  ros2 launch limo_obstacle_avoid m7_dash.launch.py dry_run:=true   # 바퀴 안 움직임 (/cmd_vel_preview)
  ros2 launch limo_obstacle_avoid m7_dash.launch.py detector:=false # 표지 인식 끔 -> 갈래 = m2_fork_default (fork:=B 로 바꿈)
  차: 3차선 시작선, 가운데|오른쪽 차로 사이 점선 위, 똑바로, 1초 정지
  시작: ros2 topic pub --once /avoid/enable std_msgs/Bool "{data: true}"
  정지: ros2 topic pub --once /avoid/enable std_msgs/Bool "{data: false}"
  갈래 강제: ros2 topic pub --once /fork_direction std_msgs/String "{data: B}"
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
    dry = LaunchConfiguration('dry_run')
    return LaunchDescription([
        DeclareLaunchArgument('bev_config', default_value=os.path.join(bring, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('lane', default_value='false'),
        DeclareLaunchArgument('bridge', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('detector', default_value='true'),
        DeclareLaunchArgument('fork', default_value='A'),               # 표지를 못 봤을 때 갈래
        DeclareLaunchArgument('m2', default_value='true'),              # false = 섬 앞에서 끝 (갈래 안 들어감)
        DeclareLaunchArgument('section_length', default_value='4.38'),  # 켠 자리 ~ 섬 앞 꼭짓점 (m)
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             parameters=[params], output='screen', condition=IfCondition(LaunchConfiguration('bridge'))),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev}], output='screen', condition=IfCondition(LaunchConfiguration('lane'))),
        # 원본 YOLO 표지 인식 (fork_a/fork_b -> 갈래 투표). 설정은 params.yaml detector_node 그대로
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=[params], output='screen', condition=IfCondition(LaunchConfiguration('detector'))),
        Node(package='limo_obstacle_avoid', executable='m7_dash_node', name='avoid_node', output='screen',
             parameters=[dash, {'bev_config': bev, 'start_on_dash': True,
                                'dry_run': ParameterValue(dry, value_type=bool),
                                'm2_replay': ParameterValue(LaunchConfiguration('m2'), value_type=bool),
                                'm2_fork_default': LaunchConfiguration('fork'),
                                'section_length': ParameterValue(LaunchConfiguration('section_length'), value_type=float)}],
             remappings=[('/avoid/cmd_vel', '/cmd_vel')]),
    ])
