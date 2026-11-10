"""전체 스택 실행.

LIMO 드라이버·카메라·라이다를 먼저 띄운 뒤 실행한다 (애커만 LIMO 예):
  ros2 launch limo_base limo_base_ackerman.launch.py
  ros2 launch limo_base open_ydlidar_launch.py
  ros2 launch orbbec_camera dabai.launch.py
  ros2 launch autorace_bringup autorace.launch.py
  ros2 launch autorace_bringup autorace.launch.py start_state:=M3_PEDESTRIAN use_detector:=false
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true            # M7을 limo_obstacle_avoid로
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true m7_dry_run:=true   # 판단만 (바퀴는 차선)
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true use_fork_node:=true  # + 갈림길 판단 노드
출발: ros2 topic pub --once /mission/start std_msgs/msg/Empty
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _avoid_params() -> str:
    """limo_obstacle_avoid가 빌드돼 있지 않아도 (m7_avoid:=false) launch가 깨지지 않게."""
    try:
        return os.path.join(get_package_share_directory('limo_obstacle_avoid'),
                            'config', 'avoid_params.yaml')
    except Exception:
        return ''


def _fork_params() -> str:
    """autorace_fork가 빌드돼 있지 않아도 (use_fork_node:=false) launch가 깨지지 않게."""
    try:
        return os.path.join(get_package_share_directory('autorace_fork'), 'config', 'fork_params.yaml')
    except Exception:
        return ''


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    params = LaunchConfiguration('params')
    bev = LaunchConfiguration('bev_config')
    start_state = LaunchConfiguration('start_state')
    use_detector = LaunchConfiguration('use_detector')
    use_bridge = LaunchConfiguration('use_ackermann_bridge')
    route = LaunchConfiguration('route_file')
    m7_avoid = LaunchConfiguration('m7_avoid')
    m7_dry = LaunchConfiguration('m7_dry_run')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('bev_config',
                              default_value=os.path.join(share, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('start_state', default_value='WAIT_START'),
        # 도면 경로 지도. 끄려면 route_file:=''
        DeclareLaunchArgument('route_file',
                              default_value=os.path.join(share, 'config', 'track_route.yaml')),
        DeclareLaunchArgument('use_detector', default_value='true'),
        DeclareLaunchArgument('use_class_commands', default_value='true'),
        # 이 LIMO의 애커만 드라이버는 /limo/ack_cmd(AckermannDrive)만 받는다
        DeclareLaunchArgument('use_ackermann_bridge', default_value='true'),
        # M7 3차선 회피: limo_obstacle_avoid의 avoid_node를 켜고 FSM이 그 명령을 쓰게 한다.
        #   avoid_node 출력 /avoid/cmd_vel -> /obstacle/cmd_vel (mission_node가 고르는 입력 칸).
        #   그 패키지의 cmd_mux는 켜지 않는다 (/cmd_vel은 mission_node 하나만 낸다).
        DeclareLaunchArgument('m7_avoid', default_value='false'),
        DeclareLaunchArgument('m7_dry_run', default_value='false'),
        DeclareLaunchArgument('m7_avoid_params', default_value=_avoid_params()),
        # M2 갈림길 판단 노드(autorace_fork): YOLO fork_a/fork_b 투표 + 지금 차선 기준 이동 차선 수 판단 -> /fork_direction.
        #   FSM이 이미 정했으면 그걸 따르므로 켜도 결정이 엇갈리지 않는다. /fork/state 로 상황을 볼 수 있다.
        DeclareLaunchArgument('use_fork_node', default_value='false'),
        DeclareLaunchArgument('fork_params', default_value=_fork_params()),
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             output='screen', condition=IfCondition(use_bridge)),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev}], output='screen'),
        Node(package='autorace_mission', executable='mission_node', name='mission_node',
             parameters=[params, {'start_state': start_state, 'route_file': route,
                                  'm7_external_avoid': ParameterValue(m7_avoid, value_type=bool),
                                  'use_class_commands': ParameterValue(
                                      LaunchConfiguration('use_class_commands'), value_type=bool)}],
             output='screen'),
        Node(package='autorace_mission', executable='parking_node', name='parking_node',
             parameters=[params], output='screen'),
        Node(package='autorace_obstacle', executable='obstacle_node', name='obstacle_node',
             parameters=[params, {'bev_config': bev}], output='screen'),
        Node(package='limo_obstacle_avoid', executable='avoid_node', name='avoid_node',
             parameters=[LaunchConfiguration('m7_avoid_params'),
                         {'dry_run': ParameterValue(m7_dry, value_type=bool)}],
             remappings=[('/avoid/cmd_vel', '/obstacle/cmd_vel')],
             output='screen', condition=IfCondition(m7_avoid)),
        Node(package='autorace_fork', executable='fork_node', name='fork_node',
             parameters=[LaunchConfiguration('fork_params')], output='screen',
             condition=IfCondition(LaunchConfiguration('use_fork_node'))),
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=[params], output='screen', condition=IfCondition(use_detector)),
        Node(package='autorace_perception', executable='class_command_node',
             name='class_command_node', parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('use_class_commands'))),
    ])
