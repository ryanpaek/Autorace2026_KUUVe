"""전체 스택 실행.

LIMO 드라이버·카메라·라이다를 먼저 띄운 뒤 실행한다 (애커만 LIMO 예):
  ros2 launch limo_base limo_base_ackerman.launch.py
  ros2 launch limo_base open_ydlidar_launch.py
  ros2 launch orbbec_camera dabai.launch.py
  ros2 launch autorace_bringup autorace.launch.py
  ros2 launch autorace_bringup autorace.launch.py start_state:=M3_PEDESTRIAN use_detector:=false
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true            # M7을 limo_obstacle_avoid로 (기본 m7_impl:=replay)
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true m7_impl:=legacy   # 예전 avoid_node 방식
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true m7_impl:=waypoint # 11/19 실험: 정차 LIMO 기준 웨이포인트 (M7만)
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true m7_dry_run:=true   # 판단만 (바퀴는 차선)
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true use_fork_node:=true  # + 갈림길 판단 노드
  ros2 launch autorace_bringup autorace.launch.py yellow_side:=true          # + 노란 선 옆 거리 (/lane/yellow_side, 11/19)
출발: ros2 topic pub --once /mission/start std_msgs/msg/Empty
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _avoid_params() -> str:
    """limo_obstacle_avoid가 빌드돼 있지 않아도 (m7_avoid:=false) launch가 깨지지 않게."""
    try:
        return os.path.join(get_package_share_directory('limo_obstacle_avoid'),
                            'config', 'avoid_params.yaml')
    except Exception:
        return ''


def _replay_params() -> str:
    try:
        return os.path.join(get_package_share_directory('limo_obstacle_avoid'),
                            'config', 'm7_replay_params.yaml')
    except Exception:
        return ''


def _waypoint_params() -> str:
    try:
        return os.path.join(get_package_share_directory('limo_obstacle_avoid'),
                            'config', 'm7_waypoint_params.yaml')
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
    m7_impl = LaunchConfiguration('m7_impl')
    # 11/17: M7 방식. replay(기본) = 녹화한 차로 변경 재생 (m7_replay_node) / legacy = 예전 avoid_node
    is_replay = IfCondition(PythonExpression(["'", m7_avoid, "' == 'true' and '", m7_impl, "' == 'replay'"]))
    # 11/19 실험: waypoint = 정차 LIMO 기준 웨이포인트 (m7_waypoint_node, M7만. M2는 FSM). obstacle_node 설정은 replay와 같게
    is_waypoint = IfCondition(PythonExpression(["'", m7_avoid, "' == 'true' and '", m7_impl, "' == 'waypoint'"]))
    is_legacy = IfCondition(PythonExpression(["'", m7_avoid, "' == 'true' and '", m7_impl, "' not in ('replay', 'waypoint')"]))
    is_link = IfCondition(PythonExpression(["'", m7_avoid, "' == 'true' and '", m7_impl, "' in ('replay', 'waypoint')"]))
    not_replay = IfCondition(PythonExpression(["not ('", m7_avoid, "' == 'true' and '", m7_impl, "' in ('replay', 'waypoint'))"]))
    # waypoint일 때 차선 노드 노란선 거리 = 직선 맞추기 (lane_core yellow_guard_fit). 다른 방식은 그대로(false)
    yg_fit = ParameterValue(PythonExpression(["'", m7_impl, "' == 'waypoint'"]), value_type=bool)
    # 재생 방식일 때 obstacle_node(m7_link) 설정: 코너 미리보기(코너 중간 켜기)·표지 보고 끄기 끔 -> 시작선 근처에서 켜고 재생 노드가 끝냄
    replay_link = {'m7_pre_enable': False, 'm7_off_mode': 'none'}
    return LaunchDescription([
        DeclareLaunchArgument('m7_impl', default_value='replay'),
        DeclareLaunchArgument('m7_replay_params', default_value=_replay_params()),
        DeclareLaunchArgument('m7_waypoint_params', default_value=_waypoint_params()),
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
        # 웹 디버그 화면 (노트북 브라우저 http://<리모IP>:8080). 구독만 함
        DeclareLaunchArgument('debug_view', default_value='true'),
        DeclareLaunchArgument('sys_monitor', default_value='true'),
        # 11/19: 노란 선 옆 거리 노드 (/lane/yellow_side). 아무도 조향에 안 씀 -> 기록·확인용. 기본 꺼짐
        DeclareLaunchArgument('yellow_side', default_value='false'),
        # 디버그 화면 미션 조작 버튼 (미션 고르기·건너뛰기·출발). 대회 때 false (비상 정지는 계속 됨)
        DeclareLaunchArgument('debug_control', default_value='true'),
        # 주차 완료 '띠링'을 차 스피커로 (chime_node). 끄려면 chime:=false
        DeclareLaunchArgument('chime', default_value='true'),
        # 주차 켜기 (parking_node enabled). 통주행은 full.launch.py가 true로 준다
        DeclareLaunchArgument('parking', default_value='false'),
        DeclareLaunchArgument('fork_params', default_value=_fork_params()),
        # 위치 확인용 수동 주행: mission_node가 /cmd_vel을 내지 않는다 (teleop으로 조종, 11/11)
        DeclareLaunchArgument('manual', default_value='false'),
        # 11/15: 브리지가 params.yaml(wheelbase 0.222)을 받게 함. 예전엔 안 넘겨 기본 0.2로 돌아 모든 회전이 약 11% 덜 꺾임
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             parameters=[params], output='screen', condition=IfCondition(use_bridge)),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev, 'yellow_guard_fit': yg_fit}], output='screen'),
        Node(package='autorace_mission', executable='mission_node', name='mission_node',
             parameters=[params, {'start_state': start_state, 'route_file': route,
                                  'm7_external_avoid': ParameterValue(m7_avoid, value_type=bool),
                                  'manual_drive': ParameterValue(
                                      LaunchConfiguration('manual'), value_type=bool),
                                  'use_class_commands': ParameterValue(
                                      LaunchConfiguration('use_class_commands'), value_type=bool)}],
             output='screen'),
        Node(package='autorace_mission', executable='debug_view', name='debug_view',
             parameters=[params, {'route_file': route, 'bev_config': bev,
                                  'control_enabled': ParameterValue(LaunchConfiguration('debug_control'),
                                                                    value_type=bool)}], output='screen',
             condition=IfCondition(LaunchConfiguration('debug_view'))),
        # 11/16: 연산량 측정 (/system/usage, 디버깅 화면 '연산량' 패널). 끄기: sys_monitor:=false
        Node(package='autorace_mission', executable='sys_monitor', name='sys_monitor', output='screen',
             condition=IfCondition(LaunchConfiguration('sys_monitor'))),
        Node(package='autorace_lane', executable='yellow_side_node', name='yellow_side_node',
             parameters=[params, {'bev_config': bev, 'lane_params_file': params}], output='screen',
             condition=IfCondition(LaunchConfiguration('yellow_side'))),
        Node(package='autorace_mission', executable='chime_node', name='chime_node',
             output='screen', condition=IfCondition(LaunchConfiguration('chime'))),
        Node(package='autorace_mission', executable='parking_node', name='parking_node',
             parameters=[params, {'enabled': ParameterValue(LaunchConfiguration('parking'), value_type=bool)}],
             output='screen'),
        Node(package='autorace_obstacle', executable='obstacle_node', name='obstacle_node',
             parameters=[params, {'bev_config': bev}], output='screen', condition=not_replay),
        Node(package='autorace_obstacle', executable='obstacle_node', name='obstacle_node',
             parameters=[params, {'bev_config': bev}, replay_link], output='screen', condition=is_link),
        Node(package='limo_obstacle_avoid', executable='avoid_node', name='avoid_node',
             parameters=[LaunchConfiguration('m7_avoid_params'),
                         {'dry_run': ParameterValue(m7_dry, value_type=bool)}],
             remappings=[('/avoid/cmd_vel', '/obstacle/cmd_vel')],
             output='screen', condition=is_legacy),
        Node(package='limo_obstacle_avoid', executable='m7_replay_node', name='avoid_node',
             parameters=[LaunchConfiguration('m7_avoid_params'), LaunchConfiguration('m7_replay_params'),
                         {'dry_run': ParameterValue(m7_dry, value_type=bool)}],
             remappings=[('/avoid/cmd_vel', '/obstacle/cmd_vel')],
             output='screen', condition=is_replay),
        Node(package='limo_obstacle_avoid', executable='m7_waypoint_node', name='avoid_node',
             parameters=[LaunchConfiguration('m7_waypoint_params'),
                         {'dry_run': ParameterValue(m7_dry, value_type=bool)}],
             remappings=[('/avoid/cmd_vel', '/obstacle/cmd_vel')],
             output='screen', condition=is_waypoint),
        Node(package='autorace_fork', executable='fork_node', name='fork_node',
             parameters=[LaunchConfiguration('fork_params')], output='screen',
             condition=IfCondition(LaunchConfiguration('use_fork_node'))),
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=[params], output='screen', condition=IfCondition(use_detector)),
        Node(package='autorace_perception', executable='class_command_node',
             name='class_command_node', parameters=[params], output='screen',
             condition=IfCondition(LaunchConfiguration('use_class_commands'))),
    ])
