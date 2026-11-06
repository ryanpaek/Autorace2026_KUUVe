"""차선 주행만 시험 (1주차 마일스톤용).

lane_node + mission_node(LANE_ONLY)만 띄운다. mission_node가 /lane/cmd_vel을 /cmd_vel로 넘긴다.
  ros2 launch autorace_bringup lane_test.launch.py
  ros2 launch autorace_bringup lane_test.launch.py speed:=0.2
  ros2 launch autorace_bringup lane_test.launch.py speed:=0.2 mask:=model   # 학습한 분할 모델
11/06 고도화 (기본 전부 꺼짐):
  memory_mm:=600.0        차선 기억 (놓쳐도 최근 본 중앙선으로)
  steer_gain:=1.3         작은 조향 보정 (브리지)
  route:=true             도면 경로 지도 켜기 -> 출발점에서 s를 맞춘 뒤
                          ros2 topic pub --once /route/set_s std_msgs/msg/Float32 "{data: 0.0}"
  ff:=0.3                 도면 곡률 미리 넣기 비율 (route:=true일 때만 의미, 놓쳤을 때는 0.8 고정)
  preview_slow:=0.25      코너 들어가기 전 감속
  look_gain:=500.0        속도 따라 lookahead 늘리기
비상 정지: ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    params = LaunchConfiguration('params')
    bev = LaunchConfiguration('bev_config')
    speed = ParameterValue(LaunchConfiguration('speed'), value_type=float)

    def f(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)
    route_on = LaunchConfiguration('route')
    route_file = PythonExpression(["'", os.path.join(share, 'config', 'track_route.yaml'),
                                   "' if '", route_on, "' == 'true' else ''"])
    ff_lost = PythonExpression(['0.8 if float(', LaunchConfiguration('ff'), ') > 0 else 0.0'])
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('bev_config',
                              default_value=os.path.join(share, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('speed', default_value='0.25'),
        DeclareLaunchArgument('mask', default_value='hsv', description='hsv | model'),
        DeclareLaunchArgument('memory_mm', default_value='0.0'),
        DeclareLaunchArgument('steer_gain', default_value='1.0'),
        DeclareLaunchArgument('route', default_value='false'),
        DeclareLaunchArgument('ff', default_value='0.0'),
        DeclareLaunchArgument('preview_slow', default_value='0.0'),
        DeclareLaunchArgument('look_gain', default_value='0.0'),
        # 이 LIMO의 애커만 드라이버는 /limo/ack_cmd(AckermannDrive)만 받는다
        DeclareLaunchArgument('use_ackermann_bridge', default_value='true'),
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             parameters=[{'small_steer_gain': f('steer_gain')}],
             output='screen', condition=IfCondition(LaunchConfiguration('use_ackermann_bridge'))),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev,
                                  'lane_mask_source': LaunchConfiguration('mask'),
                                  'memory_mm': f('memory_mm'),
                                  'route_ff_weight': f('ff'),
                                  'route_ff_weight_lost': ParameterValue(ff_lost, value_type=float),
                                  'preview_slowdown': f('preview_slow'),
                                  'lookahead_per_mps': f('look_gain')}],
             output='screen'),
        Node(package='autorace_mission', executable='mission_node', name='mission_node',
             parameters=[params, {'start_state': 'LANE_ONLY', 'base_speed': speed,
                                  'route_file': route_file}],
             output='screen'),
    ])
