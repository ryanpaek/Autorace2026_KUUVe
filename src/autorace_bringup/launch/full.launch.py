"""통주행 한 번에 띄우기: 차체 + 라이다 + 카메라 + 자율주행 전체(주차·M7 회피·갈림길 노드 켬).

  ros2 launch autorace_bringup full.launch.py
  ros2 topic pub --once /mission/start std_msgs/msg/Empty        # 출발

인자:
  sensors:=false        차체·라이다·카메라를 이미 따로 띄웠을 때 (중복 실행 금지!)
  parking:=false / m7_avoid:=false / use_fork_node:=false / use_detector:=false   하나씩 끄기
  start_state:=M4_PARKING 등   중간 구간부터 (띄우자마자 움직임)
  debug_control:=false  디버그 화면 미션 조작 버튼 끄기 (대회 때. 비상 정지 버튼은 계속 됨)
  chime:=false          주차 완료 '띠링'(차 스피커) 끄기
  cam_exp_level:=0~4    카메라 노출 단계를 이번 실행만 바꿈 (평소엔 비워 둠 -> ros2 run autorace_bringup cam_level N 으로 저장한 값. 11/19)
  manual:=true          위치 확인용 수동 주행. mission_node가 /cmd_vel을 안 냄 -> teleop으로 조종 (11/11)
카메라는 컬러만 켠다 (depth·ir를 켜고 전체 녹화하면 드라이버가 죽은 적 있음, 11/09).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _inc(pkg, *path, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), *path)),
        launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    sensors = IfCondition(LaunchConfiguration('sensors'))
    passed = ('start_state', 'parking', 'm7_avoid', 'm7_impl', 'use_fork_node', 'use_detector', 'use_class_commands',
              'manual', 'chime', 'debug_control')
    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('cam_exp_level', default_value='saved'),
        DeclareLaunchArgument('start_state', default_value='WAIT_START'),
        DeclareLaunchArgument('parking', default_value='true'),
        DeclareLaunchArgument('m7_avoid', default_value='true'),
        DeclareLaunchArgument('m7_impl', default_value='replay'),   # replay = 녹화 재생 (11/17) / legacy = 예전 회피 노드 / waypoint = 11/19 실험 (정차 LIMO 기준 웨이포인트, M7만)
        DeclareLaunchArgument('use_fork_node', default_value='true'),
        DeclareLaunchArgument('use_detector', default_value='true'),
        DeclareLaunchArgument('use_class_commands', default_value='true'),
        DeclareLaunchArgument('manual', default_value='false'),
        DeclareLaunchArgument('chime', default_value='true'),
        DeclareLaunchArgument('debug_control', default_value='true'),
        _inc('limo_base', 'launch', 'limo_base_ackerman.launch.py', condition=sensors),
        _inc('ydlidar_ros2_driver', 'launch', 'ydlidar.launch.py', condition=sensors),
        _inc('orbbec_camera', 'launch', 'dabai.launch.py',
             args={'enable_depth': 'false', 'enable_ir': 'false'}, condition=sensors),
        # 11/19: 노란 실선 포화 대응. 출발 전 한 번 노출을 계산해 고정 (단계: ros2 run autorace_bringup cam_level N 으로 저장)
        Node(package='autorace_bringup', executable='exposure_node', name='exposure_node', output='screen',
             parameters=[os.path.join(get_package_share_directory('autorace_bringup'), 'config', 'params.yaml'),
                         {'level_override': ParameterValue(LaunchConfiguration('cam_exp_level'), value_type=str)}],
             condition=IfCondition(LaunchConfiguration('sensors'))),
        _inc('autorace_bringup', 'launch', 'autorace.launch.py',
             args={k: LaunchConfiguration(k) for k in passed}),
    ])
