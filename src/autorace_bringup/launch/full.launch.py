"""통주행 한 번에 띄우기: 차체 + 라이다 + 카메라 + 자율주행 전체(주차·M7 회피·갈림길 노드 켬).

  ros2 launch autorace_bringup full.launch.py
  ros2 topic pub --once /mission/start std_msgs/msg/Empty        # 출발

인자:
  sensors:=false        차체·라이다·카메라를 이미 따로 띄웠을 때 (중복 실행 금지!)
  parking:=false / m7_avoid:=false / use_fork_node:=false / use_detector:=false   하나씩 끄기
  start_state:=M4_PARKING 등   중간 구간부터 (띄우자마자 움직임)
  chime:=false          주차 완료 '띠링'(차 스피커) 끄기
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


def _inc(pkg, *path, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), *path)),
        launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    sensors = IfCondition(LaunchConfiguration('sensors'))
    passed = ('start_state', 'parking', 'm7_avoid', 'use_fork_node', 'use_detector', 'use_class_commands',
              'manual', 'chime')
    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true'),
        DeclareLaunchArgument('start_state', default_value='WAIT_START'),
        DeclareLaunchArgument('parking', default_value='true'),
        DeclareLaunchArgument('m7_avoid', default_value='true'),
        DeclareLaunchArgument('use_fork_node', default_value='true'),
        DeclareLaunchArgument('use_detector', default_value='true'),
        DeclareLaunchArgument('use_class_commands', default_value='true'),
        DeclareLaunchArgument('manual', default_value='false'),
        DeclareLaunchArgument('chime', default_value='true'),
        _inc('limo_base', 'launch', 'limo_base_ackerman.launch.py', condition=sensors),
        _inc('ydlidar_ros2_driver', 'launch', 'ydlidar.launch.py', condition=sensors),
        _inc('orbbec_camera', 'launch', 'dabai.launch.py',
             args={'enable_depth': 'false', 'enable_ir': 'false'}, condition=sensors),
        _inc('autorace_bringup', 'launch', 'autorace.launch.py',
             args={k: LaunchConfiguration(k) for k in passed}),
    ])
