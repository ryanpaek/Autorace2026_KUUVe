"""센서만 한 번에: 차체(limo_base) + 라이다(ydlidar) + 카메라(orbbec dabai). 예전 T1·T2·T3을 한 터미널로.

  ros2 launch autorace_bringup sensors.launch.py

그 다음 자율주행은 따로 (예전 T4 그대로):
  ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true

인자 (하나만 다시 켜야 할 때):
  base:=false / lidar:=false / camera:=false
  cam_ae:=false   카메라 노출 고정(exposure_node) 끄기 -> 카메라 자동 노출 그대로 (11/19)
카메라는 full.launch.py와 같이 컬러만 켠다 (depth·ir를 켜고 전체 녹화하면 드라이버가 죽은 적 있음, 11/09).
주의: full.launch.py는 이미 센서를 포함하므로 같이 띄우지 않는다 (중복 실행).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _inc(pkg, *path, args=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), *path)),
        launch_arguments=(args or {}).items(), condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base', default_value='true'),
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('cam_ae', default_value='true'),
        _inc('limo_base', 'launch', 'limo_base_ackerman.launch.py',
             condition=IfCondition(LaunchConfiguration('base'))),
        _inc('ydlidar_ros2_driver', 'launch', 'ydlidar.launch.py',
             condition=IfCondition(LaunchConfiguration('lidar'))),
        _inc('orbbec_camera', 'launch', 'dabai.launch.py',
             args={'enable_depth': 'false', 'enable_ir': 'false'},
             condition=IfCondition(LaunchConfiguration('camera'))),
        # 11/19: 노란 실선 포화 대응. 카메라 자동 노출을 잠그고 바닥 밝기로 노출을 맞춤 (params.yaml exposure_node)
        Node(package='autorace_bringup', executable='exposure_node', name='exposure_node', output='screen',
             parameters=[os.path.join(get_package_share_directory('autorace_bringup'), 'config', 'params.yaml')],
             condition=IfCondition(PythonExpression(["'", LaunchConfiguration('camera'), "' == 'true' and '", LaunchConfiguration('cam_ae'), "' == 'true'"]))),
    ])
