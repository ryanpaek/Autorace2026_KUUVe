"""사용자 담당 범위: 카메라 → YOLO → 행동 명령 발행. 차량 제어 노드는 실행하지 않는다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    params = LaunchConfiguration('params').perform(context)
    model_path = LaunchConfiguration('model_path').perform(context)
    detector_params = [params]
    if model_path:
        detector_params.append({'model_path': model_path})
    return [
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=detector_params, output='screen'),
        Node(package='autorace_perception', executable='class_command_node',
             name='class_command_node', parameters=[params], output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        # 비어 있으면 YAML 선택을 따름. 320 설정을 고정 640 모델 경로로 덮어쓰지 않는다.
        DeclareLaunchArgument('model_path', default_value=''),
        OpaqueFunction(function=_nodes),
    ])
