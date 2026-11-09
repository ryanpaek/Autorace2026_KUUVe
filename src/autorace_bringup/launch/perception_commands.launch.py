"""사용자 담당 범위: 카메라 → YOLO → 행동 명령 발행. 차량 제어 노드는 실행하지 않는다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    params = LaunchConfiguration('params')
    model_path = LaunchConfiguration('model_path')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('model_path',
                              default_value='/home/wego/autorace_ws/models/autorace_yolo_v3.onnx'),
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=[params, {'model_path': model_path}], output='screen'),
        Node(package='autorace_perception', executable='class_command_node',
             name='class_command_node', parameters=[params], output='screen'),
    ])
