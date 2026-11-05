"""전체 스택 실행.

LIMO 드라이버·카메라·라이다를 먼저 띄운 뒤 실행한다 (애커만 LIMO 예):
  ros2 launch limo_base limo_base_ackerman.launch.py
  ros2 launch limo_base open_ydlidar_launch.py
  ros2 launch orbbec_camera dabai.launch.py
  ros2 launch autorace_bringup autorace.launch.py
  ros2 launch autorace_bringup autorace.launch.py start_state:=M3_PEDESTRIAN use_detector:=false
출발: ros2 topic pub --once /mission/start std_msgs/msg/Empty
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('autorace_bringup')
    params = LaunchConfiguration('params')
    bev = LaunchConfiguration('bev_config')
    start_state = LaunchConfiguration('start_state')
    use_detector = LaunchConfiguration('use_detector')
    use_bridge = LaunchConfiguration('use_ackermann_bridge')
    route = LaunchConfiguration('route_file')
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=os.path.join(share, 'config', 'params.yaml')),
        DeclareLaunchArgument('bev_config',
                              default_value=os.path.join(share, 'config', 'bev_limo.yaml')),
        DeclareLaunchArgument('start_state', default_value='WAIT_START'),
        # 도면 경로 지도. 끄려면 route_file:=''
        DeclareLaunchArgument('route_file',
                              default_value=os.path.join(share, 'config', 'track_route.yaml')),
        DeclareLaunchArgument('use_detector', default_value='true'),
        # 이 LIMO의 애커만 드라이버는 /limo/ack_cmd(AckermannDrive)만 받는다
        DeclareLaunchArgument('use_ackermann_bridge', default_value='true'),
        Node(package='autorace_mission', executable='ackermann_bridge', name='ackermann_bridge',
             output='screen', condition=IfCondition(use_bridge)),
        Node(package='autorace_lane', executable='lane_node', name='lane_node',
             parameters=[params, {'bev_config': bev}], output='screen'),
        Node(package='autorace_mission', executable='mission_node', name='mission_node',
             parameters=[params, {'start_state': start_state, 'route_file': route}],
             output='screen'),
        Node(package='autorace_mission', executable='parking_node', name='parking_node',
             parameters=[params], output='screen'),
        Node(package='autorace_obstacle', executable='obstacle_node', name='obstacle_node',
             parameters=[params, {'bev_config': bev}], output='screen'),
        Node(package='autorace_perception', executable='detector_node', name='detector_node',
             parameters=[params], output='screen', condition=IfCondition(use_detector)),
    ])
