"""라이다 장애물 노드 (뼈대 + 기본 도구).

/scan -> 전방 관심 영역의 덩어리(클러스터) -> /obstacle/state (String JSON)
미션 상태에 따라 /obstacle/cmd_vel을 낸다.
  M8_CONES   : 가장 가까운 좌/우 콘 사이 중점을 Pure Pursuit로 따라간다 (동작하는 초안)
  M7_OBSTACLE: TODO. 차로 점유(lane_occupancy)로 빈 차로를 고르고 차선 변경.
               갈림길 방향(/mission/state의 fork_choice)에 맞는 쪽 차로로 끝내야 한다.
               방법 A: lane 노드의 target_offset_mm(±500)으로 옆 차로 중심을 따라가게 한다.
               방법 B: 오도메트리로 미리 짠 S자 경로를 재생한다.
"""

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from autorace_obstacle.scan_utils import (cluster_points, cone_gate_target, lane_occupancy,
                                          pursuit_kappa, roi, scan_to_points)


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        p = self.declare_parameter
        p('scan_topic', '/scan')
        p('laser_x_mm', 100.0)       # TODO 실측: 기준점 -> 라이다 중심 전방 거리
        p('laser_y_mm', 0.0)
        p('laser_yaw', 0.0)          # TODO 확인: 라이다 0도가 차 정면인지
        p('roi_x_max', 2000.0)
        p('roi_y_half', 900.0)
        p('cluster_gap_mm', 60.0)
        p('cluster_min_pts', 3)
        p('lane_centers_y', [500.0, 0.0, -500.0])   # 3차선 (왼쪽 +). TODO 실측
        p('lane_half_width', 230.0)
        p('cone_speed', 0.13)
        self.mission = {}
        self.last = {}
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self._on_mission, 10)
        self.state_pub = self.create_publisher(String, '/obstacle/state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)

    def _on_mission(self, msg):
        try:
            self.mission = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_scan(self, msg: LaserScan):
        g = self.get_parameter
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max, g('laser_x_mm').value,
                             g('laser_y_mm').value, g('laser_yaw').value)
        pts = roi(pts, 0.0, g('roi_x_max').value, -g('roi_y_half').value,
                  g('roi_y_half').value)
        clusters = cluster_points(pts, g('cluster_gap_mm').value, g('cluster_min_pts').value)
        occ = lane_occupancy(clusters, g('lane_centers_y').value,
                             g('lane_half_width').value, g('roi_x_max').value)
        front = [c['near_x'] for c in clusters if abs(c['y']) < 150.0]
        state = {'clusters': clusters[:12], 'lane_occupancy': occ,
                 'front_min_mm': min(front) if front else None,
                 'm7_done': False, 'cones_done': False}
        tw = Twist()
        mstate = self.mission.get('state')
        if mstate == 'M8_CONES':
            tgt = cone_gate_target(clusters)
            if tgt is not None:
                v = g('cone_speed').value
                tw.linear.x, tw.angular.z = v, v * pursuit_kappa(tgt)
                state['cone_target'] = tgt
        elif mstate == 'M7_OBSTACLE':
            pass   # TODO: 빈 차로 선택 + 차선 변경 (모듈 설명 참고)
        self.last = state
        self.state_pub.publish(String(data=json.dumps(state)))
        self.cmd_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
