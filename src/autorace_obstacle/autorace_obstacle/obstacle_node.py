"""라이다 장애물 노드 (뼈대 + 기본 도구).

/scan -> 전방 관심 영역의 덩어리(클러스터) -> /obstacle/state (String JSON)
미션 상태에 따라 /obstacle/cmd_vel을 낸다.
  M8_CONES   : 가장 가까운 좌/우 콘 사이 중점을 Pure Pursuit로 따라간다 (동작하는 초안)
  M7_OBSTACLE: 회피 주행은 limo_obstacle_avoid 패키지의 avoid_node가 한다 (그 출력은 launch에서
               /obstacle/cmd_vel로 remap). 이 노드는 연결만 맡는다 (m7_link.py):
                 시작선에서 /avoid/enable, 갈림길 방향 -> /fork_direction,
                 /avoid/done -> /obstacle/state의 m7_done, 회피 중이면 m7_active.
               그래서 M8 말고는 /obstacle/cmd_vel을 내지 않는다 (avoid_node와 한 토픽을 나눠 쓰므로).
"""

import json
import time
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from autorace_obstacle.m7_link import M7Link, M7LinkParams

from autorace_obstacle.scan_utils import (cluster_points, cone_gate_target, lane_occupancy,
                                          pursuit_kappa, roi, scan_to_points)


def g_name(name: str) -> str:
    return name if name.startswith('/') else '/' + name


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
        # M7 연결 (m7_link.M7LinkParams 항목을 m7_ 접두어로 선언. 예: m7_start_s_mm)
        p('m7_avoid_node', '/avoid_node')   # section_length를 넣을 회피 노드 이름
        lp = M7LinkParams()
        for f in fields(lp):
            setattr(lp, f.name, p('m7_' + f.name, getattr(lp, f.name)).value)
        self.link = M7Link(lp)
        self.mission = {}
        self.lane = {}
        self.last = {}
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self._on_mission, 10)
        self.state_pub = self.create_publisher(String, '/obstacle/state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        # M7 연결
        self.create_subscription(String, '/lane/status', self._on_lane, 10)
        self.create_subscription(Bool, '/avoid/active', lambda m: self.link.on_avoid_active(m.data), 10)
        self.create_subscription(Bool, '/avoid/done', lambda m: self.link.on_avoid_done(m.data), 10)
        self.create_subscription(String, '/avoid/state', self._on_avoid_state, 10)
        self.enable_pub = self.create_publisher(Bool, '/avoid/enable', 10)
        self.fork_pub = self.create_publisher(String, '/fork_direction', 10)
        self.param_cli = self.create_client(
            SetParameters, g_name(self.get_parameter('m7_avoid_node').value) + '/set_parameters')
        self.create_timer(0.05, self._tick)

    def _on_mission(self, msg):
        try:
            self.mission = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_lane(self, msg):
        try:
            self.lane = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_avoid_state(self, msg):
        try:
            self.link.on_avoid_state(json.loads(msg.data))
        except (json.JSONDecodeError, AttributeError):
            pass

    def _tick(self):
        """20 Hz: M7 연결 + /obstacle/state 발행 (라이다가 없어도 m7_done이 FSM에 가게)."""
        for act in self.link.update(time.monotonic(), self.mission, self.lane):
            if act[0] == 'enable':
                self._enable(act[1])
            elif act[0] == 'disable':
                self.enable_pub.publish(Bool(data=False))
            elif act[0] == 'fork':
                self.fork_pub.publish(String(data=act[1]))
        for e in self.link.events:
            self.get_logger().info(f'[M7] {e}')
        self.link.events.clear()
        state = dict(self.last)
        state.update(self.link.state())
        self.state_pub.publish(String(data=json.dumps(state)))

    def _enable(self, section_m):
        """회피 노드 section_length(원점 ~ 섬 끝)를 지금 위치로 맞춘 뒤 켠다."""
        if not self.param_cli.service_is_ready():
            self.get_logger().warning('[M7] 회피 노드 파라미터 서비스 없음 -> section_length 기본값으로 켬')
            self.enable_pub.publish(Bool(data=True))
            return
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='section_length', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(section_m)))]
        self.param_cli.call_async(req).add_done_callback(
            lambda _f: self.enable_pub.publish(Bool(data=True)))

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
                 'cones_done': False}
        if self.mission.get('state') == 'M8_CONES':
            tw = Twist()
            tgt = cone_gate_target(clusters)
            if tgt is not None:
                v = g('cone_speed').value
                tw.linear.x, tw.angular.z = v, v * pursuit_kappa(tgt)
                state['cone_target'] = tgt
            self.cmd_pub.publish(tw)
        # M7은 avoid_node가 /obstacle/cmd_vel을 낸다. 여기서 0을 섞어 내면 안 된다.
        self.last = state       # /obstacle/state는 _tick에서 m7 값과 합쳐 발행


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
