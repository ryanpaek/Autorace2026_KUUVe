"""T자 주차 노드 (뼈대).

/mission/state가 M4_PARKING일 때만 /parking/cmd_vel을 낸다. 끝나면 /parking/state에 done=true.

판정: 차체 네 모서리가 모두 칸 안이면 15점, 한 곳만 걸치면 7점. 전면·후면 진입 모두 허용.
칸 크기 500(폭) x 600(깊이)mm, LIMO 322 x 220mm라 여유가 좌우 140mm, 앞뒤 139mm다.

아래 SegmentPlayer는 '속도, 곡률, 거리' 구간 목록을 오도메트리로 그대로 재생한다.
가장 단순한 출발점: 주차 칸 앞 정지 위치를 정확히 맞춘 뒤(정지선과 같은 방식),
미리 잰 구간 목록으로 들어간다. TODO 표시 값은 실측해서 채울 것.
사륜 차동 모드라면 '제자리 90도 회전 -> 직진' 두 구간으로 끝난다 (회전은 kappa 대신 yaw_rate).
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

# (speed m/s, kappa 1/m 왼쪽+, distance mm). 후진은 speed < 0
# TODO 실측: 칸 위치·진입 방향(전면/후면)을 정한 뒤 바닥에서 재고 채울 것
PLAN_ACKERMANN_EXAMPLE = [
    (0.15, 0.0, 250.0),     # 칸 입구 기준점까지 직진
    (0.15, -2.4, 600.0),    # 오른쪽으로 크게 돌며 칸 쪽으로 (최대 조향 근처)
    (-0.12, 0.0, 200.0),    # 후진으로 자세 정리
    (0.12, 0.0, 300.0),     # 칸 안으로 직진
]


class SegmentPlayer:
    def __init__(self, plan):
        self.plan = list(plan)
        self.i = 0
        self.start_dist = None

    def step(self, dist_mm):
        """현재 누적 거리를 받아 (speed, kappa, done)을 돌려준다."""
        if self.i >= len(self.plan):
            return 0.0, 0.0, True
        speed, kappa, length = self.plan[self.i]
        if self.start_dist is None:
            self.start_dist = dist_mm
        if dist_mm - self.start_dist >= length:
            self.i += 1
            self.start_dist = None
            return self.step(dist_mm)
        return speed, kappa, False


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('enabled', False)   # TODO 계획을 채운 뒤 true
        self.state = {}
        self.dist_mm = 0.0
        self._last_xy = None
        self.player = None
        self.create_subscription(String, '/mission/state', self._on_state, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, '/parking/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/parking/state', 10)
        self.create_timer(0.05, self._tick)

    def _on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self.dist_mm += 1000.0 * math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)

    def _tick(self):
        active = self.state.get('state') == 'M4_PARKING'
        done = False
        tw = Twist()
        if active and self.get_parameter('enabled').value:
            if self.player is None:
                self.player = SegmentPlayer(PLAN_ACKERMANN_EXAMPLE)
            v, k, done = self.player.step(self.dist_mm)
            tw.linear.x = v
            tw.angular.z = v * k
        elif not active:
            self.player = None
        self.cmd_pub.publish(tw)
        self.state_pub.publish(String(data=json.dumps({'active': active, 'done': done,
                                                       'stamp': time.monotonic()})))


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
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
