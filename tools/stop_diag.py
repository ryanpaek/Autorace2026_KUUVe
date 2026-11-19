#!/usr/bin/env python3
"""stop_diag.py — 시험장에서 "직진하다 멈춤" 원인 후보를 토픽으로 받기 (11/18).

LIMO에서 launch 띄운 뒤 다른 터미널에서 (colcon 빌드 필요 없음):
    python3 ~/autorace_ws/tools/stop_diag.py
    ros2 topic echo /diag/stop_reason        # 스냅샷만 보기

하는 일
  - 1초마다 한 줄 요약 출력 (cmd_vel, 실제 속도, 카메라 Hz, 라이다 정면 최소거리, 상태 토픽)
  - 출발 후 움직이다가 1초 이상 멈추면 "STOP 스냅샷"을 크게 출력 + /diag/stop_reason 발행
  - /cmd_vel, /mission/estop 을 발행하는 노드 목록을 보여줌 (다른 팀 노드가 섞이는지 확인)
  - /limo/ack_cmd(브리지 -> 차체) 속도도 봄: 명령 > 0 인데 odom 0 이면 물리적으로 걸린 것
  - 시작할 때 ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY 출력 (대회장 네트워크 섞임 확인)
"""
import os
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict

# 문자열/상태 토픽 (타입은 실행 중에 자동으로 알아냄)
STATE_TOPICS = [
    '/mission/state', '/lane/status', '/lane/cmd_vel', '/obstacle/state',
    '/perception/commands', '/avoid/state', '/fork/state', '/parking/state',
    '/mission/estop', '/mission/stage',
]
WATCH_PUBLISHERS = ['/cmd_vel', '/mission/estop', '/mission/set_state', '/mission/start']


class StopDiag(Node):
    def __init__(self):
        super().__init__('stop_diag')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('front_deg', 15.0)      # 라이다 정면 ±각도
        self.declare_parameter('stop_speed', 0.02)     # m/s 이하면 정지로 봄
        self.declare_parameter('stop_hold', 1.0)       # 이만큼 계속 멈추면 스냅샷

        self.cmd = None
        self.cmd_t = 0.0
        self.spd = 0.0
        self.odom_t = 0.0
        self.img_n = 0
        self.img_hz = 0.0
        self.img_t = 0.0
        self.scan_min = None
        self.scan_t = 0.0
        self.states = {}          # topic -> (time, text)
        self.state_subs = {}
        self.moved = False
        self.stop_since = None
        self.snap_done = False

        q = qos_profile_sensor_data
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, q)
        self.create_subscription(LaserScan, '/scan', self.on_scan, q)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_img, q)
        self.pub = self.create_publisher(String, '/diag/stop_reason', 10)

        self.ack = None
        self.ack_t = 0.0
        try:
            from ackermann_msgs.msg import AckermannDrive
            self.create_subscription(AckermannDrive, '/limo/ack_cmd', self.on_ack, 10)
        except ImportError:
            self.get_logger().warn('ackermann_msgs 없음: /limo/ack_cmd 안 봄')

        dom = os.environ.get('ROS_DOMAIN_ID', '(없음=0)')
        loc = os.environ.get('ROS_LOCALHOST_ONLY', '(없음)')
        self.get_logger().info(f'ROS_DOMAIN_ID={dom} ROS_LOCALHOST_ONLY={loc}')
        if loc != '1' and dom in ('(없음=0)', '0'):
            self.get_logger().warn('도메인 0 + localhost 아님: 같은 와이파이의 다른 팀 ROS 토픽이 섞일 수 있음')

        self.create_timer(2.0, self.attach_state_topics)
        self.create_timer(1.0, self.tick)
        self.create_timer(0.2, self.check_stop)
        self.attach_state_topics()

    # ---------- 콜백 ----------
    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_t = time.time()

    def on_ack(self, m):
        self.ack = (m.speed, m.steering_angle)
        self.ack_t = time.time()

    def on_odom(self, m):
        self.spd = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.odom_t = time.time()
        if self.spd > 0.08:
            self.moved = True

    def on_scan(self, m):
        fd = math.radians(self.get_parameter('front_deg').value)
        best = None
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= fd and m.range_min < r < m.range_max and math.isfinite(r):
                best = r if best is None else min(best, r)
        self.scan_min = best
        self.scan_t = time.time()

    def on_img(self, _m):
        self.img_n += 1
        self.img_t = time.time()

    def attach_state_topics(self):
        types = dict(self.get_topic_names_and_types())
        for t in STATE_TOPICS:
            if t in self.state_subs or t not in types:
                continue
            try:
                cls = get_message(types[t][0])
            except Exception:
                continue
            self.state_subs[t] = self.create_subscription(
                cls, t, lambda m, t=t: self.on_state(t, m), 10)

    def on_state(self, topic, m):
        try:
            d = message_to_ordereddict(m)
            txt = d['data'] if list(d.keys()) == ['data'] else str(dict(d))
        except Exception:
            txt = str(m)
        self.states[topic] = (time.time(), str(txt).replace('\n', ' ')[:160])

    # ---------- 요약 ----------
    def age(self, t):
        return '없음' if t == 0.0 else f'{time.time() - t:.1f}s전'

    def publishers(self, topic):
        try:
            infos = self.get_publishers_info_by_topic(topic)
            return [f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in infos
                    if i.node_name != 'stop_diag']
        except Exception:
            return []

    def suspects(self):
        now = time.time()
        s = []
        if self.cmd_t == 0.0 or now - self.cmd_t > 0.5:
            s.append('cmd_vel 끊김 (mission/lane 노드가 명령을 안 냄 또는 죽음)')
        elif self.cmd and abs(self.cmd[0]) < 0.01:
            s.append('cmd_vel=0 (SW가 정지 명령) -> 아래 상태 토픽 note 확인')
        elif self.cmd and abs(self.cmd[0]) >= 0.05 and self.spd < 0.02:
            if self.ack is not None and now - self.ack_t < 0.5 and abs(self.ack[0]) < 0.01:
                s.append('cmd_vel은 나가는데 /limo/ack_cmd=0 (ackermann_bridge 문제/타임아웃)')
            else:
                s.append('명령은 나가는데 차가 안 감 (턱/바퀴 걸림, 배터리, 차체 시리얼)')
        if self.img_t == 0.0 or now - self.img_t > 1.0:
            s.append('카메라 프레임 끊김')
        elif self.img_hz < 10:
            s.append(f'카메라 {self.img_hz:.0f}Hz 로 느림')
        if self.scan_min is not None and self.scan_min < 0.6:
            s.append(f'라이다 정면 {self.scan_min:.2f} m 에 물체 (장애물 정지 가능)')
        if now - self.scan_t > 1.0:
            s.append('라이다 끊김')
        cmd_pubs = self.publishers('/cmd_vel')
        if len(cmd_pubs) > 1:
            s.append(f'/cmd_vel 발행자 {len(cmd_pubs)}개: {cmd_pubs}')
        for t, (_tt, txt) in self.states.items():
            low = txt.lower()
            if any(k in low for k in ('stop', 'lost', 'estop', 'true', 'red', 'wait', 'fallback')):
                if t in ('/mission/estop',) and 'false' in low:
                    continue
                s.append(f'{t}: {txt[:80]}')
        return s

    def tick(self):
        self.img_hz = self.img_n / 1.0
        self.img_n = 0
        cmd = 'None' if self.cmd is None else f'v={self.cmd[0]:+.2f} w={self.cmd[1]:+.2f}'
        sm = '-' if self.scan_min is None else f'{self.scan_min:.2f}m'
        ms = self.states.get('/mission/state', (0, '-'))[1][:60]
        ls = self.states.get('/lane/status', (0, '-'))[1][:50]
        self.get_logger().info(
            f'cmd[{cmd} {self.age(self.cmd_t)}] spd={self.spd:.2f} cam={self.img_hz:.0f}Hz '
            f'front={sm} | mission={ms} | lane={ls}')

    def check_stop(self):
        if not self.moved:
            return
        hold = self.get_parameter('stop_hold').value
        if self.spd < self.get_parameter('stop_speed').value:
            if self.stop_since is None:
                self.stop_since = time.time()
            elif not self.snap_done and time.time() - self.stop_since > hold:
                self.snapshot()
                self.snap_done = True
        else:
            self.stop_since = None
            self.snap_done = False

    def snapshot(self):
        sus = self.suspects()
        lines = ['==================== STOP 스냅샷 ====================']
        lines.append(f'cmd_vel={self.cmd} ({self.age(self.cmd_t)}), '
                     f'ack_cmd={self.ack} ({self.age(self.ack_t)}), 실제속도={self.spd:.3f}')
        lines.append(f'카메라 {self.img_hz:.0f}Hz (마지막 {self.age(self.img_t)}), '
                     f'라이다 정면 최소 {self.scan_min}')
        for t in WATCH_PUBLISHERS + ['/limo/ack_cmd']:
            lines.append(f'  발행자 {t}: {self.publishers(t)}')
        for t, (tt, txt) in sorted(self.states.items()):
            lines.append(f'  {t} ({self.age(tt)}): {txt}')
        lines.append('원인 후보: ' + (' / '.join(sus) if sus else '자동 판정 없음 -> 위 값 직접 확인'))
        lines.append('=====================================================')
        text = '\n'.join(lines)
        self.get_logger().warn('\n' + text)
        self.pub.publish(String(data=text))


def main():
    rclpy.init()
    n = StopDiag()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
