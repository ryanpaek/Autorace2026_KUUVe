#!/usr/bin/env python3
"""
cmd_mux — 바퀴 명령 교통정리 (바퀴로 가는 명령은 이 노드 하나만 낸다)

  회피 노드가 /avoid/active true 로 '제어권'을 잡으면        → /avoid/cmd_vel
     └ 제어권을 잡은 채 회피 명령이 끊기면(노드 죽음 등)      → 정지 (차선추종으로 넘기지 않음, failsafe_hold)
  /avoid/active false 로 제어권을 놓으면                      → /lane_follow/cmd_vel (차선추종)
  아무 명령도 없으면                                          → 정지

출력 (out_mode)
  twist : Twist → out_topic(/cmd_vel)        ← 팀 ackermann_bridge 가 /limo/ack_cmd 로 변환 (브리지 켜기)
  ack   : AckermannDrive → ack_topic(/limo/ack_cmd) 직접  (브리지 끄기)

★ 차선추종 노드의 출력 토픽을 /cmd_vel → /lane_follow/cmd_vel 로 바꿔야 함
  (launch 에서 remappings=[('/cmd_vel', '/lane_follow/cmd_vel')])
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from limo_obstacle_avoid.ros_common import init_rclpy, shutdown_rclpy, declare, CmdOut


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        self.lane_topic = declare(self, 'lane_topic', '/lane_follow/cmd_vel')
        self.avoid_topic = declare(self, 'avoid_topic', '/avoid/cmd_vel')
        declare(self, 'timeout', 0.3)
        declare(self, 'publish_zero_when_idle', True)
        declare(self, 'failsafe_hold', True)
        self.out = CmdOut(self, '/cmd_vel', prefix='')
        # out_topic 이름도 받아 줌 (예전 설정 호환)
        if self.out.mode == 'twist':
            ot = declare(self, 'out_topic', self.out.topic)
            if ot != self.out.topic:
                self.out.topic = ot
                self.out.pub = self.create_publisher(Twist, ot, 10)
        self.lane = (None, 0.0)
        self.avoid = (None, 0.0)
        self.claimed = False                 # 회피 노드가 제어권을 잡고 있나
        self.create_subscription(Twist, self.lane_topic, lambda m: setattr(self, 'lane', (m, time.monotonic())), 10)
        self.create_subscription(Twist, self.avoid_topic, lambda m: setattr(self, 'avoid', (m, time.monotonic())), 10)
        self.create_subscription(Bool, '/avoid/active', self.on_active, 10)
        self.src = None
        self.t0 = time.monotonic()
        self.checked = False
        self.create_timer(1.0 / 30.0, self.loop)
        self.get_logger().info(f'cmd_mux 준비: {self.lane_topic} / {self.avoid_topic} → {self.out.topic} ({self.out.mode})')

    def on_active(self, m):
        if m.data and not self.claimed:
            self.get_logger().info('회피 노드가 제어권을 잡음')
        if not m.data and self.claimed:
            self.get_logger().info('회피 노드가 제어권을 놓음 → 차선추종')
        self.claimed = bool(m.data)

    def fresh(self, item):
        return item[0] is not None and time.monotonic() - item[1] < self.get_parameter('timeout').value

    def loop(self):
        if not self.checked and time.monotonic() - self.t0 > 3.0:
            self.checked = True
            ok, msg = self.out.check()
            (self.get_logger().info if ok else self.get_logger().warn)(msg)
        if self.claimed and self.get_parameter('failsafe_hold').value:
            if self.fresh(self.avoid):
                out, src = self.avoid[0], 'avoid'
            else:
                out, src = Twist(), 'avoid 끊김 → 정지'
        elif self.claimed and self.fresh(self.avoid):
            out, src = self.avoid[0], 'avoid'
        elif self.fresh(self.lane):
            out, src = self.lane[0], 'lane'
        else:
            out, src = (Twist() if self.get_parameter('publish_zero_when_idle').value else None), 'none'
        if src != self.src:
            log = self.get_logger().warn if '끊김' in src else self.get_logger().info
            log(f'명령 출처: {self.src} → {src}')
            self.src = src
        if out is not None:
            self.out.send_msg(out)


def main():
    init_rclpy()
    n = CmdMux()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            try:
                n.out.stop()
            except Exception:
                break
            time.sleep(0.02)
        shutdown_rclpy(n)


if __name__ == '__main__':
    main()
