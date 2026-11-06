#!/usr/bin/env python3
"""
[4단계] 차선 변경 동작만 시험  — 라이다 안 씀, 장애물 없이 (★ 바퀴 움직임)

  ① 가운데 차선 중앙, 똑바로 세워두기 (팀 브리지를 쓰면 브리지 켜기 / out_mode:=ack 면 브리지 끄기)
  ② 실행 후 시작 신호:  ros2 topic pub -t 3 /avoid/enable std_msgs/msg/Bool "{data: true}"
  ③ change_at(0.5m) 지나면 change_lanes(+1=왼쪽) 만큼 옆 차선으로 옮기고 total_dist(3m)에서 정지
  ④ 줄자로 실제 옆 이동량 재기 → 0.55m 근처인지, 몇 m 만에 옮겨졌는지(로그) 확인
  멈추기:  Ctrl+C  (정지 명령을 보내고 끝남)  또는  /avoid/enable false

튜닝: lookahead ↓ = 빨리/급하게 옮김, ↑ = 부드럽지만 길게.  min_turn_radius 는 실측값(0.53) 아래로 X
/odom 이 0.3초 넘게 끊기면 정지한다.
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner
from limo_obstacle_avoid.ros_common import load_params, PoseInputs, CmdOut, init_rclpy, shutdown_rclpy, declare


class LaneChangeTest(Node):
    def __init__(self):
        super().__init__('s4_lane_change_test')
        self.p = load_params(self, AvoidParams())
        declare(self, 'change_lanes', 1)
        declare(self, 'change_at', 0.5)
        declare(self, 'total_dist', 3.0)
        self.inp = PoseInputs(self)
        self.inp.on_odom = self.on_odom
        self.pl = AvoidPlanner(self.p)
        self.out = CmdOut(self, '/cmd_vel')
        self.create_subscription(Bool, '/avoid/enable', self.on_enable, 10)
        self.running = False
        self.reached_x = None
        self.t0 = time.monotonic()
        self.checked = False
        self.create_timer(0.05, self.loop)
        self.create_timer(0.5, self.show)
        self.get_logger().info(f'대기 중 — /avoid/enable true 를 보내면 출발 (출력 {self.out.topic}, {self.out.mode})')

    def on_enable(self, m):
        if m.data and not self.running:
            if not self.inp.ready():
                self.get_logger().warn('odom/imu 없음 (드라이버, 토픽 이름 확인)')
                return
            ok, msg = self.out.check()
            if not ok:
                self.get_logger().warn(msg + ' → 출발 안 함')
                return
            mode = self.inp.lock()
            ox, oy, yaw, _ = self.inp.get()
            self.pl.start(ox, oy, yaw)
            self.running = True
            self.reached_x = None
            self.get_logger().info(f'출발! (방향={mode})')
        elif not m.data and self.running:
            self.stop('수동 정지')

    def on_odom(self):
        if self.running:
            self.pl.pose.update(*self.inp.get())

    def stop(self, why):
        self.running = False
        self.inp.unlock()
        self.out.stop()
        self.get_logger().info(f'정지 ({why})  최종 x={self.pl.pose.x:.3f}  y={self.pl.pose.y:.3f}  '
                               f'방향={math.degrees(self.pl.pose.psi):+.1f}°')

    def loop(self):
        if not self.checked and time.monotonic() - self.t0 > 2.0:
            self.checked = True
            ok, msg = self.out.check()
            (self.get_logger().info if ok else self.get_logger().warn)(msg)
        if not self.running:
            return
        if not self.inp.fresh(0.3, 0.5):
            self.stop('odom/imu 끊김')
            return
        s = self.pl.pose
        goal = self.p.start_lane + self.get_parameter('change_lanes').value
        if s.x >= self.get_parameter('change_at').value:
            self.pl.target = goal
        if self.reached_x is None and self.pl.target == goal and \
                abs(s.y - self.pl.center(goal)) < self.p.reach_tol:
            self.reached_x = s.x
            self.get_logger().info(f'목표 차선 도착: x={s.x:.2f}m  (변경 시작 후 '
                                   f'{s.x - self.get_parameter("change_at").value:.2f}m 걸림)')
        if s.x >= self.get_parameter('total_dist').value:
            self.stop('목표 거리 도달')
            return
        in_lane = abs(s.y - self.pl.center(self.pl.target)) < self.p.reach_tol
        v = self.p.v_cruise if in_lane else self.p.v_change
        self.out.send(v, self.pl.control(v))

    def show(self):
        if self.running:
            s = self.pl.pose
            self.get_logger().info(f'x={s.x:.2f} y={s.y:+.3f} 방향={math.degrees(s.psi):+.1f}° 목표차선={self.pl.target:+d}')


def main():
    init_rclpy()
    n = LaneChangeTest()
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
