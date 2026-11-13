#!/usr/bin/env python3
"""M7에서 왜 멈추는지: 차가 서 있는 동안 원인을 한 줄로 (멈출 때·원인이 바뀔 때·서 있는 동안 1초마다).

  python3 ~/autorace_ws/tools/m7_stop_watch.py

화면과 ~/m7_stop_watch.log 에 같이 남김. 움직이는 동안은 아무것도 안 찍음 (멈춘 순간만 봄).

 기호  원인                                   어디서                     보는 것
 S    센서 끊김: /scan 0.5초·/odom 0.3초 넘게 안 옴   회피 노드 안전장치          /scan·/odom 도착 간격 (CPU 과부하면 늦게 옴)
 H    회피 노드 소식 끊김 1초 / 끝나기 전에 놓음    미션(M7 연결) 정지 유지      /obstacle/state m7_hold, m7_hold_kind
 E    범퍼 앞 0.3 m 안에 라이다 점 2개 이상       회피 노드 비상               /avoid/state emergency + 가까운 장애물
 B    가는 길 0.3 m 안에서 부딪힘 + 끝까지 꺾어도 못 빠짐   회피 노드 막힘    /avoid/state blocked, hit_m, 차로별 판단(lane_why)
 F    미션이 정지 명령 (stop)                   미션 FSM                   /mission/state source
 L    차선 노드가 정지 (선을 오래 놓침)          차선 노드 (회피 켜기 전·코너)  /lane/status source == stop
 ?    위에 해당 없음 (속도 명령은 0인데 이유 모름) -                          v, 운전 주체
"""
import json
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LANE_NO = {1: '1', 0: '2', -1: '3'}


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def judge(m, ob, av, ln, gaps):
    """-> (기호, 설명). gaps = {'scan': s, 'odom': s, 'avoid': s} 마지막 도착 뒤 지난 시간 (없으면 None)"""
    src = m.get('source')
    if ob.get('m7_hold'):
        return 'H', f'회피 노드 소식 끊김/놓음 ({ob.get("m7_hold_kind")}) -> 미션이 정지 유지'
    if src == 'obstacle':
        sg, og, ag = gaps.get('scan'), gaps.get('odom'), gaps.get('avoid')
        if (sg is not None and sg > 0.5) or (og is not None and og > 0.3):
            return 'S', f'센서 끊김 (scan {sg if sg is None else round(sg, 2)}s, odom {og if og is None else round(og, 2)}s) -> 회피 노드 정지'
        if ag is not None and ag > 0.6:
            return 'S', f'/avoid/state {ag:.1f}s 안 옴 (회피 노드 센서 끊김 정지 또는 느림)'
        near = av.get('obs_near') or []
        near_s = ' '.join(f'[앞{o[0]:+.2f} y{o[1]:+.2f}~{o[2]:+.2f}]' for o in near[:3]) or '없음'
        lane = f'{LANE_NO.get(av.get("now_lane"), "?")}->{LANE_NO.get(av.get("target"), "?")}차로 psi{av.get("psi_deg")}°'
        if av.get('emergency'):
            return 'E', f'범퍼 앞 0.3 m 안 라이다 점 (비상) {lane} 확정장애물 {near_s}'
        if av.get('blocked'):
            why = av.get('lane_why') or {}
            why_s = ' / '.join(f'{LANE_NO.get(int(k), k)}차로:{v}' for k, v in why.items())
            return 'B', f'가는 길 {av.get("hit_m")}m 앞 부딪힘, 끝까지 꺾어도 못 빠짐 {lane} 장애물 {near_s} | {why_s}'
        return '?', f'회피 운전 중인데 0 (state {av.get("state")}, squeeze {av.get("squeezing")}) {lane}'
    if src == 'stop':
        return 'F', f'미션 정지 명령 (state {m.get("state")}, sub {m.get("sub")})'
    if ln.get('source') == 'stop':
        return 'L', f'차선 노드 정지: 선 놓침 (status {ln.get("status")})'
    return '?', f'운전 {src}, 차선 {ln.get("status")}/{ln.get("source")}'


class Watch(Node):
    def __init__(self):
        super().__init__('m7_stop_watch')
        self.m, self.ob, self.av, self.ln = {}, {}, {}, {}
        self.v = None
        self.t_scan = self.t_odom = self.t_av = None
        self.stop_t, self.last_key, self.next_t = None, None, 0.0
        self.t0 = time.monotonic()
        self.log = open(os.path.expanduser('~/m7_stop_watch.log'), 'a')
        self.create_subscription(String, '/mission/state', lambda x: setattr(self, 'm', load(x)), 10)
        self.create_subscription(String, '/obstacle/state', lambda x: setattr(self, 'ob', load(x)), 10)
        self.create_subscription(String, '/avoid/state', self._on_av, 10)
        self.create_subscription(String, '/lane/status', lambda x: setattr(self, 'ln', load(x)), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda x: setattr(self, 'v', float(x.linear.x)), 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.out(f'===== 시작 {time.strftime("%H:%M:%S")} =====  M7에서 서 있을 때만 찍음 (S 센서 H 소식끊김 E 비상 B 막힘 F 미션 L 차선)')
        self.create_timer(0.1, self.tick)

    def _on_av(self, msg):
        self.av = load(msg)
        self.t_av = time.monotonic()

    def _on_scan(self, _):
        self.t_scan = time.monotonic()

    def _on_odom(self, _):
        self.t_odom = time.monotonic()

    def out(self, s):
        print(s, flush=True)
        self.log.write(s + '\n')
        self.log.flush()

    def tick(self):
        now = time.monotonic()
        if self.m.get('state') != 'M7_OBSTACLE' or self.v is None or abs(self.v) > 0.02:
            if self.stop_t is not None:
                self.out(f' {now - self.t0:7.1f}s  -> 다시 출발 (섰던 시간 {now - self.stop_t:.1f}s)')
            self.stop_t, self.last_key = None, None
            return
        gap = lambda t: None if t is None else now - t     # noqa: E731
        sym, why = judge(self.m, self.ob, self.av, self.ln,
                         {'scan': gap(self.t_scan), 'odom': gap(self.t_odom), 'avoid': gap(self.t_av)})
        if self.stop_t is None:
            self.stop_t = now
        if sym != self.last_key or now >= self.next_t:
            mark = '*' if sym != self.last_key else ' '
            self.out(f'{mark}{now - self.t0:7.1f}s  {sym}  정지{now - self.stop_t:4.1f}s  x{self.av.get("x", "-")}  {why}')
            self.last_key, self.next_t = sym, now + 1.0


def main():
    rclpy.init()
    n = Watch()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
