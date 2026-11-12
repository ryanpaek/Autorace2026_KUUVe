#!/usr/bin/env python3
"""M7 직전 코너 감시: 누가 운전하는지, 차선을 따라가는지, 언제 바뀌는지를 한 줄 O/X로.

  python3 ~/autorace_ws/tools/m7_curve_watch.py            # 0.5초마다 + 바뀌면 바로 (* 표시)
  python3 ~/autorace_ws/tools/m7_curve_watch.py 1.0        # 1초마다

화면과 ~/m7_curve_watch.log 에 같이 남김. 바뀐 칸은 줄 끝 '바뀜:'에 번호로 나옴.

 칸  뜻                                   O                    X
 1   미션 상태                            (이름: M3/M7/M2 …)
 2   운전 주체 (/mission/state source)    lane = 차선 주행      ob = 회피/콘, st = 정지
 3   차선 양쪽 선 보임 (/lane/status)     BOTH                 한쪽(L/R)·NONE
 4   차선 노드가 선을 보고 조향            source lane/memory   hold(선 놓침→직진·방향유지), route(도면), stop
 5   도면 위치 믿음 (route lost 아님)      O                    X (길 잃음: 도면 곡률 안 씀)
 6   회피(avoid) 켜짐 (/avoid/active)      O                    X
 7   M7 코너 다 돎 (IMU 80°)              O                    X(돌고 있음, 각도) / -(해당 없음)
 8   실제 /cmd_vel 회전                   L(왼)/R(오른)/0(직진) 과 각속도
"""
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

SHORT = {'WAIT_START': 'WAIT', 'M3_PEDESTRIAN': 'M3', 'M7_OBSTACLE': 'M7', 'M2_FORK': 'M2',
         'M8_CONES': 'M8', 'M6_ZIGZAG': 'M6', 'M5_BUMP': 'M5', 'M4_PARKING': 'M4', 'M1_TRAFFIC': 'M1'}


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def ox(v):
    return 'O' if v else 'X'


class Watch(Node):
    def __init__(self, period):
        super().__init__('m7_curve_watch')
        self.m, self.l, self.o = {}, {}, {}
        self.avoid = None
        self.w = 0.0
        self.last = None
        self.t0 = time.monotonic()
        self.log = open(os.path.expanduser('~/m7_curve_watch.log'), 'a')
        self.create_subscription(String, '/mission/state', lambda x: setattr(self, 'm', load(x)), 10)
        self.create_subscription(String, '/lane/status', lambda x: setattr(self, 'l', load(x)), 10)
        self.create_subscription(String, '/obstacle/state', lambda x: setattr(self, 'o', load(x)), 10)
        self.create_subscription(Bool, '/avoid/active', lambda x: setattr(self, 'avoid', bool(x.data)), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda x: setattr(self, 'w', float(x.angular.z)), 10)
        self.out(f'===== 시작 {time.strftime("%H:%M:%S")} =====')
        self.out('  시각  | 1미션 | 2운전 | 3양쪽선 | 4선보고조향 | 5도면 | 6회피 | 7코너  | 8회전')
        self.next_t = 0.0
        self.period = period
        self.create_timer(0.1, self.tick)

    def out(self, s):
        print(s, flush=True)
        self.log.write(s + '\n')
        self.log.flush()

    def cols(self):
        m, ln, o = self.m, self.l, self.o
        st = SHORT.get(m.get('state'), str(m.get('state'))[:4])
        sub = m.get('sub') or ''
        src = {'lane': 'lane', 'obstacle': 'ob', 'stop': 'st', 'parking': 'pk'}.get(m.get('source'), str(m.get('source')))
        both = ln.get('status') == 'BOTH'
        lsrc = ln.get('source')
        see = lsrc in ('lane', 'memory')
        route = m.get('route') or {}
        route_ok = (not route.get('lost')) if route else None
        av = self.avoid if self.avoid is not None else bool(o.get('m7_active'))
        corner = o.get('m7_corner')
        c7 = '-' if corner is None else ('O' if corner == 'done' else f'X{str(corner).replace("wait", "")}°')
        rot = 'L' if self.w > 0.05 else ('R' if self.w < -0.05 else '0')
        return [
            f'{st}{("/" + sub[:4]) if sub else ""}',
            src,
            f'{ox(both)}({ln.get("status", "?")})',
            f'{ox(see)}({lsrc})',
            '-' if route_ok is None else ox(route_ok),
            ox(av),
            c7,
            f'{rot}{self.w:+.2f}',
        ]

    def tick(self):
        now = time.monotonic()
        c = self.cols()
        key = tuple(c[:7]) + (c[7][0],)            # 회전은 방향만 바뀜으로 봄
        changed = [] if self.last is None else [str(i + 1) for i, (a, b) in enumerate(zip(key, self.last)) if a != b]
        if changed or now >= self.next_t:
            mark = '*' if changed else ' '
            self.out(f'{mark}{now - self.t0:6.1f}s | {c[0]:<7}| {c[1]:<5} | {c[2]:<7} | {c[3]:<11} | {c[4]:^4} | {c[5]:^4} | '
                     f'{c[6]:<6} | {c[7]}' + (f'   바뀜: {",".join(changed)}' if changed else ''))
            self.next_t = now + self.period
        self.last = key


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    rclpy.init()
    n = Watch(period)
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
