#!/usr/bin/env python3
"""M7이 켜지는지 + 차선 인식이 되는지를 한 줄 O/X로 실시간 감시.

  python3 ~/autorace_ws/tools/m7_live.py          # 0.5초마다 + 바뀌면 바로 (* 와 바뀐 번호)
  python3 ~/autorace_ws/tools/m7_live.py 1.0      # 1초마다

화면과 ~/m7_live.log 에 같이 남김.

 번호  보는 것                         O                          X / 값
 1    미션이 M7인가                    M7_OBSTACLE                 다른 상태 이름
 2    M7 연결 단계 (/obstacle/state)   running(회피 중)·done       idle(대기)·armed(켬 보냄, 회피 노드 아직)
 3    회피 노드가 실제로 도나          /avoid/active true          false
 4    운전 주체 (FSM source)           obstacle(회피)              lane(차선)·stop
 5    코너 지킴 (IMU)                  done                        wait숫자°(도는 중). '+보완' = 미션이 코너 곡률을 차선 노드에 넣는 중 (M7 직전·M2 뒤 코너)
 6    앞 장애물 (라이다, 켜기 판단)     값(mm). 1300 안이면 켤 조건
 7    차선 양쪽 선                     BOTH                        L/R(한쪽)·NONE
 8    차선 노드가 선 보고 조향          lane/memory                 hold(놓침)·route(코너 곡률)·stop
 9    카메라 차로 번호 / 지금 차로      1·2·3                       -(모름)
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


LANE_NO = {1: '1', 0: '2', -1: '3'}


def cols(m, ob, avoid, ln, m7lane):
    st = m.get('state')
    ph = ob.get('m7_phase')
    src = m.get('source')
    corner = ob.get('m7_corner')
    front = ob.get('m7_front_mm')
    lstat, lsrc = ln.get('status'), ln.get('source')
    return [
        ('1', 'O' if st == 'M7_OBSTACLE' else 'X', str(st)[:13]),
        ('2', 'O' if ph in ('running', 'done') else 'X', str(ph)),
        ('3', 'O' if avoid else 'X', 'on' if avoid else 'off'),
        ('4', 'O' if src == 'obstacle' else 'X', str(src)),
        ('5', '-' if corner is None else ('O' if corner == 'done' else 'X'),
         ('-' if corner is None else str(corner)) + ('+보완' if m.get('corner_preview') else '')),
        ('6', '-', '-' if front is None else f'{front}mm'),
        ('7', 'O' if lstat == 'BOTH' else 'X', str(lstat)),
        ('8', 'O' if lsrc in ('lane', 'memory') else 'X', str(lsrc)),
        ('9', '-', f'cam{LANE_NO.get(ln.get("lane_id"), "-")}/now{m7lane.get("lane", "-")}'),
    ]


class Live(Node):
    def __init__(self, period):
        super().__init__('m7_live')
        self.m, self.ob, self.ln, self.m7lane = {}, {}, {}, {}
        self.avoid = False
        self.last, self.period, self.next_t = None, period, 0.0
        self.t0 = time.monotonic()
        self.log = open(os.path.expanduser('~/m7_live.log'), 'a')
        self.create_subscription(String, '/mission/state', lambda x: setattr(self, 'm', load(x)), 10)
        self.create_subscription(String, '/obstacle/state', lambda x: setattr(self, 'ob', load(x)), 10)
        self.create_subscription(String, '/lane/status', lambda x: setattr(self, 'ln', load(x)), 10)
        self.create_subscription(String, '/m7/lane', lambda x: setattr(self, 'm7lane', load(x)), 10)
        self.create_subscription(Bool, '/avoid/active', lambda x: setattr(self, 'avoid', bool(x.data)), 10)
        self.out(f'===== 시작 {time.strftime("%H:%M:%S")} =====  (O 정상 / X 아님)')
        self.out('  시각   | 1 미션          | 2 M7단계     | 3 회피노드 | 4 운전     | 5 코너      | 6 앞장애물 '
                 '| 7 양쪽선  | 8 선보고조향 | 9 차로(카메라/지금)')
        self.create_timer(0.1, self.tick)

    def out(self, s):
        print(s, flush=True)
        self.log.write(s + '\n')
        self.log.flush()

    def tick(self):
        now = time.monotonic()
        c = cols(self.m, self.ob, self.avoid, self.ln, self.m7lane)
        key = tuple(x[1] for x in c[:5]) + (c[6][1], c[7][1], c[8][2])
        idx = ['1', '2', '3', '4', '5', '7', '8', '9']
        changed = [] if self.last is None else [idx[i] for i in range(len(key)) if key[i] != self.last[i]]
        if changed or now >= self.next_t:
            w = [15, 12, 9, 9, 10, 9, 8, 10, 12]
            cells = ' | '.join(f'{ok} {v}'.ljust(w[i]) for i, (_, ok, v) in enumerate(c))
            self.out(f'{"*" if changed else " "}{now - self.t0:6.1f}s | {cells}'
                     + (f'  바뀜: {",".join(changed)}' if changed else ''))
            self.next_t = now + self.period
        self.last = key


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    rclpy.init()
    n = Live(period)
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
