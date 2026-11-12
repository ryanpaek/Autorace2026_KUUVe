#!/usr/bin/env python3
"""M7 후반(마지막 장애물 ~ 섬 ~ M2)에서 차선을 안 지키는 원인 후보를 한 줄 O/X로 감시.

  python3 ~/autorace_ws/tools/m7_end_watch.py        # 0.5초마다 + 바뀌면 바로 (* 와 바뀐 번호)

O = 정상, X = 의심. ~/m7_end_watch.log 에도 남김.

 번호  후보                                   X 조건
 E1   회피 노드 위치 추정이 카메라와 어긋남      lane_fix_dy 가 10 cm 넘음 (15 cm 넘으면 카메라 보정을 아예 못 받음)
 E2   카메라 차로 번호가 회피 목표와 다름        cam_lane 이 있고 target 과 다름 (그러면 차선 노드 조향을 안 섞음)
 E3   차선 노드 조향을 안 섞고 혼자 감          차로 안(옮기는 중 아님)인데 lane_blend 꺼짐
 E4   비스듬함                                  psi 8° 넘음
 E5   출구 차로로 옮기기가 늦음/못 함           exit_plan 이 too_late 또는 wait
 E6   차선 노드가 선을 놓침                      /lane/status source 가 lane/memory 아님
 E7   M2 넘길 때 모드                           M2인데 left_only/right_only 아님, 또는 status 가 L/R/BOTH 아님
"""
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def judge(m, av, ln):
    out = []
    dy = av.get('lane_fix_dy_cm')
    out.append(('E1', 'X' if dy is not None and abs(dy) > 10 else 'O', f'{dy}cm'))
    cam, tg = av.get('cam_lane'), av.get('target')
    out.append(('E2', 'X' if cam is not None and tg is not None and cam != tg else 'O', f'cam{cam}/tg{tg}'))
    in_lane = tg is not None and av.get('y') is not None and abs(av['y'] - 0.52 * tg) < 0.08
    out.append(('E3', 'X' if (av.get('state') == 'RUN' and in_lane and not av.get('lane_blend')) else 'O',
                'blend' if av.get('lane_blend') else '-'))
    psi = av.get('psi_deg')
    out.append(('E4', 'X' if psi is not None and abs(psi) > 8 else 'O', f'{psi}°'))
    plan = av.get('exit_plan')
    out.append(('E5', 'X' if plan in ('too_late', 'wait') else 'O', f'{plan}/{av.get("exit_rem_m")}m'))
    src = ln.get('source')
    out.append(('E6', 'O' if src in ('lane', 'memory') else 'X', f'{src}/{ln.get("status")}'))
    mode = (m.get('lane_cfg') or {}).get('mode')
    bad7 = m.get('state') == 'M2_FORK' and (mode not in ('left_only', 'right_only') or ln.get('status') == 'NONE')
    out.append(('E7', 'X' if bad7 else 'O', f'{mode}'))
    return out


class Watch(Node):
    def __init__(self, period):
        super().__init__('m7_end_watch')
        self.m, self.av, self.ln = {}, {}, {}
        self.last, self.period, self.next_t = None, period, 0.0
        self.t0 = time.monotonic()
        self.log = open(os.path.expanduser('~/m7_end_watch.log'), 'a')
        self.create_subscription(String, '/mission/state', lambda x: setattr(self, 'm', load(x)), 10)
        self.create_subscription(String, '/avoid/state', lambda x: setattr(self, 'av', load(x)), 10)
        self.create_subscription(String, '/lane/status', lambda x: setattr(self, 'ln', load(x)), 10)
        self.out(f'===== 시작 {time.strftime("%H:%M:%S")} =====  (O 정상 / X 의심)')
        self.out('  시각   미션 회피  x     y    | E1 카메라어긋남 | E2 차로번호 | E3 섞기 | E4 비스듬 | E5 출구 | E6 선놓침 | E7 M2모드')
        self.create_timer(0.1, self.tick)

    def out(self, s):
        print(s, flush=True)
        self.log.write(s + '\n')
        self.log.flush()

    def tick(self):
        now = time.monotonic()
        res = judge(self.m, self.av, self.ln)
        key = tuple(r[1] for r in res) + (self.m.get('state'), self.av.get('state'))
        changed = [] if self.last is None else [res[i][0] for i in range(len(res)) if key[i] != self.last[i]]
        if changed or now >= self.next_t:
            st = str(self.m.get('state'))[:2]
            cells = ' | '.join(f'{h} {ok} {v}'.ljust(14) for h, ok, v in res)
            self.out(f'{"*" if changed else " "}{now - self.t0:6.1f}s {st:<3}{str(self.av.get("state"))[:4]:<5}'
                     f'{self.av.get("x", "-")!s:>5} {self.av.get("y", "-")!s:>5} | {cells}'
                     + (f'  바뀜: {",".join(changed)}' if changed else ''))
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
