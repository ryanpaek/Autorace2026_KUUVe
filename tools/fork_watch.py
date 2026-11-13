#!/usr/bin/env python3
"""갈림길(M7 후반 ~ M2) 4가지만 시간순으로: ① 무엇을 인식했나 ② 어떻게 판단했나 ③ 지금 몇 차로 ④ 어디로 옮기려 하나.

  python3 ~/autorace_ws/tools/fork_watch.py          # 바뀔 때만 한 줄 (+ 2초마다 한 줄)

화면과 ~/fork_watch.csv (엑셀로 열 수 있음, 같은 내용) 에 남김.

 ① 인식   /perception/detections   이번 프레임에 본 갈림길 표지와 신뢰도  예) A 0.83 / B 0.71 / - (안 보임)
 ② 판단   /perception/commands      투표 A n/필요, B n/필요 와 확정(SELECT_ROUTE)  +  /mission/state fork_choice (FSM 최종)
 ③ 지금   /m7/lane                  1·2·3 차로 (근거 avoid/camera/last/start)
 ④ 목표   /avoid/state              회피 노드가 가려는 차로와 출구 계획(go/wait/too_late, 섬까지 남은 m)
                                    M2에서는 차선 모드 (left_only = A 바깥선, right_only = B 바깥선)
"""
import csv
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

LANE_NO = {1: '1', 0: '2', -1: '3'}
SHORT = {'M7_OBSTACLE': 'M7', 'M2_FORK': 'M2', 'M8_CONES': 'M8', 'M3_PEDESTRIAN': 'M3', 'WAIT_START': 'WAIT'}


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def seen(det):
    best = {}
    for d in det.get('detections', []) or []:
        c = {'fork_a': 'A', 'fork_b': 'B'}.get(d.get('cls'))
        if c:
            best[c] = max(best.get(c, 0.0), float(d.get('conf', 0.0)))
    return ' '.join(f'{k} {v:.2f}' for k, v in sorted(best.items())) or '-'


def judged(cmd, m):
    v = cmd.get('vote') or {}
    cnt, need = v.get('counts') or {}, v.get('need')
    route = [c.get('route') for c in cmd.get('commands', []) if c.get('command') == 'SELECT_ROUTE']
    vote = f'A{cnt.get("fork_a", 0)}/B{cnt.get("fork_b", 0)} 필요{need}' if cmd else '-'
    conf = f'확정{route[0]}' if route else ('충돌' if 'fork_direction' in (cmd.get('conflicts') or []) else '')
    return f'{vote} {conf}'.strip(), str(m.get('fork_choice') or '-')


def target(av, m):
    if m.get('state') == 'M2_FORK':
        mode = (m.get('lane_cfg') or {}).get('mode')
        return {'left_only': 'A 바깥선 따라감', 'right_only': 'B 바깥선 따라감'}.get(mode, f'차선 {mode}')
    if av.get('state') != 'RUN':
        return '-'
    tg = LANE_NO.get(av.get('target'), '?')
    now = LANE_NO.get(av.get('now_lane'), '?')
    move = f'{now}->{tg}차로' if tg != now else f'{tg}차로 유지'
    plan = av.get('exit_plan')
    if plan:
        rem = av.get('exit_rem_m')
        move += f'  출구 {plan}' + (f' (섬까지 {rem}m)' if rem is not None else '')
    return move


class Watch(Node):
    def __init__(self):
        super().__init__('fork_watch')
        self.m, self.det, self.cmd, self.lane, self.av = {}, {}, {}, {}, {}
        self.last, self.next_t = None, 0.0
        self.t0 = time.monotonic()
        path = os.path.expanduser('~/fork_watch.csv')
        new = not os.path.exists(path)
        self.f = open(path, 'a', newline='')
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(['시각(s)', '미션', '①인식', '②투표·확정', '②FSM결정', '③지금차로', '④목표'])
        self.w.writerow([f'==== 시작 {time.strftime("%H:%M:%S")} ===='])
        for topic, attr in (('/mission/state', 'm'), ('/perception/detections', 'det'),
                            ('/perception/commands', 'cmd'), ('/m7/lane', 'lane'), ('/avoid/state', 'av')):
            self.create_subscription(String, topic, lambda x, a=attr: setattr(self, a, load(x)), 10)
        print(f'===== 시작 {time.strftime("%H:%M:%S")} =====', flush=True)
        print(f'{"  시각":>8} | {"미션":<4} | {"① 인식":<12} | {"② 투표·확정":<20} | {"② FSM":<5} | '
              f'{"③ 지금":<12} | ④ 목표', flush=True)
        self.create_timer(0.1, self.tick)

    def tick(self):
        now = time.monotonic()
        st = SHORT.get(self.m.get('state'), str(self.m.get('state'))[:4])
        a = seen(self.det)
        vote, fsm = judged(self.cmd, self.m)
        ln = self.lane
        cur = f'{ln.get("lane", "-")}차로({ln.get("src", "-")})' if ln else '-'
        tg = target(self.av, self.m)
        row = (st, a, vote, fsm, cur, tg)
        if row != self.last or now >= self.next_t:
            mark = '*' if row != self.last else ' '
            t = f'{now - self.t0:7.1f}s'
            print(f'{mark}{t} | {st:<4} | {a:<12} | {vote:<20} | {fsm:<5} | {cur:<12} | {tg}', flush=True)
            self.w.writerow([f'{now - self.t0:.1f}', *row])
            self.f.flush()
            self.last, self.next_t = row, now + 2.0


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
    sys.exit(main())
