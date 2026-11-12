#!/usr/bin/env python3
"""M2(갈림길) ~ M8 앞 코너에서 '누가 조향을 잡고 있나'를 시간순 한 줄로. 차선 기반으로 안 가면 무엇이 끼어들었는지 본다.

  python3 ~/autorace_ws/tools/after_m2_watch.py

M7 후반 ~ M2 ~ M8(콘 들어가기 전)에서만 찍음. 바뀔 때 바로(*), 아니면 0.5초마다. ~/after_m2_watch.log 에도 남김.

 판정(누가 조향)         뜻
 차선(양쪽)              차선 노드가 양쪽 선으로 조향 = 정상
 차선(한쪽 L/R)          한쪽 선만 보고 조향 (M2 left_only/right_only면 정상)
 차선+코너80%            선은 보지만 미션의 코너 미리보기(IMU) 곡률이 80% 섞임 (route_corner_weight)
 코너곡률만(선 놓침)      선을 놓쳐 코너 미리보기 곡률로만 돎 (source route)
 직진유지(선 놓침)        선을 놓쳐 마지막 방향 유지 (source hold) -> 길게 가면 차선 무시처럼 보임
 차선노드 정지            선을 오래 놓쳐 정지 (source stop)
 미션정지 / 콘 노드 / 회피노드 / 주차   미션이 차선 노드가 아닌 쪽을 고름 (mission source)
 덮어씀!                 미션은 lane을 골랐는데 /cmd_vel 조향이 /lane/cmd_vel 과 0.15 넘게 다름 (다른 노드가 같은 토픽에 냄)

 칸: 시각 | 미션(sub) | 차선모드 | 선(status) | 차선노드 source | 안쪽선 | 차선k / 코너k | lane w -> cmd w | 판정
"""
import json
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

SHORT = {'M7_OBSTACLE': 'M7', 'M2_FORK': 'M2', 'M8_CONES': 'M8'}


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def verdict(m, ln, w_lane, w_cmd):
    src = m.get('source')
    if src != 'lane':
        return {'stop': '미션정지', 'obstacle': '콘 노드' if m.get('state') == 'M8_CONES' else '회피노드',
                'parking': '주차'}.get(src, f'미션 source {src}')
    if w_lane is not None and w_cmd is not None and abs(w_lane - w_cmd) > 0.15:
        return '덮어씀!'
    ls, st = ln.get('source'), ln.get('status')
    if ls == 'stop':
        return '차선노드 정지'
    if ls == 'hold':
        return '직진유지(선 놓침)'
    if ls == 'route':
        return '코너곡률만(선 놓침)'
    if m.get('corner_preview'):
        return '차선+코너80%'
    if st == 'BOTH':
        return '차선(양쪽)'
    if st in ('L', 'R'):
        return f'차선(한쪽 {st})'
    return f'? ({ls}/{st})'


def num(v):
    return '-' if v is None else f'{v:+.2f}'


class Watch(Node):
    def __init__(self):
        super().__init__('after_m2_watch')
        self.m, self.ln = {}, {}
        self.w_lane = self.w_cmd = None
        self.last, self.next_t = None, 0.0
        self.t0 = time.monotonic()
        self.log = open(os.path.expanduser('~/after_m2_watch.log'), 'a')
        self.create_subscription(String, '/mission/state', lambda x: setattr(self, 'm', load(x)), 10)
        self.create_subscription(String, '/lane/status', lambda x: setattr(self, 'ln', load(x)), 10)
        self.create_subscription(Twist, '/lane/cmd_vel', lambda x: setattr(self, 'w_lane', float(x.angular.z)), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda x: setattr(self, 'w_cmd', float(x.angular.z)), 10)
        self.out(f'===== 시작 {time.strftime("%H:%M:%S")} =====')
        self.out('    시각  | 미션(sub)    | 차선모드    | 선   | 차선노드 | 안쪽 | 차선k/코너k   | lane w->cmd w  | 판정')
        self.create_timer(0.1, self.tick)

    def out(self, s):
        print(s, flush=True)
        self.log.write(s + '\n')
        self.log.flush()

    def tick(self):
        st = self.m.get('state')
        if st not in ('M7_OBSTACLE', 'M2_FORK', 'M8_CONES'):
            return
        if st == 'M8_CONES' and self.m.get('sub') not in ('', 'approach', None):
            return                                      # 콘 구간부터는 안 봄
        now = time.monotonic()
        mode = (self.m.get('lane_cfg') or {}).get('mode', '-')
        v = verdict(self.m, self.ln, self.w_lane, self.w_cmd)
        key = (st, self.m.get('sub'), mode, self.ln.get('status'), self.ln.get('source'), self.ln.get('inner'), v)
        if key != self.last or now >= self.next_t:
            mark = '*' if key != self.last else ' '
            ms = f'{SHORT.get(st, st)}({self.m.get("sub") or "-"})'
            ks = f'{num(self.ln.get("kappa"))}/{num(self.ln.get("ff_kappa"))}'
            ws = f'{num(self.w_lane)}->{num(self.w_cmd)}'
            self.out(f'{mark}{now - self.t0:7.1f}s | {ms:<12} | {mode:<10} | {str(self.ln.get("status")):<4} | '
                     f'{str(self.ln.get("source")):<8} | {str(self.ln.get("inner") or "-"):<4} | {ks:<13} | {ws:<14} | {v}')
            self.last, self.next_t = key, now + 0.5


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
