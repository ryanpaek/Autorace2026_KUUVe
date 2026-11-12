#!/usr/bin/env python3
"""M7 직전 코너(횡단보도 끝 -> R640 오른쪽 90°)에서 차선을 안 따라가고 직진하는 원인 후보 8개를 한 줄 O/X로 감시.

  python3 ~/autorace_ws/tools/curve_diag.py          # 0.5초마다 + 바뀌면 바로 (* 와 바뀐 번호)
  python3 ~/autorace_ws/tools/curve_diag.py 1.0      # 1초마다

O = 정상, X = 그 후보가 의심됨 (괄호 안이 그때 값). 화면과 ~/curve_diag.log 에 같이 남고,
같은 내용을 /diag/curve (std_msgs/String, JSON) 로 발행해 rosbag에도 남는다 (record_bag.sh에 포함).

 번호  후보                               보는 토픽                          X 조건
 H1   차선 노드가 선을 놓침(방향 유지 직진) /lane/status source            source가 lane/memory 아님 (hold=놓침, stop, route=도면만)
 H2   양쪽 선이 안 보임                    /lane/status status            BOTH 아님 (L/R 한쪽, NONE)
 H3   M7 3차선 설정이 코너에 맞지 않음      /mission/state lane_cfg        M7인데 코너를 다 안 돌았고 색 both(흰 선도 차선으로 봄)
 H4   도면 곡률을 못 씀                    /mission/state route, ff_kappa  길 잃음(lost) 또는 ff_kappa 없음
 H5   꺾는 양이 모자람·끝까지 꺾음          /lane/status kappa·path_kappa  코너(|필요 곡률|>0.8)인데 명령이 필요의 60% 미만, 또는 kappa_max 도달
 H6   차선 노드가 아닌 쪽이 운전            /mission/state source, /avoid/active  source가 lane 아님 또는 회피 켜짐
 H7   차선 명령이 바퀴까지 안 감            /lane/cmd_vel vs /cmd_vel       차선 운전인데 각속도가 0.15 넘게 다름
 H8   차선 노드가 느림                      /lane/status 주기               10 Hz 미만
"""
import json
import os
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

SHORT = {'WAIT_START': 'WAIT', 'M3_PEDESTRIAN': 'M3', 'M7_OBSTACLE': 'M7', 'M2_FORK': 'M2', 'M8_CONES': 'M8'}
KAPPA_MAX = 2.1          # params.yaml lane_node kappa_max
CORNER_K = 0.8           # 이보다 크게 휘면 코너로 봄 (R640 = 1.56)


def load(msg):
    try:
        d = json.loads(msg.data)
        return d if isinstance(d, dict) else {}
    except (ValueError, TypeError):
        return {}


def num(v, f='{:+.2f}'):
    return '-' if v is None else f.format(v)


def judge(m, ln, ob, avoid, w_cmd, w_lane, lane_hz):
    """-> [(번호, 'O'/'X', 값 문자열)] 8개. ROS 없이 시험할 수 있게 따로 둠."""
    out = []
    src = ln.get('source')
    out.append(('H1', 'O' if src in ('lane', 'memory') else 'X', str(src)))
    st = ln.get('status')
    out.append(('H2', 'O' if st == 'BOTH' else 'X', str(st)))
    cfg = m.get('lane_cfg') or {}
    color = cfg.get('lane_color', 'yellow')
    corner = ob.get('m7_corner')
    in_corner = m.get('state') == 'M7_OBSTACLE' and corner != 'done'
    out.append(('H3', 'X' if (in_corner and color == 'both') else 'O',
                f'{color}/{cfg.get("lane_width_mm", "-")}/{cfg.get("mode", "-")}'))
    route = m.get('route') or {}
    ff = ln.get('ff_kappa')
    out.append(('H4', 'X' if (route.get('lost') or ff is None) else 'O',
                ('lost ' if route.get('lost') else '') + num(ff)))
    k, pk = ln.get('kappa'), ln.get('path_kappa')
    need = pk if pk is not None else ff
    bad5 = False
    if k is not None:
        if abs(k) >= KAPPA_MAX - 0.05:
            bad5 = True
        elif need is not None and abs(need) > CORNER_K and (k * need <= 0 or abs(k) < 0.6 * abs(need)):
            bad5 = True
    out.append(('H5', 'X' if bad5 else 'O', f'{num(k)}/{num(need)}'))
    fsrc = m.get('source')
    out.append(('H6', 'X' if (fsrc != 'lane' or avoid) else 'O', f'{fsrc}{"+avoid" if avoid else ""}'))
    bad7 = fsrc == 'lane' and w_cmd is not None and w_lane is not None and abs(w_cmd - w_lane) > 0.15
    out.append(('H7', 'X' if bad7 else 'O', f'{num(w_lane)}->{num(w_cmd)}'))
    out.append(('H8', 'X' if (lane_hz is not None and lane_hz < 10.0) else 'O', num(lane_hz, '{:.0f}Hz')))
    return out


class Diag(Node):
    def __init__(self, period):
        super().__init__('curve_diag')
        self.m, self.ln, self.ob = {}, {}, {}
        self.avoid = False
        self.w_cmd = self.w_lane = None
        self.lane_t = deque(maxlen=40)
        self.last = None
        self.period, self.next_t = period, 0.0
        self.t0 = time.monotonic()
        self.log = open(os.path.expanduser('~/curve_diag.log'), 'a')
        self.pub = self.create_publisher(String, '/diag/curve', 10)
        self.create_subscription(String, '/mission/state', lambda x: setattr(self, 'm', load(x)), 10)
        self.create_subscription(String, '/lane/status', self._on_lane, 10)
        self.create_subscription(String, '/obstacle/state', lambda x: setattr(self, 'ob', load(x)), 10)
        self.create_subscription(Bool, '/avoid/active', lambda x: setattr(self, 'avoid', bool(x.data)), 10)
        self.create_subscription(Twist, '/cmd_vel', lambda x: setattr(self, 'w_cmd', float(x.angular.z)), 10)
        self.create_subscription(Twist, '/lane/cmd_vel', lambda x: setattr(self, 'w_lane', float(x.angular.z)), 10)
        self.out(f'===== 시작 {time.strftime("%H:%M:%S")} =====  (O 정상 / X 의심)')
        self.out('  시각   미션  | H1 선놓침   | H2 양쪽선 | H3 M7설정         | H4 도면곡률   | H5 꺾는양(명령/필요) '
                 '| H6 운전주체   | H7 명령전달     | H8 속도')
        self.create_timer(0.1, self.tick)

    def _on_lane(self, msg):
        self.ln = load(msg)
        self.lane_t.append(time.monotonic())

    def out(self, s):
        print(s, flush=True)
        self.log.write(s + '\n')
        self.log.flush()

    def hz(self):
        ts = [t for t in self.lane_t if time.monotonic() - t < 2.0]
        return (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 and ts[-1] > ts[0] else None

    def tick(self):
        now = time.monotonic()
        res = judge(self.m, self.ln, self.ob, self.avoid, self.w_cmd, self.w_lane, self.hz())
        key = tuple(r[1] for r in res) + (self.m.get('state'),)
        changed = [] if self.last is None else [res[i][0] for i in range(len(res)) if key[i] != self.last[i]]
        st = SHORT.get(self.m.get('state'), str(self.m.get('state'))[:4])
        corner = self.ob.get('m7_corner')
        self.pub.publish(String(data=json.dumps({
            't': round(now - self.t0, 2), 'state': self.m.get('state'), 'm7_corner': corner,
            'flags': {h: ok for h, ok, _ in res}, 'values': {h: v for h, _, v in res}}, ensure_ascii=False)))
        if changed or now >= self.next_t:
            w = [11, 9, 16, 12, 20, 13, 15, 6]
            cells = ' | '.join(f'{h} {ok} {v}'[:w[i] + 6].ljust(w[i] + 6) for i, (h, ok, v) in enumerate(res))
            self.out(f'{"*" if changed else " "}{now - self.t0:6.1f}s {st:<4}| {cells}'
                     + (f'  바뀜: {",".join(changed)}' if changed else ''))
            self.next_t = now + self.period
        self.last = key


def main():
    period = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    rclpy.init()
    n = Diag(period)
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
