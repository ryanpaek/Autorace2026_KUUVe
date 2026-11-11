"""M7 끄기(YOLO 갈림길 표지) 시뮬레이션 (ROS 없이): 끄는 위치가 섬 앞 끝 기준 어디인지, 얼마나 흔들리는지.

진짜 코드: M7Link(m7_link.py)의 표지 끄기 (off_mode: sign, 확인 3/5프레임, 지나감 = 연속 안 보임, 섬 앞 1.5 m 문턱,
          옆 차로일 때만). 모형: 핀홀 카메라(가로 화각 HFOV, 640x480), 표지 = 섬 앞 끝 가운데 (높이 SIGN_H),
          YOLO(주기 HZ, 상자 높이 MIN_PX 이상이면 90% 검출), 차는 옆 차로(±0.52 m)를 일정 속도로 직진.
          카메라 = 앞바퀴 위치로 가정. 표지 실제 위치·크기·카메라 화각은 실측 전이라 가정값 (아래 인자로 바꿈).

  python3 src/autorace_obstacle/test/m7_off_sim.py
"""
import argparse
import math
import os
import random
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from autorace_obstacle.m7_link import M7Link, M7LinkParams   # noqa: E402

START_S, ISLAND_S = 13589.0, 17973.0
TIP = (ISLAND_S - START_S) / 1000.0          # 시작선 -> 섬 앞 끝 (m)
IMG_W, IMG_H = 640, 480
SIGN_H = 0.15                                # 표지 높이 (m, 가정)
MIN_PX = 12.0                                # YOLO가 잡는 최소 상자 높이 (px, 가정)


def run(method, seed, hfov=70.0, hz=5.0, v=0.25, lane=-1, map_err_mm=0.0, near_px=0.0, lost=5,
        edge_px=0.0, after_mm=0.0):
    rnd = random.Random(seed)
    f = (IMG_W / 2) / math.tan(math.radians(hfov) / 2)          # 정사각 화소 가정 -> 세로도 같은 f
    p = M7LinkParams(off_mode='sign', off_sign_lost_frames=lost,
                     off_sign_near_h_px=near_px if method == 'near' else 0.0,
                     off_sign_edge_px=edge_px, off_sign_after_mm=after_mm)
    link = M7Link(p)
    y_car = 0.52 * lane + rnd.gauss(0.0, 0.03)
    x = 0.5                                                      # 앞바퀴, 시작선 기준 (회피 중)
    dt, t, next_det = 0.05, 0.0, 0.0
    # 회피 노드가 이미 켜져 옆 차로에 있다고 둠
    link.update(0.0, {'state': 'M7_OBSTACLE', 'dist_mm': 0.0, 'route': {'s_mm': START_S - 300.0}}, {})
    link.phase, link.claimed = 'running', True
    link.on_avoid_state({'state': 'RUN', 'target': lane, 'now_lane': lane})
    off_at = None
    while x < TIP + 1.0:
        if t >= next_det:
            next_det = t + (1.0 / hz) * rnd.uniform(0.85, 1.15)
            depth = TIP - x
            dets = []
            if depth > 0.10:
                ang = math.atan2(-y_car, depth)                  # 표지는 섬 가운데(y=0)
                u = IMG_W / 2 - f * math.tan(ang)
                h = f * SIGN_H / depth
                x1, x2 = max(0.0, u - h / 2), min(float(IMG_W), u + h / 2)   # 검출기처럼 화면 끝에서 자름
                if x2 - x1 >= 0.5 * h and h >= MIN_PX and rnd.random() < 0.9:  # 절반 이상 보여야 검출
                    cls = 'fork_b' if lane < 0 else 'fork_a'
                    dets = [{'cls': cls, 'conf': 0.8, 'bbox': [x1, IMG_H / 2 - h / 2, x2 - x1, h]}]
            link.on_detections(dets)
        s_mm = START_S + x * 1000.0 + map_err_mm
        acts = link.update(t, {'state': 'M7_OBSTACLE', 'dist_mm': x * 1000.0, 'route': {'s_mm': s_mm}}, {})
        if ('disable',) in acts and off_at is None:
            off_at = x - TIP
            break
        x += v * dt
        t += dt
    return off_at


def summarize(vals):
    got = [v for v in vals if v is not None]
    if not got:
        return '  끄지 못함 (회피 노드 종료/FSM 강제로 끝남)'
    q = sorted(got)
    return (f'끈 판 {len(got):2d}/{len(vals)}  섬 앞 끝 기준 중앙 {statistics.median(q) * 100:+5.0f} cm  '
            f'범위 {q[0] * 100:+5.0f} ~ {q[-1] * 100:+5.0f} cm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=40)
    a = ap.parse_args()
    cases = [
        ('기준: 화각70° YOLO5Hz 0.25m/s', dict()),
        ('화각 60°', dict(hfov=60.0)),
        ('화각 80°', dict(hfov=80.0)),
        ('YOLO 2Hz (과부하)', dict(hz=2.0)),
        ('속도 0.18 m/s', dict(v=0.18)),
        ('1차선(A)', dict(lane=1)),
        ('도면 위치 -0.3 m', dict(map_err_mm=-300.0)),
    ]
    # '가까움' 상자 높이: 기준 화각에서 섬 앞 끝 0.45 m 전 높이 (실차에서 원하는 지점의 높이를 재서 넣는 방식)
    f70 = (IMG_W / 2) / math.tan(math.radians(70.0) / 2)
    near_px = f70 * SIGN_H / 0.45
    # 화각 70°에서 표지가 화면 옆으로 나가는 곳 = 섬 앞 끝 0.52/tan35° = 0.74 m 전 -> 0.54 m 더 가면 섬 앞 끝 0.2 m 전
    methods = [('예전: 5프레임 안 보임', dict(method='lost', lost=5)),
               ('예전 + 상자 높이로 가까움', dict(method='near', near_px=near_px, lost=5)),
               ('새: 화면 옆으로 나가면 바로 (after 0)', dict(method='lost', lost=5, edge_px=10.0)),
               ('새: 나간 뒤 0.54 m 더 (after 540)', dict(method='lost', lost=5, edge_px=10.0, after_mm=540.0))]
    for mname, mk in methods:
        print(f'\n=== {mname} ===')
        for cname, ck in cases:
            vals = [run(seed=i, **mk, **ck) for i in range(a.n)]
            print(f'{cname:32s} {summarize(vals)}')


if __name__ == '__main__':
    main()
