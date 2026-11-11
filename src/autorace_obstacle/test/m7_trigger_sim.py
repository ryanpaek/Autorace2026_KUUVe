"""M7 켜기 시뮬레이션 (ROS 없이): 시작선 앞에서 차선 주행 -> 라이다로 앞 장애물 -> 회피 노드 켬 -> 통과.

진짜 코드: M7Link(m7_link.py), obstacle_node의 앞 장애물 거리 계산(scan_utils: 점 -> 묶기 -> 차 중심 ±200 mm),
          AvoidPlanner(limo_obstacle_avoid). 모형: 차선 주행(직진 0.18 m/s), 라이다(1° 360점 10 Hz),
          켜기 지연(파라미터 서비스 0.05 s + 회피 노드 active -> FSM 넘김 0.10 s).

잰 것 (켜기 트리거 거리마다):
  켤 때 앞 거리   : M7Link가 켠 순간 라이다 -> 장애물 (mm)
  넘김 앞 거리    : FSM이 회피 명령을 쓰기 시작한 순간
  5cm 옆 앞 거리 : 차가 실제로 옆으로 5 cm 움직인 순간 (눈으로 '꺾기 시작'으로 보이는 때)
  성공 / 정지 / 후진 / 차로 밖

  python3 src/autorace_obstacle/test/m7_trigger_sim.py              # 1000 / 850 / 700 mm 비교
  python3 src/autorace_obstacle/test/m7_trigger_sim.py --n 20 --trigger 700
"""
import argparse
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..')
for d in ('autorace_obstacle', 'limo_obstacle_avoid', os.path.join('limo_obstacle_avoid', 'tools')):
    sys.path.insert(0, os.path.join(SRC, d))

from autorace_obstacle.m7_link import M7Link, M7LinkParams                      # noqa: E402
from autorace_obstacle.scan_utils import cluster_points, roi, scan_to_points     # noqa: E402
from limo_obstacle_avoid import sim_common as sc                                 # noqa: E402
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base  # noqa: E402
import stress_sim as ss                                                          # noqa: E402

START_S, ISLAND_S = 13589.0, 17973.0        # track_route.yaml (mm)
WHEELBASE = 0.20                            # 기준점(뒤차축) -> 앞바퀴. 도면 위치 s는 앞바퀴
LIDAR_X = 0.10
V_LANE = 0.18                               # FSM approach_speed
N_BEAM = 360


def obstacle_node_front(rng, a_min, a_inc, hw_mm):
    """obstacle_node._on_scan 과 같은 계산 -> 라이다에서 앞 차로 안 가장 가까운 장애물 (mm) 또는 None."""
    pts = scan_to_points(rng, a_min, a_inc, 0.05, 8.0, LIDAR_X * 1000.0, 0.0, 0.0)
    front = roi(pts, 0.0, 2000.0, -900.0, 900.0)
    cl = cluster_points(front, 60.0, 3)
    ahead = [c['near_x'] - LIDAR_X * 1000.0 for c in cl
             if abs(c['y']) <= hw_mm and c['near_x'] > LIDAR_X * 1000.0]
    return min(ahead) if ahead else None


def make_layout(rnd, enable_front=-0.25):
    """첫 줄에 가운데 차로 장애물(앞면 = 시작선 + 0.0~1.5 m)이 있는, 지나갈 수 있는 배치 (stress_sim 검사 그대로).
    섬 = 시작선 + 4.384 m (도면). 회피 노드는 섬을 0.2 m 이르게 본다(앞바퀴 기준 거리를 기준점에서 씀) -> 그 위치로 검사."""
    island = (ISLAND_S - START_S) / 1000.0 - WHEELBASE
    for _ in range(20000):
        x_first = rnd.uniform(0.0, 1.5)
        rows, _ = ss.random_layout(rnd, x0=0.9)
        cars = [(x - rows[0][0] + x_first, lane) for x, lane in rows]
        if not any(lane == 0 and abs(x - x_first) < 1e-9 for x, lane in cars):
            continue
        cars = [(x, lane) for x, lane in cars if x + 0.32 + 0.8 <= island]
        grouped = {}
        for x, lane in cars:
            grouped.setdefault(round(x, 6), []).append(lane)
        rows2 = sorted(grouped.items())
        # stress_sim feasible 은 '켜는 순간 기준점'에서 잰 거리 (첫 줄 0.85 m 이상 등).
        #   켜는 곳 = 앞바퀴가 시작선 enable_front -> 기준점 = enable_front - 0.2
        base = enable_front - WHEELBASE
        rel = [(x - base, ls) for x, ls in rows2]
        if ss.feasible(rel + [(island - base, [0])]):
            return cars
    raise RuntimeError('배치 생성 실패')


CORNER_END = -0.288                         # 시작선 기준 코너 끝 (m, 앞바퀴). 그 전엔 카메라 차선 방향이 크게 꺾여 보임


def run(cars, trig_mm, seed, map_err_mm=0.0, delay=0, odom_scale=1.0, imu_drift_deg=0.0,
        noise=0.01, fsm_lag=0.10, svc_lag=0.05, gate_early_mm=None, rechoose_dy=None):
    rnd = random.Random(seed)
    lw = 0.52
    rects = sc.layout_rects(cars, lw, jitter=0.03, seed=seed)
    segs = sc.rects_to_segments(rects)
    car = sc.SimCar(-1.3, 0.0, 0.0)                 # 앞바퀴 = 시작선 -1.1 m (M7 진입 직후)
    car.odom_yaw = car.imu_yaw = car.yaw
    lp = M7LinkParams(enable_mode='obstacle', trigger_front_mm=trig_mm, off_mode='none')
    if gate_early_mm is not None:
        lp.trigger_gate_early_mm = gate_early_mm
    link = M7Link(lp)
    pl, p = None, AvoidParams()
    if rechoose_dy is not None:
        p.rechoose_dy = rechoose_dy
    a_min, a_inc = -math.pi, 2 * math.pi / N_BEAM
    dt, t = 0.05, 0.0
    q = [(0.0, 0.0)] * delay
    ev = {'enable': None, 'handover': None, 'side5': None}
    start_at = handover_at = None
    front_now = None
    last_v, collided, off_edge, stop_n, rev = 0.0, False, 0.0, 0, 0.0
    edge = 1.5 * lw
    first_x = min(x for x, lane in cars if lane == 0)

    def lidar_to_first():
        return (first_x - (car.x + LIDAR_X)) * 1000.0

    for k in range(int(70 / dt)):
        if k % 2 == 0:
            lx, ly = car.x + LIDAR_X * math.cos(car.yaw), car.y + LIDAR_X * math.sin(car.yaw)
            rng = sc.raycast(lx, ly, car.yaw, a_min, a_inc, N_BEAM, segs, noise=noise, rnd=rnd)
            front_now = obstacle_node_front(rng, a_min, a_inc, link.p.trigger_half_width_mm)
            link.on_front(front_now)
            if pl is not None:
                pl.update_scan(scan_to_base(rng, a_min, a_inc, p))
        s_mm = START_S + (car.x + WHEELBASE) * 1000.0 + map_err_mm
        mission = {'state': 'M7_OBSTACLE', 'dist_mm': (car.x + 1.3) * 1000.0, 'fork_choice': None,
                   'route': {'s_mm': s_mm}}
        front_true = car.x + WHEELBASE
        lane = {'status': 'BOTH', 'heading_rad': math.radians(25.0 if front_true < CORNER_END else rnd.gauss(0.0, 1.0))}
        for act in link.update(t, mission, lane):
            if act[0] == 'enable' and start_at is None:
                start_at = t + svc_lag
                ev['enable'] = lidar_to_first()
                p.section_length = act[1]
        if start_at is not None and pl is None and t >= start_at:
            pl = AvoidPlanner(p)
            pl.start(car.odom_x, car.odom_y, car.imu_yaw, t)
            handover_at = t + fsm_lag
            link.on_avoid_active(True, t)
        if pl is not None:
            drift = math.radians(imu_drift_deg) * min(1.0, t / 20.0)
            pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw, forward=last_v >= 0.0)
            v_a, w_a = pl.step(t)
            link.on_avoid_active(True, t)
        if handover_at is not None and t >= handover_at:
            if ev['handover'] is None:
                ev['handover'] = lidar_to_first()
            v, w = v_a, w_a
        else:
            v, w = V_LANE, -1.5 * car.y - 1.0 * math.sin(car.yaw)    # 차선 주행 (가운데 유지)
            drift = 0.0
        q.append((v, w))
        v2, w2 = q.pop(0)
        car.step(v2, w2, dt, odom_scale=odom_scale, imu_drift=drift if pl else 0.0, rnd=rnd)
        if v2 < 0:
            rev += -v2 * dt
        if pl is not None and pl.state == 'RUN' and (pl.blocked or pl.emergency):
            stop_n += 1
        last_v = v2
        t += dt
        if ev['side5'] is None and abs(car.y) >= 0.05:
            ev['side5'] = lidar_to_first()
        g = sc.min_gap(car.footprint(), rects)
        collided |= g < 0
        for (_, cy) in sc.rect_corners(*car.footprint()):
            off_edge = max(off_edge, abs(cy) - edge)
        if pl is not None and pl.state == 'DONE':
            break
    done = pl is not None and pl.state == 'DONE'
    return dict(ok=done and not collided and off_edge <= 0, done=done, collided=collided,
                off=off_edge > 0, stop=stop_n > 0, rev=rev, **ev)


CONDS = [
    ('이상적', {}),
    ('조향 지연 0.15s', dict(delay=3)),
    ('도면 위치 -0.3 m 늦음', dict(map_err_mm=-300.0)),
    ('도면 위치 +0.3 m 빠름', dict(map_err_mm=300.0)),
    ('복합: 지연+odom4%+드리프트2°+잡음2cm', dict(delay=3, odom_scale=1.04, imu_drift_deg=2.0, noise=0.02)),
]


def fmt(v):
    return '  -  ' if v is None else f'{v:5.0f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--trigger', type=float, nargs='*', default=[1000.0, 850.0, 700.0])
    ap.add_argument('--gate-early', type=float, default=None, help='trigger_gate_early_mm (기본: 코드 기본값)')
    ap.add_argument('--rechoose', type=float, default=None, help='rechoose_dy (0 = 다시 고르기 끔 = 11/10 main)')
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    lays = [make_layout(rnd) for _ in range(a.n)]
    for trig in a.trigger:
        print(f'\n=== 켜기 거리 {trig:.0f} mm ===')
        print(f'{"조건":34s} 성공   충돌 차로밖 정지판 후진합   | 켤때 앞  넘김 앞  5cm옆 앞 (mm, 중앙값)')
        for name, cond in CONDS:
            res = [run(l, trig, i, gate_early_mm=a.gate_early, rechoose_dy=a.rechoose, **cond)
                   for i, l in enumerate(lays)]
            med = {k: (float(np.median([r[k] for r in res if r[k] is not None]))
                       if any(r[k] is not None for r in res) else None)
                   for k in ('enable', 'handover', 'side5')}
            print(f'{name:34s} {sum(r["ok"] for r in res):2d}/{len(res)} {sum(r["collided"] for r in res):4d} '
                  f'{sum(r["off"] for r in res):5d} {sum(r["stop"] for r in res):5d} '
                  f'{sum(r["rev"] for r in res):5.2f}m  | {fmt(med["enable"])}    {fmt(med["handover"])}   '
                  f'{fmt(med["side5"])}', flush=True)


if __name__ == '__main__':
    main()
