#!/usr/bin/env python3
"""
stress_sim.py — 무작위 7대 배치 × 실차 오차로 회피 로직 강건성 시험 (ROS 없이, matplotlib 불필요)

  python3 tools/stress_sim.py                       # 기본: 40개 배치 × 여러 오차 조건
  python3 tools/stress_sim.py --n 100 --trigger 1.2
  python3 tools/stress_sim.py --no-align            # 정렬 보정 끄고 비교

배치는 '회전반경 0.53m 로 실제로 지나갈 수 있는 것'만 만든다 (앞줄 차가 있던 차선으로 들어가려면 줄 간격 1.35m 이상).
오차: 조향 지연, 출발 방향/옆 위치 틀어짐, odom 배율, IMU 드리프트, 정차 LIMO 위치·각도 흔들림.
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from limo_obstacle_avoid import sim_common as sc                                     # noqa: E402
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base   # noqa: E402


def feasible(rows, s_hard=1.35):
    prev, px, pblocked = {0: 0.0}, None, set()
    for x, blocked in rows:
        free = [l for l in (-1, 0, 1) if l not in blocked]
        nxt = {}
        for b in free:
            for a in prev:
                if abs(a - b) > 1:
                    continue
                if a != b and px is not None and b in pblocked and x - px < s_hard:
                    continue
                if a != b and px is None and x < 0.85:
                    continue
                nxt[b] = x
        if not nxt:
            return False
        prev, px, pblocked = nxt, x, set(blocked)
    return True


def random_layout(rnd, n_cars=7, x0=0.9, gap=(1.1, 1.7)):
    """→ ([(앞면 x, 차선)], 섬 시작 x)"""
    for _ in range(5000):
        rows, x, cars, last = [], x0, 0, x0
        while cars < n_cars:
            k = 2 if (n_cars - cars >= 2 and rnd.random() < 0.7) else 1
            rows.append((x, rnd.sample([-1, 0, 1], k)))
            cars += k
            last = x
            x += rnd.uniform(*gap)
        island_x = last + 0.32 + 0.8
        if last > 5.0 or not feasible(rows + [(island_x, [0])]):      # 가운데 섬도 '가운데 차선을 막는 줄'로 검사
            continue
        return [(xx, l) for xx, ls in rows for l in ls], island_x
    raise RuntimeError('배치 생성 실패')


def run(layout, p, island_x, delay=0, head_err_deg=0.0, dy0=0.0, odom_scale=1.0, imu_drift_deg=0.0,
        noise=0.01, seed=0, obs_dy=0.0, obs_yaw_deg=0.0):
    rnd = random.Random(seed)
    p.section_length = island_x
    rects = sc.layout_rects(layout, p.lane_width, jitter=0.05, seed=seed)
    r2 = random.Random(seed + 999)
    rects = [(cx, cy + r2.uniform(-obs_dy, obs_dy), L, W, math.radians(r2.uniform(-obs_yaw_deg, obs_yaw_deg)))
             for (cx, cy, L, W, _) in rects]
    segs = sc.rects_to_segments(rects)
    car = sc.SimCar(0.0, dy0, math.radians(head_err_deg))
    car.odom_yaw = car.imu_yaw = car.yaw
    pl = AvoidPlanner(p)
    pl.start(car.odom_x, car.odom_y, car.imu_yaw, 0.0)
    n_beam, a_min = 360, -math.pi
    a_inc = 2 * math.pi / n_beam
    dt, t = 0.05, 0.0
    q = [(0.0, 0.0)] * delay
    worst, collided, stuck, off_edge = 1e9, False, 0, 0.0
    edge = (p.num_side_lanes + 0.5) * p.lane_width
    for k in range(int(80 / dt)):
        drift = math.radians(imu_drift_deg) * min(1.0, t / 20.0)
        pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw)
        if k % 2 == 0:
            lx = car.x + p.lidar_x * math.cos(car.yaw)
            ly = car.y + p.lidar_x * math.sin(car.yaw)
            rng = sc.raycast(lx, ly, car.yaw, a_min, a_inc, n_beam, segs, noise=noise, rnd=rnd)
            pl.update_scan(scan_to_base(rng, a_min, a_inc, p))
        v, w = pl.step(t)
        q.append((v, w))
        v2, w2 = q.pop(0)
        car.step(v2, w2, dt, odom_scale=odom_scale, imu_drift=drift, rnd=rnd)
        t += dt
        g = sc.min_gap(car.footprint(), rects)
        worst = min(worst, g)
        collided |= g < 0
        for (_, cy) in sc.rect_corners(*car.footprint()):
            off_edge = max(off_edge, abs(cy) - edge)
        stuck = stuck + 1 if v == 0 else 0
        if pl.state == 'DONE' or stuck > 60:
            break
    done = pl.state == 'DONE'
    return dict(ok=done and not collided and off_edge <= 0, collided=collided, done=done,
                worst=worst, off_edge=off_edge, t=t)


CONDITIONS = [
    ('이상적', {}),
    ('조향 지연 0.15s', dict(delay=3)),
    ('출발 방향 +3°', dict(head_err_deg=3)),
    ('출발 방향 -5°', dict(head_err_deg=-5)),
    ('출발 옆 6cm', dict(dy0=0.06)),
    ('정차 LIMO ±6cm, ±10°', dict(obs_dy=0.06, obs_yaw_deg=10)),
    ('복합: 지연+3°+6cm+odom4%+드리프트2°', dict(delay=3, head_err_deg=3, dy0=0.06, odom_scale=1.04, imu_drift_deg=2)),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--seed', type=int, default=123)
    ap.add_argument('--trigger', type=float)
    ap.add_argument('--lookahead', type=float)
    ap.add_argument('--no-align', action='store_true')
    ap.add_argument('--only', type=int, help='조건 번호 하나만 (0부터)')
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    layouts = [random_layout(rnd) for _ in range(a.n)]
    total_ok = total = 0
    for ci, (name, cond) in enumerate(CONDITIONS):
        if a.only is not None and ci != a.only:
            continue
        res = []
        for i, (lay, island_x) in enumerate(layouts):
            p = AvoidParams()
            if a.trigger is not None:
                p.trigger_dist = a.trigger
            if a.lookahead is not None:
                p.lookahead = a.lookahead
            p.lane_align = not a.no_align
            res.append(run(lay, p, island_x, seed=i, **cond))
        ok = sum(r['ok'] for r in res)
        total_ok += ok
        total += len(res)
        print(f"{name:34s} 성공 {ok:3d}/{len(res)}  충돌 {sum(r['collided'] for r in res):2d}  "
              f"미완료 {sum(not r['done'] for r in res):2d}  가장자리이탈 {sum(r['off_edge'] > 0 for r in res):2d}  "
              f"최소간격 {min(r['worst'] for r in res) * 100:5.1f}cm")
    print(f'\n전체 {total_ok}/{total}')


if __name__ == '__main__':
    main()
