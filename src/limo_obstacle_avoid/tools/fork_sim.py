#!/usr/bin/env python3
"""
fork_sim.py — 갈림길(미션 2) 출구 차선 이동 시험 (ROS 없이)

무작위 7대 배치(지나갈 수 있는 것만) × 마지막 차~섬 여유(gap) × 출구 방향(A/B) × 표지 인식 시점으로
'요청한 쪽 길(A=+1, B=-1)로 섬을 지났는지', 충돌, 멈춤을 센다.

  python3 tools/fork_sim.py                 # 새 방식(fork_shift) vs 예전 방식 비교
  python3 tools/fork_sim.py --n 30 --only-new
표지 인식 시점: start(처음부터), 섬 앞 끝 2.5/2.0/1.5/1.0 m 전에 /fork_direction 이 도착했다고 가정
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

from limo_obstacle_avoid import sim_common as sc                                     # noqa: E402
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base   # noqa: E402
import stress_sim                                                                    # noqa: E402

TIMINGS = [('처음부터', None), ('섬 2.5m 전', 2.5), ('섬 2.0m 전', 2.0), ('섬 1.5m 전', 1.5), ('섬 1.0m 전', 1.0)]


def run(layout, p, island_x, exit_dir, sign_before, seed=0, delay=2, head_err_deg=0.0):
    rnd = random.Random(seed)
    p.section_length = island_x
    p.exit_lane = 'auto'
    rects = sc.layout_rects(layout, p.lane_width, jitter=0.05, seed=seed)
    segs = sc.rects_to_segments(rects)
    car = sc.SimCar(0.0, 0.0, math.radians(head_err_deg))
    car.odom_yaw = car.imu_yaw = car.yaw
    pl = AvoidPlanner(p)
    pl.start(car.odom_x, car.odom_y, car.imu_yaw, 0.0)
    if sign_before is None:
        pl.set_exit(exit_dir)
    sent = sign_before is None
    lane_at_sign = 0 if sent else None
    q = [(0.0, 0.0)] * delay
    dt, t = 0.05, 0.0
    worst, collided, stuck = 1e9, False, 0
    y_at_island = None
    for k in range(int(80 / dt)):
        pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw)
        if not sent and island_x - (pl.pose.x + p.robot_front) <= sign_before:
            pl.set_exit(exit_dir)
            sent = True
            lane_at_sign = pl.target
        if k % 2 == 0:
            rng = sc.raycast(car.x + p.lidar_x * math.cos(car.yaw), car.y + p.lidar_x * math.sin(car.yaw), car.yaw,
                             -math.pi, 2 * math.pi / 360, 360, segs, noise=0.01, rnd=rnd)
            pl.update_scan(scan_to_base(rng, -math.pi, 2 * math.pi / 360, p))
        v, w = pl.step(t)
        q.append((v, w))
        v2, w2 = q.pop(0)
        car.step(v2, w2, dt, rnd=rnd)
        t += dt
        g = sc.min_gap(car.footprint(), rects)
        worst = min(worst, g)
        collided |= g < 0
        if y_at_island is None and car.x + p.robot_front >= island_x:
            y_at_island = car.y                       # 앞바퀴(앞범퍼)가 섬 앞 끝에 닿을 때 차의 옆 위치
        stuck = stuck + 1 if v == 0 else 0
        if pl.state == 'DONE' or stuck > 60:
            break
    want = 1 if exit_dir == 'A' else -1
    # 섬 앞 끝에서 차체 중심이 출구 쪽 섬 경계 바깥(섬 반폭 0.3·w 넘어서)에 있으면 그쪽 길로 들어간 것
    right_side = y_at_island is not None and want * y_at_island > 0.3 * p.lane_width
    return dict(ok=right_side and not collided and pl.state == 'DONE', right_side=right_side, collided=collided,
                done=pl.state == 'DONE', worst=worst, lane_at_sign=lane_at_sign, events=list(pl.events))


def layouts_with_gap(n, seed, gap):
    """마지막 줄 차 뒤끝 ~ 섬 앞 끝 = gap [m]"""
    rnd = random.Random(seed)
    out = []
    for _ in range(n):
        lay, _ = stress_sim.random_layout(rnd)
        last = max(x for x, _ in lay)
        out.append((lay, last + 0.32 + gap))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=15)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--only-new', action='store_true')
    ap.add_argument('--timing', type=int, help='표지 시점 번호 하나만 (0~4)')
    ap.add_argument('--gap', type=float, default=2.0, help='마지막 차 ~ 섬 앞 끝 여유 [m]')
    a = ap.parse_args()
    layouts = layouts_with_gap(a.n, a.seed, a.gap)
    print(f'[마지막 차 ~ 섬 여유 {a.gap} m, 배치 {a.n}개 × A/B]')
    modes = [('새 방식', True)] if a.only_new else [('예전', False), ('새 방식', True)]
    for ti, (tname, before) in enumerate(TIMINGS):
        if a.timing is not None and ti != a.timing:
            continue
        for mname, fs in modes:
            tot = ok = col = stuck = 0
            need2 = need2_ok = 0
            for i, (lay, ix) in enumerate(layouts):
                for ex in ('A', 'B'):
                    p = AvoidParams()
                    p.fork_shift = fs
                    r = run(lay, p, ix, ex, before, seed=i)
                    tot += 1
                    ok += r['ok']
                    col += r['collided']
                    stuck += not r['done']
                    if r['lane_at_sign'] is not None and abs((1 if ex == 'A' else -1) - r['lane_at_sign']) == 2:
                        need2 += 1
                        need2_ok += r['ok']
            print(f'{tname:10s} {mname:5s} 요청한 길로 통과 {ok:3d}/{tot}  충돌 {col}  멈춤 {stuck}  '
                  f'(인식 때 2차선 이동 필요 {need2}건 중 성공 {need2_ok})')


if __name__ == '__main__':
    main()
