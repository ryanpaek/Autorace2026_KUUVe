#!/usr/bin/env python3
"""차로 바꾸기 시작 거리(trigger_dist)·조향(lookahead, max_heading_deg) 고르기 (ROS 없음).

stress_sim 무작위 7대 배치 + avoid_params.yaml(실차 값)으로 돌리며 잰다:
  성공 / 후진한 판 / 정지 시간 / 평균 완주
  도착 여유: 차로를 바꾼 이유였던 장애물(옛 차로 앞)의 앞면까지, 새 차로에 '똑바로 도착'(중심 8 cm·8° 안)했을 때 앞범퍼 거리 (m)
             작으면(0 근처) 장애물 바로 옆에서 막 도착 = 늦지도 이르지도 않음. 음수 = 장애물 옆을 아직 비스듬히 지남.
             크면 너무 일찍 바꿈 (새 차로 뒤 장애물에 다시 대비할 거리는 남지만, 옛 차로를 빨리 떠나 다음 판단이 어려움)
  옆 도착률 : 장애물 앞면에 앞범퍼가 닿을 때 이미 똑바로 도착해 있던 비율

  python3 tools/change_sweep.py                      # 기본 격자
  python3 tools/change_sweep.py --trig 1.2 1.5 --ld 0.35 0.45 --hd 35 40 --n 40
"""
import argparse
import math
import os
import random
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
from dataclasses import fields  # noqa: E402

import stress_sim as ss  # noqa: E402
from limo_obstacle_avoid import sim_common as sc  # noqa: E402
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base  # noqa: E402

EXTRA = {}
YAML = os.path.join(HERE, '..', 'config', 'avoid_params.yaml')


def team_params(**kw):
    p = AvoidParams()
    with open(YAML) as fp:
        y = yaml.safe_load(fp)['/**']['ros__parameters']
    for f in fields(p):
        if f.name in y:
            v = y[f.name]
            setattr(p, f.name, float(v) if isinstance(getattr(p, f.name), float) and isinstance(v, int) else v)
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def run(layout, island_x, seed, p, delay=0, noise=0.01, head_err_deg=0.0, dy0=0.0):
    rnd = random.Random(seed)
    p.section_length = island_x
    rects = sc.layout_rects(layout, p.lane_width, jitter=0.05, seed=seed)
    segs = sc.rects_to_segments(rects)
    car = sc.SimCar(0.0, dy0, math.radians(head_err_deg))
    car.odom_yaw = car.imu_yaw = car.yaw
    pl = AvoidPlanner(p)
    pl.start(car.odom_x, car.odom_y, car.imu_yaw, 0.0)
    a_min, n_beam = -math.pi, 360
    a_inc = 2 * math.pi / n_beam
    dt, t, last_v = 0.05, 0.0, 0.0
    q = [(0.0, 0.0)] * delay
    collided, off_edge, rev, stop_t, worst = False, 0.0, 0.0, 0.0, 9.0
    over = 0.0                    # 바깥 차로(1·3)에서 차체 바깥 모서리가 차로 중심 + 반폭 - 차 반폭을 넘은 최대 (= 바깥 선 쪽으로 넘침)
    edge = (p.num_side_lanes + 0.5) * p.lane_width
    changes = []                  # 차로 바꿈: {'face': 이유 장애물 앞면 x, 'to': 새 차로, 'arrive': 똑바로 도착한 앞범퍼 x}
    tgt = pl.target
    for k in range(int(80 / dt)):
        pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw, forward=last_v >= 0.0)
        if k % 2 == 0:
            lx, ly = car.x + p.lidar_x * math.cos(car.yaw), car.y + p.lidar_x * math.sin(car.yaw)
            rng = sc.raycast(lx, ly, car.yaw, a_min, a_inc, n_beam, segs, noise=noise, rnd=rnd)
            pl.update_scan(scan_to_base(rng, a_min, a_inc, p))
        old = pl.target
        ne = len(pl.events)
        v, w = pl.step(t)
        why = ' '.join(e[1] for e in pl.events[ne:])
        if pl.target != old and not why.startswith('차선 '):
            if changes and changes[-1]['arrive'] is None:
                changes[-1]['arrive'] = float('nan')      # 장애물 때문이 아닌 차로 바꿈(가운데 모으기·출구 등)이 끼어듦 -> 빼고 셈
        elif pl.target != old:
            # 진짜 위치로: 옛 차로 앞 실제 장애물 앞면
            faces = [x for x, ln in layout if ln == old and x + 0.32 > car.x - 0.16]
            face = min(faces) if faces else None
            if face is not None and face - (car.x + 0.2) < 3.0:
                changes.append({'face': face, 'to': pl.target, 'arrive': None, 'at': face - (car.x + 0.2)})
            elif changes and changes[-1]['arrive'] is None:
                changes[-1]['to'] = pl.target
        if changes and changes[-1]['arrive'] is None:
            c = changes[-1]
            if (abs(car.y - c['to'] * p.lane_width) < 0.08 and abs(car.yaw) < math.radians(8.0)
                    and pl.target == c['to']):
                c['arrive'] = car.x + 0.2
        q.append((v, w))
        v2, w2 = q.pop(0)
        car.step(v2, w2, dt, rnd=rnd)
        if v2 < 0:
            rev += -v2 * dt
        if pl.state == 'RUN' and abs(v2) < 1e-3:
            stop_t += dt
        last_v = v2
        t += dt
        g = sc.min_gap(car.footprint(), rects)
        worst = min(worst, g)
        collided |= g < 0
        for (_, cy) in sc.rect_corners(*car.footprint()):
            off_edge = max(off_edge, abs(cy) - edge)
            over = max(over, abs(cy) - (edge - 0.02))   # 노란 실선 안쪽 가장자리까지
        if pl.state == 'DONE':
            break
    done = pl.state == 'DONE'
    changes = [c for c in changes if not (c['arrive'] is not None and math.isnan(c['arrive']))]
    margins = [c['face'] - c['arrive'] for c in changes if c['arrive'] is not None]
    late = sum(1 for c in changes if c['arrive'] is None or c['arrive'] > c['face'])
    return dict(ok=done and not collided and off_edge <= 0, rev=rev, over=over, stop_t=stop_t, t=t, worst=worst,
                margins=margins, n_change=len(changes), late=late, starts=[c['at'] for c in changes], off=off_edge > 0, collided=collided)


CONDS = [('이상적', {}), ('지연0.15s+잡음2cm+방향3°', dict(delay=3, noise=0.02, head_err_deg=3.0))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--seed', type=int, default=123)
    ap.add_argument('--trig', type=float, nargs='*', default=[1.0, 1.2, 1.4, 1.6, 1.8])
    ap.add_argument('--ld', type=float, nargs='*', default=[0.35, 0.45, 0.55])
    ap.add_argument('--hd', type=float, nargs='*', default=[30.0, 40.0])
    ap.add_argument('--set', nargs='*', default=[], help='이름=값 (AvoidParams)')
    a = ap.parse_args()
    for kv in a.set:
        k, v = kv.split('=')
        EXTRA[k] = {'true': True, 'false': False}.get(v.lower(), None) if v.lower() in ('true', 'false') else float(v)
    rnd = random.Random(a.seed)
    lays = [ss.random_layout(rnd) for _ in range(a.n)]
    print(f'{"trig":>5s} {"ld":>5s} {"hd":>4s} | {"성공":>5s} {"후진판":>4s} {"후진합":>6s} {"정지":>5s} {"완주":>5s} '
          f'{"최소간격":>6s} | 도착여유 중앙/10%/90% (m)  옆도착률')
    for trig in a.trig:
        for ld in a.ld:
            for hd in a.hd:
                for name, ck in CONDS:
                    res = [run(l, isl, i, team_params(trigger_dist=trig, lookahead=ld, max_heading_deg=hd,
                                                      **EXTRA), **ck)
                           for i, (l, isl) in enumerate(lays)]
                    m = [x for r in res for x in r['margins']]
                    st = [x for r in res for x in r['starts']]
                    nch = sum(r['n_change'] for r in res)
                    late = sum(r['late'] for r in res)
                    q = (np.percentile(m, [50, 10, 90]) if m else [float('nan')] * 3)
                    print(f'{trig:5.2f} {ld:5.2f} {hd:4.0f} | {sum(r["ok"] for r in res):2d}/{len(res)} '
                          f'{sum(r["rev"] > 0.01 for r in res):4d} {sum(r["rev"] for r in res):5.2f}m '
                          f'{np.mean([r["stop_t"] for r in res]):4.1f}s {np.mean([r["t"] for r in res]):5.1f}s '
                          f'{min(r["worst"] for r in res) * 100:5.1f}cm | {q[0]:+.2f} / {q[1]:+.2f} / {q[2]:+.2f}   '
                          f'{(nch - late) / max(1, nch):.0%}  선넘음 {sum(r["over"] > 0 for r in res)}판 최대 {max(r["over"] for r in res) * 100:+.1f}cm  시작 {np.median(st) if st else float("nan"):.2f}m  {name}', flush=True)


if __name__ == '__main__':
    main()
