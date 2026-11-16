#!/usr/bin/env python3
"""차선 노드(lane_core) 그대로 넣은 코너 폐루프 시뮬레이션 (11/15).
직선 1.3 m -> R640 오른쪽 90° -> 직선. 양쪽 노란 실선(중심 간 515). 카메라가 실제로 보는 영역(bev_limo.yaml 호모그래피)만 보임.
카메라 hz, 조향 지연(프레임), 선 잡음, 도면 곡률 미리보기(route_k, 미션 노드처럼 차선 노드에 넣음).

  python3 tools/lane_corner_sim.py                 # 기본 비교
  python3 tools/lane_corner_sim.py trace           # 코너 구간 프레임별 기록 (곡률·옆 오차)
"""
import math
import os
import random
import sys

import cv2
import numpy as np

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'autorace_lane')
sys.path.insert(0, SRC)
from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams  # noqa: E402

CFG = BevConfig.from_yaml(os.path.join(SRC, '..', 'autorace_bringup', 'config', 'bev_limo.yaml'))
VALID = CFG.warp(np.full((480, 640), 255, np.uint8)) > 0       # 카메라에 실제로 보이는 BEV 칸
W = 515.0
R = 640.0
L0 = 1300.0                                                       # 코너 앞 직선


def center_pose(s):
    """중심선 위 s(mm) -> (x, y, heading). 동쪽으로 가다 오른쪽 90° (남쪽)"""
    if s <= L0:
        return s - L0, 0.0, 0.0
    a = (s - L0) / R
    if a <= math.pi / 2:
        return R * math.sin(a), -R * (1 - math.cos(a)), -a
    d = s - L0 - R * math.pi / 2
    return R, -R - d, -math.pi / 2


def line_points(off):
    pts = []
    for s in np.arange(0.0, L0 + R * math.pi / 2 + 2500.0, 4.0):
        x, y, h = center_pose(s)
        pts.append((x - off * math.sin(h), y + off * math.cos(h)))
    return np.array(pts)


LINES = [line_points(+W / 2), line_points(-W / 2)]


def track_err(x, y):
    """중심선에서 옆 오차 (mm, + = 코너 바깥쪽 = 왼쪽), 진행 s"""
    if x <= 0.0:
        return y, x + L0
    if y >= -R:
        ang = math.atan2(x, R + y)                     # 0..90°
        if ang <= math.pi / 2 + 1e-9:
            r = math.hypot(x, y + R)
            return r - R, L0 + R * ang
    return x - R, L0 + R * math.pi / 2 + (-R - y)


def render(x0, y0, h0, rnd, noise_mm):
    img = np.zeros((CFG.bev_h, CFG.bev_w), np.uint8)
    c, s = math.cos(h0), math.sin(h0)
    for pts in LINES:
        dx, dy = pts[:, 0] - x0, pts[:, 1] - y0
        f = dx * c + dy * s                              # 앞
        lx = -dx * s + dy * c                            # 왼쪽
        ok = (f > 0) & (f < 2500)
        f, lx = f[ok], lx[ok]
        if noise_mm > 0:
            lx = lx + rnd.gauss(0, noise_mm)
        rows = np.round(CFG.y_mm_to_row(f)).astype(int)
        cols = np.round(CFG.vehicle_col + (-lx) / CFG.mm_per_px_x).astype(int)
        for r_, c_ in zip(rows, cols):
            if 0 <= r_ < CFG.bev_h and 2 <= c_ < CFG.bev_w - 2:
                img[r_, c_ - 2:c_ + 3] = 255
    img[~VALID] = 0
    return img


def run(kw, v=0.25, hz=13.0, delay=3, noise_mm=6.0, route=True, s_err=0.0, seed=1, trace=False, plant=1.0):
    """plant: 실제 곡률 / 명령 곡률 (브리지 축거 0.2 vs 실제 0.222면 약 0.9)"""
    rnd = random.Random(seed)
    p = LaneParams(lane_width_mm=W, lane_color='yellow', lookahead_mm=650.0, kappa_alpha=0.5,
                   kappa_rate_max=0.6, kappa_max=2.1, route_ff_weight_lost=0.8, curve_slowdown=0.3)
    for k, val in kw.items():
        setattr(p, k, val)
    f = LaneFollower(CFG, p)
    x, y, h = -L0 + 50.0, 0.0, 0.0
    dt = 1.0 / hz
    q = [0.0] * delay
    errs, rows = [], []
    kmax_car = 1.0 / 0.53
    for i in range(int(14.0 * hz)):
        e, s = track_err(x, y)
        if route:
            f.route_step_mm = 100.0
            ks = []
            for j in range(14):
                sj = s + s_err + 200.0 + j * 100.0               # 앞바퀴 = 기준점 + 200
                ks.append(-1000.0 / R if L0 <= sj <= L0 + R * math.pi / 2 else 0.0)
            f.route_k = ks
        img = render(x, y, h, rnd, noise_mm)
        out = f.process_bev(img, np.zeros_like(img), yaw=h, speed=v)
        q.append(out.kappa * (out.speed_scale if out.speed_scale else 1.0) ** 0)
        k = max(-kmax_car, min(kmax_car, plant * q.pop(0)))
        vv = v * (out.speed_scale if out.speed_scale is not None else 1.0)
        h += vv * k * dt
        x += vv * 1000.0 * math.cos(h) * dt
        y += vv * 1000.0 * math.sin(h) * dt
        errs.append(e)
        if trace and L0 - 600 <= s <= L0 + R * math.pi / 2 + 900:
            rows.append(f'  s {s - L0:+6.0f}mm(코너시작 0) 옆오차 {e:+5.0f}mm  명령k {out.kappa:+.2f} 필요k {(-1000/R if L0<=s<=L0+R*math.pi/2 else 0):+.2f}  '
                        f'{out.status:<4} {out.source:<6} 지킴 {out.yellow_guard or "-"} 방향 {math.degrees(h):+5.0f}°')
        if s > L0 + R * math.pi / 2 + 1500:
            break
    errs = np.array(errs)
    out_max = float(np.max(errs))                       # 바깥쪽 최대
    in_max = float(-np.min(errs))                       # 안쪽 최대
    cross = (out_max + 120.0 > W / 2) or (in_max + 120.0 > W / 2)
    return out_max, in_max, cross, rows


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'trace':
        kw = eval(sys.argv[2]) if len(sys.argv) > 2 else {}
        o, i_, c, rows = run(kw, trace=True)
        print('\n'.join(rows[::2]))
        print(f'바깥 최대 {o:.0f}mm 안쪽 최대 {i_:.0f}mm 선 밟음 {c}')
        sys.exit(0)
    VAR = eval(os.environ.get('VAR', '{}'))
    cases = {'지금 main': {}}
    cases.update(VAR)
    for name, kw in cases.items():
        res = [run(kw, seed=sd, **cond) for sd in range(3)
               for cond in (dict(), dict(delay=4, hz=12.0, noise_mm=10.0), dict(s_err=+200.0), dict(s_err=-200.0), dict(v=0.3))]
        o = np.array([r[0] for r in res]); i_ = np.array([r[1] for r in res]); c = sum(r[2] for r in res)
        print(f'{name:14s} 바깥쪽 최대 평균 {o.mean():4.0f} 최악 {o.max():4.0f}mm | 안쪽 최대 평균 {i_.mean():4.0f} 최악 {i_.max():4.0f}mm | '
              f'선 밟음 {c}/{len(res)}판', flush=True)
