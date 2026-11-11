"""M7 코너 시뮬레이션 (ROS 없이): 횡단보도 끝 -> 우하단 코너(R640, 90°) -> 시작선 -> 3차선 회피.

통주행에서만 생기는 문제를 본다: 도면 위치가 앞서면(또는 코너에서 라이다 앞에 벽이 보이면) 코너 안에서 회피를 켜고,
회피 노드는 켠 순간 방향으로 직진 -> 코너 바깥(노란 실선·금속 벽)으로 나가고 정차 LIMO 위치도 틀어진다.

진짜 코드: M7Link(코너 지킴 포함), obstacle_node 앞 장애물 계산, AvoidPlanner.
모형: 도면 치수 (횡단보도 끝 -> 코너 시작 390 mm, R640 90°, 코너 끝 -> 시작선 288 mm), 코너 바깥 벽(차로 가장자리 + 15 cm),
     3차선 왼쪽 벽, 차선 주행 = 중심선 pure pursuit 0.18 m/s, 카메라 차선 방향 = 0.35 m 앞 경로 방향 - 차 방향.

  python3 src/autorace_obstacle/test/m7_corner_sim.py
"""
import argparse
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m7_trigger_sim as ts                                                       # noqa: E402
from autorace_obstacle.m7_link import M7Link, M7LinkParams                       # noqa: E402
from limo_obstacle_avoid import sim_common as sc                                 # noqa: E402
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base  # noqa: E402

R = 0.64
CE = -0.288                      # 코너 끝 (시작선 기준 x)
PRE = 0.39                       # 횡단보도 끝 -> 코너 시작
U_START = PRE + math.pi * R / 2 - CE   # 횡단보도 끝 -> 시작선 (경로 거리) = 1.683
HALF = 0.26                      # 1차로 반폭 (도면 500 + 20)


def path_point(u):
    """경로 거리 u(횡단보도 끝 = 0) -> (x, y, 방향)."""
    if u <= PRE:
        return CE - R, -R - PRE + u, math.pi / 2
    if u <= PRE + math.pi * R / 2:
        th = math.pi - (u - PRE) / R
        return CE + R * math.cos(th), -R + R * math.sin(th), th - math.pi / 2
    return CE + (u - PRE - math.pi * R / 2), 0.0, 0.0


US = np.arange(-0.5, 8.0, 0.005)
PTS = np.array([path_point(max(0.0, u)) if u >= 0 else (CE - R, -R - PRE + u, math.pi / 2) for u in US])


def nearest_u(x, y):
    d = (PTS[:, 0] - x) ** 2 + (PTS[:, 1] - y) ** 2
    i = int(np.argmin(d))
    return float(US[i]), float(math.sqrt(d[i]))


def left_offset(x, y):
    """중심선 기준 왼쪽 거리 (코너 바깥 = 왼쪽 +)."""
    u, _ = nearest_u(x, y)
    px, py, h = path_point(max(u, 0.0))
    return -(x - px) * math.sin(h) + (y - py) * math.cos(h)


def walls():
    segs = []
    rw = R + 1.5 * 0.52 + 0.15            # 코너 바깥 벽: 3차선 왼쪽 가장자리 + 15 cm 선과 이어짐 (실제 위치는 실측 필요)
    n = 18
    for i in range(n):
        a0 = math.pi - (math.pi / 2) * i / n
        a1 = math.pi - (math.pi / 2) * (i + 1) / n
        segs.append(((CE + rw * math.cos(a0), -R + rw * math.sin(a0)), (CE + rw * math.cos(a1), -R + rw * math.sin(a1))))
    segs.append(((CE - rw, -R - PRE - 0.5), (CE - rw, -R)))                # 코너 전 직선 왼쪽 벽
    segs.append(((CE, 1.5 * 0.52 + 0.15), (5.0, 1.5 * 0.52 + 0.15)))       # 3차선 왼쪽 가장자리 + 15 cm
    return segs


def run(cars, seed, guard=True, map_err_mm=0.0, trig_mm=1000.0, delay=0, noise=0.01):
    rnd = random.Random(seed)
    rects = sc.layout_rects(cars, 0.52, jitter=0.03, seed=seed)
    segs = sc.rects_to_segments(rects)
    segs += walls()
    u0 = 0.06                                   # M7 진입 = 횡단보도 끝 + 6 cm (앞바퀴)
    fx, fy, fh = path_point(u0)
    car = sc.SimCar(fx - ts.WHEELBASE * math.cos(fh), fy - ts.WHEELBASE * math.sin(fh), fh)
    car.odom_yaw = car.imu_yaw = car.yaw
    lp = M7LinkParams(enable_mode='obstacle', trigger_front_mm=trig_mm, off_mode='none')
    if not guard:
        lp.corner_turn_deg = 0.0
    link = M7Link(lp)
    pl, p = None, AvoidParams()
    a_min, a_inc = -math.pi, 2 * math.pi / ts.N_BEAM
    dt, t, dist = 0.05, 0.0, 0.0
    q = [(0.0, 0.0)] * delay
    start_at = handover_at = None
    ev = dict(enable_pos=None, enable_head=None)
    max_left_pre, collided, off_edge = -9.0, False, 0.0
    v_a = w_a = 0.0
    for k in range(int(80 / dt)):
        if k % 2 == 0:
            lx, ly = car.x + ts.LIDAR_X * math.cos(car.yaw), car.y + ts.LIDAR_X * math.sin(car.yaw)
            rng = sc.raycast(lx, ly, car.yaw, a_min, a_inc, ts.N_BEAM, segs, noise=noise, rnd=rnd)
            link.on_front(ts.obstacle_node_front(rng, a_min, a_inc, link.p.trigger_half_width_mm))
            if pl is not None:
                pl.update_scan(scan_to_base(rng, a_min, a_inc, p))
        ffx, ffy = car.x + ts.WHEELBASE * math.cos(car.yaw), car.y + ts.WHEELBASE * math.sin(car.yaw)
        u_f, _ = nearest_u(ffx, ffy)
        pos_true = u_f - U_START
        s_mm = ts.START_S + pos_true * 1000.0 + map_err_mm
        _, _, h_ahead = path_point(max(u_f + 0.35, 0.0))
        lane = {'status': 'BOTH', 'heading_rad': math.atan2(math.sin(h_ahead - car.yaw), math.cos(h_ahead - car.yaw))}
        link.on_yaw(car.imu_yaw)
        mission = {'state': 'M7_OBSTACLE', 'dist_mm': dist * 1000.0, 'fork_choice': None, 'route': {'s_mm': s_mm}}
        for act in link.update(t, mission, lane):
            if act[0] == 'enable' and start_at is None:
                start_at = t + 0.05
                p.section_length = act[1]
                p.start_heading_deg = link.start_heading_deg or 0.0     # obstacle_node._enable 과 같게
                ev['enable_pos'] = pos_true
                ev['enable_head'] = math.degrees(car.yaw)
        if start_at is not None and pl is None and t >= start_at:
            pl = AvoidPlanner(p)
            pl.start(car.odom_x, car.odom_y, car.imu_yaw, t)
            handover_at = t + 0.10
        if pl is not None:
            pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw, forward=True)
            v_a, w_a = pl.step(t)
            link.on_avoid_active(True, t)
        if handover_at is not None and t >= handover_at:
            v, w = v_a, w_a
        else:                                   # 차선 주행: 중심선 pure pursuit
            ub, _ = nearest_u(car.x, car.y)
            tx, ty, _ = path_point(max(ub + 0.35, 0.0))
            dx, dy = tx - car.x, ty - car.y
            yr = -dx * math.sin(car.yaw) + dy * math.cos(car.yaw)
            v = ts.V_LANE
            w = v * 2.0 * yr / max(1e-6, dx * dx + dy * dy)
        q.append((v, w))
        v2, w2 = q.pop(0)
        car.step(v2, w2, dt, rnd=rnd)
        dist += abs(v2) * dt
        t += dt
        corners = sc.rect_corners(*car.footprint())
        if pos_true < 0.0:
            max_left_pre = max(max_left_pre, max(left_offset(cx, cy) for cx, cy in corners))
        g = sc.min_gap(car.footprint(), rects)
        collided |= g < 0
        if pos_true > 0.3:
            for (_, cy) in corners:
                off_edge = max(off_edge, abs(cy) - 1.5 * 0.52)
        if pl is not None and pl.state == 'DONE':
            break
    done = pl is not None and pl.state == 'DONE'
    depart = max_left_pre > HALF
    return dict(ok=done and not collided and off_edge <= 0 and not depart, done=done, collided=collided,
                depart=depart, max_left_pre=max_left_pre, off=off_edge > 0, **ev)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=30)
    a = ap.parse_args()
    rnd = random.Random(3)
    lays = [ts.make_layout(rnd, enable_front=-0.16) for _ in range(a.n)]   # 코너 끝(80°)에서 켜도 피할 수 있는 배치
    conds = [('도면 위치 정확', dict()), ('도면 위치 +0.3 m 앞섬', dict(map_err_mm=300.0)),
             ('도면 위치 +0.5 m 앞섬', dict(map_err_mm=500.0)), ('도면 위치 -0.3 m 늦음', dict(map_err_mm=-300.0))]
    for guard in (False, True):
        print(f'\n=== 코너 지킴 {"켬 (새)" if guard else "끔 (11/11 main)"} ===')
        print(f'{"조건":24s} 성공   충돌 코너밖 3차선밖 | 켠 위치(시작선 기준, 코너 끝 -0.29) 최소/중앙  켤 때 방향 최대')
        for name, ck in conds:
            res = [run(l, i, guard=guard, **ck) for i, l in enumerate(lays)]
            ep = [r['enable_pos'] for r in res if r['enable_pos'] is not None]
            eh = [abs(r['enable_head']) for r in res if r['enable_head'] is not None]
            print(f'{name:24s} {sum(r["ok"] for r in res):2d}/{len(res)} {sum(r["collided"] for r in res):4d} '
                  f'{sum(r["depart"] for r in res):5d} {sum(r["off"] for r in res):6d}   | '
                  f'{min(ep):+.2f} / {float(np.median(ep)):+.2f} m   {max(eh):4.0f}°', flush=True)


if __name__ == '__main__':
    main()
