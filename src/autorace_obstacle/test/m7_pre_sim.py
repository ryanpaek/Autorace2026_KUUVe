"""M7 코너 미리보기 시뮬레이션 (ROS 없이): 코너 직후 2차선에 정차 LIMO가 있는 배치만.

11/14 실차: 우하단 코너(R640, 90°)를 다 돈 뒤(80°) 회피 노드를 켜고, 지도가 쌓인 뒤에야 2차선 장애물에 반응 ->
늦게 꺾어 서고 후진(온 길 되돌아가기)하며 시간을 까먹음.
새 방식(m7_link 설명 8): 코너 도는 중 라이다 점을 3차선 좌표로 쌓아 '코너 직후 2차선 막힘'과 1·3차선 장애물을 같이 보고
빈 쪽을 미리 정함 -> 코너 70°에서 켜고 회피 노드는 켜자마자 그 차로로.

진짜 코드: M7Link(+CornerPreview), obstacle_node 계산(scan_to_points), AvoidPlanner. 모형은 m7_corner_sim과 같음
(횡단보도 끝 -> 코너 R640 90° -> 시작선, 차선 주행 = 중심선 pure pursuit, 코너 바깥 벽).

  python3 src/autorace_obstacle/test/m7_pre_sim.py            # 끔(지금 main) vs 켬, 조건별
  python3 src/autorace_obstacle/test/m7_pre_sim.py --n 60 --turn 60 70 80
"""
import argparse
import math
import os
import random
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import m7_corner_sim as cs                                                       # noqa: E402
import m7_trigger_sim as ts                                                      # noqa: E402
import stress_sim as ss                                                          # noqa: E402
from autorace_obstacle.m7_link import M7Link, M7LinkParams                       # noqa: E402
from autorace_obstacle.scan_utils import scan_to_points                          # noqa: E402
from limo_obstacle_avoid import sim_common as sc                                 # noqa: E402
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base  # noqa: E402

ISLAND = (ts.ISLAND_S - ts.START_S) / 1000.0     # 시작선 -> 섬 앞 끝 (m)
YAML = os.path.join(HERE, '..', '..', 'limo_obstacle_avoid', 'config', 'avoid_params.yaml')


def team_params():
    """실차와 같게 avoid_params.yaml 값을 넣은 AvoidParams (후진 recover 켬, 속도 0.2 등)."""
    import yaml
    from dataclasses import fields
    p = AvoidParams()
    with open(YAML) as fp:
        y = yaml.safe_load(fp)['/**']['ros__parameters']
    for f in fields(p):
        if f.name in y:
            v = y[f.name]
            setattr(p, f.name, float(v) if isinstance(getattr(p, f.name), float) and isinstance(v, int) else v)
    return p


def make_layout(rnd, x_range=(0.0, 1.0), side_p=0.5):
    """첫 줄에 2차선 장애물(앞면 = 시작선 + x_range). side_p 확률로 첫 줄에 옆 차로 하나 더 (그쪽은 못 감).
    뒤는 stress_sim 무작위 줄 (줄 간격 1.1~1.7 m). 차로 조합으로 지나갈 수 있는 배치만."""
    for _ in range(20000):
        x_first = rnd.uniform(*x_range)
        first = [0] + ([rnd.choice([-1, 1])] if rnd.random() < side_p else [])
        try:
            rows, _ = ss.random_layout(rnd, n_cars=7 - len(first), x0=x_first + rnd.uniform(1.1, 1.7))
        except RuntimeError:
            continue
        grouped = {}
        for x, lane in rows:
            grouped.setdefault(round(x, 6), []).append(lane)
        rows2 = [(x_first, first)] + sorted(grouped.items())
        rows2 = [(x, ls) for x, ls in rows2 if x + 0.32 + 0.8 <= ISLAND - ts.WHEELBASE]
        if len(rows2) < 2:
            continue
        rel = [(x - x_first + 0.9, ls) for x, ls in rows2]          # 조합 검사 (첫 줄 거리 조건은 뺌)
        if ss.feasible(rel + [(ISLAND - ts.WHEELBASE - x_first + 0.9, [0])]):
            return [(x, lane) for x, ls in rows2 for lane in ls]
    raise RuntimeError('배치 생성 실패')


def run(cars, seed, pre=True, turn_deg=60.0, map_err_mm=0.0, delay=0, noise=0.01, lidar_yaw_deg=0.0,
        real_r=cs.R, odom_scale=1.0, lane_dy=0.0, link_kw=None, cam=False, avoid_kw=None, car_min_r=0.53):
    """real_r: 실제 코너 반경 (모형 640과 다를 때), lidar_yaw_deg: 라이다 방향 오차, lane_dy: 차선 주행이 중심선에서 옆으로 치우침."""
    rnd = random.Random(seed)
    rects = sc.layout_rects(cars, 0.52, jitter=0.03, seed=seed)
    segs = sc.rects_to_segments(rects) + cs.walls()
    R0 = cs.R
    cs.R = real_r                                   # 경로 모형만 바꿈 (m7_link 모형은 640 그대로)
    try:
        return _run(cars, rects, segs, rnd, seed, pre, turn_deg, map_err_mm, delay, noise, lidar_yaw_deg,
                    odom_scale, lane_dy, link_kw or {}, cam, avoid_kw or {}, car_min_r)
    finally:
        cs.R = R0


def fake_lane_status(car, rnd, lw=0.52, row=0.55, ld=0.6):
    """차선 노드 /lane/status 흉내 (3차선 직선 구간): 카메라 행(기준점 앞 row)이 들어 있는 차로를 본다.
    offset_mm = 그 차로 중심의 옆 위치(차 기준, 왼쪽 +), heading_rad = 차로 방향 - 차 방향, kappa = 그 차로 중심 pure pursuit,
    lane_id = 그 차로, 선 종류 = 바깥 차로면 바깥쪽 노란 실선. 잡음: 옆 15 mm, 방향 1°."""
    yr = car.y + row * math.sin(car.yaw)
    lane = max(-1, min(1, int(round(yr / lw))))
    c = lane * lw
    psi = car.yaw
    off = (c - yr) * math.cos(psi) + rnd.gauss(0, 0.015)
    h = -psi + math.radians(rnd.gauss(0, 1.0))
    dy = c - car.y
    xr = ld * math.cos(psi) + dy * math.sin(psi)
    yrr = -ld * math.sin(psi) + dy * math.cos(psi)
    k = 2.0 * yrr / (xr * xr + yrr * yrr)
    yellow = {'color': 'yellow', 'dashed': False}
    white = {'color': 'white', 'dashed': True}
    st = 'BOTH' if abs(math.degrees(psi)) < 25.0 else ('L' if psi < 0 else 'R')
    return {'status': st, 'offset_mm': off * 1000.0, 'heading_rad': h, 'kappa': max(-1.9, min(1.9, k)),
            'lane_id': lane, 'line_left': yellow if lane == 1 else white, 'line_right': yellow if lane == -1 else white}


def _run(cars, rects, segs, rnd, seed, pre, turn_deg, map_err_mm, delay, noise, lidar_yaw_deg, odom_scale, lane_dy,
         link_kw, cam=False, avoid_kw=None, car_min_r=0.53):
    u0 = 0.06
    fx, fy, fh = cs.path_point(u0)
    car = sc.SimCar(fx - ts.WHEELBASE * math.cos(fh), fy - ts.WHEELBASE * math.sin(fh), fh, min_r=car_min_r)
    car.odom_yaw = car.imu_yaw = car.yaw
    lp = M7LinkParams(enable_mode='obstacle', trigger_front_mm=1800.0, off_mode='none',
                      pre_enable=pre, pre_enable_turn_deg=turn_deg, **link_kw)
    link = M7Link(lp)
    pl, p = None, team_params()
    for kk, vv in (avoid_kw or {}).items():
        setattr(p, kk, vv)
    cam_q = []
    a_min, a_inc = -math.pi, 2 * math.pi / ts.N_BEAM
    dt, t, dist = 0.05, 0.0, 0.0
    q = [(0.0, 0.0)] * delay
    start_at = handover_at = None
    ev = dict(enable_pos=None, enable_turn=None, pre_lane=None, first_target=None, t_enable=None)
    collided, off_edge, rev, stop_t, last_v = False, 0.0, 0.0, 0.0, 0.0
    first_x = min([x for x, lane in cars if lane == 0] or [None], key=lambda v: 9e9 if v is None else v)
    min_gap = 9.9
    depart_n, was_out = 0, False
    over = -1.0
    for k in range(int(90 / dt)):
        if k % 2 == 0:
            lx, ly = car.x + ts.LIDAR_X * math.cos(car.yaw), car.y + ts.LIDAR_X * math.sin(car.yaw)
            rng = sc.raycast(lx, ly, car.yaw + math.radians(lidar_yaw_deg), a_min, a_inc, ts.N_BEAM, segs,
                             noise=noise, rnd=rnd)
            link.on_front(ts.obstacle_node_front(rng, a_min, a_inc, link.p.trigger_half_width_mm))
            link.on_scan_points(scan_to_points(rng, a_min, a_inc, 0.05, 8.0, ts.LIDAR_X * 1000.0, 0.0, 0.0))
            if pl is not None:
                pl.update_scan(scan_to_base(rng, a_min, a_inc, p))
        ffx, ffy = car.x + ts.WHEELBASE * math.cos(car.yaw), car.y + ts.WHEELBASE * math.sin(car.yaw)
        u_f, _ = cs.nearest_u(ffx, ffy)
        pos_true = u_f - cs.U_START
        s_mm = ts.START_S + pos_true * 1000.0 + map_err_mm
        _, _, h_ahead = cs.path_point(max(u_f + 0.35, 0.0))
        lane = {'status': 'BOTH', 'heading_rad': math.atan2(math.sin(h_ahead - car.yaw), math.cos(h_ahead - car.yaw))}
        link.on_yaw(car.imu_yaw)
        mission = {'state': 'M7_OBSTACLE', 'dist_mm': dist * 1000.0, 'fork_choice': None, 'route': {'s_mm': s_mm}}
        for act in link.update(t, mission, lane):
            if act[0] == 'enable' and start_at is None:
                start_at = t + 0.05                                  # 파라미터 서비스
                p.section_length = act[1]
                p.start_heading_deg = link.start_heading_deg or 0.0  # obstacle_node._enable 과 같게
                p.start_target = link.start_target
                p.start_y = link.start_y_m
                ev.update(enable_pos=pos_true, enable_turn=math.degrees(link.corner.turned),
                          pre_lane=link.start_target or None, t_enable=t)
        if start_at is not None and pl is None and t >= start_at:
            pl = AvoidPlanner(p)
            pl.start(car.odom_x, car.odom_y, car.imu_yaw, t)
            handover_at = t + 0.10
        if pl is not None:
            pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw, forward=last_v >= 0.0)
            if cam and car.x > 0.0 and k % 2 == 0:              # 카메라 10 Hz, 0.1 s 늦게
                cam_q.append(fake_lane_status(car, rnd))
                if len(cam_q) > 1:
                    pl.lane_fix(cam_q.pop(0), t)
            v_a, w_a = pl.step(t)
            link.on_avoid_active(True, t)
            if ev['first_target'] is None and pl.target != 0:
                ev['first_target'] = pl.target
        if handover_at is not None and t >= handover_at:
            v, w = v_a, w_a
        else:                                       # 차선 주행: 중심선(+lane_dy) pure pursuit
            ub, _ = cs.nearest_u(car.x, car.y)
            tx, ty, th = cs.path_point(max(ub + 0.35, 0.0))
            tx, ty = tx - lane_dy * math.sin(th), ty + lane_dy * math.cos(th)
            dx, dy = tx - car.x, ty - car.y
            yr = -dx * math.sin(car.yaw) + dy * math.cos(car.yaw)
            v = ts.V_LANE
            w = v * 2.0 * yr / max(1e-6, dx * dx + dy * dy)
        q.append((v, w))
        v2, w2 = q.pop(0)
        car.step(v2, w2, dt, odom_scale=odom_scale, rnd=rnd)
        dist += abs(v2) * dt
        if v2 < 0:
            rev += -v2 * dt
        if pl is not None and pl.state == 'RUN' and abs(v2) < 1e-3:
            stop_t += dt
        last_v = v2
        t += dt
        g = sc.min_gap(car.footprint(), rects)
        min_gap = min(min_gap, g)
        collided |= g < 0
        if pos_true > 0.3:
            for (_, cy) in sc.rect_corners(*car.footprint()):
                off_edge = max(off_edge, abs(cy) - 1.5 * 0.52)
                over = max(over, abs(cy) - (1.5 * 0.52 - 0.03))   # 바깥 노란 실선(두께 20 mm) 안쪽 가장자리를 차체가 넘음
        out_now = departed(car)
        depart_n += int(out_now and not was_out)
        was_out = out_now
        if pl is not None and pl.state == 'DONE':
            break
    done = pl is not None and pl.state == 'DONE'
    return dict(ok=done and not collided and off_edge <= 0, done=done, collided=collided, off=off_edge > 0,
                depart=depart_n, over=over,
                rev=rev, stop_t=stop_t, t=t, first_x=first_x, min_gap=min_gap, **ev)


TAPER = 0.58          # 도면: 시작선부터 3차선 바깥 선이 약 45°로 벌어져 0.58 m 뒤 전체 폭 (그 전은 한 차로 500 mm)


def edge_at(u):
    """경로 거리 u(횡단보도 끝 = 0) 지점의 차로 가장자리 (중심선 기준 옆 m)."""
    x = u - cs.U_START
    if x <= 0.0:
        return 0.25
    return 0.25 + min(1.0, x / TAPER) * 0.52


def departed(car):
    """차로 이탈 (규정: 한쪽 앞·뒷바퀴 모두 선 밖). 바퀴 = 차 중심선 옆 ±0.10 m, 앞바퀴 = 기준점 + 0.2 m."""
    sides = []
    for d in (0.0, ts.WHEELBASE):
        px, py = car.x + d * math.cos(car.yaw), car.y + d * math.sin(car.yaw)
        u, _ = cs.nearest_u(px, py)
        off = cs.left_offset(px, py)
        e = edge_at(u)
        sides.append(1 if off - 0.10 > e else -1 if off + 0.10 < -e else 0)
    return sides[0] != 0 and sides[0] == sides[1]


CONDS = [
    ('이상적', {}),
    ('조향 지연 0.15 s', dict(delay=3)),
    ('실제 코너 R600 (모형 640)', dict(real_r=0.60)),
    ('실제 코너 R680', dict(real_r=0.68)),
    ('라이다 방향 2° 틀림', dict(lidar_yaw_deg=2.0)),
    ('차선 주행 5 cm 치우침', dict(lane_dy=0.05)),
    ('복합: 지연+R600+라이다1°+잡음2cm', dict(delay=3, real_r=0.60, lidar_yaw_deg=1.0, noise=0.02)),
]


def summary(res):
    rv = [r['rev'] for r in res]
    tt = [r['t'] for r in res if r['done']]
    ep = [r['enable_turn'] for r in res if r['enable_turn'] is not None]
    return (f'{sum(r["ok"] for r in res):2d}/{len(res)} {sum(r["collided"] for r in res):3d} {sum(r["off"] for r in res):4d} '
            f'{sum(r["depart"] for r in res):4d} '
            f'{sum(1 for v in rv if v > 0.01):4d} {sum(rv):5.2f}m {np.mean([r["stop_t"] for r in res]):5.1f}s '
            f'{(np.mean(tt) if tt else float("nan")):5.1f}s {sum(r["pre_lane"] is not None for r in res):4d}  '
            f'{(np.median(ep) if ep else float("nan")):3.0f}°')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=40)
    ap.add_argument('--seed', type=int, default=5)
    ap.add_argument('--turn', type=float, nargs='*', default=[70.0])
    ap.add_argument('--x', type=float, nargs=2, default=[0.0, 1.0], help='첫 2차선 장애물 앞면 범위 (시작선 기준 m)')
    ap.add_argument('--conds', type=int, default=len(CONDS))
    a = ap.parse_args()
    rnd = random.Random(a.seed)
    lays = [make_layout(rnd, tuple(a.x)) for _ in range(a.n)]
    head = f'{"조건":32s} 성공 충돌 차로밖 이탈 후진판 후진합 정지평균 완주평균 미리정함 켠각'
    for name, ck in CONDS[:a.conds]:
        print(f'\n--- {name} ---\n{head}')
        base = [run(l, i, pre=False, **ck) for i, l in enumerate(lays)]
        print(f'{"끔 (지금 main)":32s} {summary(base)}', flush=True)
        for tr in a.turn:
            res = [run(l, i, pre=True, turn_deg=tr, **ck) for i, l in enumerate(lays)]
            print(f'{f"켬 (미리보기, {tr:.0f}°에서 켬)":32s} {summary(res)}', flush=True)


if __name__ == '__main__':
    main()
