"""m7_waypoint_core 오프라인 시뮬 (ROS 없음).

  python3 -m limo_obstacle_avoid.m7_waypoint_sim --runs 40 --entry straight
  python3 -m limo_obstacle_avoid.m7_waypoint_sim --runs 40 --entry wide --reverse 3

세계: 3차선(차로 0.52 m, 바깥 노란선 ±0.78), 정차 LIMO 7대(가로 0.22 x 0.32), 벽 ±1.3 m.
1차 대회 배치(11/19)를 앞뒤 ±0.2 m·옆 ±3 cm·각도 ±4°, 전체 ±0.15 m 흔들어 씀. M2 방향 무작위.
라이다 240°·1°·10 Hz·노이즈 8 mm, 조향: 명령 0.3 s 지연 → 실제 곡률 = gain x 명령 + bias, 반응 0.1 s, |곡률| <= 1.6.
odom 거리 배율 오차, IMU 자이로 배율·편향 → PoseFusion 그대로.
진입: straight(0±1.5°) / normal(+5±3°, 바깥 +5 cm) / wide(+10±3°, 바깥 +15 cm) / inside(−1차로 ±4°).
성공 = 끝까지 + 정차 LIMO 접촉 0 + 바깥 노란선 밖으로 한쪽 바퀴 둘 다 나감 0.
"""
import argparse
import math
import random

from limo_obstacle_avoid.m7_waypoint_core import M7Waypoint, WaypointParams, rect, rect_ref, sat_overlap

BASE = [(1.08, 1, 0, 0), (1.09, 0, 0, 1), (4.41, 1, 0, 3), (3.27, 0, 3, 1), (2.17, 1, -4, 0), (2.16, -1, 0, 0), (4.42, -1, -2, 2)]
ENTRY = {'straight': (0.0, 0.04, 0.0, 1.5), 'normal': (0.05, 0.08, 5.0, 3.0),
         'wide': (0.15, 0.08, 10.0, 3.0), 'inside': (-0.52, 0.06, 0.0, 4.0)}
LW, ALONG, ACROSS, LINE = 0.52, 0.22, 0.32, 0.78


def layout(seed):
    r = random.Random(seed * 7 + 3)
    sh = r.uniform(-0.15, 0.15)
    obs = [(x + sh + r.uniform(-0.2, 0.2), k * LW + (lat + r.uniform(-3, 3)) / 100, math.radians(a + r.uniform(-4, 4)))
           for x, k, lat, a in BASE]
    return obs, ('A' if r.random() < 0.5 else 'B')


def segments(obs):
    segs = []
    for x, y, th in obs:
        c = rect(x, y, th, ALONG, ACROSS)
        segs += [(c[i][0], c[i][1], c[(i + 1) % 4][0], c[(i + 1) % 4][1]) for i in range(4)]
    segs += [(-1, 1.3, 8, 1.3), (-1, -1.3, 8, -1.3)]
    return segs


def raycast(x, y, psi, segs, lidar_x, rng, rmax=3.0):
    sx, sy = x + lidar_x * math.cos(psi), y + lidar_x * math.sin(psi)
    out = []
    for i in range(-120, 121):
        a = math.radians(i)
        dx, dy = math.cos(psi + a), math.sin(psi + a)
        best = rmax
        for x1, y1, x2, y2 in segs:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - sx) * ey - (y1 - sy) * ex) / den
            u = ((x1 - sx) * dy - (y1 - sy) * dx) / den
            if 0.05 < t < best and 0.0 <= u <= 1.0:
                best = t
        if best < rmax:
            t = best + rng.gauss(0, 0.008)
            out.append((lidar_x + t * math.cos(a), t * math.sin(a), a))
    return out


def run(seed, entry='straight', overrides=None, gain=0.74 * 1.2, bias=-0.051 + 0.075, t_max=120.0):
    rng = random.Random(seed * 104729 + 7)
    obs, fork = layout(seed)
    segs = segments(obs)
    rects = [rect(x, y, th, ALONG, ACROSS) for x, y, th in obs]
    ym, ys, pm, ps = ENTRY[entry]
    p = WaypointParams()
    for k, v in (overrides or {}).items():
        setattr(p, k, v)
    p.section_length = 5.6 + p.end_before
    if entry == 'inside':
        p.start_lane = -1
    core = M7Waypoint(p)
    core.set_exit(fork)
    # 위치를 다 알 때 통과 가능한가 (같은 계획기)
    k0 = -1 if entry == 'inside' else 0
    feasible = core._plan([{'x': x, 'y': y, 'near': True} for x, y, _ in obs], 0.0, k0, None, None) is not None
    # 실제 상태 (차로 좌표)
    X, Y, PSI, K = 0.0, ym + rng.gauss(0, ys), math.radians(pm + rng.gauss(0, ps)), 0.0
    odo_scale, gyro_scale, gyro_bias = 1 + rng.gauss(0, 0.03), 1 + rng.gauss(0, 0.03), math.radians(rng.gauss(0, 0.05))
    # odom 좌표 (임의 원점·방향)
    oyaw0 = rng.uniform(-math.pi, math.pi)
    ox = oy = 0.0
    oyaw = oyaw0
    dt = 0.01
    t = 0.0
    for _ in range(150):                    # PoseFusion 자이로 편향 잼 (정지 1.5 s)
        t += dt
        core.on_imu(t, gyro_bias + rng.gauss(0, 0.002))
        if int(t * 100) % 2 == 0:
            core.on_odom(t, ox, oy, oyaw, 0.0)
    core.start()
    q = [0.0] * int(round(0.3 / dt))
    v = k_cmd = 0.0
    next_scan, next_ctl = t, t
    contact = depart = False
    t0 = t
    min_gap = 9.9
    while t - t0 < t_max:
        if t >= next_scan:
            next_scan += 0.1
            core.on_scan(t, raycast(X, Y, PSI, segs, p.lidar_x, rng), math.radians(-120), math.radians(120))
        if t >= next_ctl:
            next_ctl += 0.05
            v, k_cmd = core.step(t)
            if core.state == 'DONE':
                break
        q.append(k_cmd)
        kd = q.pop(0)
        K += (max(-1.6, min(1.6, gain * kd + bias)) - K) * dt / 0.1
        dpsi = v * K * dt
        PSI += dpsi
        X += v * math.cos(PSI) * dt
        Y += v * math.sin(PSI) * dt
        ds = v * dt * odo_scale
        oyaw += dpsi
        ox += ds * math.cos(oyaw)
        oy += ds * math.sin(oyaw)
        t += dt
        core.on_imu(t, dpsi / dt * gyro_scale + gyro_bias + rng.gauss(0, 0.002))
        if int(round(t * 100)) % 2 == 0:
            core.on_odom(t, ox, oy, oyaw, v * odo_scale)
        me = rect_ref(X, Y, PSI, p.robot_front, p.robot_rear, p.robot_half_width)
        if any(sat_overlap(me, r) for r in rects):
            contact = True
        c, s = math.cos(PSI), math.sin(PSI)
        wl = [Y + ax * s + ay * c for ax, ay in ((0.1, 0.09), (-0.1, 0.09))]
        wr = [Y + ax * s + ay * c for ax, ay in ((0.1, -0.09), (-0.1, -0.09))]
        if min(wl) > LINE + 0.01 or max(wr) < -(LINE + 0.01):
            depart = True
    reached = core.state == 'DONE' and X >= 5.6 - 0.4
    st = core.status()
    return {'seed': seed, 'feasible': feasible, 'reached': reached, 'contact': contact, 'depart': depart,
            'success': reached and not contact and not depart, 'x': round(X, 2), 't': round(t - t0, 1),
            'n_rev': st['n_rev'], 'why': st['why'], 'y_err': round(core.pose()[1] - Y, 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=30)
    ap.add_argument('--entry', default='straight', choices=list(ENTRY))
    ap.add_argument('--reverse', type=int, default=0)
    ap.add_argument('--roll', type=float, default=1.0)
    ap.add_argument('--prior', action='store_true', help='1차 대회 배치를 예상 배치로 넣음')
    ap.add_argument('--start', type=int, default=0)
    a = ap.parse_args()
    ov = {'reverse_max': a.reverse, 'roll_dist': a.roll}
    if a.prior:
        ov['prior_xy'] = [v for x, k, lat, _ in BASE for v in (x, k * LW + lat / 100)]
    rs_all = [run(a.start + i, a.entry, ov) for i in range(a.runs)]
    rs = [r for r in rs_all if r['feasible']]
    print(f'위치를 다 알아도 못 지나가는 배치 {len(rs_all) - len(rs)}/{len(rs_all)} 제외')
    n = max(1, len(rs))
    pc = lambda key: f'{100 * sum(1 for r in rs if r[key]) / n:.0f}%'
    stuck = sum(1 for r in rs if not r['reached'])
    print(f'[{a.entry}] 후진 {a.reverse} 굴려보기 {a.roll} m: 성공 {pc("success")} | 접촉 {pc("contact")} | 이탈 {pc("depart")} | '
          f'끝 못 감 {100 * stuck / n:.0f}% | 후진/판 {sum(r["n_rev"] for r in rs) / n:.2f}')
    for r in rs:
        if not r['success']:
            print('  실패', r)


if __name__ == '__main__':
    main()
