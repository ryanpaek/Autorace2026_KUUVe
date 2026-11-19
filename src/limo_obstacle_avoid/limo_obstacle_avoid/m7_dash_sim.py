"""m7_dash_core 오프라인 시뮬 (ROS 없음). 세계·라이다·조향 모델은 m7_waypoint_sim(#152) 그대로 씀.

  python3 -m limo_obstacle_avoid.m7_dash_sim --runs 40
  python3 -m limo_obstacle_avoid.m7_dash_sim --runs 40 --orient mixed --clear 0.03

좌표: m7_waypoint_sim은 가운데 차로 중심 = 0 → 여기선 +0.26 옮겨 1|2 점선 = 0.
배치: 1차 대회 배치 흔든 것 (앞뒤 ±0.2 m, 옆 ±3 cm, 각 ±4°). 왼쪽 끝 차로 LIMO도 라이다엔 보임(무시해야 함).
--orient side(가로, 기본) / long(세로) / mixed(무작위).  --jitter 0 이면 사용자 말대로 '차로 중심 정확히'.
점선 카메라: 0.1 s마다 전방 0.30~0.70 m 점선 점 (노이즈 1 cm, 절반은 끊김). 시작 오차: 옆 ±2 cm, 방향 ±1.5°.
성공 = 끝까지 + 접촉 0 + 오른쪽 노란선(-0.52) 밖으로 바퀴 안 나감.
"""
import argparse
import math
import random

from limo_obstacle_avoid.m7_dash_core import DashParams, M7Dash
from limo_obstacle_avoid.m7_waypoint_sim import BASE, LW, ALONG, ACROSS, raycast
from limo_obstacle_avoid.sim_common import min_gap

SHIFT = LW / 2          # 가운데 차로 중심 -> 점선 기준


def layout(seed, orient='side', jitter=1.0):
    r = random.Random(seed * 7 + 3)
    sh = r.uniform(-0.15, 0.15) * jitter
    obs = []
    for x, k, lat, a in BASE:
        th = math.radians(a + r.uniform(-4, 4)) * jitter
        if orient == 'long' or (orient == 'mixed' and r.random() < 0.5):
            th += math.pi / 2
        obs.append((x + sh + r.uniform(-0.2, 0.2) * jitter, k * LW + SHIFT + (lat + r.uniform(-3, 3)) * jitter / 100, th))
    return obs


def segments(obs):
    from limo_obstacle_avoid.m7_waypoint_core import rect
    segs = []
    for x, y, th in obs:
        c = rect(x, y, th, ALONG, ACROSS)
        segs += [(c[i][0], c[i][1], c[(i + 1) % 4][0], c[(i + 1) % 4][1]) for i in range(4)]
    return segs + [(-1, 1.3 + SHIFT, 8, 1.3 + SHIFT), (-1, -1.3 + SHIFT, 8, -1.3 + SHIFT)]


def run(seed, orient='side', jitter=1.0, overrides=None, start_deg=1.5, dash_drop=0.5, gain=0.74 * 1.2, bias=-0.051 + 0.075, t_max=60.0, trace=False):
    rng = random.Random(seed * 104729 + 7)
    obs = layout(seed, orient, jitter)
    segs = segments(obs)
    rects = [(x, y, ALONG, ACROSS, th) for x, y, th in obs]
    p = DashParams()
    for k, v in (overrides or {}).items():
        setattr(p, k, v)
    p.end_x = 5.0
    core = M7Dash(p)
    X, Y, PSI, K = 0.0, rng.uniform(-0.02, 0.02), math.radians(rng.uniform(-start_deg, start_deg)), 0.0
    odo_scale, gyro_scale, gyro_bias = 1 + rng.gauss(0, 0.03), 1 + rng.gauss(0, 0.03), math.radians(rng.gauss(0, 0.05))
    ox = oy = 0.0
    oyaw = rng.uniform(-math.pi, math.pi)
    dt, t = 0.01, 0.0
    for _ in range(150):
        t += dt
        core.on_imu(t, gyro_bias + rng.gauss(0, 0.002))
        if int(t * 100) % 2 == 0:
            core.on_odom(t, ox, oy, oyaw, 0.0)
    core.start()
    q = [0.0] * int(round(0.3 / dt))
    v = k_cmd = 0.0
    next_scan = next_ctl = next_cam = t
    t0 = t
    gap_min, depart, contact = 9.9, False, False
    y_hi, y_lo = -9.9, 9.9
    tr = []
    while t - t0 < t_max:
        if t >= next_scan:
            next_scan += 0.1
            core.on_scan(t, raycast(X, Y, PSI, segs, p.lidar_x, rng), math.radians(-120), math.radians(120))
        if t >= next_cam:
            next_cam += 0.1
            pts = []
            for f in (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70):
                if rng.random() < dash_drop:
                    continue
                l = -(Y + f * math.sin(PSI)) / math.cos(PSI)
                pts.append((1000 * f, -1000 * l + rng.gauss(0, 10)))
            pred = core.dash_pred_mm
            core.on_dash([(f, xm) for f, xm in pts if abs(xm - pred(f)) < 1000 * p.dash_gate])
        if t >= next_ctl:
            next_ctl += 0.05
            v, k_cmd = core.step(t)
            if core.state == 'DONE':
                break
            if trace:
                ex, ey, _ = core.pose()
                tr.append((round(X, 2), round(Y, 3), round(ey - Y, 3), round(core.y_ref(X), 3), core.why))
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
        c, s = math.cos(PSI), math.sin(PSI)
        off = (p.robot_front - p.robot_rear) / 2
        g = min_gap((X + off * c, Y + off * s, p.robot_front + p.robot_rear, 2 * p.robot_half_width, PSI), rects)
        gap_min = min(gap_min, g)
        contact = contact or g < 0
        wr = [Y + ax * s - 0.09 * c for ax in (0.1, -0.1)]
        depart = depart or max(wr) < -LW - 0.01
        body = [Y + a * s + b * c for a in (p.robot_front, -p.robot_rear) for b in (p.robot_half_width, -p.robot_half_width)]
        y_hi, y_lo = max(y_hi, max(body)), min(y_lo, min(body))
    reached = core.state == 'DONE'
    lane3 = y_hi > LW                      # 차체가 2|3차로 점선(+0.52)을 넘음
    return {'seed': seed, 'reached': reached, 'contact': contact, 'depart': depart, 'lane3': lane3,
            'body_hi_cm': round(100 * y_hi, 1), 'body_lo_cm': round(100 * y_lo, 1),
            'n_rejudge': core.n_rejudge, 'n_rev': core.n_rev,
            'success': reached and not contact and not depart and not lane3, 'gap_cm': round(100 * gap_min, 1),
            't': round(t - t0, 1), 'why': core.why, 'y_err_cm': round(100 * (core.pose()[1] - Y), 1),
            'trace': tr, 'status': core.status()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=30)
    ap.add_argument('--start', type=int, default=0)
    ap.add_argument('--orient', default='side', choices=['side', 'long', 'mixed'])
    ap.add_argument('--jitter', type=float, default=1.0)
    ap.add_argument('--set', nargs='*', default=[], help='파라미터 덮어쓰기 이름=값')
    a = ap.parse_args()
    ov = {k: float(v) for k, v in (s.split('=') for s in a.set)}
    rs = [run(a.start + i, a.orient, a.jitter, ov) for i in range(a.runs)]
    n = len(rs)
    pc = lambda key: f'{100 * sum(1 for r in rs if r[key]) / n:.0f}%'
    gaps = sorted(r['gap_cm'] for r in rs)
    print(f'[{a.orient} jitter {a.jitter} {ov}] 성공 {pc("success")} | 접촉 {pc("contact")} | 이탈 {pc("depart")} | '
          f'끝 못 감 {100 * sum(1 for r in rs if not r["reached"]) / n:.0f}% | '
          f'최소 간격 cm: 최저 {gaps[0]} 중앙 {gaps[n // 2]} | 시간 중앙 {sorted(r["t"] for r in rs)[n // 2]} s')
    for r in rs:
        if not r['success']:
            print('  실패', {k: r[k] for k in ('seed', 'contact', 'depart', 'reached', 'gap_cm', 'why', 'y_err_cm')})


if __name__ == '__main__':
    main()
