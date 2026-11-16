"""M7 재생 방식(m7_replay_core) 시뮬레이션 — ROS 없음.

  python3 src/limo_obstacle_avoid/tools/m7_replay_sim.py            # 배치 여러 개 요약
  python3 src/limo_obstacle_avoid/tools/m7_replay_sim.py -v 2       # 2번 배치 자세히 (이벤트 로그)
  python3 src/limo_obstacle_avoid/tools/m7_replay_sim.py --left m7_left.csv --right m7_right.csv   # 실제 녹화로

모형: 애커만(축거 0.222, 최대 조향 0.40, 조향 지연 0.25 s, 실제 회전 = 명령의 90%), 라이다 0.5° 360°, 정차 LIMO 0.32 x 0.23 m,
      odom 방향 +12% 오차, 자이로 잡음·편향, 카메라 /lane/status(양쪽 선, 옆 위치·방향 잡음), 켤 때 옆 ±3 cm·방향 ±3°.
녹화 경로가 없으면 같은 모형으로 '키보드 녹화'를 만들어 씀 (546 mm 앞 장애물, 왼쪽 S자).
"""
import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, load_change, mirror  # noqa: E402
from limo_obstacle_avoid.path_track import Path, PoseFusion  # noqa: E402

WB, MAXD, LAG, GAIN = 0.222, 0.40, 0.25, 0.90
LW = 0.52
TRIGGER_MM = 546.0   # 실제 녹화와 같은 값. 모형 녹화(record_left)는 main에서 바꿈
OBS_L, OBS_W = 0.32, 0.23
FRONT, REAR, HALF = 0.20, 0.16, 0.12


class Car:
    def __init__(self, x, y, th):
        self.x, self.y, self.th, self.d, self.v, self.w = x, y, th, 0.0, 0.0, 0.0
        self.ox, self.oy, self.oth = 0.0, 0.0, 0.0

    def step(self, v, kappa, dt):
        delta = max(-MAXD, min(MAXD, math.atan(WB * kappa))) if abs(v) >= 0.02 else 0.0
        self.d += (delta - self.d) * min(1.0, dt / LAG)
        self.v = v
        self.w = v * math.tan(self.d) / WB * GAIN
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += self.w * dt
        self.ox += v * math.cos(self.oth) * dt
        self.oy += v * math.sin(self.oth) * dt
        self.oth += self.w * 1.12 * dt


def rect(cx, cy, L, W, th=0.0):
    c, s = math.cos(th), math.sin(th)
    return [(cx + c * a - s * b, cy + s * a + c * b) for a, b in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2))]


def seg_hit(ox, oy, dx, dy, ax, ay, bx, by):
    ex, ey = bx - ax, by - ay
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / den
    u = ((ax - ox) * dy - (ay - oy) * dx) / den
    return t if t > 0 and 0 <= u <= 1 else None


def scan(car, obstacles, lidar_x=0.10, step_deg=0.5, rmax=3.0):
    lx, ly = car.x + lidar_x * math.cos(car.th), car.y + lidar_x * math.sin(car.th)
    pts = []
    for i in range(int(360 / step_deg)):
        a = math.radians(-180 + i * step_deg)
        dx, dy = math.cos(car.th + a), math.sin(car.th + a)
        best = rmax
        for poly in obstacles:
            for j in range(4):
                t = seg_hit(lx, ly, dx, dy, *poly[j], *poly[(j + 1) % 4])
                if t is not None and t < best:
                    best = t
        if best < rmax:
            r = best + random.gauss(0, 0.005)
            pts.append((lidar_x + r * math.cos(a), r * math.sin(a)))
    return pts


def poly_dist(P, Q):
    def pd(px, py, ax, ay, bx, by):
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey
        u = max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / L2))
        return math.hypot(px - ax - u * ex, py - ay - u * ey)
    best = 1e9
    for A, B in ((P, Q), (Q, P)):
        for px, py in A:
            for j in range(4):
                best = min(best, pd(px, py, *B[j], *B[(j + 1) % 4]))
    return best


def record_left(trigger=0.546):
    """키보드처럼 왼쪽 S자를 녹화한 경로 (PoseFusion 좌표)."""
    car = Car(0.0, 0.0, 0.0)
    fu = PoseFusion()
    t, dt = 0.0, 0.02
    xs, ys, hs, ss = [], [], [], []
    T1 = 2.95
    while t < 1.2 + 0.3 + 2 * T1 + 2.0:
        tt = t - 1.2
        if tt < 0:
            v, k = 0.0, 0.0
        elif tt < 0.3:
            v, k = 0.2, 0.0
        elif tt < 0.3 + T1:
            v, k = 0.2, math.tan(0.40) / WB
        elif tt < 0.3 + 2 * T1:
            v, k = 0.2, -math.tan(0.40) / WB
        elif tt < 0.3 + 2 * T1 + 1.5:
            v, k = 0.2, 0.0
        else:
            v, k = 0.0, 0.0
        car.step(v, k, dt)
        t += dt
        fu.on_imu(t, car.w + random.gauss(0, 0.01))
        fu.on_odom(t, car.ox, car.oy, car.oth, v, v)
        if fu.ready:
            xs.append(fu.x)
            ys.append(fu.y)
            hs.append(fu.yaw)
    # 일정 간격으로
    out = ([xs[0]], [ys[0]], [hs[0]])
    for x, y, h in zip(xs, ys, hs):
        if math.hypot(x - out[0][-1], y - out[1][-1]) >= 0.02:
            out[0].append(x)
            out[1].append(y)
            out[2].append(h)
    return [Path(*out, 1)]


def run(layout, left, right, seed=0, verbose=False, fork='A', start_err=True, p=None, t_max=90.0):
    random.seed(seed)
    p = p or ReplayParams(trigger_mm=TRIGGER_MM)
    p.exit_lane = fork
    obstacles = [rect(x + OBS_L / 2, k * LW, OBS_L, OBS_W) for k, x in layout]
    island = p.section_length
    # 섬은 바닥 선 (라이다에 안 보임). 넘으면 '섬 침범'으로 셈
    island_poly = rect(island + 0.85, 0.0, 1.7, 0.50)
    ey, eth = (random.uniform(-0.03, 0.03), math.radians(random.uniform(-3, 3))) if start_err else (0.0, 0.0)
    car = Car(0.0, ey, eth)
    m = M7Replay(p, left, right)
    t, dt = 0.0, 0.02
    for _ in range(60):          # 자이로 보정 (서 있음)
        t += dt
        m.on_imu(t, random.gauss(0, 0.01))
        m.on_odom(t, car.ox, car.oy, car.oth, 0.0)
    m.p.start_heading_deg = 0.0
    m.p.start_y = 0.0
    m.start()
    k = 0
    min_clear = 9.0
    rev = 0.0
    n_ev = 0
    isl_hit = 0
    while t < t_max and m.state != 'DONE':
        t += dt
        k += 1
        m.on_imu(t, car.w + 0.002 + random.gauss(0, 0.01))
        if k % 2 == 0:
            m.on_odom(t, car.ox, car.oy, car.oth, car.v)
        if k % 5 == 0:
            m.on_scan_points(t, scan(car, obstacles))
        if k % 3 == 0:
            # 카메라: 지금 차로 양쪽 선이 보이면 (방향 12° 안)
            lane = max(-1, min(1, int(round(car.y / LW))))
            yin = car.y - lane * LW
            if abs(math.degrees(car.th)) < 15 and abs(yin) < 0.15:
                h = -car.th
                off = (-(yin) + 0.55 * math.sin(h)) / math.cos(h)
                m.on_lane({'status': 'BOTH', 'offset_mm': 1000 * off + random.gauss(0, 8),
                           'heading_rad': h + random.gauss(0, 0.01), 'lane_id': lane})
        v, kap = m.step(t)
        car.step(v, kap, dt)
        if v < 0:
            rev += -v * dt
        carp = rect(car.x + (FRONT - REAR) / 2 * math.cos(car.th), car.y + (FRONT - REAR) / 2 * math.sin(car.th),
                    FRONT + REAR, 2 * HALF, car.th)
        for o in obstacles:
            min_clear = min(min_clear, poly_dist(carp, o))
        if any(island <= qx <= island + 1.7 and abs(qy) < 0.25 for qx, qy in carp):
            isl_hit += 1
        if verbose:
            for e in m.events[n_ev:]:
                print(f'  {t:6.2f}s {e}')
            n_ev = len(m.events)
    ok = m.state == 'DONE'
    exit_lane = m.lane
    return {'ok': ok, 't': t, 'clear_cm': 100 * min_clear, 'rev_m': rev, 'lane': exit_lane, 'n_exec': m.n_exec,
            'y_err_cm': 100 * (car.y - exit_lane * LW), 'fork_ok': (exit_lane == (1 if fork == 'A' else -1)), 'island': isl_hit}


LAYOUTS = [
    # (차로, 장애물 앞면 x m) — 시작선 기준, 섬 앞 끝 4.38. 한 '벽'마다 옆 칸 하나는 비게 (대회 배치처럼 지나갈 수 있게)
    [(0, 1.2), (1, 2.8), (-1, 2.8), (0, 4.0)],
    [(0, 1.0), (-1, 1.0), (1, 2.8), (-1, 2.8)],
    [(1, 0.6), (0, 1.2), (-1, 2.7), (1, 3.3)],
    [(0, 1.3), (1, 1.3), (-1, 3.0), (1, 3.0)],
    [(0, 0.9), (1, 2.5), (-1, 2.6), (0, 3.9), (1, 4.1)],
    [(0, 1.1), (1, 1.1), (-1, 2.7), (1, 2.9), (0, 4.0), (-1, 4.2), (1, 0.3)],
    [(0, 1.2), (-1, 1.2), (1, 2.9), (-1, 2.9), (-1, 0.4), (1, 0.45), (1, 4.2)],
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', type=int, default=None, help='이 번호 배치를 자세히')
    ap.add_argument('--left')
    ap.add_argument('--right')
    ap.add_argument('--seeds', type=int, default=3)
    ap.add_argument('--trigger', type=float, default=None, help='트리거 거리 mm (기본: 녹화 파일이면 546, 모형이면 700)')
    ap.add_argument('--check', action='store_true',
                    help='녹화 경로만 점검: 가운데 차로 장애물 앞 trigger에서 시작했을 때 차체와 장애물 최소 간격')
    a = ap.parse_args()
    global TRIGGER_MM
    if a.left or a.right:
        left, src_l = load_change(a.left, a.right, +1, LW)
        right, src_r = load_change(a.right, a.left, -1, LW)
        print(f'녹화: 왼쪽 {src_l} / 오른쪽 {src_r}')
    else:
        random.seed(99)
        left = record_left()
        TRIGGER_MM = 700.0   # 모형 녹화(최대 조향 S자, 조향 지연·회전 90%)는 546 mm에서 시작하면 장애물 모서리에 닿음 -> 700
        right = mirror(left)
        print(f'녹화(모형): 왼쪽 S자 길이 {left[0].length:.2f} m, 끝 y {left[0].y[-1]:+.3f} m')
    if a.trigger is not None:
        TRIGGER_MM = a.trigger
    if a.check:
        from limo_obstacle_avoid.m7_replay_core import fit_lateral
        obs = rect(1.0 + OBS_L / 2, 0.0, OBS_L, OBS_W)
        x0 = 1.0 - 0.10 - TRIGGER_MM / 1000.0
        for name, segs in (('왼쪽', left), ('오른쪽', right)):
            segs, k = fit_lateral(segs, LW)
            best = 9.0
            for g in segs:
                for x, y, h in zip(g.x, g.y, g.h):
                    c = (x0 + x + (FRONT - REAR) / 2 * math.cos(h), y + (FRONT - REAR) / 2 * math.sin(h))
                    best = min(best, poly_dist(rect(c[0], c[1], FRONT + REAR, 2 * HALF, h), obs))
            need = ReplayParams().hit_margin
            print(f'{name}: 장애물(LIMO {OBS_L}x{OBS_W} m) 앞 {TRIGGER_MM:.0f} mm에서 시작 -> 차체 최소 간격 {100 * best:.1f} cm '
                  + ('OK' if best > need + 0.01 else f'<- 너무 가까움: 차로 고를 때 막힌 걸로 봄 (여유 {100 * need:.0f} cm). '
                     f'trigger_mm를 늘리거나 hit_margin을 줄이거나 더 일찍 꺾게 다시 녹화'))
        return
    if a.v is not None:
        r = run(LAYOUTS[a.v], left, right, seed=0, verbose=True)
        print(r)
        return
    tot = ok = fk = 0
    worst = 9
    for i, L in enumerate(LAYOUTS):
        for fork in ('A', 'B'):
            for s in range(a.seeds):
                r = run(L, left, right, seed=s, fork=fork)
                tot += 1
                ok += r['ok'] and r['clear_cm'] > 0.5 and r['island'] == 0
                fk += r['fork_ok']
                worst = min(worst, r['clear_cm'])
                print(f'배치 {i} 출구 {fork} seed {s}: {"성공" if r["ok"] else "실패"} {r["t"]:5.1f}s  최소 간격 {r["clear_cm"]:4.1f} cm  '
                      f'후진 {r["rev_m"]:.2f} m  재생 {r["n_exec"]}번  끝 차로 {r["lane"]:+d}{"" if r["fork_ok"] else " (출구 아님)"} '
                      f'옆 {r["y_err_cm"]:+.1f} cm' + (' 섬 침범!' if r['island'] else ''))
    print(f'\n합계: 성공(부딪힘 없이 끝) {ok}/{tot}, 출구 차로 맞음 {fk}/{tot}, 최소 간격 {worst:.1f} cm')


if __name__ == '__main__':
    main()
