"""M7 재생 방식 스트레스 시뮬레이션 — m7_replay_sim.py보다 훨씬 나쁜 조건 (ROS 없음).

  python3 src/limo_obstacle_avoid/tools/m7_stress_sim.py                  # 3단계(mild/hard/brutal) x 신뢰 방식 2개 비교
  python3 src/limo_obstacle_avoid/tools/m7_stress_sim.py --level hard -n 60
  python3 src/limo_obstacle_avoid/tools/m7_stress_sim.py --level brutal --one 12 -v   # 한 판 이벤트 로그

m7_replay_sim과 다른 점 (판마다 무작위):
  차체   : 실제 회전 = 명령의 gain배, 조향 지연, 조향 트림(바퀴가 한쪽으로 쏠림), 실제 속도 배율, 명령 지연·주기 빠짐
  odom   : 거리 배율 오차, 방향 오차, 자이로 편향 + 잡음
  라이다 : 거리 잡음, 레이 빠짐, 유령 점, 장착 각 오차, 스캔 지연
  장애물 : 위치 ±x/±y 흔들림, 비스듬히 놓임
  카메라 : 지연, 옆 위치·방향·곡률 잡음, 놓침 구간(NONE), low_conf, 한쪽 선만(L/R),
           차로 번호 틀림, '가짜 차로'(바깥 차로에서 트랙 밖 바닥을 흰 선으로 봄 -> 곡률이 바깥으로 끌고, 번호도 틀림)
  배치   : 고정 7개 + 무작위 배치 (벽마다 한 칸은 비게)
판정: 완주(DONE) / 충돌(간격 0) / 탈선(차체가 바깥 노란선 밖) / 섬 침범 / 시간 초과
'gate' = PR #117 이전 (카메라 차로 번호가 내부 추정과 같을 때만 차선 곡률), 'trust' = 지금 (번호 상관없이 차선 곡률)
"""
import argparse
import math
import os
import random
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, mirror  # noqa: E402
from limo_obstacle_avoid.path_track import MAX_CURV  # noqa: E402
import m7_replay_sim as base  # noqa: E402

LW = base.LW
EDGE = 1.5 * LW                 # 바깥 노란 실선 (차로 중심 ±0.52에서 반폭 바깥)
FRONT, REAR, HALF = base.FRONT, base.REAR, base.HALF

LEVELS = {
    #            mild    hard    brutal
    'gain':     ((0.85, 0.95), (0.75, 1.00), (0.68, 1.05)),
    'lag':      ((0.20, 0.30), (0.20, 0.40), (0.20, 0.50)),
    'trim_deg': (1.0, 2.0, 3.0),
    'vscale':   ((0.92, 1.05), (0.85, 1.10), (0.78, 1.12)),
    'cmd_lat':  ((0.02, 0.06), (0.04, 0.12), (0.06, 0.20)),
    'cmd_skip': (0.02, 0.06, 0.12),
    'odom_s':   (0.03, 0.06, 0.10),
    'odom_h':   ((1.05, 1.15), (0.95, 1.20), (0.90, 1.25)),
    'gyro_b':   (0.002, 0.005, 0.010),
    'gyro_n':   (0.01, 0.02, 0.03),
    'lid_n':    (0.008, 0.015, 0.025),
    'lid_drop': (0.05, 0.15, 0.30),
    'lid_ghost': (2, 6, 15),
    'lid_yaw_deg': (0.5, 1.5, 3.0),
    'lid_lat':  (0.05, 0.10, 0.20),
    'obs_dx':   (0.03, 0.06, 0.10),
    'obs_dy':   (0.02, 0.05, 0.08),
    'obs_rot_deg': (5, 12, 20),
    'cam_lat':  ((0.08, 0.15), (0.10, 0.25), (0.15, 0.40)),
    'cam_off':  (12, 20, 30),          # mm
    'cam_h':    (0.015, 0.03, 0.05),   # rad
    'cam_k':    (0.06, 0.12, 0.20),    # 1/m
    'cam_lost': (0.03, 0.08, 0.15),    # 1초당 놓침 시작 확률 x
    'cam_lost_len': ((0.2, 0.5), (0.3, 1.0), (0.4, 1.5)),
    'cam_lowc': (0.03, 0.08, 0.15),
    'cam_id_bad': (0.05, 0.15, 0.30),
    'fake_rate': (0.05, 0.20, 0.45),   # 바깥 차로에서 1초당 가짜 차로 시작 확률
    'fake_len': ((0.3, 0.6), (0.4, 1.2), (0.5, 2.0)),
    'start_y':  (0.04, 0.06, 0.09),
    'start_deg': (4, 6, 9),
}
LV = {'mild': 0, 'hard': 1, 'brutal': 2}
# 요인 묶음: --ablate 로 하나씩 이상값으로 꺼 보고 실패가 얼마나 줄어드는지 봄
GROUPS = {
    '차체(조향 gain·지연·트림·속도)': {'gain': 0.9, 'lag': 0.25, 'trim_deg': 0.0, 'vscale': 1.0},
    '명령 지연·주기 빠짐': {'cmd_lat': 0.0, 'cmd_skip': 0.0},
    'odom·자이로': {'odom_s': 0.0, 'odom_h': 1.12, 'gyro_b': 0.002, 'gyro_n': 0.01},
    '라이다': {'lid_n': 0.005, 'lid_drop': 0.0, 'lid_ghost': 0, 'lid_yaw_deg': 0.0, 'lid_lat': 0.0},
    '장애물 비뚤게 놓임': {'obs_dx': 0.0, 'obs_dy': 0.0, 'obs_rot_deg': 0},
    '카메라 지연·잡음': {'cam_lat': 0.0, 'cam_off': 8, 'cam_h': 0.01, 'cam_k': 0.03},
    '카메라 놓침·low_conf': {'cam_lost': 0.0, 'cam_lowc': 0.0},
    '카메라 차로 번호 틀림': {'cam_id_bad': 0.0},
    '가짜 차로(트랙 밖 흰 선)': {'fake_rate': 0.0},
    '시작 자세 오차': {'start_y': 0.03, 'start_deg': 3},
}


def pick(name, lv, r, off=None):
    if off and name in off:
        v = off[name]
        return r.uniform(*v) if isinstance(v, tuple) else v
    v = LEVELS[name][lv]
    return r.uniform(*v) if isinstance(v, tuple) else v


class Car:
    def __init__(self, x, y, th, c):
        self.x, self.y, self.th, self.d, self.v, self.w = x, y, th, 0.0, 0.0, 0.0
        self.ox, self.oy, self.oth = 0.0, 0.0, 0.0
        self.c = c

    def step(self, v, kappa, dt):
        c = self.c
        delta = max(-base.MAXD, min(base.MAXD, math.atan(base.WB * kappa))) if abs(v) >= 0.02 else 0.0
        delta = max(-base.MAXD, min(base.MAXD, delta + c['trim']))
        self.d += (delta - self.d) * min(1.0, dt / c['lag'])
        va = v * c['vscale']
        self.v = va
        self.w = va * math.tan(self.d) / base.WB * c['gain']
        self.x += va * math.cos(self.th) * dt
        self.y += va * math.sin(self.th) * dt
        self.th += self.w * dt
        vo = va * c['odom_s']
        self.ox += vo * math.cos(self.oth) * dt
        self.oy += vo * math.sin(self.oth) * dt
        self.oth += self.w * c['odom_h'] * dt


def scan(car, obstacles, c, r, step_deg=0.5, rmax=3.0, lidar_x=0.10):
    lx, ly = car.x + lidar_x * math.cos(car.th), car.y + lidar_x * math.sin(car.th)
    pts = []
    near = [o for o in obstacles if math.hypot(o[0][0] - lx, o[0][1] - ly) < rmax + 0.5]
    for i in range(int(360 / step_deg)):
        if r.random() < c['lid_drop']:
            continue
        a = math.radians(-180 + i * step_deg)
        dx, dy = math.cos(car.th + a), math.sin(car.th + a)
        best = rmax
        for poly in near:
            for j in range(4):
                t = base.seg_hit(lx, ly, dx, dy, *poly[j], *poly[(j + 1) % 4])
                if t is not None and t < best:
                    best = t
        if best < rmax:
            d = best + r.gauss(0, c['lid_n'])
            am = a - c['lid_yaw']             # 장착 각 오차: 차는 다른 각으로 안다
            pts.append((lidar_x + d * math.cos(am), d * math.sin(am)))
    for _ in range(r.randint(0, c['lid_ghost'])):   # 유령 점 (반사·먼지): 흩어진 낱점
        d, a = r.uniform(0.2, 2.5), r.uniform(-math.pi, math.pi)
        pts.append((lidar_x + d * math.cos(a), d * math.sin(a)))
    return pts


def car_poly(car):
    return base.rect(car.x + (FRONT - REAR) / 2 * math.cos(car.th), car.y + (FRONT - REAR) / 2 * math.sin(car.th),
                     FRONT + REAR, 2 * HALF, car.th)


def cam_msg(st, c, r, cam):
    """st = (x, y, th) 지연된 실제 자세. 카메라 차선 노드 출력 흉내."""
    x, y, th = st
    t = cam['t']
    if cam['lost_until'] > t:
        return {'status': 'NONE', 'source': 'hold', 'kappa': 0.0}
    if r.random() < c['cam_lost'] * 0.1:          # 0.1 s마다 부름
        cam['lost_until'] = t + r.uniform(*c['cam_lost_len'])
        return {'status': 'NONE', 'source': 'hold', 'kappa': 0.0}
    lane = max(-1, min(1, int(round(y / LW))))
    yc = lane * LW
    # 가짜 차로: 바깥 차로(±1)에서 트랙 밖 바닥을 흰 선으로 봄 -> 노란선과 그 선 사이를 차로로 착각
    if cam['fake_until'] > t and lane != 0:
        yc = lane * (LW + LW)                     # 한 칸 바깥 (트랙 밖) 을 차로 중심으로
        fake_id = -lane                          # 왼쪽 흰·오른쪽 노랑 = 반대 바깥 차로 모양
    else:
        fake_id = None
        if lane != 0 and r.random() < c['fake_rate'] * 0.1:
            cam['fake_until'] = t + r.uniform(*c['fake_len'])
    yin = y - yc
    if abs(math.degrees(th)) > 20 or (abs(yin) > 0.40 and fake_id is None):
        return {'status': 'NONE', 'source': 'hold', 'kappa': 0.0}
    status = 'BOTH' if (abs(yin) < 0.15 or fake_id is not None) and abs(math.degrees(th)) < 15 else r.choice(['L', 'R'])
    h = -th
    off = (-yin + 0.55 * math.sin(h)) / math.cos(h)
    xr = 0.5 * math.cos(-th) - (-yin) * math.sin(-th)
    yr = 0.5 * math.sin(-th) + (-yin) * math.cos(-th)
    kap = max(-2.1, min(2.1, 2 * yr / (xr * xr + yr * yr))) + r.gauss(0, c['cam_k'])
    lid = fake_id if fake_id is not None else lane
    if fake_id is None and r.random() < c['cam_id_bad']:
        lid = r.choice([k for k in (-1, 0, 1, None) if k != lane])
    return {'status': status, 'offset_mm': 1000 * off + r.gauss(0, c['cam_off']), 'source': 'lane',
            'heading_rad': h + r.gauss(0, c['cam_h']), 'lane_id': lid, 'kappa': kap,
            'low_conf': r.random() < c['cam_lowc']}


class GateReplay(M7Replay):
    """PR #117 이전: 카메라 차로 번호가 내부 추정과 같을 때만 차선 곡률."""

    def _cruise_cmd(self, t):
        ln = self.lane_msg
        if ln and ln.get('lane_id') not in (None, self.lane):
            from limo_obstacle_avoid.m7_replay_core import Track, _straight
            self.steer_src = 'pose'
            x, y, psi = self.pose()
            tr = Track([_straight(self.center(self.lane), x - 0.5, x + 3.0)], self.p, self.p.speed, self.p.speed)
            return tr.step(x, y, psi, t, True)
        return super()._cruise_cmd(t)


class SanityReplay(M7Replay):
    """절충안(시뮬레이션만): 한 칸 차이는 믿고, 두 칸 떨어진 차로 번호이거나 차로 중심이 30 cm 넘게 떨어졌다고 하면 곡률 안 씀."""

    def _cruise_cmd(self, t):
        ln = self.lane_msg
        if ln and ((ln.get('lane_id') is not None and abs(ln['lane_id'] - self.lane) >= 2)
                   or (ln.get('offset_mm') is not None and abs(ln['offset_mm']) > 300.0)):
            from limo_obstacle_avoid.m7_replay_core import Track, _straight
            self.steer_src = 'pose'
            x, y, psi = self.pose()
            tr = Track([_straight(self.center(self.lane), x - 0.5, x + 3.0)], self.p, self.p.speed, self.p.speed)
            return tr.step(x, y, psi, t, True)
        return super()._cruise_cmd(t)


def random_layout(r):
    xs, x = [], r.uniform(0.6, 1.3)
    while x < 4.1 and len(xs) < 3:
        xs.append(x)
        x += r.uniform(1.2, 1.9)
    out = []
    for x in xs:
        nb = r.choice([1, 2, 2])
        for k in r.sample([-1, 0, 1], nb):
            out.append((k, x + r.uniform(-0.1, 0.1)))
    return out


def run(args):
    idx, level, mode, left, right, trigger, layout, fork, verbose, off = args
    lv = LV[level]
    r = random.Random(1000 * idx + lv)
    c = {k: pick(k, lv, r, off) for k in LEVELS}
    c['cam_lost_len'], c['fake_len'] = LEVELS['cam_lost_len'][lv], LEVELS['fake_len'][lv]   # 구간 길이는 매번 뽑음
    c['trim'] = math.radians(r.uniform(-c['trim_deg'], c['trim_deg']))
    c['odom_s'] = 1.0 + r.uniform(-c['odom_s'], c['odom_s'])
    c['gyro_b'] = r.uniform(-c['gyro_b'], c['gyro_b'])
    c['lid_yaw'] = math.radians(r.uniform(-c['lid_yaw_deg'], c['lid_yaw_deg']))
    rr = random.Random(7 + 1000 * idx + lv)   # 실행 중 잡음 (두 방식이 같은 판을 보도록 판 조건은 r로 먼저 다 뽑음)
    obstacles = []
    for k, x in layout:
        obstacles.append(base.rect(x + base.OBS_L / 2 + r.uniform(-c['obs_dx'], c['obs_dx']),
                                   k * LW + r.uniform(-c['obs_dy'], c['obs_dy']), base.OBS_L, base.OBS_W,
                                   math.radians(r.uniform(-c['obs_rot_deg'], c['obs_rot_deg']))))
    p = ReplayParams(trigger_mm=trigger)
    p.exit_lane = fork
    island = p.section_length
    sy = r.uniform(-c['start_y'], c['start_y'])
    sth = math.radians(r.uniform(-c['start_deg'], c['start_deg']))
    car = Car(0.0, sy, sth, c)
    m = {'gate': GateReplay, 'sanity': SanityReplay}.get(mode, M7Replay)(p, left, right)
    t, dt = 0.0, 0.02
    for _ in range(60):
        t += dt
        m.on_imu(t, c['gyro_b'] + rr.gauss(0, c['gyro_n']))
        m.on_odom(t, car.ox, car.oy, car.oth, 0.0)
    m.start()
    hist = []          # (t, x, y, th)
    cmdq = []          # (t_apply, v, k)
    cur = (0.0, 0.0)
    cam = {'t': 0.0, 'lost_until': -1, 'fake_until': -1}
    k = 0
    min_clear, rev, n_ev = 9.0, 0.0, 0
    isl_hit = off_hit = 0
    n_lane = n_pose = 0
    pend_scan = []
    while t < 90.0 and m.state != 'DONE':
        t += dt
        k += 1
        hist.append((t, car.x, car.y, car.th))
        m.on_imu(t, car.w + c['gyro_b'] + rr.gauss(0, c['gyro_n']))
        if k % 2 == 0:
            m.on_odom(t, car.ox, car.oy, car.oth, car.v)
        if k % 5 == 0:
            pend_scan.append((t + c['lid_lat'], scan(car, obstacles, c, rr)))
        while pend_scan and pend_scan[0][0] <= t:
            m.on_scan_points(t, pend_scan.pop(0)[1])
        if k % 5 == 0:     # 카메라 10 Hz (실차 차선 노드 + YOLO 병목)
            tc = t - c['cam_lat']
            st = next(((x, y, th) for (tt, x, y, th) in reversed(hist) if tt <= tc), (car.x, car.y, car.th))
            cam['t'] = t
            m.on_lane(cam_msg(st, c, rr, cam), t)
        if rr.random() >= c['cmd_skip']:      # 주기 빠짐: 이번엔 계산 못 함 -> 이전 명령 유지
            v, kap = m.step(t)
            cmdq.append((t + c['cmd_lat'], v, kap))
            if m.state == 'CRUISE':
                n_lane += m.steer_src == 'lane'
                n_pose += m.steer_src == 'pose'
        while cmdq and cmdq[0][0] <= t:
            cur = cmdq.pop(0)[1:]
        car.step(cur[0], cur[1], dt)
        if car.v < 0:
            rev += -car.v * dt
        cp = car_poly(car)
        for o in obstacles:
            min_clear = min(min_clear, base.poly_dist(cp, o))
        if any(island <= qx <= island + 1.7 and abs(qy) < 0.25 for qx, qy in cp):
            isl_hit += 1
        if any(abs(qy) > EDGE for _, qy in cp):
            off_hit += 1
        if verbose:
            for e in m.events[n_ev:]:
                print(f'  {t:6.2f}s {e}')
            n_ev = len(m.events)
        if min_clear <= 0.003 or off_hit > 25:   # 부딪힘(3 mm 안) / 0.5 s 넘게 탈선 = 그 판 끝
            break
    hit = min_clear <= 0.003
    if hit:
        res = '충돌'
    elif off_hit > 25:
        res = '탈선'
    elif m.state != 'DONE':
        res = '시간초과'
    elif isl_hit:
        res = '섬침범'
    else:
        res = '완주'
    return {'idx': idx, 'res': res, 't': t, 'clear_cm': 100 * min_clear, 'off_n': off_hit, 'rev_m': rev,
            'lane': m.lane, 'fork_ok': m.lane == (1 if fork == 'A' else -1), 'exec': m.n_exec,
            'lane_pct': 100 * n_lane / max(1, n_lane + n_pose), 'layout': layout,
            'c': {kk: round(vv, 3) if isinstance(vv, float) else vv for kk, vv in c.items()}}


def model_paths():
    random.seed(99)
    left = base.record_left()
    return left, mirror(left), 700.0


def _ideal_ok(a):
    lay, fork, left, right, trig = a
    base.TRIGGER_MM = trig
    r = base.run(lay, left, right, seed=0, fork=fork)
    return r['ok'] and r['clear_cm'] > 0.5 and r['island'] == 0


def make_layouts(n, left, right, trig, pool):
    """고정 7개(x2 출구) + 무작위. 무작위는 노이즈 없는 m7_replay_sim에서 통과하는 것만 (통과 못 하는 배치 수도 셈)."""
    out = [(base.LAYOUTS[i % 7], 'A' if i < 7 else 'B') for i in range(min(n, 14))]
    bad, k = 0, 0
    while len(out) < n:
        cand = []
        for _ in range(2 * (n - len(out)) + 4):
            r = random.Random(50000 + k)
            k += 1
            cand.append((random_layout(r), 'A' if k % 2 else 'B'))
        oks = pool.map(_ideal_ok, [(L, f, left, right, trig) for L, f in cand])
        for (L, f), ok in zip(cand, oks):
            if len(out) >= n:
                break
            if ok:
                out.append((L, f))
            else:
                bad += 1
    return out, bad


def jobs(level, mode, lays, left, right, trig, verbose=False, only=None, off=None):
    return [(i, level, mode, left, right, trig, L, f, verbose, off)
            for i, (L, f) in enumerate(lays) if only is None or i == only]


def summarize(rs):
    from collections import Counter
    c = Counter(x['res'] for x in rs)
    n = len(rs)
    clear = sorted(x['clear_cm'] for x in rs)
    near = sum(1 for x in rs if 0 < x['clear_cm'] < 2.0)
    fork = sum(x['fork_ok'] for x in rs if x['res'] == '완주')
    lane = sum(x['lane_pct'] for x in rs) / max(1, n)
    return (f"완주 {c['완주']:3d}/{n} ({100 * c['완주'] / n:4.0f}%) | 충돌 {c['충돌']:3d} 탈선 {c['탈선']:3d} "
            f"섬 {c['섬침범']:2d} 시간초과 {c['시간초과']:2d} | 아슬(<2cm) {near:3d} | 간격 하위10% "
            f"{clear[max(0, n // 10 - 1)]:4.1f}cm | 출구맞음 {fork}/{c['완주']} | 평소 차선조향 {lane:3.0f}%")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--level', choices=list(LV), action='append')
    ap.add_argument('--mode', choices=['trust', 'gate', 'sanity'], action='append')
    ap.add_argument('-n', type=int, default=80)
    ap.add_argument('--one', type=int)
    ap.add_argument('-v', action='store_true')
    ap.add_argument('--fails', action='store_true', help='실패 판 목록')
    ap.add_argument('--ablate', action='store_true', help='요인 묶음을 하나씩 끄고 다시 (첫 level·mode로)')
    a = ap.parse_args()
    left, right, trig = model_paths()
    levels = a.level or ['mild', 'hard', 'brutal']
    modes = a.mode or ['trust', 'gate']
    with Pool() as pool:
        n = max(a.n, (a.one or 0) + 1)
        lays, bad = make_layouts(n, left, right, trig, pool)
        if a.one is not None:
            for lv in levels:
                for md in modes:
                    print(f'== {lv} / {md} / 판 {a.one}')
                    r = run(jobs(lv, md, lays, left, right, trig, a.v, a.one)[0])
                    print({k: v for k, v in r.items() if k != 'c'})
                    print(r['c'])
            return
        print(f'배치 {n}개 (고정 14 + 무작위 {n - 14}). 노이즈 없이도 못 지나가 뺀 무작위 배치 {bad}개')
        if a.ablate:
            lv, md = levels[0], modes[0]
            rs = pool.map(run, jobs(lv, md, lays, left, right, trig))
            print(f'{lv}/{md} 전부 켬          : {summarize(rs)}', flush=True)
            for g, off in GROUPS.items():
                rs = pool.map(run, jobs(lv, md, lays, left, right, trig, off=off))
                print(f'  - {g:24s} 끔: {summarize(rs)}', flush=True)
            return
        for lv in levels:
            for md in modes:
                rs = pool.map(run, jobs(lv, md, lays, left, right, trig))
                print(f'{lv:6s} {md:5s}: {summarize(rs)}', flush=True)
                if a.fails:
                    for x in rs:
                        if x['res'] != '완주':
                            print(f"    판 {x['idx']:3d} {x['res']} 간격 {x['clear_cm']:.1f}cm 차선조향 {x['lane_pct']:.0f}% "
                                  f"배치 {[(kk, round(xx, 2)) for kk, xx in x['layout']]}")


if __name__ == '__main__':
    main()
