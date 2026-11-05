"""라바콘 구간 닫힌 루프 시뮬레이터 (테스트·튜닝용, ROS 없음).

도면 좌표(mm, 좌하단 원점, x 동쪽, y 북쪽)에서
  M2 합류 끝(하단 차로, 서쪽 진행) -> 좌하단 코너 R640 -> 좌측 직선(라바콘 구간) -> 좌상단 코너
를 달린다. 코너까지는 '이상적인 차선 주행'(차로 중심선 Pure Pursuit)으로, 시작선 300mm 앞부터는
ConeFollower로, done 뒤에는 다시 차선 주행으로 넘긴다 (실제 FSM 순서와 같음).

실제와 비슷하게 넣은 것
  - 가상 2D 라이다 (0.5°, 10Hz, 거리 잡음), 콘 = 스캔 높이 단면 원
  - 조향 지연 0.3초, 실제 최소 회전 반지름 약 0.53m (명령 한계는 kappa 2.1)
  - IMU yaw는 도면 방향과 임의 각도만큼 돌아가 있음 (축 학습 오차도 넣을 수 있음)
결과: 차체 직사각형과 콘 바닥 원 사이 최소 여유(mm), 완료 위치, 궤적

그림으로 보기: python3 src/autorace_obstacle/test/cone_sim.py  (PNG가 현재 폴더에 생김)
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_obstacle.cone_core import (ConeFollower, ConeParams, Pose2D,  # noqa: E402
                                         PoseIntegrator, pursue, wrap)
from autorace_obstacle.scan_utils import scan_to_points  # noqa: E402

LANE_X = 720.0               # 좌측 차로 중심 (도면)
LINE_W, LINE_E = 470.0, 970.0  # 차로 양쪽 선 (대략)
START_Y, END_Y = 1820.0, 3920.0
CAR_REAR, CAR_FRONT, CAR_HALF_W = 61.0, 261.0, 110.0   # 뒤차축 기준 차체 (LIMO 322 x 220)


def centerline(step: float = 10.0) -> np.ndarray:
    pts = []
    for x in np.arange(1969.0, 1360.0, -step):                     # 하단, 서쪽으로
        pts.append((x, 940.0))
    for a in np.arange(-90.0, -180.0, -step / 640.0 * 180 / math.pi):  # 좌하단 코너 (우회전)
        r = math.radians(a)
        pts.append((1360.0 + 640.0 * math.cos(r), 1580.0 + 640.0 * math.sin(r)))
    for y in np.arange(1580.0, 4200.0, step):                      # 좌측 직선, 북쪽으로
        pts.append((LANE_X, y))
    for a in np.arange(180.0, 90.0, -step / 640.0 * 180 / math.pi):   # 좌상단 코너 (우회전)
        r = math.radians(a)
        pts.append((1360.0 + 640.0 * math.cos(r), 4200.0 + 640.0 * math.sin(r)))
    for x in np.arange(1360.0, 3000.0, step):
        pts.append((x, 4840.0))
    return np.array(pts)


def fake_scan(lidar: Pose2D, cones: np.ndarray, r_scan: float, rng: np.random.Generator,
              n: int = 720, noise: float = 5.0, extra: Optional[np.ndarray] = None,
              fov_deg: float = 360.0):
    """라이다 위치(도면 좌표)에서 본 원기둥들의 스캔. extra: [(x, y, r)] 콘 아닌 물체.

    fov_deg: 정면 기준 시야 (LIMO 라이다는 뒤쪽이 가려져 약 240도). 스캔 메시지도 그 범위만 담는다.
    """
    inc = 2 * math.pi / n
    fov_half = math.radians(fov_deg) / 2.0
    n = int(round(2 * fov_half / inc)) if fov_deg < 360.0 else n
    a_min = -fov_half if fov_deg < 360.0 else -math.pi
    angles = a_min + np.arange(n) * inc
    ranges = np.full(n, np.inf)
    objs = [(c[0], c[1], r_scan) for c in cones]
    if extra is not None:
        objs += [tuple(e) for e in extra]
    for x, y, r in objs:
        cx, cy = x - lidar.x, y - lidar.y
        cr, sr = math.cos(-lidar.yaw), math.sin(-lidar.yaw)
        c = np.array([cr * cx - sr * cy, sr * cx + cr * cy])
        dist = float(np.hypot(*c))
        if dist > 8000:
            continue
        ang = math.atan2(c[1], c[0])
        half = math.asin(min(1.0, r / dist)) if dist > r else math.pi
        diff = (angles - ang + math.pi) % (2 * math.pi) - math.pi
        idx = np.nonzero(np.abs(diff) <= half)[0]
        for i in idx:
            d = np.array([math.cos(angles[i]), math.sin(angles[i])])
            b = d @ c
            disc = b * b - (c @ c - r * r)
            if disc >= 0 and b > 0:
                ranges[i] = min(ranges[i], (b - math.sqrt(disc)) / 1000.0)
    ranges = ranges + rng.normal(0.0, noise / 1000.0, n)
    return ranges, a_min, inc


def footprint_clearance(car: Pose2D, cone_xy, base_r: float) -> float:
    c, s = math.cos(car.yaw), math.sin(car.yaw)
    dx, dy = cone_xy[0] - car.x, cone_xy[1] - car.y
    vx, vy = c * dx + s * dy, -s * dx + c * dy
    ex = max(-CAR_REAR - vx, 0.0, vx - CAR_FRONT)
    ey = max(-CAR_HALF_W - vy, 0.0, vy - CAR_HALF_W)
    return math.hypot(ex, ey) - base_r


@dataclass
class SimResult:
    traj: np.ndarray
    modes: List[str]
    min_clear: float
    clear_by_cone: List[float]
    done: bool
    done_y: Optional[float]
    exit_x_err: Optional[float]          # 끝선에서 차로 중심과의 x 차이 (mm)
    follower: ConeFollower
    corridor_path_track: Optional[np.ndarray] = None
    log: Dict[str, float] = field(default_factory=dict)


def run(cones_track, params: Optional[ConeParams] = None, imu_offset: float = 0.7,
        axis_err: float = 0.0, lane_err_mm: float = 0.0, seed: int = 0,
        r_scan: float = 30.0, base_r: float = 50.0, steer_delay: float = 0.30,
        lane_speed: float = 0.13, extra=None, drop_scan_prob: float = 0.0,
        t_max: float = 60.0, handoff_kappa: bool = True,
        lidar_yaw_err: float = 0.0, fov_deg: float = 240.0) -> SimResult:
    p = params or ConeParams()
    cones = np.asarray(cones_track, dtype=np.float64).reshape(-1, 2)
    rng = np.random.default_rng(seed)
    route = centerline()
    dt, lidar_x, L = 0.05, 100.0, 210.0
    car = Pose2D(1969.0, 940.0, math.pi)              # 도면 좌표, 서쪽을 봄
    fol = ConeFollower(p)
    integ = PoseIntegrator()
    integ.on_imu(wrap(car.yaw + imu_offset))
    integ.on_odom(0.0, 0.0)
    odom_xy = np.zeros(2)
    delay = deque([0.0] * max(1, int(round(steer_delay / dt))))
    mode, t, k_lane = 'lane', 0.0, 0.0
    traj, modes = [], []
    clear = [math.inf] * len(cones)
    done_y = None
    exit_err = None
    next_scan = 0.0
    while t < t_max:
        # --- 센서 ---
        integ.on_imu(wrap(car.yaw + imu_offset))
        integ.on_odom(odom_xy[0] / 1000.0, odom_xy[1] / 1000.0, 0.1)
        pose_w = Pose2D(integ.pose.x, integ.pose.y, integ.pose.yaw)
        if t >= next_scan:
            next_scan += 0.1
            if rng.random() >= drop_scan_prob:
                lid = Pose2D(car.x + lidar_x * math.cos(car.yaw),
                             car.y + lidar_x * math.sin(car.yaw), car.yaw + lidar_yaw_err)
                rr, amin, inc = fake_scan(lid, cones, r_scan, rng, extra=extra, fov_deg=fov_deg)
                pts = scan_to_points(rr, amin, inc, 0.05, 8.0, lidar_x, 0.0, 0.0)
                fol.on_scan(pts, pose_w, (lidar_x, 0.0))
        front_y = car.y + 200.0 * math.sin(car.yaw)
        front_x = car.x + 200.0 * math.cos(car.yaw)
        # --- 모드 전환 (FSM 흉내) ---
        if (mode == 'lane' and front_x < 1100 and front_y >= START_Y - 300.0
                and fol.corridor is None):
            axis = math.pi / 2 + imu_offset + axis_err
            # 차선 노드가 본 차로 중심점: BEV 맨 아래(뒤차축 550mm 앞) 행에서 차로 중심
            ax_ = car.x + 550.0 * math.cos(car.yaw)
            ay_ = car.y + 550.0 * math.sin(car.yaw)
            lane_pt_w = np.array([LANE_X + lane_err_mm, ay_ if ax_ < 1360 else 1580.0])
            c, s = math.cos(car.yaw), math.sin(car.yaw)
            d = lane_pt_w - np.array([car.x, car.y])
            lane_v = (c * d[0] + s * d[1], -s * d[0] + c * d[1])
            fol.start(pose_w, axis, lane_v, 'route',
                      kappa0=k_lane if handoff_kappa else 0.0, v0=lane_speed)
            mode = 'cones'
        if mode == 'cones' and fol.done:
            mode = 'lane'
            done_y = car.y
        if exit_err is None and front_y >= END_Y and front_x < 1100:
            exit_err = car.x - LANE_X
        # --- 제어 ---
        if mode == 'cones':
            v, k = fol.control(pose_w)
        else:
            v = lane_speed
            pred = Pose2D(car.x, car.y, car.yaw)
            k_raw, _, _ = pursue(route, pred, 650.0)
            k_lane += float(np.clip(k_raw - k_lane, -0.6, 0.6))
            k = float(np.clip(k_lane, -2.1, 2.1))
        delta_cmd = float(np.clip(math.atan(0.2 * k), -0.40, 0.40))
        delay.append(delta_cmd)
        delta = delay.popleft()
        k_act = math.tan(delta * 0.95) / (L / 1000.0)       # 실제: 명령보다 약 5% 덜 꺾임
        # --- 차 움직임 ---
        ds = v * 1000.0 * dt
        nxt = Pose2D(car.x, car.y, car.yaw)
        nxt.yaw = wrap(car.yaw + ds * k_act / 1000.0)
        ym = car.yaw + 0.5 * wrap(nxt.yaw - car.yaw)
        nxt.x = car.x + ds * math.cos(ym)
        nxt.y = car.y + ds * math.sin(ym)
        # 오도메트리는 드라이버 좌표(시작점 원점, 시작 방향 x)로
        rot = -math.pi
        odom_xy = odom_xy + ds * np.array([math.cos(ym + rot), math.sin(ym + rot)])
        car = nxt
        t += dt
        for i, cxy in enumerate(cones):
            clear[i] = min(clear[i], footprint_clearance(car, cxy, base_r))
        traj.append((car.x, car.y, car.yaw))
        modes.append(mode)
        if car.x > 1500 and car.y > 4500:
            break
    path_track = None
    if fol.corridor is not None and fol.plan is not None:
        # 구간 좌표 경로 -> 월드(적분) -> 도면 좌표
        pw = fol.corridor.to_w(fol.plan.path)
        integ0 = Pose2D(0.0, 0.0, wrap(math.pi + imu_offset))
        c0 = math.cos(-imu_offset)
        s0 = math.sin(-imu_offset)
        # 월드(적분) 원점 = 출발점 (1969, 940), 회전 = imu_offset
        rel = pw - np.array([integ0.x, integ0.y])
        path_track = np.stack([1969.0 + c0 * rel[:, 0] - s0 * rel[:, 1],
                               940.0 + s0 * rel[:, 0] + c0 * rel[:, 1]], axis=1)
    return SimResult(np.array(traj), modes, min(clear) if clear else math.inf, clear,
                     fol.done, done_y, exit_err, fol, path_track)


# ---------------------------------------------------------------------------
# 배치 예시 (도면 좌표). 통로 폭 = 콘 중심 간 거리
# ---------------------------------------------------------------------------
def corridor(width=500.0, shift=0.0, y0=2000.0, y1=3800.0, step=400.0, stagger=0.0,
             wave_amp=0.0, wave_len=2400.0, drop_left=(), drop_right=()):
    out = []
    ys = np.arange(y0, y1 + 1, step)
    for i, y in enumerate(ys):
        cx = LANE_X + shift + wave_amp * math.sin(2 * math.pi * (y - y0) / wave_len)
        if i not in drop_left:
            out.append((cx - width / 2, y))
        yr = y + stagger
        cxr = LANE_X + shift + wave_amp * math.sin(2 * math.pi * (yr - y0) / wave_len)
        if i not in drop_right and yr <= y1 + 1:
            out.append((cxr + width / 2, yr))
    return np.array(out)


SCENARIOS = {
    'straight_500': lambda: corridor(500),
    'narrow_440_shift_w': lambda: corridor(440, shift=-80),
    'wave_460': lambda: corridor(460, wave_amp=120, wave_len=2000),
    'staggered_480': lambda: corridor(480, stagger=200),
    'gap_left_480': lambda: corridor(480, drop_left=(2, 3)),
    'entry_gate_at_line': lambda: corridor(480, y0=1860, step=350),
    'slalom_center': lambda: np.array([(LANE_X, 2300.0), (LANE_X, 3000.0), (LANE_X, 3600.0)]),
}


def plot(name: str, res: SimResult, cones, path: str) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5, 7))
    ax.axhline(START_Y, color='0.6', lw=3)
    ax.axhline(END_Y, color='0.6', lw=3)
    for x in (LINE_W, LINE_E):
        ax.axvline(x, color='gold', lw=2, alpha=0.7)
    cl = centerline()
    ax.plot(cl[:, 0], cl[:, 1], color='0.8', lw=1, ls='--')
    for i, c in enumerate(cones):
        ax.add_patch(plt.Circle(c, 50, color='tab:orange'))
        ax.text(c[0] + 60, c[1], f'{res.clear_by_cone[i]:.0f}', fontsize=7)
    tr = res.traj
    cones_mode = np.array([m == 'cones' for m in res.modes])
    ax.plot(tr[:, 0], tr[:, 1], color='tab:blue', lw=1, label='lane')
    ax.plot(np.where(cones_mode, tr[:, 0], np.nan), np.where(cones_mode, tr[:, 1], np.nan),
            color='tab:red', lw=2, label='cones')
    if res.corridor_path_track is not None:
        ax.plot(res.corridor_path_track[:, 0], res.corridor_path_track[:, 1], color='tab:green',
                lw=1, label='plan')
    ax.set_xlim(200, 1500)
    ax.set_ylim(800, 4600)
    ax.set_aspect('equal')
    ax.set_title(f'{name}\nmin clear {res.min_clear:.0f} mm, done={res.done}, '
                 f'exit dx={res.exit_x_err if res.exit_x_err is None else round(res.exit_x_err)}',
                 fontsize=9)
    ax.legend(fontsize=7, loc='lower right')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    os.makedirs(out_dir, exist_ok=True)
    for name, make in SCENARIOS.items():
        cones = make()
        res = run(cones, axis_err=math.radians(2.0), lane_err_mm=30.0)
        print(f'{name:22s} min_clear={res.min_clear:6.0f}mm done={res.done} '
              f'done_y={res.done_y} exit_dx={res.exit_x_err} {res.follower.state_dict()}')
        plot(name, res, cones, os.path.join(out_dir, f'cone_sim_{name}.png'))
