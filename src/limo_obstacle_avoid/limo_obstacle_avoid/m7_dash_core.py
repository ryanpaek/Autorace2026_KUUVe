"""M7 점선 기준 회피 (11/19 실험, main 반영 전 단독 실행용). ROS 없음 — m7_dash_node가 입출력.

좌표 (구간 좌표): x = 시작선(커브 뒤 흰 가로선)부터 앞, y = 왼쪽 +, y = 0 이 1|2차로 사이 점선.
  1차로 = 오른쪽(안쪽, 중심 y = -0.26), 2차로 = 가운데(중심 +0.26). 왼쪽 끝 차로(+0.78)는 안 씀.

기본: 점선(y = 0) 위 직진. 방향 기준 = 시작 때 IMU 방향 (PoseFusion: odom 거리 + IMU yaw).
보조 보정 (조향은 안 건드리고 '내 위치 추정'만 고침 → 조향 판단끼리 안 싸움):
  - 라이다 장애물: 앞면이 다 보인 LIMO 중심 y → 차로 중심(±0.26)에 맞춤 (가장 믿음)
  - 라이다 장애물: 상자 방향 → 방향(psi) 보정 (#152 fit_box 그대로)
  - 카메라 점선: 예상 위치 ±dash_gate 안 흰 점들 → y (+점 펼침이 길면 방향). 장애물이 점선 근처에 보이면 안 씀
  - 노란선(/lane/status yellow_r_mm, 있으면): 오른쪽 노란선 y = yellow_r_y 로 y 약하게
회피: 장애물마다 '비킴 구간' [앞 - front - clear_x, 뒤 + rear + clear_x] 동안
  2차로 장애물 → 오른쪽 (y ≤ 안쪽 모서리 - 반폭 - clear), 1차로 장애물 → 왼쪽.
  구간 사이는 코사인 S자. 빈 칸이 2 x shift_len보다 짧으면 점선 복귀 없이 바로 반대쪽으로 (연속 회피 1→2→1).
  지금 장애물 옆이면 그 구간 쪽으로는 목표를 안 넘김 (먼저 꺾어 뒤 모서리 긁는 것 방지).
예외(나중에 채움): 같은 x에 양쪽 막힘 / 점선 위 장애물 → 천천히 다가가 정지. 후진 없음.
끝: x ≥ end_x → DONE (정지).
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from limo_obstacle_avoid.m7_waypoint_core import cluster, fit_box, median
from limo_obstacle_avoid.path_track import PoseFusion, wrap


@dataclass
class DashParams:
    lane_width: float = 0.52
    robot_front: float = 0.20
    robot_rear: float = 0.16
    robot_half_width: float = 0.115
    lidar_x: float = 0.10
    # 정차 LIMO: 가로·세로 둘 다 있음 → 모르면 긴 쪽(0.32)으로 봄
    obs_long: float = 0.32
    obs_short: float = 0.22
    # 회피
    clear: float = 0.04                 # 옆 여유 (장애물 안쪽 모서리 ~ 내 차 옆면)
    clear_x: float = 0.03               # 앞뒤 여유
    hold_rear: float = 0.05             # 장애물 뒤 모서리 지나 이만큼 더 가야 옆 유지 끝 (차 기준점 기준, 뒤 오버행 일부)
    shift_len: float = 0.70             # 점선 <-> 비킴 S자 길이
    max_offset: float = 0.20            # |목표 y| 상한
    # 주행
    speed: float = 0.20
    tight_speed: float = 0.14           # S자 빈 칸이 shift_len보다 짧을 때
    lookahead: float = 0.35
    max_curv: float = 1.0 / 0.53
    end_x: float = 4.6
    # 라이다
    cluster_gap: float = 0.08
    min_pts: int = 3
    max_range: float = 2.5
    edge_deg: float = 12.0
    band_y: float = 0.50                # |y| 이 안에 걸친 덩어리만 (오른 노란선 -0.52 ~ 왼쪽 점선 +0.52)
    straddle_y: float = 0.06            # 덩어리 중심 |y| 이 안이면 '점선 위' → 예외
    # 위치 보정
    snap_gain: float = 0.35
    snap_gate: float = 0.10
    snap_range: float = 1.4
    psi_gain: float = 0.15
    psi_gate_deg: float = 8.0
    dash_gain: float = 0.15
    dash_gate: float = 0.12
    dash_psi_gain: float = 0.05
    dash_min_pts: int = 4
    dash_min_span_mm: float = 150.0
    yellow_gain: float = 0.05
    yellow_gate: float = 0.10
    yellow_r_y: float = -0.52           # 오른쪽 노란선(안쪽 모서리) y — 실측으로 맞출 것
    # 안전
    stop_gap: float = 0.08              # 차 폭 안 앞 간격 이보다 작으면 정지
    creep_v: float = 0.10
    creep_stop: float = 0.30


def dash_points(bev_white, cfg, pred_mm, gate_mm, max_f_mm=800.0, band_px=6, max_w_mm=45.0):
    """BEV 흰색 마스크에서 점선 후보 [(전방 f_mm, 횡 x_mm 오른쪽 +)].
    pred_mm(f_mm): 그 전방 거리에서 점선 예상 횡위치. 예상 ±gate_mm 안, 폭 max_w_mm 이하 띠만 (옆 점선·LIMO 흰 부분·바닥 반사 거름)."""
    out = []
    h, w = bev_white.shape[:2]
    for r1 in range(h - 1, band_px - 1, -band_px):
        f = cfg.row_to_y_mm(r1 - band_px / 2)
        if f > max_f_mm:
            break
        xp = pred_mm(f)
        c0 = int(max(0, (xp - gate_mm) / cfg.mm_per_px_x + cfg.vehicle_col))
        c1 = int(min(w, (xp + gate_mm) / cfg.mm_per_px_x + cfg.vehicle_col + 1))
        if c1 - c0 < 2:
            continue
        cols = np.nonzero(bev_white[r1 - band_px + 1:r1 + 1, c0:c1].any(axis=0))[0]
        if len(cols) < 2 or (cols[-1] - cols[0] + 1) * cfg.mm_per_px_x > max_w_mm:
            continue
        out.append((f, cfg.col_to_x_mm(c0 + float(cols.mean()))))
    return out


class M7Dash:
    def __init__(self, p: DashParams):
        self.p = p
        self.fu = PoseFusion()
        self.offset = (0.0, 0.0, 0.0)
        self.state = 'IDLE'
        self.events: List[str] = []
        self.reset()

    def reset(self):
        self.mem: List[dict] = []
        self.holds: List[Tuple[float, float, float, int]] = []    # (a, b, y, side)
        self.block_x: Optional[float] = None
        self.front_gap = math.inf
        self.n_scan = 0
        self.last_v = 0.0
        self.lane_msg, self.lane_t = None, None
        self.why = ''
        self.n_fix = {'snap': 0, 'psi': 0, 'dash': 0, 'yellow': 0}

    # ---------- 위치 (#152와 같은 방식: 융합 위치 ∘ 보정 offset) ----------
    def pose(self) -> Tuple[float, float, float]:
        dx, dy, dpsi = self.offset
        c, s = math.cos(dpsi), math.sin(dpsi)
        fx, fy = self.fu.x, self.fu.y
        return dx + c * fx - s * fy, dy + s * fx + c * fy, wrap(self.fu.yaw + dpsi)

    def _shift(self, ddy=0.0, ddpsi=0.0):
        x, y, _ = self.pose()
        dx, dy, dpsi = self.offset
        dpsi2 = dpsi + ddpsi
        c, s = math.cos(dpsi2), math.sin(dpsi2)
        fx, fy = self.fu.x, self.fu.y
        self.offset = (x - (c * fx - s * fy), y + ddy - (s * fx + c * fy), dpsi2)

    def on_imu(self, t, gz):
        self.fu.on_imu(t, gz)

    def on_odom(self, t, x, y, yaw, v):
        self.fu.on_odom(t, x, y, yaw, v, self.last_v)

    def on_lane(self, msg: dict, t: float):
        self.lane_msg, self.lane_t = msg, t

    def start(self, start_y=0.0, start_heading_deg=0.0):
        self.reset()
        self.fu.x = self.fu.y = 0.0
        self.fu.s = self.fu.dist = 0.0
        self.offset = (0.0, start_y, math.radians(start_heading_deg) - self.fu.yaw)
        self.state = 'RUN'
        self.events.append(f'시작: 점선 기준, 끝 x={self.p.end_x:.2f} m, 옆 여유 {100 * self.p.clear:.0f} cm')

    def stop(self):
        self.state = 'IDLE'

    # ---------- 라이다 ----------
    def on_scan(self, t: float, pts: Sequence[Tuple[float, float, float]], a_min: float, a_max: float):
        """pts: 차 기준 (x 앞, y 왼쪽, 라이다 각도) 각도 순."""
        if self.state != 'RUN':
            return
        p = self.p
        self.n_scan += 1
        self.front_gap = min([x - p.robot_front for x, y, _ in pts
                              if abs(y) < p.robot_half_width + 0.03 and x > p.robot_front], default=math.inf)
        ex, ey, epsi = self.pose()
        edge = math.radians(p.edge_deg)
        cl = [c for c in cluster(pts, p.cluster_gap)
              if len(c) >= p.min_pts and not any(q[2] < a_min + edge or q[2] > a_max - edge for q in c)
              and min(math.hypot(q[0], q[1]) for q in c) < p.max_range]
        # 방향 보정: 상자 변 방향 (가로·세로 상관없이 변은 차로 축과 나란함)
        th = []
        for c in cl:
            b = fit_box(c, -epsi, p.obs_short, p.obs_long, p.lidar_x)
            if b is not None and b[3]:
                th.append(-b[2])
        if th:
            d = wrap(median(th) - epsi)
            if abs(d) < math.radians(p.psi_gate_deg):
                self._shift(ddpsi=p.psi_gain * d)
                self.n_fix['psi'] += 1
                ex, ey, epsi = self.pose()
        co, si = math.cos(epsi), math.sin(epsi)
        dets = []
        for c in cl:
            q = [(ex + x * co - y * si, ey + x * si + y * co) for x, y, _ in c]
            xs, ys = [a for a, _ in q], [b for _, b in q]
            d = {'x0': min(xs), 'x1': max(xs), 'y0': min(ys), 'y1': max(ys), 'ym': sum(ys) / len(ys),
                 'r': min(math.hypot(a[0], a[1]) for a in c)}
            if d['y0'] > p.band_y or d['y1'] < -p.band_y or d['x1'] < ex - 1.0:
                continue
            if d['x1'] - d['x0'] > 1.5:          # 벽처럼 긴 것 (예외: 붙어 있는 장애물 줄은 나중에)
                continue
            dets.append(d)
        # 옆 보정: 앞면이 다 보인 LIMO 중심 → 차로 중심 ±0.26
        res = []
        for d in dets:
            w = d['y1'] - d['y0']
            if d['r'] < p.snap_range and p.obs_short - 0.04 <= w <= p.obs_long + 0.04 and d['x0'] > ex + p.robot_front:
                yc = (d['y0'] + d['y1']) / 2
                r = yc - math.copysign(p.lane_width / 2, yc)
                if abs(r) < p.snap_gate:
                    res.append(r)
        if res:
            self._shift(ddy=-p.snap_gain * median(res))
            self.n_fix['snap'] += 1
            dy = -p.snap_gain * median(res)
            for d in dets:
                d['y0'] += dy
                d['y1'] += dy
                d['ym'] += dy
        # 기억
        for d in dets:
            side = 0 if abs(d['ym']) < p.straddle_y else (1 if d['ym'] > 0 else -1)
            w = d['y1'] - d['y0']
            m = next((o for o in self.mem if o['side'] == side and d['x0'] < o['x1'] + 0.15 and d['x1'] > o['x0'] - 0.15), None)
            if m is None:
                m = {'side': side, 'x0': d['x0'], 'x1': d['x1'], 'y0': d['y0'], 'y1': d['y1'], 'w': 0.0, 'n': 0, 'seen': t}
                self.mem.append(m)
            m['x0'] = min(m['x0'], d['x0'])
            m['x1'] = max(m['x1'], d['x1'])
            m['y0'] = min(m['y0'], d['y0']) if m['n'] else d['y0']
            m['y1'] = max(m['y1'], d['y1']) if m['n'] else d['y1']
            if d['x0'] > ex + p.robot_front:      # 앞면 폭 = 가로/세로 판단
                m['w'] = max(m['w'], w)
            m['n'] += 1
            m['seen'] = t
        self.mem = [o for o in self.mem if o['n'] >= 2 or t - o['seen'] < 0.3]
        self._replan()

    # ---------- 계획 ----------
    def _hold_of(self, o):
        p = self.p
        half = p.lane_width / 2
        if o['w'] < 0.15:                         # 앞면 못 봄 → 둘 다 긴 쪽으로 (보수)
            across = along = p.obs_long
        elif o['w'] < (p.obs_short + p.obs_long) / 2:   # 앞면 좁음 = 세로로 놓임
            across, along = p.obs_short, p.obs_long
        else:                                     # 앞면 넓음 = 가로로 놓임
            across, along = p.obs_long, p.obs_short
        x1 = max(o['x1'], o['x0'] + along)
        a = o['x0'] - p.robot_front - p.clear_x
        b = x1 + p.hold_rear + p.clear_x
        if o['side'] > 0:
            edge = min(o['y0'], half - across / 2)
            y = edge - p.robot_half_width - p.clear
        else:
            edge = max(o['y1'], -half + across / 2)
            y = edge + p.robot_half_width + p.clear
        return a, b, max(-p.max_offset, min(p.max_offset, y))

    def _replan(self):
        p = self.p
        hs = []
        self.block_x = None
        for o in sorted(self.mem, key=lambda o: o['x0']):
            if o['n'] < 2:
                continue
            if o['side'] == 0:
                self.block_x = o['x0'] if self.block_x is None else min(self.block_x, o['x0'])
                continue
            a, b, y = self._hold_of(o)
            hs.append([a, b, y, o['side']])
        hs.sort()
        out = []
        for h in hs:
            if out and h[0] <= out[-1][1]:
                q = out[-1]
                if q[3] != h[3]:                    # 같은 x에 양쪽 → 길 없음 (예외)
                    bx = h[0] + p.robot_front + p.clear_x
                    self.block_x = bx if self.block_x is None else min(self.block_x, bx)
                    continue
                q[1] = max(q[1], h[1])
                q[2] = min(q[2], h[2]) if h[3] > 0 else max(q[2], h[2])
            else:
                out.append(h)
        self.holds = [tuple(h) for h in out]

    def y_ref(self, x: float) -> float:
        """계획 옆 위치. 비킴 구간 사이는 코사인 S자, 빈 칸 짧으면 점선 복귀 없이 바로."""
        L = self.p.shift_len
        hs = self.holds
        if not hs:
            return 0.0

        def ramp(x, x0, x1, y0, y1):
            u = max(0.0, min(1.0, (x - x0) / max(x1 - x0, 1e-6)))
            return y0 + (y1 - y0) * (1 - math.cos(math.pi * u)) / 2

        for i, (a, b, y, _) in enumerate(hs):
            if a <= x <= b:
                return y
            prev_b = hs[i - 1][1] if i else -math.inf
            prev_y = hs[i - 1][2] if i else 0.0
            if prev_b < x < a:
                gap = a - prev_b
                if i and gap < 2 * L:               # 연속 회피: 바로 반대쪽(또는 같은 쪽)으로
                    return ramp(x, prev_b, a, prev_y, y)
                if x >= a - L:
                    return ramp(x, a - L, a, 0.0, y)
                if i and x <= prev_b + L:
                    return ramp(x, prev_b, prev_b + L, prev_y, 0.0)
                return 0.0
        a, b, y, _ = hs[-1]
        return ramp(x, b, b + L, y, 0.0)

    def _tight(self, x):
        for i in range(1, len(self.holds)):
            pb, a = self.holds[i - 1][1], self.holds[i][0]
            if pb - 0.3 < x < a and a - pb < self.p.shift_len:
                return True
        return False

    # ---------- 카메라 점선 / 노란선 (위치 보정만) ----------
    def dash_pred_mm(self, f_mm: float) -> float:
        """BEV 전방 f_mm에서 점선이 있을 BEV 횡위치(mm, 오른쪽 +)."""
        _, y, psi = self.pose()
        f = f_mm / 1000.0
        l = -(y + f * math.sin(psi)) / max(math.cos(psi), 0.5)
        return -1000.0 * l

    def _dash_blocked(self):
        x, _, _ = self.pose()
        return any(o['x1'] > x and o['x0'] < x + 1.3 for o in self.mem)

    def on_dash(self, pts_mm: Sequence[Tuple[float, float]]):
        """pts_mm: 점선 후보 (전방 f_mm, 횡 x_mm 오른쪽 +) — 노드가 예상 위치 ±dash_gate 안만 넘김."""
        p = self.p
        if self.state != 'RUN' or len(pts_mm) < p.dash_min_pts or self._dash_blocked():
            return
        _, y, psi = self.pose()
        ys = []
        for f, xm in pts_mm:
            f, l = f / 1000.0, -xm / 1000.0
            ys.append(-(f * math.sin(psi) + l * math.cos(psi)))   # 이 점이 점선(y=0)이라면 내 y
        r = median(ys) - y
        if abs(r) > p.dash_gate:
            return
        self._shift(ddy=p.dash_gain * r)
        self.n_fix['dash'] += 1
        fs = [f for f, _ in pts_mm]
        if max(fs) - min(fs) >= p.dash_min_span_mm and p.dash_psi_gain > 0:
            n = len(pts_mm)
            mf, mx = sum(fs) / n, sum(xm for _, xm in pts_mm) / n
            sxx = sum((f - mf) ** 2 for f in fs)
            slope = sum((f - mf) * (xm - mx) for f, xm in pts_mm) / sxx     # 오른쪽 + / 앞
            d = wrap(math.atan(slope) - psi)       # 선이 앞으로 갈수록 오른쪽 = 내가 왼쪽을 봄 (psi +)
            if abs(d) < math.radians(p.psi_gate_deg):
                self._shift(ddpsi=p.dash_psi_gain * d)

    def _yellow_fix(self, t):
        p = self.p
        ln = self.lane_msg
        if p.yellow_gain <= 0 or not ln or self.lane_t is None or t - self.lane_t > 0.3:
            return
        d = ln.get('yellow_r_mm')
        if d is None:
            return
        f = float(ln.get('yellow_f_mm') or 575.0) / 1000.0
        _, y, psi = self.pose()
        y_m = p.yellow_r_y + float(d) / 1000.0 * math.cos(psi) - f * math.sin(psi)
        if abs(y_m - y) < p.yellow_gate:
            self._shift(ddy=p.yellow_gain * (y_m - y))
            self.n_fix['yellow'] += 1
        self.lane_msg = None                      # 한 메시지는 한 번만

    # ---------- 주기 (20 Hz) ----------
    def step(self, t: float) -> Tuple[float, float]:
        p = self.p
        if self.state != 'RUN':
            return 0.0, 0.0
        self._yellow_fix(t)
        x, y, psi = self.pose()
        if x >= p.end_x:
            self.state = 'DONE'
            self.events.append(f'[x={x:.2f}] 끝')
            return self._out(0.0, 0.0, '끝')
        if self.n_scan < 2:
            return self._out(0.0, 0.0, '라이다 기다림')
        v = p.tight_speed if self._tight(x + 0.3) else p.speed
        why = '점선 주행' if abs(self.y_ref(x + p.lookahead)) < 0.01 else '회피'
        # 예외: 길 없음 → 다가가서 정지
        if self.block_x is not None and self.block_x - x < 1.2:
            if self.front_gap > p.creep_stop:
                v, why = p.creep_v, '길 없음 -> 천천히'
            else:
                v, why = 0.0, '길 없음 -> 정지'
        yt = self.y_ref(x + p.lookahead)
        for a, b, yh, side in self.holds:        # 장애물 옆이면 그쪽으로 안 꺾음
            if a <= x <= b:
                yt = min(yt, yh) if side > 0 else max(yt, yh)
        dx, dy = p.lookahead, yt - y
        c, s = math.cos(psi), math.sin(psi)
        xr, yr = dx * c + dy * s, -dx * s + dy * c
        k = 2 * yr / (xr * xr + yr * yr)
        if self.front_gap < p.stop_gap and v > 0:
            v, why = 0.0, f'앞 {100 * self.front_gap:.0f} cm 정지'
        return self._out(v, max(-p.max_curv, min(p.max_curv, k)), why)

    def _out(self, v, k, why):
        if why != self.why:
            x, y, _ = self.pose()
            self.events.append(f'[x={x:.2f} y={100 * y:+.0f}cm] {why}')
        self.why = why
        self.last_v = v
        return v, k

    def status(self) -> dict:
        x, y, psi = self.pose()
        return {'state': self.state, 'impl': 'dash', 'why': self.why,
                'x': round(x, 3), 'y': round(y, 3), 'psi_deg': round(math.degrees(psi), 1),
                'y_ref': round(self.y_ref(x), 3),
                'holds': [[round(a, 2), round(b, 2), round(yy, 3), s] for a, b, yy, s in self.holds],
                'block_x': None if self.block_x is None else round(self.block_x, 2),
                'mem': [[round(o['x0'], 2), round(o['x1'], 2), round(o['y0'], 2), round(o['y1'], 2), o['side'], o['n']]
                        for o in self.mem],
                'front_gap_mm': None if not math.isfinite(self.front_gap) else round(1000 * self.front_gap),
                'fix': dict(self.n_fix)}
