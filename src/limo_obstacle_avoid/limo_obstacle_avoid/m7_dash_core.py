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
안전: 지금 곡률 + 조향 지연(steer_delay) 동안 낸 곡률로 차 외곽선을 굴려 라이다 점에 닿으면 정지.
멈추면(recover_wait): 기억 비우고 0.4 s 다시 봄 → 길 있으면 감 / 다른 곡률로 비켜 감(서서 바퀴 먼저) /
  곧게 후진 15 cm(최대 3번). 같은 자리에서 또 멈추면 계획 여유를 1 cm씩 줄임(clear_min까지, 충돌 검사 여유는 안 줄임).
  0.5 m 나아가면 원래대로. 비켜 가기는 차체 |y| ≤ 0.44, 방향 ≤ 20° 안에서만 (노란선·3차로 침범 방지).
예외(나중에 채움): 같은 x에 양쪽 막힘 / 점선 위 장애물 → 다가가 정지 후 재판단 반복.
끝: 표지판(fork_a/b) 상자 높이 ≥ end_sign_h_px가 end_sign_frames 연속 + x ≥ end_min_x. 못 보면 x ≥ end_x에서 끝.
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
    clear: float = 0.05                 # 옆 여유 (장애물 안쪽 모서리 ~ 내 차 옆면)
    clear_x: float = 0.03               # 앞뒤 여유
    hold_rear: float = 0.05             # 장애물 뒤 모서리 지나 이만큼 더 가야 옆 유지 끝 (차 기준점 기준, 뒤 오버행 일부)
    shift_len: float = 0.70             # 점선 <-> 비킴 S자 길이
    max_offset: float = 0.20            # |목표 y| 상한
    # 주행
    speed: float = 0.20
    tight_speed: float = 0.14           # S자 빈 칸이 shift_len보다 짧을 때
    lookahead: float = 0.25
    latency: float = 0.2                # 조향 지연 보상: 이만큼(s) 뒤 위치로 pure pursuit
    max_curv: float = 1.0 / 0.53
    max_y: float = 0.25                 # |추정 y| 상한 (차 옆면 0.365 < 노란선·3차로 경계 0.52)
    # 끝: 표지판(fork_a/b) 상자 높이 >= end_sign_h_px 가 end_sign_frames 연속, x >= end_min_x 일 때.
    #     end_x = 거리 상한 (표지판 못 봐도 여기서 끝)
    end_by_sign: bool = True
    end_sign_h_px: float = 60.0         # 실측해서 맞출 것 (/m7dash/state sign_h 보고)
    end_sign_frames: int = 3
    end_min_x: float = 2.5
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
    # 안전: 지금 곡률로 차 외곽선을 굴려(arc) 라이다 점에 닿는 거리
    stop_gap: float = 0.08              # 굴린 경로 이 거리 안에 닿으면 정지
    path_margin: float = 0.015
    creep_v: float = 0.10
    creep_stop: float = 0.30
    # 멈추면 재판단 → 다시 감
    recover_wait: float = 0.8           # 이만큼 멈춰 있으면 재판단 (기억 비우고 다시 봄)
    escape_need: float = 0.30           # 다른 곡률로 이만큼 안 닿고 갈 수 있으면 그 곡률로 천천히
    escape_dist: float = 0.30
    recover_rev_max: int = 3            # 그래도 못 가면 곧게 후진 (0 = 후진 안 함)
    recover_rev_dist: float = 0.15
    recover_rev_v: float = 0.10
    relax_step: float = 0.01            # 같은 자리에서 재판단할 때마다 여유를 이만큼 줄임
    clear_min: float = 0.015            # 줄여도 이 아래로는 안 감 (옆·앞뒤 여유)
    stop_gap_min: float = 0.05
    resume_extra: float = 0.03          # 멈춘 뒤 다시 가려면 stop_gap + 이만큼 비어야 (떨림 방지)
    steer_delay: float = 0.35           # 조향 반응 지연: 이 시간 동안 낸 곡률 전부로 굴려 봄, 비키기·후진 전엔 이만큼 서서 바퀴 먼저
    progress_reset: float = 0.5         # 이만큼 나아가면 여유·후진 횟수 원래대로
    esc_max_deg: float = 20.0           # 비켜 갈 때 방향 상한
    body_max_y: float = 0.44            # 비켜 간 뒤 차체 모서리 |y| 상한 (노란선·3차로 경계 0.52 - 8 cm)


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
        self.pts: List[Tuple[float, float]] = []
        self.n_scan = 0
        self.last_v = 0.0
        self.last_k = 0.0
        self.t_last = None
        self.stop_t = 0.0
        self.esc = None                           # {'k', 'left'} 탈출 곡률 주행 중
        self.rev = None                           # 남은 후진 거리
        self.rev_wait = 0.0
        self.n_rev = 0
        self.relax = 0
        self.k_hist: List[Tuple[float, float]] = []
        self.halted = False
        self.rj_x = None                          # 마지막 재판단 위치
        self.n_rejudge = 0
        self.sign_h = None
        self.sign_n = 0
        self.end_why = ''
        self.rj_t = None
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
        p = self.p
        self.events.append('시작: 점선 기준, 끝 = ' + (f'표지판 {p.end_sign_h_px:.0f} px (x {p.end_min_x:.1f} m 뒤) / ' if p.end_by_sign else '')
                           + f'거리 {p.end_x:.2f} m, 옆 여유 {100 * p.clear:.0f} cm')

    def stop(self):
        self.state = 'IDLE'

    # ---------- 라이다 ----------
    def on_scan(self, t: float, pts: Sequence[Tuple[float, float, float]], a_min: float, a_max: float):
        """pts: 차 기준 (x 앞, y 왼쪽, 라이다 각도) 각도 순."""
        if self.state != 'RUN':
            return
        p = self.p
        self.n_scan += 1
        self.pts = [(x, y) for x, y, _ in pts if -0.6 < x < 1.0 and abs(y) < 0.7]
        self.front_gap = self.arc_clear(0.0)
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
            # 앞면이 보이면 최신값 (odom 배율 오차가 min으로 쌓이지 않게), 옆·뒤에서 보면 앞면 아님 → 기존 유지
            m['x0'] = d['x0'] if d['x0'] > ex + p.robot_front else min(m['x0'], d['x0'])
            m['x1'] = max(m['x1'], d['x1'])
            # 옆 모서리는 최신값: 지금 위치 추정과 같은 기준이라 위치 오차가 상쇄됨 (min 누적하면 순간 오차가 굳어 과하게 비킴)
            m['y0'], m['y1'] = d['y0'], d['y1']
            if d['x0'] > ex + p.robot_front:      # 앞면 폭 = 가로/세로 판단
                m['w'] = max(m['w'], w)
            m['n'] += 1
            m['seen'] = t
        self.mem = [o for o in self.mem if o['n'] >= 2 or t - o['seen'] < 0.3]
        self._replan()

    # ---------- 계획 ----------
    def _eff(self, v, lo):
        return max(lo, v - self.relax * self.p.relax_step) if v > lo else v

    def _hold_of(self, o):
        p = self.p
        clear, clear_x, hold_rear = (self._eff(p.clear, p.clear_min), self._eff(p.clear_x, 0.0),
                                     self._eff(p.hold_rear, 0.0))
        half = p.lane_width / 2
        if o['w'] < 0.15:                         # 앞면 못 봄 → 둘 다 긴 쪽으로 (보수)
            across = along = p.obs_long
        elif o['w'] < (p.obs_short + p.obs_long) / 2:   # 앞면 좁음 = 세로로 놓임
            across, along = p.obs_short, p.obs_long
        else:                                     # 앞면 넓음 = 가로로 놓임
            across, along = p.obs_long, p.obs_short
        x1 = max(o['x1'], o['x0'] + along)
        a = o['x0'] - p.robot_front - clear_x
        b = x1 + hold_rear + clear_x
        if o['side'] > 0:
            edge = min(o['y0'], half - across / 2)
            y = edge - p.robot_half_width - clear
        else:
            edge = max(o['y1'], -half + across / 2)
            y = edge + p.robot_half_width + clear
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

    # ---------- 표지판 (끝) ----------
    def on_sign(self, h_px: Optional[float]):
        """detector 한 프레임: fork_a/fork_b 상자 높이(px) 최대값, 못 봤으면 None."""
        self.sign_h = h_px
        self.sign_n = self.sign_n + 1 if h_px is not None and h_px >= self.p.end_sign_h_px else 0

    # ---------- 굴려 보기: 곡률 k로 앞으로 갈 때 처음 닿는 거리 ----------
    def arc_clear(self, k: float, dist: float = 0.6) -> float:
        p = self.p
        m = p.path_margin
        f, r, hw = p.robot_front + m, p.robot_rear + m, p.robot_half_width + m
        best = math.inf
        for px, py in self.pts:
            if -r <= px <= f and abs(py) <= hw:
                if px > p.robot_front - 0.02:     # 이미 앞 범퍼에 걸림
                    return 0.0
                continue                          # 옆·뒤에 이미 붙은 점은 앞으로 가도 안 나아짐 -> 무시
            s = 0.02
            while s <= min(dist, best):
                th = k * s
                if abs(k) < 1e-6:
                    cx, cy = s, 0.0
                else:
                    cx, cy = math.sin(th) / k, (1 - math.cos(th)) / k
                c, sn = math.cos(th), math.sin(th)
                u, v = (px - cx) * c + (py - cy) * sn, -(px - cx) * sn + (py - cy) * c
                if -r <= u <= f and abs(v) <= hw:
                    best = s
                    break
                s += 0.02
        return best

    def _escape(self, x, y, psi):
        """멈춘 자리에서 안 닿고 escape_need 이상 갈 수 있는 곡률 중 계획 y에 가장 가까운 것."""
        p = self.p
        d = p.escape_dist
        best = None
        for i in range(11):
            k = -p.max_curv + 2 * p.max_curv * i / 10
            if self.arc_clear(k, p.escape_need + 0.05) < p.escape_need:
                continue
            ye = y + (d * math.sin(psi) if abs(k) < 1e-6 else (math.cos(psi) - math.cos(psi + k * d)) / k)
            th = psi + k * d
            body = abs(ye) + p.robot_half_width * math.cos(th) + p.robot_front * abs(math.sin(th))
            if abs(ye) > p.max_y or abs(th) > math.radians(p.esc_max_deg) or body > p.body_max_y:
                continue
            sc = abs(ye - self.y_ref(x + d))
            if best is None or sc < best[0]:
                best = (sc, k)
        return None if best is None else best[1]

    # ---------- 주기 (20 Hz) ----------
    def step(self, t: float) -> Tuple[float, float]:
        p = self.p
        dt = 0.05 if self.t_last is None else max(1e-3, min(0.2, t - self.t_last))
        self.t_last = t
        if self.state != 'RUN':
            return 0.0, 0.0
        self._yellow_fix(t)
        x, y, psi = self.pose()
        if p.end_by_sign and x >= p.end_min_x and self.sign_n >= p.end_sign_frames:
            self.end_why = f'표지판 가까움 (상자 {self.sign_h:.0f} px)'
        elif x >= p.end_x:
            self.end_why = '거리 상한' + (' (표지판 못 봄)' if p.end_by_sign else '')
        if self.end_why:
            self.state = 'DONE'
            self.events.append(f'[x={x:.2f}] 끝: {self.end_why}')
            return self._out(0.0, 0.0, '끝')
        if self.n_scan < 2:
            return self._out(0.0, 0.0, '라이다 기다림')
        # 재판단 중: 기억 비우고 0.4 s 새로 봄 (그동안 정지)
        if self.rj_t is not None:
            if t - self.rj_t < 0.4:
                return self._out(0.0, 0.0, '재판단: 다시 보는 중')
            self.rj_t = None
            self.stop_t = 0.0
            v, k, why = self._plan_cmd(x, y, psi)
            if v > 0:
                self.events.append(f'[x={x:.2f}] 재판단: 길 있음 -> 다시 감')
            elif self.block_x is not None and self.block_x - x < 1.2:
                self.events.append(f'[x={x:.2f}] 재판단: 양쪽 막힘 그대로 -> {p.recover_wait:.1f} s 뒤 또 봄')
            else:
                ke = self._escape(x, y, psi)
                if ke is not None:
                    self.esc = {'k': ke, 'left': p.escape_dist, 'wait': p.steer_delay}
                    self.events.append(f'[x={x:.2f}] 재판단: 곡률 {ke:+.2f}로 {100 * p.escape_dist:.0f} cm 비켜 감')
                elif self.n_rev < p.recover_rev_max:
                    self.rev = p.recover_rev_dist
                    self.rev_wait = p.steer_delay
                    self.n_rev += 1
                    self.events.append(f'[x={x:.2f}] 재판단: 비킬 곳 없음 -> 곧게 후진 {100 * p.recover_rev_dist:.0f} cm '
                                       f'({self.n_rev}/{p.recover_rev_max})')
                else:
                    self.events.append(f'[x={x:.2f}] 재판단: 후진 다 씀 -> 대기 후 또 봄')
        # 후진 (곧게, 지나온 길)
        if self.rev is not None:
            if self.rev_wait > 0:                 # 바퀴 먼저 곧게
                self.rev_wait -= dt
                return self._out(0.0, 0.0, '후진 준비 (바퀴 곧게)')
            self.rev -= p.recover_rev_v * dt
            if self.rev <= 0:
                self.rev = None
                self._rejudge(t, '후진 끝')
            return self._out(-p.recover_rev_v, 0.0, '후진')
        # 탈출 곡률
        if self.esc is not None:
            if self.esc['wait'] > 0:              # 서서 바퀴 먼저 돌림
                self.esc['wait'] -= dt
                return self._out(0.0, self.esc['k'], '비켜 가기 준비 (바퀴 먼저)')
            if self.arc_clear(self.esc['k']) < self._eff(p.stop_gap, p.stop_gap_min):
                self.esc = None
            else:
                k = self.esc['k']
                self.esc['left'] -= p.creep_v * dt
                if self.esc['left'] <= 0:
                    self.esc = None
                return self._out(p.creep_v, k, '비켜 가는 중')
        if self.rj_x is not None and x > self.rj_x + p.progress_reset:
            self.events.append(f'[x={x:.2f}] {100 * p.progress_reset:.0f} cm 나아감 -> 여유·후진 횟수 원래대로')
            self.rj_x, self.relax, self.n_rev = None, 0, 0
        v, k, why = self._plan_cmd(x, y, psi)
        self.stop_t = self.stop_t + dt if v <= 0 else 0.0
        if self.stop_t >= p.recover_wait:
            self._rejudge(t, why)
        return self._out(v, k, why)

    def _rejudge(self, t, why):
        x, _, _ = self.pose()
        self.n_rejudge += 1
        if self.rj_x is not None and x < self.rj_x + self.p.progress_reset:
            self.relax += 1
        self.rj_x = x if self.rj_x is None else max(self.rj_x, x)
        why += f', 여유 -{self.relax} cm' if self.relax else ''
        self.mem, self.holds, self.block_x = [], [], None
        self.rj_t = t
        self.stop_t = 0.0
        self.events.append(f'[x={x:.2f}] 재판단 {self.n_rejudge} ({why}): 기억 비우고 다시 봄')

    def _plan_cmd(self, x, y, psi):
        p = self.p
        d = self.last_v * p.latency               # 조향 지연 보상: latency 뒤 위치에서 pure pursuit
        if d > 0:
            x += d * math.cos(psi + self.last_k * d / 2)
            y += d * math.sin(psi + self.last_k * d / 2)
            psi += self.last_k * d
        v = p.tight_speed if self._tight(x + 0.3) else p.speed
        yt = self.y_ref(x + p.lookahead)
        why = '점선 주행' if abs(yt) < 0.01 else '회피'
        for a, b, yh, side in self.holds:        # 장애물 옆이면 그쪽으로 안 꺾음
            if a <= x <= b:
                yt = min(yt, yh) if side > 0 else max(yt, yh)
        yt = max(-p.max_offset, min(p.max_offset, yt))
        dx, dy = p.lookahead, yt - y
        c, s = math.cos(psi), math.sin(psi)
        xr, yr = dx * c + dy * s, -dx * s + dy * c
        k = max(-p.max_curv, min(p.max_curv, 2 * yr / (xr * xr + yr * yr)))
        ks = [kk for tt, kk in self.k_hist if self.t_last is not None and self.t_last - tt <= p.steer_delay]
        gap = min(self.arc_clear(kk) for kk in {k, min(ks + [k]), max(ks + [k])})
        self.front_gap = gap
        if self.block_x is not None and self.block_x - x < 1.2:    # 예외: 길 없음 -> 다가가서 정지
            v, why = (p.creep_v, '길 없음 -> 천천히') if gap > p.creep_stop else (0.0, '길 없음 -> 정지')
        sg = self._eff(p.stop_gap, p.stop_gap_min) + (p.resume_extra if self.halted else 0.0)
        self.halted = gap < sg
        if self.halted:
            v, why = 0.0, f'굴려 보니 {100 * gap:.0f} cm 뒤 닿음 -> 정지'
        return v, k, why

    def _out(self, v, k, why):
        if why != self.why:
            x, y, _ = self.pose()
            self.events.append(f'[x={x:.2f} y={100 * y:+.0f}cm] {why}')
        self.why = why
        self.last_v, self.last_k = v, k
        self.k_hist.append((self.t_last or 0.0, k))
        if len(self.k_hist) > 12:
            self.k_hist.pop(0)
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
                'fix': dict(self.n_fix), 'sign_h': self.sign_h, 'sign_n': self.sign_n, 'end': self.end_why,
                'n_rejudge': self.n_rejudge, 'n_rev': self.n_rev, 'relax_cm': self.relax}
