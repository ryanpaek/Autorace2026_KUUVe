"""M7 3차선 정적 장애물 — 정차 LIMO 기준 웨이포인트 방식 (11/19, 시뮬 'C' 방식 이식).

녹화 경로 재생(m7_replay_core)과 따로 둔 실험용 구현. launch m7_impl:=waypoint 로만 켜진다.
ROS 없음 (m7_waypoint_node가 입출력).

흐름 (라이다 한 바퀴마다)
  1) 덩어리 나누기 → L자/한 변 맞추기(closeness) → 정차 LIMO 중심·방향 (차 기준)
     - 시야 끝(edge_deg 안)에 걸린 덩어리는 버림 (일부만 보여 중심이 틀어짐)
  2) 위치 보정 (지금 믿는 위치 = PoseFusion(odom 거리 + IMU yaw) ∘ offset)
     - 방향: 긴 변 또는 뒷면이 보이는 LIMO 각도의 중앙값 (한 바퀴 값).
       11/19 실차: 1.5 m 앞 LIMO 뒷면 한 바퀴 값이 정지 중에도 ±8° 튐 -> 최근 psi_win_s 동안 값(odom yaw 빼서 쌓음)의
       중앙값으로만 고침. psi_min_n개 이상 모이고 그 psi_near_frac(80%)이 중앙값 ±psi_ok_deg 안이면 '방향 맞음'
     - 옆: '방향 맞음' 뒤에만, snap_range 안 LIMO의 '차로 격자 나머지' 중앙값으로 (막힘 y_unlock_s 넘으면 그냥 품)
  3) 기억: 차로 좌표 장애물 목록 (예상 배치 prior를 미리 넣을 수 있음, 보이면 덮어씀)
     - 11/19 실차: 지나가는 사람·방향 흔들림으로 잘못 찍힌 점이 계속 남아 '길 없음'이 88 s 이어짐 ->
       보여야 할 자리(시야 안, 가리는 것 없음)인데 forget_n 바퀴 연속 안 보이면 지움
     - 바로 옆(앞뒤 0.6 m) LIMO는 지금 라이다로 잰 차 기준 옆 거리로 차로를 다시 정함
  4) 계획: 차로 3개 × 5 cm 칸 DP. 차로 변경 change_len(1.1 m), 길 없을 때만 change_len_short(0.9 m, 느리게)
     - 1.5 m 안 LIMO가 차로 경계에 걸쳐 보이면 두 차로 다 막힘으로
  5) 따라가기: 계획 차로 높이 = 옆 차로 LIMO 중심 ± offset(0.5 m) (LIMO 기준 웨이포인트), pure pursuit
안전
  - 길 없음 → 지금 차로 막힘까지 짧은 차로 변경 앞부분만 남으면 wait_s 동안 서서 기다림(길이 열리면 옮길 자리)
             → 그래도 막히면 creep_v로 지금 차로 따라 접근 → 차 폭 안 앞 간격 creep_stop에서 정지
  - 굴려 보기: 지금 자세로 roll_dist 굴려 정차 LIMO·노란선(차로 좌표)에 닿으면 정지
  - 후진: reverse_max > 0일 때만 (기본 0 = 후진 없음)
  - 노란선: /lane/status yellow_l/r_mm으로 m7_replay_core._exec_yellow_check와 같은 예측 → 반대로 최대 조향
끝: x >= section_length - end_before → 지금 차로를 target으로 DONE (M2는 FSM)
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from limo_obstacle_avoid.path_track import PoseFusion, wrap

LANE_NO = {1: 1, 0: 2, -1: 3}


@dataclass
class WaypointParams:
    # 차로·차
    lane_width: float = 0.52
    num_side_lanes: int = 1
    robot_front: float = 0.20
    robot_rear: float = 0.16
    robot_half_width: float = 0.115
    lidar_x: float = 0.10
    # 켤 때 (obstacle_node가 넣음)
    section_length: float = 4.38        # 켜는 점 ~ 섬 앞 끝 (m)
    start_heading_deg: float = 0.0
    start_y: float = 0.0
    start_target: int = 0               # 호환 (안 씀)
    start_lane: int = 0                 # 켤 때 차로 (+1 왼쪽, 0 가운데, -1 오른쪽). start_lane_no가 있으면 그쪽
    start_lane_no: int = 0              # 1/2/3 (0 = start_lane)
    m2_test_start: bool = False         # 이 노드는 M2를 안 함 → 켜지면 바로 끝 (FSM 차선 주행)
    end_before: float = 1.0             # 섬 앞 끝 이만큼 전에서 끝 (M2는 FSM)
    # 정차 LIMO 크기 (11/19 실차 영상·라이다: 차로 방향으로 세움 -> 차로 방향 0.32, 폭 방향 0.22. 뒷면 폭 0.25로 잼)
    obs_along: float = 0.32
    obs_across: float = 0.22
    # 라이다 처리
    cluster_gap: float = 0.08
    min_pts: int = 4
    max_range: float = 2.6
    edge_deg: float = 12.0              # 시야 끝 이만큼 안에 걸친 덩어리 버림
    corridor_y: float = 0.8             # 차로 좌표 |y| 이보다 바깥(벽·사람) 버림 (노란선 0.78. 11/19 1.0이면 트랙 밖 물체가 바깥 차로를 막음)
    # 위치 보정
    psi_gain: float = 0.15
    psi_gain_start: float = 0.5
    psi_ok_deg: float = 3.0
    psi_first_n: int = 3                # (호환: 안 씀)
    psi_win_s: float = 1.5              # 방향 값 모으는 시간
    psi_min_n: int = 5                  # 이만큼 모여야 고침
    psi_near_frac: float = 0.8          # 모인 값의 이 비율 이상이 중앙값 ±psi_ok_deg 안이어야 '방향 맞음'
    psi_max_jump_deg: float = 20.0
    y_gain: float = 0.4
    snap_range: float = 1.3
    snap_max: float = 0.15              # 차로 격자 나머지가 이보다 크면 안 씀. 11/19 시뮬: 0.2면 처음 옆 오차 0.25 m 넘을 때 한 차로 틀리게 붙어 노란선 밖 (wide 3/12)
    y_unlock_s: float = 3.0
    # 기억
    prior_xy: List[float] = field(default_factory=list)   # 예상 배치 [x1, y1, x2, y2, ...] (켜는 점 기준, m)
    match_dx: float = 0.30
    match_dy: float = 0.25
    side_fix_dx: float = 0.6
    near_range: float = 1.5
    forget_n: int = 5                   # 보여야 할 자리인데 이만큼 연속 안 보이면 기억에서 지움 (0 = 안 지움)
    forget_gap: float = 0.25            # 그 방향 점이 모두 기억 위치보다 이만큼 넘게 멀면 '뚫림'
    # 계획
    change_len: float = 1.1
    change_len_short: float = 0.9
    short_cost: float = 1.3
    short_speed: float = 0.75
    clear: float = 0.05
    plan_ds: float = 0.05
    # 따라가기
    speed: float = 0.20
    offset: float = 0.50                # 옆 차로 정차 LIMO 중심에서 이만큼
    lookahead: float = 0.40
    max_curv: float = 1.0 / 0.53
    edge_margin: float = 0.05           # 목표 옆 위치가 바깥 차로 중심 + 이만큼을 넘지 않게
    # 안전
    creep_v: float = 0.10
    creep_stop: float = 0.30
    wait_s: float = 6.0                 # 길 없음: 지금 차로 막힘까지 '짧은 차로 변경 앞부분(0.6 x change_len_short)' 만큼 남으면
                                        #   이 시간 동안 서서 기다림 (그 뒤 길이 열려도 옮길 자리를 남김). 지나면 예전처럼 creep
    safety_stop: float = 0.06
    rollout: bool = True
    roll_dist: float = 1.0
    roll_delay: float = 0.30
    roll_tau: float = 0.10
    roll_kmax: float = 1.6
    roll_margin: float = 0.02
    roll_stop_s: float = 0.5
    roll_hold_s: float = 1.0           # 계획이 닿을 때 고른 '차로 직선'을 이만큼 유지
    yellow_line_y: float = 0.775        # 차로 좌표 바깥 노란선 (굴려 보기용, 차로 1.5배 + 반 두께)
    reverse_max: int = 0                # 0 = 후진 없음
    reverse_dist: float = 0.25
    reverse_speed: float = 0.12
    reverse_aim_deg: float = 10.0
    stuck_reverse_after: float = 1.0
    # 노란선 (차선 노드 /lane/status, m7_replay_core._exec_yellow_check와 같은 식)
    yellow_check: bool = True
    eyg_clear_mm: float = 50.0
    eyg_psi_deg: float = 2.0
    eyg_frames: int = 2
    eyg_front: float = 0.20
    eyg_latency: float = 0.15
    eyg_fresh_s: float = 0.3


def rect(x, y, th, along, across):
    c, s = math.cos(th), math.sin(th)
    ha, hc = along / 2.0, across / 2.0
    return [(x + a * c - b * s, y + a * s + b * c) for a, b in ((ha, hc), (-ha, hc), (-ha, -hc), (ha, -hc))]


def rect_ref(x, y, th, front, rear, hw):
    c, s = math.cos(th), math.sin(th)
    return [(x + a * c - b * s, y + a * s + b * c) for a, b in ((front, hw), (-rear, hw), (-rear, -hw), (front, -hw))]


def sat_overlap(A, B) -> bool:
    for poly in (A, B):
        for i in range(4):
            p, q = poly[i], poly[(i + 1) % 4]
            nx, ny = q[1] - p[1], p[0] - q[0]
            a = [v[0] * nx + v[1] * ny for v in A]
            b = [v[0] * nx + v[1] * ny for v in B]
            if max(a) < min(b) or max(b) < min(a):
                return False
    return True


def median(v):
    s = sorted(v)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def cluster(pts: Sequence[Tuple[float, float, float]], gap: float) -> List[List[Tuple[float, float, float]]]:
    """pts: 라이다 각도 순 (x, y, a). 이웃 점 거리 gap 넘으면 끊음."""
    out, cur, last = [], [], None
    for p in pts:
        if last is not None and math.hypot(p[0] - last[0], p[1] - last[1]) > gap:
            out.append(cur)
            cur = []
        cur.append(p)
        last = p
    if cur:
        out.append(cur)
    return out


def fit_box(c, th_exp, along, across, lidar_x):
    """L자/한 변 맞추기 (closeness). 반환 (cx, cy, th, good, e1, e2) 차 기준, 아니면 None."""
    scores = []
    d = -0.4
    while d <= 0.4001:
        th = th_exp + d
        co, si = math.cos(th), math.sin(th)
        c1 = [p[0] * co + p[1] * si for p in c]
        c2 = [-p[0] * si + p[1] * co for p in c]
        mn1, mx1, mn2, mx2 = min(c1), max(c1), min(c2), max(c2)
        sc = 0.0
        for a, b in zip(c1, c2):
            sc += 1.0 / max(min(min(mx1 - a, a - mn1), min(mx2 - b, b - mn2)), 0.01)
        scores.append((th, sc))
        d += 0.01
    best = max(sc for _, sc in scores)
    tied = [th for th, sc in scores if sc >= 0.995 * best]     # 한 변만 보이면 여러 각이 같은 점수 -> 가운데
    b_th = sum(tied) / len(tied)
    co, si = math.cos(b_th), math.sin(b_th)
    c1 = [p[0] * co + p[1] * si for p in c]
    c2 = [-p[0] * si + p[1] * co for p in c]
    mn1, mx1, mn2, mx2 = min(c1), max(c1), min(c2), max(c2)
    e1, e2 = mx1 - mn1, mx2 - mn2
    if e1 > 0.45 or e2 > 0.40:
        return None
    s1, s2 = lidar_x * co, -lidar_x * si
    if e1 > along - 0.06:
        m1 = (mn1 + mx1) / 2
    elif s1 < mn1:
        m1 = mn1 + along / 2
    elif s1 > mx1:
        m1 = mx1 - along / 2
    else:
        m1 = (mn1 + mx1) / 2
    if e2 > across - 0.05:
        m2 = (mn2 + mx2) / 2
    elif s2 < mn2:
        m2 = mn2 + across / 2
    elif s2 > mx2:
        m2 = mx2 - across / 2
    else:
        m2 = (mn2 + mx2) / 2
    good = len(c) >= 10 and (e1 > 0.6 * along + 0.07 or e2 > 0.6 * across + 0.04)
    return (m1 * co - m2 * si, m1 * si + m2 * co, b_th, good, e1, e2)


@dataclass
class Trans:
    x0: float
    x1: float
    a: int
    b: int
    short: bool = False


class M7Waypoint:
    def __init__(self, p: WaypointParams):
        self.p = p
        self.fu = PoseFusion()
        self.offset = (0.0, 0.0, 0.0)
        self.state = 'IDLE'
        self.events: List[str] = []
        self.exit_fork: Optional[str] = None
        self.lane = 0
        self.target = 0
        self.reset()

    # ---------- 위치 ----------
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

    def lane_of(self, y: float) -> int:
        n = self.p.num_side_lanes
        return max(-n, min(n, int(round(y / self.p.lane_width))))

    # ---------- 입력 ----------
    def on_imu(self, t, gz):
        self.fu.on_imu(t, gz)

    def on_odom(self, t, x, y, yaw, v):
        self.fu.on_odom(t, x, y, yaw, v, self.last_v)

    def set_exit(self, fork: str):
        f = (fork or '').strip().upper()
        self.exit_fork = f if f in ('A', 'B') else None

    def exit_lane(self) -> Optional[int]:
        return {'A': 1, 'B': -1}.get(self.exit_fork) if self.exit_fork else None

    def on_lane(self, msg: dict, t: float):
        self.lane_msg, self.lane_t = msg, t

    # ---------- 켜기·끄기 ----------
    def reset(self):
        self.mem: List[dict] = []
        self.plan_tr: Optional[List[Trans]] = None
        self.plan_lane0 = 0
        self.blocked = False
        self.blocked_t = 0.0
        self.blocked_sum = 0.0
        self.wait_t = 0.0
        self.creep = False
        self.danger = False
        self.danger_t = 0.0
        self.rev = None
        self.n_rev = 0
        self.psi_n = 0
        self.psi_ok = 0
        self.psi_buf: List[Tuple[float, float]] = []
        self.y_unlocked = False
        self.n_forgot = 0
        self.front_gap = math.inf
        self.dets = []
        self.first_block: Dict[int, Optional[float]] = {}
        self.n_scan = 0
        self.t_last = None
        self.last_v = 0.0
        self.last_k = 0.0
        self.k_hist: List[Tuple[float, float]] = []
        self.lane_msg, self.lane_t = None, None
        self.eyg_n = 0
        self.eyg_hits = 0
        self.yg = None
        self.hold = None
        self._roll_t = -1e9
        self.why = ''
        self.stop_why = ''

    def start(self):
        p = self.p
        self.reset()
        self.fu.x = self.fu.y = 0.0
        self.fu.s = self.fu.dist = 0.0
        k0 = {1: 1, 2: 0, 3: -1}.get(int(p.start_lane_no), int(p.start_lane))
        self.lane = self.target = k0
        self.offset = (0.0, k0 * p.lane_width + p.start_y, math.radians(p.start_heading_deg) - self.fu.yaw)
        pr = list(p.prior_xy or [])
        for i in range(0, len(pr) - 1, 2):
            self.mem.append({'x': float(pr[i]), 'y': float(pr[i + 1]), 'n': 2, 'seen': -1.0, 'prior': True})
        self.state = 'RUN'
        self.events.append(f'시작: 차로 {LANE_NO[k0]}, 섬까지 {p.section_length:.2f} m, 끝 {p.section_length - p.end_before:.2f} m'
                           + (f', 예상 배치 {len(pr) // 2}대' if pr else '') + f', 후진 {"끔" if p.reverse_max <= 0 else p.reverse_max}')
        if p.m2_test_start:
            self.state = 'DONE'
            self.events.append('M2만 모드: 이 노드는 M2를 안 함 -> 바로 끝 (FSM 차선 주행)')

    def stop(self):
        self.state = 'IDLE'

    # ---------- 라이다 ----------
    def on_scan(self, t: float, pts: Sequence[Tuple[float, float, float]], a_min: float, a_max: float):
        """pts: 차 기준점 좌표 (x 앞, y 왼쪽, a 라이다 각도) 각도 순."""
        if self.state != 'RUN':
            return
        p = self.p
        self.n_scan += 1
        fg = math.inf
        for x, y, _ in pts:
            if abs(y) < p.robot_half_width + 0.03 and x > p.robot_front:
                fg = min(fg, x - p.robot_front)
        self.front_gap = fg
        ex, ey, epsi = self.pose()
        edge = math.radians(p.edge_deg)
        dets = []
        for c in cluster(pts, p.cluster_gap):
            if len(c) < p.min_pts:
                continue
            if any(q[2] < a_min + edge or q[2] > a_max - edge for q in c):
                continue
            b = fit_box(c, -epsi, p.obs_along, p.obs_across, p.lidar_x)
            if b is None or math.hypot(b[0], b[1]) > p.max_range:
                continue
            dets.append(b)
        # 방향 보정
        goods = [d for d in dets if d[3]]
        if goods:
            psi_m = -median([d[2] for d in goods])
            dpsi = wrap(psi_m - epsi)
            if abs(dpsi) < math.radians(p.psi_max_jump_deg):
                # odom yaw 기준 '차로 방향 - odom yaw' 로 쌓아 차가 돌아도 같은 값끼리 비교
                self.psi_buf.append((t, wrap(psi_m - self.fu.yaw)))
        self.psi_buf = [q for q in self.psi_buf if t - q[0] <= p.psi_win_s]
        if len(self.psi_buf) >= p.psi_min_n:
            vals = [q[1] for q in self.psi_buf]
            ref = vals[-1]
            vals = [ref + wrap(v - ref) for v in vals]
            med = median(vals)
            near = sum(1 for v in vals if abs(v - med) < math.radians(p.psi_ok_deg))
            dpsi = wrap(med + self.fu.yaw - epsi)
            if near >= p.psi_near_frac * len(vals):
                g = p.psi_gain_start if self.psi_ok < 3 else p.psi_gain
                self._shift(ddpsi=g * dpsi)
                self.psi_n += 1
                self.psi_ok += 1
            else:
                self.psi_ok = 0
        if self.psi_ok >= 3 or self.blocked_sum > p.y_unlock_s:
            self.y_unlocked = True
        ex, ey, epsi = self.pose()

        def to_lane(d):
            c, s = math.cos(epsi), math.sin(epsi)
            return ex + d[0] * c - d[1] * s, ey + d[0] * s + d[1] * c

        # 옆 보정 (차로 격자 스냅)
        if self.y_unlocked:
            res = []
            for d in dets:
                if math.hypot(d[0], d[1]) > p.snap_range:
                    continue
                ox, oy = to_lane(d)
                if abs(oy) > 0.9:
                    continue
                r = oy - p.lane_width * round(oy / p.lane_width)
                if abs(r) < p.snap_max:
                    res.append(r)
            if res:
                m = median(res)
                if abs(m) < p.snap_max:
                    self._shift(ddy=-p.y_gain * m)
                    ex, ey, epsi = self.pose()
        # 기억
        self.dets = []
        for d in dets:
            ox, oy = to_lane(d)
            self.dets.append((ox, oy, d[0], d[1]))
            if abs(oy) > p.corridor_y:
                continue
            m = next((q for q in self.mem if abs(q['x'] - ox) < (0.35 if q.get('prior') else p.match_dx)
                      and abs(q['y'] - oy) < p.match_dy), None)
            if m:
                m.update(x=ox, y=oy, seen=t, prior=False)
                m['n'] += 1
            else:
                self.mem.append({'x': ox, 'y': oy, 'n': 1, 'seen': t, 'prior': False})
        self._forget(t, pts, a_min, a_max)
        self.mem = [q for q in self.mem if q.get('prior') or q['n'] >= 2 or t - q['seen'] < 0.25]
        self._replan(t)

    def _forget(self, t, pts, a_min, a_max):
        """기억한 물체 자리를 라이다가 '뚫고' 지나가면(그 방향 점이 모두 물체보다 forget_gap 넘게 멀거나 없음) miss +1.
        forget_n 바퀴 연속이면 지움. 그 방향에 물체 근처나 앞쪽 점이 하나라도 있으면(보임·가림) miss 0."""
        p = self.p
        if p.forget_n <= 0 or not pts:
            return
        ex, ey, epsi = self.pose()
        c, s = math.cos(epsi), math.sin(epsi)
        edge = math.radians(p.edge_deg)
        keep = []
        for q in self.mem:
            dx, dy = q['x'] - ex, q['y'] - ey
            rx, ry = dx * c + dy * s - p.lidar_x, -dx * s + dy * c
            rng, brg = math.hypot(rx, ry), math.atan2(ry, rx)
            if rx < 0.25 or not (0.3 < rng < p.max_range - 0.3) or not (a_min + edge < brg < a_max - edge):
                keep.append(q)
                continue
            hw = math.atan2(0.10, rng)             # 물체 가운데 ±10 cm 방향만
            rays = [math.hypot(x - p.lidar_x, y) for x, y, a in pts if abs(wrap(math.atan2(y, x - p.lidar_x) - brg)) < hw]
            if any(r < rng + p.forget_gap for r in rays):
                q['miss'] = 0
                keep.append(q)
                continue
            q['miss'] = q.get('miss', 0) + 1
            if q['miss'] >= p.forget_n:
                self.n_forgot += 1
                continue
            keep.append(q)
        self.mem = keep

    # ---------- 계획 ----------
    def _plan(self, obs, x0, lane0, target, fixed: Optional[Trans]):
        p = self.p
        ds = p.plan_ds
        x_end = p.section_length - p.end_before
        n = max(2, int(math.ceil((x_end - x0) / ds)) + 1)
        half = (p.robot_front + p.robot_rear) / 2 + p.obs_along / 2 + p.clear
        bl = {-1: [], 0: [], 1: []}
        for o in obs:
            r = o['y'] - p.lane_width * round(o['y'] / p.lane_width)
            ks = {self.lane_of(o['y'])}
            if abs(r) > 0.12 and o.get('near', True):
                ks.add(self.lane_of(o['y'] + math.copysign(0.25, r)))
            for k in ks:
                bl[k].append(o['x'])

        xs = [x0 + j * ds for j in range(n + int(math.ceil(max(p.change_len, p.change_len_short) / ds)) + 2)]
        B = {k: [any(abs(x - ox) < half for ox in bl[k]) for x in xs] for k in (-1, 0, 1)}
        pre = {k: [0] for k in (-1, 0, 1)}
        for k in (-1, 0, 1):
            for b in B[k]:
                pre[k].append(pre[k][-1] + (1 if b else 0))
        M = len(xs)

        def blocked(k, x):
            j = int(round((x - x0) / ds))
            return B[k][j] if 0 <= j < M else False

        def free(k, xa, xb):
            ja, jb = max(0, int(round((xa - x0) / ds))), min(M - 1, int(round((xb - x0) / ds)))
            return jb < ja or pre[k][jb + 1] - pre[k][ja] == 0

        CH = [(p.change_len, 1.0), (p.change_len_short, p.short_cost)] if p.change_len_short > 0 else [(p.change_len, 1.0)]
        INF = 1e9
        cost = [[INF] * 3 for _ in range(n)]
        act = [[None] * 3 for _ in range(n)]
        for j in range(n - 1, -1, -1):
            x = x0 + j * ds
            for k in (-1, 0, 1):
                if blocked(k, x):
                    continue
                if j == n - 1:
                    cost[j][k + 1] = 0.0 if (target is None or k == target) else 5.0
                    act[j][k + 1] = 's'
                    continue
                best, a = INF, None
                for D, cc in CH:
                    for dd in (-1, 1):
                        k2 = k + dd
                        if k2 < -1 or k2 > 1:
                            continue
                        j2 = min(n - 1, j + int(round(D / ds)))
                        if not free(k, x, x + 0.6 * D) or not free(k2, x + 0.4 * D, x + D):
                            continue
                        c = cc + cost[j2][k2 + 1]
                        if c < best - 1e-9:
                            best, a = c, (dd, D)
                cs = cost[j + 1][k + 1] + 0.001
                if cs < best - 1e-9:
                    best, a = cs, 's'
                cost[j][k + 1], act[j][k + 1] = best, a
        if cost[0][lane0 + 1] >= INF / 2:
            return None
        tr = [fixed] if fixed else []
        j, k = 0, lane0
        while j < n - 1:
            a = act[j][k + 1]
            if a == 's' or a is None:
                j += 1
                continue
            x = x0 + j * ds
            tr.append(Trans(x, x + a[1], k, k + a[0], a[1] < p.change_len))
            k += a[0]
            j = min(n - 1, j + int(round(a[1] / ds)))
        return tr

    def _lane_at(self, x):
        k = self.plan_lane0
        for t in self.plan_tr or []:
            if x >= t.x1:
                k = t.b
            elif x >= t.x0:
                return k, t
        return k, None

    def _replan(self, t):
        p = self.p
        ex, ey, epsi = self.pose()
        egoL = self.lane_of(ey)
        corr = p.robot_half_width + p.obs_across / 2 + 0.03
        obs = []
        for m in self.mem:
            if m['n'] < 2:
                continue
            o = {'x': m['x'], 'y': m['y'], 'near': m['x'] - ex < p.near_range}
            if abs(m['x'] - ex) < p.side_fix_dx and t - m['seen'] < 0.25:
                c, s = math.cos(epsi), math.sin(epsi)
                dx, dy = m['x'] - ex, m['y'] - ey
                ry = -dx * s + dy * c
                if abs(ry) > corr and self.lane_of(m['y']) == egoL:
                    o['y'], o['near'] = (egoL + (1 if ry > 0 else -1)) * p.lane_width, False
                if abs(ry) <= corr * 0.6:
                    o['y'], o['near'] = egoL * p.lane_width, False
            obs.append(o)
        fixed, x0 = None, ex
        if self.plan_tr is not None:
            k, tr = self._lane_at(ex)
            if tr:
                fixed, x0, lane0 = tr, tr.x1, tr.b
            else:
                lane0 = k
        else:
            lane0 = egoL
        tgt = self.exit_lane()
        pl = self._plan(obs, x0, lane0, tgt, fixed)
        if pl is not None and fixed:
            self.plan_lane0 = fixed.a
        if pl is None and fixed:
            for k0 in (fixed.a, fixed.b):
                pl = self._plan(obs, ex, k0, tgt, None)
                if pl is not None:
                    self.plan_lane0 = k0
                    break
        elif pl is not None and not fixed:
            self.plan_lane0 = lane0
        if pl is not None:
            self.plan_tr = pl
            self.blocked = False
        else:
            self.blocked = True
        # 진단: 차로별 첫 막힘
        half = (p.robot_front + p.robot_rear) / 2 + p.obs_along / 2 + p.clear
        fb = {-1: None, 0: None, 1: None}
        for o in obs:
            r = o['y'] - p.lane_width * round(o['y'] / p.lane_width)
            ks = [self.lane_of(o['y'])]
            if abs(r) > 0.12 and o['near']:
                ks.append(self.lane_of(o['y'] + math.copysign(0.25, r)))
            for k in ks:
                d = o['x'] - half - ex
                if d > -2 * half and (fb[k] is None or d < fb[k]):
                    fb[k] = d
        self.first_block = fb

    def y_des(self, x):
        p = self.p
        if self.plan_tr is None:
            return self.pose()[1]

        def level(k, xq):
            base = k * p.lane_width
            ws = acc = 0.0
            for o in self.mem:
                if o['n'] < 2:
                    continue
                j = self.lane_of(o['y'])
                if abs(j - k) != 1:
                    continue
                w = max(0.0, min(1.0, (0.6 - abs(xq - o['x'])) / 0.2))
                if w <= 0:
                    continue
                acc += w * (o['y'] + (k - j) * p.offset)
                ws += w
            if ws <= 0:
                return base
            w = min(1.0, ws)
            y = w * (acc / ws) + (1 - w) * base
            lim = p.num_side_lanes * p.lane_width + p.edge_margin
            return max(-lim, min(lim, y))

        k, tr = self._lane_at(x)
        if tr is None:
            return level(k, x)
        u = max(0.0, min(1.0, (x - tr.x0) / (tr.x1 - tr.x0)))
        b = (1 - math.cos(math.pi * u)) / 2
        return (1 - b) * level(tr.a, x) + b * level(tr.b, x)

    # ---------- 굴려 보기 ----------
    def _roll_hit(self, yfn=None) -> bool:
        """지금 믿는 자세에서 yfn(기본: 지금 계획)을 roll_dist 굴려 봄 (명령 지연 roll_delay, 반응 roll_tau, |곡률| <= roll_kmax)."""
        p = self.p
        yfn = yfn or self.y_des
        x, y, psi = self.pose()
        k = self.last_k
        dt = 0.05
        nd = max(1, int(round(p.roll_delay / dt)))
        q = [kk for _, kk in self.k_hist][-nd:]
        q = [self.last_k] * (nd - len(q)) + q
        obs = [rect(m['x'], m['y'], 0.0, p.obs_along + p.roll_margin, p.obs_across + p.roll_margin)
               for m in self.mem if m['n'] >= 2]
        s0, v = 0.0, p.speed
        while s0 < p.roll_dist:
            ydes = yfn(x + p.lookahead)
            dx, dy = p.lookahead, ydes - y
            c, s = math.cos(psi), math.sin(psi)
            xr, yr = dx * c + dy * s, -dx * s + dy * c
            q.append(2 * yr / (xr * xr + yr * yr))
            kd = q.pop(0)
            k += (max(-p.roll_kmax, min(p.roll_kmax, kd)) - k) * min(1.0, dt / max(p.roll_tau, 1e-3))
            psi += v * k * dt
            x += v * math.cos(psi) * dt
            y += v * math.sin(psi) * dt
            s0 += v * dt
            me = rect_ref(x, y, psi, p.robot_front, p.robot_rear, p.robot_half_width)
            if any(sat_overlap(me, o) for o in obs):
                return True
            if any(abs(pt[1]) > p.yellow_line_y for pt in me):
                return True
        return False

    # ---------- 노란선 (차선 노드 값) ----------
    def _yellow(self, t, psi) -> Optional[int]:
        p = self.p
        ln = self.lane_msg
        if not p.yellow_check or not ln or self.lane_t is None or t - self.lane_t > p.eyg_fresh_s:
            self.eyg_n = 0
            return None
        f = ln.get('yellow_f_mm')
        f = float(f) / 1000.0 if f is not None else 0.575
        best = None
        for side, key in ((1, 'yellow_l_mm'), (-1, 'yellow_r_mm')):
            d = ln.get(key)
            if d is None:
                continue
            a = side * psi
            if math.degrees(a) < p.eyg_psi_deg:
                continue
            d_ref = float(d) / 1000.0 + f * math.tan(a)
            clr = (d_ref - p.robot_half_width - p.eyg_front * math.sin(a) - p.speed * p.eyg_latency * math.sin(a)
                   - (1.0 / p.max_curv) * (1 - math.cos(a)))
            if best is None or clr < best[1]:
                best = (side, clr)
        if best is None or 1000.0 * best[1] >= p.eyg_clear_mm:
            self.eyg_n = 0
            return None
        self.eyg_n += 1
        return best[0] if self.eyg_n >= p.eyg_frames else None

    # ---------- 주기 ----------
    def step(self, t: float) -> Tuple[float, float]:
        """(속도 m/s, 곡률 1/m). 20 Hz."""
        p = self.p
        dt = 0.05 if self.t_last is None else max(1e-3, min(0.2, t - self.t_last))
        self.t_last = t
        if self.state != 'RUN':
            return 0.0, 0.0
        x, y, psi = self.pose()
        self.lane = self.lane_of(y)
        if x >= p.section_length - p.end_before:
            self.target = self.lane
            self.state = 'DONE'
            self.events.append(f'[x={x:.2f}] 3차선 끝 -> 차로 {LANE_NO[self.lane]}로 끝 (출구 {self.exit_fork or "?"})')
            return 0.0, 0.0
        if self.n_scan < 2:
            return self._out(0.0, 0.0, '라이다 2바퀴 기다림')
        v = p.speed
        # 후진 중
        if self.rev is not None:
            kc = max(-p.max_curv, min(p.max_curv, 2.5 * (psi - self.rev['aim'])))
            self.rev['left'] -= p.reverse_speed * dt
            if self.rev['left'] <= 0:
                self.rev = None
                self.plan_tr = None
                self.danger = False
            return self._out(-p.reverse_speed, kc, '후진')
        if self.plan_tr:
            k, tr = self._lane_at(x + 0.2)
            if tr is not None and tr.short:
                v = p.speed * p.short_speed
        # 굴려 보기
        if self.hold is not None and t > self.hold[1]:
            self.hold = None
        if p.rollout and self.plan_tr is not None and not self.blocked:
            if t - self._roll_t >= 0.1:
                self._roll_t = t
                if self.hold is not None:
                    k_h = self.hold[0]
                    self.danger = self._roll_hit(lambda _x: k_h * p.lane_width)
                else:
                    self.danger = self._roll_hit()
                if self.danger:
                    # 다른 선택지: 지금 차로 유지 / 옆 차로로 (차로 가운데 직선) — 안 닿는 게 있으면 roll_hold_s 동안 그쪽
                    for k_c in sorted({self.lane, self.lane - 1, self.lane + 1}, key=lambda kk: abs(kk - self.lane)):
                        if abs(k_c) > p.num_side_lanes:
                            continue
                        if not self._roll_hit(lambda _x, kk=k_c: kk * p.lane_width):
                            self.hold = (k_c, t + p.roll_hold_s)
                            self.danger = False
                            self.events.append(f'[x={x:.2f}] 계획대로 가면 닿음 -> {LANE_NO[k_c]}차선 직선으로 {p.roll_hold_s:.1f} s')
                            break
        else:
            self.danger = False
            self.hold = None
        # 길 없음 → 천천히 다가감
        self.creep = False
        waiting = False
        if self.blocked:
            self.blocked_sum += dt
            room = self.first_block.get(self.lane)
            if (room is not None and room <= 0.6 * p.change_len_short + 0.05 and self.wait_t < p.wait_s):
                waiting = True
                self.wait_t += dt
                v = 0.0
            elif self.front_gap > p.creep_stop:
                v, self.creep, self.blocked_t = p.creep_v, True, 0.0
            else:
                v = 0.0
                self.blocked_t += dt
        else:
            self.blocked_t = 0.0
            self.wait_t = 0.0
        if self.danger and not self.blocked:
            v = 0.0
            self.danger_t += dt
        else:
            self.danger_t = 0.0
        # 후진 (reverse_max > 0일 때만)
        if p.reverse_max > 0 and self.n_rev < p.reverse_max:
            want = None
            if self.blocked and not self.creep and self.blocked_t > p.stuck_reverse_after and self.front_gap <= p.creep_stop + 0.02:
                want = '길 없음 + 앞 가까움'
            elif self.danger and self.danger_t > p.roll_stop_s:
                want = '지금 자세로 가면 닿음'
            if want:
                k, tr = self._lane_at(x + 0.5) if self.plan_tr else (self.lane, None)
                tgt = tr.b if tr else k
                aim = math.copysign(math.radians(p.reverse_aim_deg), tgt * p.lane_width - y) if tgt * p.lane_width != y else 0.0
                self.rev = {'left': p.reverse_dist, 'aim': aim}
                self.n_rev += 1
                self.blocked_t = self.danger_t = 0.0
                self.events.append(f'[x={x:.2f}] 후진 {self.n_rev}/{p.reverse_max} ({want})')
                return self._out(-p.reverse_speed, 0.0, '후진 시작')
        # 조향
        if self.blocked and self.creep:
            ydes = self.lane * p.lane_width
        elif self.hold is not None:
            ydes = self.hold[0] * p.lane_width
        else:
            ydes = self.y_des(x + p.lookahead)
        dx, dy = p.lookahead, ydes - y
        c, s = math.cos(psi), math.sin(psi)
        xr, yr = dx * c + dy * s, -dx * s + dy * c
        kc = 2 * yr / (xr * xr + yr * yr)
        side = self._yellow(t, psi) if v > 0 else None
        self.yg = None
        if side is not None:
            kc = -side * p.max_curv
            self.yg = side
            self.eyg_hits += 1
        # 안전 정지
        if self.front_gap < p.safety_stop and v > 0:
            v = 0.0
            self.stop_why = f'차 앞 {100 * self.front_gap:.0f} cm'
        why = ('길 없음 -> 천천히 접근' if self.creep else
               f'길 없음 -> 옮길 자리 남기고 기다림 ({self.wait_t:.0f}/{p.wait_s:.0f} s)' if waiting else
               '길 없음 -> 정지' if self.blocked else
               '굴려 보니 닿음 -> 정지' if self.danger else '계획 따라감')
        return self._out(v, max(-p.max_curv, min(p.max_curv, kc)), why)

    def _out(self, v, k, why):
        self.why = why
        self.last_v, self.last_k = v, k
        self.k_hist.append((self.t_last or 0.0, k))
        if len(self.k_hist) > 8:
            self.k_hist.pop(0)
        return v, k

    def status(self) -> dict:
        x, y, psi = self.pose()
        return {'state': 'RUN' if self.state == 'RUN' else self.state, 'impl': 'waypoint', 'phase': self.why,
                'x': round(x, 3), 'y': round(y, 3), 'psi_deg': round(math.degrees(psi), 1),
                'now_lane': self.lane, 'now_lane_no': LANE_NO.get(self.lane), 'target': self.target,
                'lane_no': LANE_NO.get(self.lane), 'end_lane': None, 'exit': self.exit_fork,
                'plan': [[round(t.x0, 2), t.a, t.b, bool(t.short)] for t in (self.plan_tr or [])],
                'blocked': self.blocked, 'creep': self.creep, 'danger': self.danger,
                'front_gap_mm': None if not math.isfinite(self.front_gap) else round(1000 * self.front_gap),
                'first_block_mm': {str(k): (None if v is None else round(1000 * v)) for k, v in self.first_block.items()},
                'mem': [[round(m['x'], 2), round(m['y'], 2), m['n'], bool(m.get('prior'))] for m in self.mem],
                'hold': None if self.hold is None else self.hold[0], 'psi_ok': self.psi_ok, 'y_unlocked': self.y_unlocked, 'n_rev': self.n_rev,
                'n_forgot': self.n_forgot,
                'yellow': self.yg, 'eyg_hits': self.eyg_hits, 'why': self.why, 'stop': self.stop_why,
                'lane_why': {'reason': self.why}}
