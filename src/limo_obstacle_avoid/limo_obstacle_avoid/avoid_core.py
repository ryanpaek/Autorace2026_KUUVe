"""
avoid_core.py  —  3차선 정적 장애물(정차 LIMO) 회피 핵심 로직

★ ROS를 전혀 쓰지 않는 순수 파이썬 파일입니다.
  → ROS 노드(s2~s6)도 이 파일을 쓰고,
  → 집에서 LIMO 없이 돌리는 tools/offline_sim.py 도 이 파일을 씁니다.

[좌표계: "미션 로컬 좌표"]
  미션 시작(enable) 순간의 로봇 위치를 원점, 그때의 진행 방향을 +x 로 잡습니다.
  y 는 왼쪽이 +.  (3차선 구간은 직선이라 이 좌표 하나로 충분)
  GPS 없이 '시작점으로부터 얼마나 왔나'만 오도메트리(거리) + IMU(방향)로 계산합니다.

[차선 번호]  (진행 방향 기준)
  +1 = 왼쪽 차선 (도면 A 쪽)
   0 = 가운데 차선 (진입 차선)
  -1 = 오른쪽 차선 (도면 B 쪽)
  차선 중심 y = 차선번호 × lane_width
"""
import math
from dataclasses import dataclass

from limo_obstacle_avoid.fork_exit import shift_distance, lane_name

INF = float('inf')


def wrap(a):
    """각도를 -pi ~ +pi 로"""
    return math.atan2(math.sin(a), math.cos(a))


def parse_exit(s):
    """'A' → +1(왼쪽), 'B' → -1(오른쪽), 그 외 → None(자동)"""
    s = str(s).strip().upper()
    if s in ('A', 'LEFT', 'L', '+1', '1'):
        return 1
    if s in ('B', 'RIGHT', 'R', '-1'):
        return -1
    return None


# ──────────────────────────────────────────────────────────────
# 파라미터 (config/avoid_params.yaml 에서 덮어씀)
# ──────────────────────────────────────────────────────────────
@dataclass
class AvoidParams:
    # ── 트랙 ──
    lane_width: float = 0.52          # 이웃 차로 중심 간격 (도면: 선 사이 500 + 점선 20 mm)
    num_side_lanes: int = 1           # 가운데 기준 양옆 차선 수 → 3차선
    start_lane: int = 0               # enable 순간 로봇이 있는 차선
    section_length: float = 4.3       # enable 지점 ~ 3차선 끝(가운데 섬 시작)까지 거리 [실측!]
    island_length: float = 1.0        # 가운데 섬(육각형) 길이 → 가상 장애물로 넣음
    use_island: bool = True           # 섬을 가상 장애물로 넣을지
    exit_lane: str = 'auto'           # 'auto' | 'A'(왼쪽 +1) | 'B'(오른쪽 -1)
    done_margin: float = 0.3          # 섬 시작 + 이만큼 지나고 차선에 붙으면 완료
    # ── 차량 (LIMO PRO) ──
    robot_front: float = 0.20         # base_link → 앞 범퍼
    robot_rear: float = 0.16          # base_link → 뒤 범퍼
    robot_half_width: float = 0.12    # 차폭/2 (+약간)
    side_margin: float = 0.05         # 옆 여유
    # ── 라이다 ──
    lidar_x: float = 0.10             # base_link 기준 라이다 앞쪽 위치
    scan_yaw_offset_deg: float = 0.0  # 라이다 0도가 정면이 아니면 보정 (s1에서 확인)
    scan_flip: bool = False           # 각도 방향이 반대면 True (s1에서 확인)
    range_min: float = 0.10
    range_max: float = 3.0
    roi_front: float = 2.5            # 앞으로 이만큼까지만 봄
    roi_back: float = 0.6             # 뒤로 이만큼까지 (옆 차 지나가는 중 확인용)
    roi_side_extra: float = 0.10      # 3차선 바깥으로 이만큼만 더 봄 (트랙 밖 사람 무시)
    cluster_gap: float = 0.12         # 점 사이가 이보다 멀면 다른 물체
    cluster_min_pts: int = 3          # 점이 이보다 적으면 노이즈
    merge_dist: float = 0.20          # 같은 장애물로 합치는 거리
    max_obs_width: float = 0.45       # 장애물 하나의 최대 폭 (LIMO 0.22)
    max_obs_len: float = 0.70         # 한 번에 본 덩어리가 앞뒤로 이보다 길면 벽·트랙 테두리로 보고 버림 (LIMO 0.32)
    min_obs_size: float = 0.08        # 덩어리 가로·세로 중 긴 쪽이 이보다 작으면 버림 (기둥·다리. 리모 면 0.22)
    obs_min_len: float = 0.33         # 앞면만 보여도 이 길이만큼 차가 있다고 가정 (LIMO 길이)
    confirm_hits: int = 3             # 이 횟수 이상 보여야 진짜 장애물로 인정
    # ── 판단 ──
    trigger_dist: float = 1.0         # 내 차선 앞 장애물이 (앞범퍼 기준) 이 안이면 차선 변경
    change_len: float = 1.0           # 차선 1칸 옮기는 데 필요한 앞쪽 여유
    clear_back: float = 0.0           # 옮길 차선의 차 끝이 (뒤범퍼 - 이값)보다 뒤여야 들어감 (음수=더 일찍)
    better_margin: float = 0.3        # 옮길 차선이 지금보다 이만큼 더 비어야 옮김
    hold_time: float = 0.8            # 차선 바꾼 뒤 최소 유지 시간 [s]
    stop_dist: float = 0.25           # 갈 곳 없고 앞이 이만큼 가까우면 정지
    emerg_min_pts: int = 2            # 비상정지 박스 안에 점이 이만큼 있어야 비상 (라이다 점 하나 튀어서 서는 것 방지)
    # ── 제어 (pure pursuit) ──
    v_cruise: float = 0.25            # 차선 안에서 속도
    v_change: float = 0.20            # 차선 변경 중 속도
    lookahead: float = 0.40           # pure pursuit 전방주시거리 (11/10 0.50 -> 0.40: 시뮬레이션 장애물 최소 간격 7.7 -> 12cm 이상)
    min_turn_radius: float = 0.53     # LIMO 아커만 최소 회전반경 실측값 (이보다 작게 두면 못 도는 경로를 계획)
    max_heading_deg: float = 40.0     # 이 이상 비틀어지면 더 꺾지 않음
    reach_tol: float = 0.08           # 차선 중심에서 이 안이면 '도착'
    # ── 갈림길(미션 2) 출구 차선 미리 이동 ──
    fork_shift: bool = True           # 갈림길 방향을 알면 섬 앞 끝 전에 출구 차선(A=+1, B=-1)으로 미리 옮김 (0/1/2차선)
    exit_margin: float = 0.20         # 섬 앞 끝까지 남겨 둘 여유 [m] (조향 지연·차체 모서리)
    exit_min_time: float = 0.6        # 시작 후 이 시간 동안은 미리 이동 안 함 (라이다 지도가 쌓일 때까지)
    # ── 정렬 보정 (정차 LIMO 중심이 차선 중심선에 오도록 내 좌표를 돌리고 옮김) ──
    lane_align: bool = True           # 출발 방향/위치 오차 보정 (라이다만 사용)
    align_gain: float = 0.5           # 한 번에 반영하는 비율 (0~1)
    align_sigma_meas: float = 0.03    # 차 중심 측정 오차 [m] (차가 차선 중앙에서 벗어나 서 있는 정도 포함)
    align_sigma_dy: float = 0.05      # 한 번 보정에서 옆 오프셋 사전 크기 [m]
    align_sigma_heading_deg: float = 2.0  # 한 번 보정에서 방향 사전 크기 [deg]
    align_max_dy: float = 0.12        # 누적 옆 보정 한계 [m]
    align_max_heading_deg: float = 6.0    # 누적 방향 보정 한계 [deg]
    align_w_min: float = 0.14         # 폭이 이 범위인 장애물만 사용 (일부만 보인 것 제외)
    align_w_max: float = 0.32
    align_min_span: float = 0.8       # 방향 보정은 앞뒤로 이만큼 떨어진 장애물들이 있을 때만 (가까우면 옆 위치만)
    align_max_psi_deg: float = 15.0   # 차가 이보다 비틀어져 있거나 후진(탈출) 중이면 정렬 보정 안 함
    # ── 카메라 차선 보정 (팀 lane_node의 /lane/status 를 그대로 받아 옆 위치·방향을 맞춤) ──
    #   차선 코드를 복사하지 않고 같은 노드의 결과를 쓰므로 차선 주행과 항상 같은 기준
    lane_fix: bool = True
    lane_fix_gain: float = 0.30       # 한 번에 옆 오차의 이만큼 반영 (차선 노드 약 20Hz). 11/10 0.15 -> 0.30
    lane_fix_heading_gain: float = 0.2   # 11/10 0.1 -> 0.2
    # 차로 번호: 차선 노드가 선 종류로 본 3차선 번호(lane_id)가 연속 lane_id_need/lane_id_window번 같고
    #   지금 추정 차로와 다르면 옆 위치를 그 차로로 바로 옮긴다 (시작 위치·방향 오차로 차로를 잘못 아는 것 방지)
    lane_id_fix: bool = True
    lane_id_window: int = 7
    lane_id_need: int = 5
    # 차로 안에서 똑바로 갈 때 조향 = (1-a) x 회피 pure pursuit + a x 차선 노드 곡률 (양쪽 선 보임 + 같은 차로일 때만)
    lane_keep_blend: float = 1.0      # 11/10 오후 0.5 -> 1.0: 차로 안 직진은 팀 차선 주행과 완전히 같은 조향 (v x kappa)
    lane_fresh_sec: float = 0.3
    lane_fix_gate: float = 0.15       # 지금 추정과 이만큼 넘게 다르면 다른 차로를 본 것 → 안 씀 [m]
    lane_fix_max_deg: float = 10.0    # 차선 방향이 이보다 틀어져 보이면 안 씀 (차로 변경 중·3차선 입구 사선)
    lane_row: float = 0.55            # 차선 노드 offset_mm 을 재는 행 (기준점 앞 m)
    # ── 멈춤 탈출: 막혀서 서 있으면 상황이 안 바뀌어 계속 선다 → 조금 후진하며 크게 꺾어 다시 잡음 ──
    recover: bool = True
    recover_after: float = 0.4        # 이만큼 계속 막혀 있으면 탈출 시작 [s] (0.8 -> 0.4: 서 있는 시간 줄임)
    recover_dist: float = 0.25        # 후진 거리 [m] (방금 지나온 길이라 비어 있음)
    recover_speed: float = 0.10       # 후진 속도 [m/s]
    recover_time: float = 4.0         # 이 시간 안에 못 끝내도 멈추고 다시 계획 [s]
    recover_max: int = 3              # trail 방식 횟수
    recover_swing_max: int = 6        # trail을 다 쓰고도 막히면: 후진하며 최대 조향으로 머리를 빈 쪽으로 돌리기 횟수 (0이면 예전처럼 정지)
    # 온 길 되돌아가기(11/10): 막히면 지나온 길(trail)을 그대로 후진하면서, 갈 수 있는 차로가 생기는 순간 멈추고 그리로 간다
    recover_mode: str = 'trail'       # 'trail' | 'swing'(예전: 25cm 후진하며 크게 꺾기)
    trail_step: float = 0.05          # 지나온 길 점 간격 [m]
    rollback_lookahead: float = 0.30  # 후진할 때 따라갈 뒤쪽 점 거리 [m]
    rollback_max_dist: float = 1.50   # 한 번 되돌아갈 최대 거리 [m]
    rollback_time: float = 15.0       # 한 번 되돌아가기 최대 시간 [s]


# ──────────────────────────────────────────────────────────────
# 1) 로컬 위치 추정 : 오도메트리 거리 + IMU 방향
# ──────────────────────────────────────────────────────────────
class LocalPose:
    def __init__(self):
        self.ready = False
        self.x = self.y = self.psi = 0.0
        self.dist = 0.0

    def reset(self, ox, oy, yaw, y0=0.0):
        self._ox, self._oy = ox, oy      # 직전 odom 위치
        self.yaw0 = yaw                  # 기준 방향 (enable 순간)
        self.x, self.y, self.psi = 0.0, y0, 0.0
        self.dist = 0.0
        self.ready = True

    def update(self, ox, oy, yaw, forward=True):
        if not self.ready:
            return
        ds = math.hypot(ox - self._ox, oy - self._oy)   # odom 이 움직인 거리만 사용
        if not forward:
            ds = -ds
        psi_new = wrap(yaw - self.yaw0)                 # 방향은 IMU(또는 odom) yaw
        mid = self.psi + wrap(psi_new - self.psi) / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.psi = psi_new
        self.dist += abs(ds)
        self._ox, self._oy = ox, oy


# ──────────────────────────────────────────────────────────────
# 2) 라이다 → 로봇 좌표 점
# ──────────────────────────────────────────────────────────────
def scan_to_base(ranges, angle_min, angle_inc, p):
    """LaserScan ranges → [(x, y) 또는 None, ...]  (base_link 기준, 순서 유지)"""
    off = math.radians(p.scan_yaw_offset_deg)
    out = []
    for i, r in enumerate(ranges):
        if not (p.range_min <= r <= p.range_max):      # nan / inf 도 여기서 걸러짐
            out.append(None)
            continue
        a = angle_min + i * angle_inc
        if p.scan_flip:
            a = -a
        a += off
        out.append((p.lidar_x + r * math.cos(a), r * math.sin(a)))
    return out


class Obstacle:
    """로컬 좌표의 사각형(bbox) 장애물"""
    def __init__(self, xmin, xmax, ymin, ymax, virtual=False, tag=''):
        self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax
        self.hits = 1
        self.virtual = virtual
        self.tag = tag

    def x_end(self, min_len):
        """뒤쪽 끝.  앞면만 봤어도 차 길이만큼은 있다고 가정"""
        if self.virtual:
            return self.xmax
        return max(self.xmax, self.xmin + min_len)


# ──────────────────────────────────────────────────────────────
# 3) 회피 플래너
# ──────────────────────────────────────────────────────────────
class AvoidPlanner:
    def __init__(self, params=None):
        self.p = params or AvoidParams()
        self.pose = LocalPose()
        self.reset()

    # ---------- 상태 ----------
    def reset(self):
        self.state = 'IDLE'          # IDLE → RUN → DONE
        self.t_start = 0.0
        self.obs = []
        self.target = self.p.start_lane
        self.last_switch = -1e9
        self.emergency = False
        self.blocked = False
        self.exit_pref = parse_exit(self.p.exit_lane)
        self.note = ''
        self.events = []             # (x, '설명') 기록
        self.align_dy = 0.0          # 누적 옆 보정 [m]
        self.align_th = 0.0          # 누적 방향 보정 [rad]
        self.exit_info = {}          # 출구 이동 판단 (status 로 내보냄)
        self._late_logged = False
        self._given_up_for = None
        self.stuck_since = None      # 막혀서 서 있기 시작한 시각
        self.rec = None              # 탈출 중이면 {'t0', 'x0', 'y0', 'side'}
        self.rb = None               # 되돌아가기 중이면 {'t0', 'x0', 'y0', 'dist', 'blocked'}
        self.rec_n = 0               # 탈출 횟수 (trail + swing)
        self.rec_side_last = None    # 마지막 swing 방향
        self.rec_same_side = 0
        self.lane_fix_n = 0          # 카메라 차선으로 보정한 횟수 (status)
        self.lane_fix_dy = 0.0       # 마지막 보정량 [m]
        self.trail = []              # 지나온 길 [(x, y)] (되돌아가기)
        self.lane_ids = []           # 차선 노드 차로 번호 기록
        self.lane_id = None          # 확정된 카메라 차로 번호
        self.lane_id_fix_n = 0
        self.lane_last = None        # 마지막 /lane/status
        self.lane_t = None
        self.rollback_m = 0.0

    def start(self, ox, oy, yaw, t=0.0):
        self.reset()
        self.t_start = t
        self.pose.reset(ox, oy, yaw, y0=self.p.start_lane * self.p.lane_width)
        self.state = 'RUN'
        self.last_switch = t - self.p.hold_time

    def set_exit(self, s):
        new = parse_exit(s)
        if new is not None and self.exit_info.get('plan') == 'given_up' and new == self._given_up_for:
            return                                   # 이미 포기한 방향을 다시 받아도 또 시도하지 않음
        if self.exit_info.get('plan') == 'given_up':
            self.exit_info = {}
        self.exit_pref = new

    @property
    def lanes(self):
        n = self.p.num_side_lanes
        return list(range(-n, n + 1))

    def center(self, lane):
        return lane * self.p.lane_width

    def nearest_lane(self, y):
        n = self.p.num_side_lanes
        return max(-n, min(n, int(round(y / self.p.lane_width))))

    # ---------- 라이다 처리 ----------
    def update_scan(self, pts_base):
        """base_link 점들 → (비상정지 확인) → 로컬 좌표 → 묶기 → 지도에 합치기"""
        p = self.p
        x, y, psi = self.pose.x, self.pose.y, self.pose.psi
        c, s = math.cos(psi), math.sin(psi)
        ylim = (p.num_side_lanes + 0.5) * p.lane_width + p.roi_side_extra

        emerg_n = 0
        clusters, cur = [], []
        prev = None

        def close():
            if len(cur) >= p.cluster_min_pts:
                xs = [q[0] for q in cur]
                ys = [q[1] for q in cur]
                lx, wy = max(xs) - min(xs), max(ys) - min(ys)
                # 너무 크면 벽·테두리(길이 max_obs_len, 폭 max_obs_width 초과), 너무 작으면 기둥·다리 -> 버림 (11/10)
                if lx <= p.max_obs_len and wy <= p.max_obs_width and max(lx, wy) >= p.min_obs_size:
                    clusters.append((min(xs), max(xs), min(ys), max(ys)))
            cur.clear()

        for pt in pts_base:
            if pt is None:
                close(); prev = None
                continue
            px, py = pt
            # 비상정지 박스: 앞범퍼 바로 앞, 차폭 안
            if p.robot_front < px < p.robot_front + p.stop_dist and abs(py) < p.robot_half_width:
                emerg_n += 1
            X = x + px * c - py * s
            Y = y + px * s + py * c
            if not (x - p.roi_back <= X <= x + p.roi_front and -ylim <= Y <= ylim):
                close(); prev = None
                continue
            if prev is not None and math.hypot(X - prev[0], Y - prev[1]) > p.cluster_gap:
                close()
            cur.append((X, Y))
            prev = (X, Y)
        close()

        self.emergency = emerg_n >= max(1, p.emerg_min_pts)
        self.last_clusters = clusters
        self._merge(clusters)
        self._align()
        return clusters

    # ---------- 정렬 보정 ----------
    def _align(self):
        """확정된 정차 LIMO들의 중심 y 가 차선 중심(k × lane_width)에 오도록
        잔차 r = y중심 - k·w 를  r ≈ a + b·x  로 맞춘 뒤(사전값이 있는 최소제곱),
        좌표 전체를 원점 기준 δ=-b 만큼 회전 + t=-a 만큼 옆으로 옮긴다."""
        p = self.p
        if not p.lane_align:
            return
        if self.rec is not None or self.rb is not None or abs(math.degrees(self.pose.psi)) > p.align_max_psi_deg:
            return      # 후진·크게 비틀어진 동안은 장애물 옆면이 비스듬히 보여 중심이 틀어짐 -> 정렬에 안 씀
        xs, rs = [], []
        for o in self.obs:
            if o.virtual or o.hits < p.confirm_hits:
                continue
            w = o.ymax - o.ymin
            if not (p.align_w_min <= w <= p.align_w_max):
                continue
            yc = 0.5 * (o.ymin + o.ymax)
            k = self.nearest_lane(yc)
            r = yc - k * p.lane_width
            if abs(r) > 0.5 * p.lane_width - 0.05:      # 차선 경계 근처면 어느 차선인지 애매 → 제외
                continue
            xs.append(o.xmin + 0.5 * p.obs_min_len)
            rs.append(r)
        if not xs:
            return
        im = 1.0 / p.align_sigma_meas ** 2
        s0 = len(xs) * im + 1.0 / p.align_sigma_dy ** 2
        s1 = sum(xs) * im
        s2 = sum(x * x for x in xs) * im + 1.0 / math.radians(p.align_sigma_heading_deg) ** 2
        t0 = sum(rs) * im
        t1 = sum(x * r for x, r in zip(xs, rs)) * im
        det = s0 * s2 - s1 * s1
        if det <= 0:
            return
        a = (t0 * s2 - s1 * t1) / det
        b = (s0 * t1 - s1 * t0) / det
        if max(xs) - min(xs) < p.align_min_span:
            # 장애물들이 앞뒤로 가까이 붙어 있으면 방향(기울기)은 잴 수 없다 (5cm 오차 / 15cm 간격 = 18°) -> 옆 위치만
            a, b = t0 / s0, 0.0
        g = p.align_gain
        d = -g * b
        t = -g * a
        mth, mdy = math.radians(p.align_max_heading_deg), p.align_max_dy
        d = max(-mth - self.align_th, min(mth - self.align_th, d))
        t = max(-mdy - self.align_dy, min(mdy - self.align_dy, t))
        if abs(d) < 1e-5 and abs(t) < 1e-5:
            return
        self._apply_transform(d, t)
        self.align_th += d
        self.align_dy += t

    def _apply_transform(self, d, t):
        """로컬 좌표 전체를 원점 기준 d[rad] 회전 + y 로 t[m] 이동 (위치·방향·장애물 모두)"""
        c, s = math.cos(d), math.sin(d)
        P = self.pose
        P.x, P.y = c * P.x - s * P.y, s * P.x + c * P.y + t
        P.psi = wrap(P.psi + d)
        P.yaw0 -= d                      # 이후 odom/IMU 갱신도 같은 좌표로 이어지게
        self.trail = [(c * tx - s * ty, s * tx + c * ty + t) for tx, ty in self.trail]
        for o in self.obs:               # 장애물은 중심만 옮김 (크기는 그대로 → 반복 회전에도 안 커짐)
            cx, cy = 0.5 * (o.xmin + o.xmax), 0.5 * (o.ymin + o.ymax)
            nx, ny = c * cx - s * cy, s * cx + c * cy + t
            dx, dy = nx - cx, ny - cy
            o.xmin += dx; o.xmax += dx
            o.ymin += dy; o.ymax += dy

    def _merge(self, clusters):
        p, md = self.p, self.p.merge_dist
        for (x0, x1, y0, y1) in clusters:
            merged = False
            for o in self.obs:
                if x0 <= o.xmax + md and x1 >= o.xmin - md and y0 <= o.ymax + md and y1 >= o.ymin - md:
                    ny0, ny1 = min(o.ymin, y0), max(o.ymax, y1)
                    if ny1 - ny0 > p.max_obs_width:
                        continue
                    if max(o.xmax, x1) - min(o.xmin, x0) > p.max_obs_len:   # 벽 조각이 이어 붙어 길어지는 것 방지
                        continue
                    o.xmin, o.xmax = min(o.xmin, x0), max(o.xmax, x1)
                    o.ymin, o.ymax = ny0, ny1
                    o.hits += 1
                    merged = True
                    break
            if not merged:
                self.obs.append(Obstacle(x0, x1, y0, y1))

    def obstacles(self, with_pref=True):
        """확정된 실제 장애물 + 가상 장애물(섬, 출구 선호)"""
        p = self.p
        out = [o for o in self.obs if o.hits >= p.confirm_hits]
        L, lw = p.section_length, p.lane_width
        if p.use_island:
            out.append(Obstacle(L, L + p.island_length, -0.3 * lw, 0.3 * lw, True, 'island'))
        if with_pref and self.exit_pref is not None:
            other = -self.exit_pref                      # 반대쪽 차선을 섬 구간부터 막음
            out.append(Obstacle(L, L + p.island_length,
                                other * lw - 0.3 * lw, other * lw + 0.3 * lw, True, 'exit'))
        return out

    # ---------- 차선별 여유 ----------
    def _in_corridor(self, o, lane):
        c = self.center(lane)
        hw = self.p.robot_half_width + self.p.side_margin
        return not (o.ymax < c - hw or o.ymin > c + hw)

    def free_ahead(self, lane, obs):
        """그 차선에서 앞범퍼 ~ 가장 가까운 장애물까지 거리 (없으면 INF, 옆에 붙어있으면 음수)"""
        p = self.p
        x = self.pose.x
        xf = x + p.robot_front
        d = INF
        for o in obs:
            if not self._in_corridor(o, lane):
                continue
            if o.x_end(p.obs_min_len) < x - p.robot_rear:   # 이미 지나감
                continue
            d = min(d, o.xmin - xf)
        return d

    def path_clear(self, frm, to, obs):
        """frm → to 로 옮길 때 지나가는 차선들이 '지금 옆 + 앞쪽 change_len' 동안 비었나"""
        p = self.p
        x = self.pose.x
        ahead = x + p.robot_front + p.change_len * abs(to - frm)
        back = x - p.robot_rear - p.clear_back
        step = 1 if to > frm else -1
        for lane in range(frm + step, to + step, step):
            for o in obs:
                if not self._in_corridor(o, lane):
                    continue
                if o.xmin < ahead and o.x_end(p.obs_min_len) > back:
                    return False
        return True

    def choose(self, cur, obs):
        """막힌 차선 cur 대신 갈 차선 고르기 (없으면 None)"""
        p = self.p
        f_cur = self.free_ahead(cur, obs)
        best, best_key = None, None
        for lane in self.lanes:
            if lane == cur or not self.path_clear(cur, lane, obs):
                continue
            f = self.free_ahead(lane, obs)
            if f < f_cur + p.better_margin:
                continue
            key = (min(f, 3.0), -abs(lane - cur), 1 if lane == self.exit_pref else 0)
            if best is None or key > best_key:
                best, best_key = lane, key
        return best

    # ---------- 갈림길 출구 차선 이동 ----------
    def _lanes_between(self, frm, to):
        """frm 에서 to 로 갈 때 새로 지나가는 차선들 (frm 제외, to 포함)"""
        step = 1 if to > frm else -1
        return list(range(frm + step, to + step, step))

    def exit_plan(self, obs):
        """출구 차선으로 지금 옮겨도 되나 → ('keep'|'go'|'wait'|'too_late', 정보)
        keep    : 이미 출구 차선 (또는 방향 모름)
        go      : 지금 옮기기 시작 (지나갈 차선이 비었고, 섬 앞 끝까지 거리가 충분)
        wait    : 장애물 때문에 아직 못 옮김 (계속 다시 봄)
        too_late: 섬 앞 끝까지 남은 거리가 이동에 필요한 거리보다 짧음"""
        p = self.p
        ex, cur = self.exit_pref, self.target
        if ex is None or cur == ex:
            return 'keep', {'need_lanes': 0}
        n = ex - cur
        clear, settle = shift_distance(p, n)
        x = self.pose.x
        rem = p.section_length - (x + p.robot_front)
        info = {'need_lanes': n, 'need_m': round(clear + p.exit_margin, 2), 'rem_m': round(rem, 2)}
        if rem < clear + p.exit_margin:
            return 'too_late', info
        # 미리 옮기는 판단은 보수적으로: 아직 확정 전(1~2번 본) 장애물도 막힌 것으로 본다
        real = [o for o in self.obs if not o.virtual]
        back = x - p.robot_rear - p.clear_back
        ahead = x + p.robot_front + settle
        for lane in self._lanes_between(cur, ex):          # 지나갈 차선: 옮기는 동안 옆·앞이 비어야 함
            for o in real:
                if self._in_corridor(o, lane) and o.xmin < ahead and o.x_end(p.obs_min_len) > back:
                    info['blocked_by'] = lane
                    return 'wait', info
        for o in real:                                      # 출구 차선: 섬 앞 끝까지 쭉 비어야 함 (옮긴 뒤 또 막히지 않게)
            if self._in_corridor(o, ex) and o.x_end(p.obs_min_len) > back and o.xmin < p.section_length:
                info['blocked_by'] = ex
                return 'wait', info
        return 'go', info

    # ---------- 한 주기 ----------
    # ---------- 카메라 차선 보정 ----------
    def lane_fix(self, lane, t=None):
        """팀 lane_node 의 /lane/status (dict) 로 옆 위치·방향 보정. 쓰면 True.

        offset_mm  : 기준점 앞 lane_row 행에서 차로 중심의 횡위치 (왼쪽 +, 차 기준)
        heading_rad: 차로 방향 (왼쪽 +, 차 기준)  →  차 기준 차로 직선에서 차의 옆 위치
                     y_car = -off*cos(h) + row*sin(h),  차의 방향(차로 기준) = -h
        같은 차로를 봤다고 확신할 때만 쓴다: 양쪽 선(BOTH), 방향 작음, 지금 추정과 가까움."""
        p = self.p
        if self.state != 'RUN' or not lane:
            return False
        self.lane_last, self.lane_t = lane, t
        self._lane_id_fix(lane)
        if not p.lane_fix:
            return False
        off, h = lane.get('offset_mm'), lane.get('heading_rad')
        if lane.get('status') != 'BOTH' or lane.get('low_conf') or off is None or h is None:
            return False
        h = float(h)
        if abs(math.degrees(h)) > p.lane_fix_max_deg:
            return False
        off = float(off) / 1000.0
        y_in_lane = -off * math.cos(h) + p.lane_row * math.sin(h)
        lane_idx = self.nearest_lane(self.pose.y)
        dy = self.center(lane_idx) + y_in_lane - self.pose.y
        if abs(dy) > p.lane_fix_gate:
            return False
        dpsi = wrap(-h - self.pose.psi)
        d = p.lane_fix_heading_gain * dpsi if abs(math.degrees(dpsi)) <= 5.0 else 0.0
        self._apply_transform(d, p.lane_fix_gain * dy)
        self.lane_fix_n += 1
        self.lane_fix_dy = dy
        return True

    def _lane_id_fix(self, lane):
        """차선 노드의 3차선 번호가 여러 번 같게 나오면, 지금 추정 차로가 다를 때 옆 위치를 그 차로로 옮긴다."""
        p = self.p
        lid = lane.get('lane_id')
        self.lane_ids.append(lid if lid in self.lanes else None)
        self.lane_ids = self.lane_ids[-p.lane_id_window:]
        best = None
        for k in self.lanes:
            if self.lane_ids.count(k) >= p.lane_id_need:
                best = k
        self.lane_id = best
        if not p.lane_id_fix or best is None or self.rb is not None:
            return
        now = self.nearest_lane(self.pose.y)
        if best == now or self.target != now:          # 같은 차로 / 차로 바꾸는 중이면 안 건드림
            return
        h = lane.get('heading_rad')
        off = lane.get('offset_mm')
        if h is None or off is None or abs(math.degrees(float(h))) > p.lane_fix_max_deg:
            return
        off = float(off) / 1000.0
        y_in_lane = -off * math.cos(float(h)) + p.lane_row * math.sin(float(h))
        dy = self.center(best) + y_in_lane - self.pose.y
        self._apply_transform(0.0, dy)
        self.target = best
        self.lane_id_fix_n += 1
        self.lane_ids = []
        self.events.append((self.pose.x, f'카메라 선 종류로 차로 확인: {lane_name(now)} → {lane_name(best)} (옆 {dy * 100:+.0f}cm)'))

    # ---------- 온 길 되돌아가기 ----------
    def _record_trail(self):
        x, y = self.pose.x, self.pose.y
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= self.p.trail_step:
            self.trail.append((x, y))
            if len(self.trail) > 400:
                self.trail = self.trail[-400:]

    def _rollback_step(self, t):
        """지나온 길을 따라 후진. 갈 차로가 생기면 그 차로로 정하고 끝(None). 너무 멀리·오래면 끝."""
        p, rb = self.p, self.rb
        x, y, psi = self.pose.x, self.pose.y, self.pose.psi
        rb['dist'] = max(rb['dist'], math.hypot(x - rb['x0'], y - rb['y0']))
        self.rollback_m = rb['dist']
        obs = self.obstacles(True)
        cur = self.nearest_lane(y)
        nxt = self.choose(cur, obs)
        if nxt is None and self.free_ahead(cur, obs) >= p.trigger_dist and cur != rb['blocked']:
            nxt = cur                                  # 지금 차로 앞이 충분히 비었으면 그대로
        if nxt is not None and nxt != cur:
            # 옆 차로로 틀 공간이 생길 때까지는 더 물러난다 (차체가 지금 차로 장애물 옆을 벗어나는 데 필요한 앞 거리)
            need = shift_distance(p, nxt - cur)[0] + 0.05
            if self.free_ahead(cur, obs) < need:
                nxt = None
        if nxt is not None and rb['dist'] >= 0.05:
            self.events.append((x, f'되돌아가기 {rb["dist"]:.2f}m → 갈 길 찾음: {lane_name(nxt)}'))
            self.target = nxt
            self.last_switch = t
            self.rb = None
            self.stuck_since = None
            return None
        while len(self.trail) > 1 and math.hypot(self.trail[-1][0] - x, self.trail[-1][1] - y) < p.rollback_lookahead:
            self.trail.pop()
        if (rb['dist'] >= p.rollback_max_dist or t - rb['t0'] >= p.rollback_time or not self.trail):
            self.events.append((x, f'되돌아가기 끝 ({rb["dist"]:.2f}m, 갈 길 없음) → 다시 계획'))
            self.rb = None
            self.stuck_since = None
            self.last_switch = t - p.hold_time
            return None
        tx, ty = self.trail[-1]
        dx, dy = tx - x, ty - y
        xr = dx * math.cos(psi) + dy * math.sin(psi)
        yr = -dx * math.sin(psi) + dy * math.cos(psi)
        d2 = xr * xr + yr * yr
        k = 0.0 if d2 < 1e-6 else 2.0 * yr / d2
        kmax = 1.0 / p.min_turn_radius
        k = max(-kmax, min(kmax, k))
        v = -p.recover_speed
        return v, v * k

    # ---------- 멈춤 탈출 ----------
    def _recover_side(self, obs):
        """후진하며 머리를 돌릴 쪽 (+1 왼쪽, -1 오른쪽).

        차로를 옮기는 중이면(목표 차로 ≠ 지금 차로) 가던 차로 쪽 (옮기다 모서리에 걸린 것).
        아니면 갈 수 있는 차로 쪽, 그것도 없으면 더 비어 있는 옆 차로 쪽."""
        y = self.pose.y
        now = self.nearest_lane(y)
        want = self.target if self.target != now else self.choose(now, obs)
        if want is None or want == now:
            best, best_f = None, -1.0
            for lane in (now - 1, now + 1):
                if lane in self.lanes:
                    f = self.free_ahead(lane, obs)
                    if f > best_f:
                        best, best_f = lane, f
            want = best
        if want is None:
            return 1 if y < 0 else -1
        d = self.center(want) - y
        if abs(d) < 1e-6:
            return 1 if want > now else -1
        return 1 if d > 0 else -1

    def _recover_limit(self):
        p = self.p
        if p.recover_mode == 'trail':
            return p.recover_max + p.recover_swing_max
        return p.recover_max

    def _recover_step(self, t):
        """후진하며 크게 꺾어 머리를 갈 쪽으로 돌린다. 끝나면 None."""
        p, r = self.p, self.rec
        back = math.hypot(self.pose.x - r['x0'], self.pose.y - r['y0'])
        if back >= p.recover_dist or t - r['t0'] >= p.recover_time:
            self.events.append((self.pose.x, f'탈출 끝 ({back:.2f}m 후진) → 다시 계획'))
            self.rec = None
            self.stuck_since = None
            self.last_switch = t - p.hold_time        # 바로 차로를 다시 고를 수 있게
            return None
        v = -p.recover_speed
        k = -r['side'] / p.min_turn_radius            # 후진 + 반대 조향 → 머리가 side 쪽으로 돈다
        return v, v * k

    def step(self, t):
        """→ (v, w).  20Hz 정도로 호출"""
        p = self.p
        if self.state != 'RUN':
            return 0.0, 0.0
        if self.rec is not None:
            out = self._recover_step(t)
            if out is not None:
                return out
        if self.rb is not None:
            out = self._rollback_step(t)
            if out is not None:
                return out
        self._record_trail()
        x, y = self.pose.x, self.pose.y

        obs = self.obstacles(True)
        cur = self.target
        f_cur = self.free_ahead(cur, obs)

        # ① 내 차선 앞이 막혔나? → ② 갈 차선 고르기
        if f_cur < p.trigger_dist and t - self.last_switch >= p.hold_time:
            nxt = self.choose(cur, obs)
            if nxt is None and self.exit_pref is not None:
                obs2 = self.obstacles(False)            # 출구 선호 때문에 막혔으면 선호 포기
                nxt = self.choose(cur, obs2)
                if nxt is not None:
                    self._given_up_for = self.exit_pref
                    self.exit_pref = None
                    self.exit_info = {'plan': 'given_up'}
                    self.events.append((x, '출구선호 포기'))
                    obs = obs2
            if nxt is None and self.exit_pref is not None:
                # 지금 차선이 '출구 선호' 가상벽 때문에만 막힌 거면 → 선호 포기하고 그대로 직진
                obs2 = self.obstacles(False)
                if self.free_ahead(cur, obs2) > self.free_ahead(cur, obs) + 1e-6:
                    self._given_up_for = self.exit_pref
                    self.exit_pref = None
                    self.exit_info = {'plan': 'given_up'}
                    self.events.append((x, '출구선호 포기(현재 차선 유지)'))
                    obs = obs2
            if nxt is not None:
                self.events.append((x, f'차선 {cur:+d} → {nxt:+d}  (앞 {f_cur:.2f}m)'))
                self.target = nxt
                self.last_switch = t

        # ②' 갈림길 방향을 알면 섬 앞 끝 전에 출구 차선으로 미리 옮김 (0/1/2차선)
        if p.fork_shift and self.exit_pref is not None and self.target != self.exit_pref \
                and t - self.last_switch >= p.hold_time and t - self.t_start >= p.exit_min_time:
            decision, info = self.exit_plan(obs)
            self.exit_info = dict(info, plan=decision)
            if decision == 'go':
                self.events.append((x, f'갈림길 출구로 이동: {lane_name(self.target)} → {lane_name(self.exit_pref)} '
                                        f'({abs(info["need_lanes"])}차선, 섬까지 {info["rem_m"]:.2f}m / 필요 {info["need_m"]:.2f}m)'))
                self.target = self.exit_pref
                self.last_switch = t
            elif decision == 'too_late' and not self._late_logged:
                self._late_logged = True
                self.events.append((x, f'갈림길 출구 이동 불가: 섬까지 {info["rem_m"]:.2f}m < 필요 {info["need_m"]:.2f}m '
                                        f'({lane_name(self.target)} 유지)'))
        elif self.exit_pref is not None and self.target == self.exit_pref:
            self.exit_info = {'need_lanes': 0, 'plan': 'keep'}

        # ③ 속도
        in_lane = abs(y - self.center(self.target)) < p.reach_tol
        v = p.v_cruise if in_lane else p.v_change
        f_t = self.free_ahead(self.target, obs)
        self.blocked = f_t < p.stop_dist
        if self.blocked or self.emergency:
            v = 0.0
            # 서 있으면 상황이 안 바뀐다 → 잠시 뒤 후진하며 크게 꺾어 다시 잡기 (횟수 제한)
            if self.stuck_since is None:
                self.stuck_since = t
            elif (p.recover and p.recover_mode == 'trail' and self.rec_n < p.recover_max
                  and t - self.stuck_since >= p.recover_after and len(self.trail) > 1):
                self.rb = {'t0': t, 'x0': x, 'y0': y, 'dist': 0.0, 'blocked': self.target}
                self.rec_n += 1
                why = '앞 비상' if self.emergency else '갈 차선 막힘'
                self.events.append((x, f'멈춤 {t - self.stuck_since:.1f}s ({why}) → 온 길 되돌아가기 '
                                       f'{self.rec_n}/{p.recover_max} (최대 {p.rollback_max_dist:.1f}m, 갈 길 생기면 멈춤)'))
                out = self._rollback_step(t)
                if out is not None:
                    return out
            elif (p.recover and t - self.stuck_since >= p.recover_after
                  and self.rec_n < self._recover_limit()):
                # swing: trail을 다 썼거나(또는 recover_mode: swing) 지나온 길이 없음 -> 후진하며 최대 조향으로 머리를 빈 쪽으로
                side = self._recover_side(obs)
                if self.rec_side_last == side and self.rec_same_side >= 2:
                    side = -side                       # 같은 쪽으로 두 번 돌려도 안 되면 반대쪽도 해 본다
                self.rec_same_side = self.rec_same_side + 1 if side == self.rec_side_last else 1
                self.rec_side_last = side
                self.rec = {'t0': t, 'x0': x, 'y0': y, 'side': side}
                self.rec_n += 1
                why = '앞 비상' if self.emergency else '갈 차선 막힘'
                self.events.append((x, f'멈춤 {t - self.stuck_since:.1f}s ({why}) → 탈출 {self.rec_n}/{self._recover_limit()}: '
                                       f'{p.recover_dist:.2f}m 후진하며 {"왼쪽" if side > 0 else "오른쪽"}으로 최대 조향'))
                return self._recover_step(t)
        else:
            self.stuck_since = None

        # ④ 끝났나?
        L = p.section_length
        if (x > L + p.done_margin and in_lane) or x > L + p.island_length:
            self.state = 'DONE'
            self.events.append((x, f'완료 (차선 {self.target:+d})'))
            return 0.0, 0.0

        return v, self.control(v, t, in_lane)

    # ---------- pure pursuit (직선 y = 목표차선중심 따라가기) ----------
    def control(self, v, t=None, in_lane=False):
        p = self.p
        y, psi = self.pose.y, self.pose.psi
        Ld = p.lookahead
        dy = self.center(self.target) - y
        # 목표점 (Ld, dy) 을 로봇 기준으로 회전
        xr = Ld * math.cos(psi) + dy * math.sin(psi)
        yr = -Ld * math.sin(psi) + dy * math.cos(psi)
        k = 2.0 * yr / (xr * xr + yr * yr)
        kmax = 1.0 / p.min_turn_radius
        k = max(-kmax, min(kmax, k))
        mh = math.radians(p.max_heading_deg)
        if (psi > mh and k > 0) or (psi < -mh and k < 0):
            k = 0.0
        # 차로 안에서 똑바로 갈 때는 팀 차선 노드 곡률을 섞는다 (양쪽 선 보임, 같은 차로, 최근 값)
        ln = self.lane_last
        self.blend_used = False
        if (p.lane_keep_blend > 0 and in_lane and ln and t is not None and self.lane_t is not None
                and t - self.lane_t <= p.lane_fresh_sec and ln.get('status') == 'BOTH' and not ln.get('low_conf')
                and ln.get('kappa') is not None and self.target == self.nearest_lane(y)
                and (self.lane_id is None or self.lane_id == self.target)):
            a = p.lane_keep_blend
            k = max(-kmax, min(kmax, (1.0 - a) * k + a * float(ln['kappa'])))
            self.blend_used = True
        return v * k

    def status(self):
        return {
            'state': self.state,
            'x': round(self.pose.x, 3), 'y': round(self.pose.y, 3),
            'psi_deg': round(math.degrees(self.pose.psi), 1),
            'target': self.target,
            'now_lane': self.nearest_lane(self.pose.y),
            'emergency': self.emergency, 'blocked': self.blocked,
            'recovering': self.rec is not None or self.rb is not None, 'recover_n': self.rec_n,
            'rollback': self.rb is not None, 'rollback_m': round(self.rollback_m, 2),
            'cam_lane': self.lane_id, 'lane_id_fix_n': self.lane_id_fix_n,
            'lane_blend': getattr(self, 'blend_used', False),
            'lane_fix_n': self.lane_fix_n, 'lane_fix_dy_cm': round(self.lane_fix_dy * 100, 1),
            'exit_pref': self.exit_pref,
            'n_obs': len([o for o in self.obs if o.hits >= self.p.confirm_hits]),
            'align_dy_cm': round(self.align_dy * 100, 1),
            'align_deg': round(math.degrees(self.align_th), 2),
            'section_length': round(self.p.section_length, 3),
            'exit_plan': self.exit_info.get('plan'),
            'exit_need_lanes': self.exit_info.get('need_lanes'),
            'exit_need_m': self.exit_info.get('need_m'),
            'exit_rem_m': self.exit_info.get('rem_m'),
        }
