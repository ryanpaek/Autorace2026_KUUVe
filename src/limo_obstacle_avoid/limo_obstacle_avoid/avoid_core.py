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
    lane_width: float = 0.55          # 차선 폭 (규정 550mm)
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
    obs_min_len: float = 0.33         # 앞면만 보여도 이 길이만큼 차가 있다고 가정 (LIMO 길이)
    confirm_hits: int = 3             # 이 횟수 이상 보여야 진짜 장애물로 인정
    # ── 판단 ──
    trigger_dist: float = 1.0         # 내 차선 앞 장애물이 (앞범퍼 기준) 이 안이면 차선 변경
    change_len: float = 1.0           # 차선 1칸 옮기는 데 필요한 앞쪽 여유
    clear_back: float = 0.0           # 옮길 차선의 차 끝이 (뒤범퍼 - 이값)보다 뒤여야 들어감 (음수=더 일찍)
    better_margin: float = 0.3        # 옮길 차선이 지금보다 이만큼 더 비어야 옮김
    hold_time: float = 0.8            # 차선 바꾼 뒤 최소 유지 시간 [s]
    stop_dist: float = 0.25           # 갈 곳 없고 앞이 이만큼 가까우면 정지
    # ── 제어 (pure pursuit) ──
    v_cruise: float = 0.25            # 차선 안에서 속도
    v_change: float = 0.20            # 차선 변경 중 속도
    lookahead: float = 0.50           # pure pursuit 전방주시거리
    min_turn_radius: float = 0.53     # LIMO 아커만 최소 회전반경 실측값 (이보다 작게 두면 못 도는 경로를 계획)
    max_heading_deg: float = 40.0     # 이 이상 비틀어지면 더 꺾지 않음
    reach_tol: float = 0.08           # 차선 중심에서 이 안이면 '도착'
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

    def start(self, ox, oy, yaw, t=0.0):
        self.reset()
        self.pose.reset(ox, oy, yaw, y0=self.p.start_lane * self.p.lane_width)
        self.state = 'RUN'
        self.last_switch = t - self.p.hold_time

    def set_exit(self, s):
        self.exit_pref = parse_exit(s)

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

        emerg = False
        clusters, cur = [], []
        prev = None

        def close():
            if len(cur) >= p.cluster_min_pts:
                xs = [q[0] for q in cur]
                ys = [q[1] for q in cur]
                clusters.append((min(xs), max(xs), min(ys), max(ys)))
            cur.clear()

        for pt in pts_base:
            if pt is None:
                close(); prev = None
                continue
            px, py = pt
            # 비상정지 박스: 앞범퍼 바로 앞, 차폭 안
            if p.robot_front < px < p.robot_front + p.stop_dist and abs(py) < p.robot_half_width:
                emerg = True
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

        self.emergency = emerg
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

    # ---------- 한 주기 ----------
    def step(self, t):
        """→ (v, w).  20Hz 정도로 호출"""
        p = self.p
        if self.state != 'RUN':
            return 0.0, 0.0
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
                    self.exit_pref = None
                    self.events.append((x, '출구선호 포기'))
                    obs = obs2
            if nxt is None and self.exit_pref is not None:
                # 지금 차선이 '출구 선호' 가상벽 때문에만 막힌 거면 → 선호 포기하고 그대로 직진
                obs2 = self.obstacles(False)
                if self.free_ahead(cur, obs2) > self.free_ahead(cur, obs) + 1e-6:
                    self.exit_pref = None
                    self.events.append((x, '출구선호 포기(현재 차선 유지)'))
                    obs = obs2
            if nxt is not None:
                self.events.append((x, f'차선 {cur:+d} → {nxt:+d}  (앞 {f_cur:.2f}m)'))
                self.target = nxt
                self.last_switch = t

        # ③ 속도
        in_lane = abs(y - self.center(self.target)) < p.reach_tol
        v = p.v_cruise if in_lane else p.v_change
        f_t = self.free_ahead(self.target, obs)
        self.blocked = f_t < p.stop_dist
        if self.blocked or self.emergency:
            v = 0.0

        # ④ 끝났나?
        L = p.section_length
        if (x > L + p.done_margin and in_lane) or x > L + p.island_length:
            self.state = 'DONE'
            self.events.append((x, f'완료 (차선 {self.target:+d})'))
            return 0.0, 0.0

        return v, self.control(v)

    # ---------- pure pursuit (직선 y = 목표차선중심 따라가기) ----------
    def control(self, v):
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
        return v * k

    def status(self):
        return {
            'state': self.state,
            'x': round(self.pose.x, 3), 'y': round(self.pose.y, 3),
            'psi_deg': round(math.degrees(self.pose.psi), 1),
            'target': self.target,
            'now_lane': self.nearest_lane(self.pose.y),
            'emergency': self.emergency, 'blocked': self.blocked,
            'exit_pref': self.exit_pref,
            'n_obs': len([o for o in self.obs if o.hits >= self.p.confirm_hits]),
            'align_dy_cm': round(self.align_dy * 100, 1),
            'align_deg': round(math.degrees(self.align_th), 2),
        }
