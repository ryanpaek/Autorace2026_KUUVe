"""M8 라바콘 회피 핵심 로직 (ROS 의존성 없음).

흐름
  1) 라이다 점 -> 작은 덩어리 = 콘 후보                         cone_candidates()
  2) 월드 좌표(오도메트리 거리 + IMU yaw로 적분한 위치)에 쌓아 추적  ConeTracker
     지나간 콘도 기억해서 차 옆·뒤 콘까지 경로 계산에 쓴다.
     카메라가 "그 자리에 주황색이 없다"를 여러 번 말하면 그 콘은 뺀다 (기둥·사람 다리 거르기).
  3) 라바콘 모드에 들어가는 순간 '구간 좌표계'를 고정            Corridor
     x = 라바콘 구간 직선 방향(도면상 북쪽), y = 왼쪽 +, 원점 = 진입 순간 차 기준점(뒤차축)
     y_lane = 그때 차선 노드가 본 차로 중심 (출구에서 여기로 돌아온다)
  4) 좌/우 콘 쌍(게이트)의 중점을 잇고, 짝 없는 콘은 옆으로 비켜 가는 점을 넣어 경로를 만든다
     -> 콘과의 여유 거리를 밀어내기로 확보 -> 부드럽게             plan_path()
  5) 경로를 Pure Pursuit로 따라간다 (조향 지연 보상)              ConeFollower.control()
  6) 마지막 콘을 뒤차축이 지나면 done -> 미션 노드가 차선 주행으로 넘긴다

좌표·단위: mm, rad. 차량 좌표 x 전방 +, y 왼쪽 + (ROS). 곡률 kappa는 1/m, 왼쪽 +.
배치가 슬라럼(콘 하나씩 좌우 교대)이어도 같은 코드로 돈다: 짝 없는 콘은 한쪽으로 비켜 간다.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import asdict, dataclass, field, fields
from typing import Dict, List, Optional, Tuple

import numpy as np

from autorace_obstacle.scan_utils import cluster_groups


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# 파라미터
# ---------------------------------------------------------------------------
@dataclass
class ConeParams:
    # --- 콘 후보 (라이다) ---
    cluster_gap_mm: float = 50.0        # 이웃 점 간격이 이보다 크면 다른 덩어리
    cone_min_pts: int = 2               # 콘 위쪽은 가늘어서 점이 2~3개만 맞을 수 있다
    cone_max_size_mm: float = 140.0     # 덩어리 크기(끝점 사이) 상한. LIMO(폭 220)·벽은 여기서 걸러진다
    cone_max_range_mm: float = 2600.0   # 라이다에서 이보다 먼 덩어리는 안 쓴다
    cone_min_range_mm: float = 120.0    # 이보다 가까운 점 덩어리는 차체(가림 경계)로 보고 버린다
    lidar_fov_deg: float = 240.0        # LIMO 라이다는 뒤쪽이 가려져 약 240도. '보여야 정상' 판단에만 씀
    cone_scan_radius_mm: float = 30.0   # TODO 실측: 라이다 높이에서 콘 단면 반지름 (보이는 면 -> 중심 보정)
    cone_base_radius_mm: float = 50.0   # TODO 실측: 콘 바닥 반지름 (부딪힘 여유 계산)
    # --- 추적 ---
    assoc_gate_mm: float = 90.0         # 이 거리 안의 기존 콘과 같은 콘으로 본다
    merge_mm: float = 70.0              # 두 트랙이 이보다 가까우면 하나로 합친다
    confirm_hits: int = 3               # 이만큼 보이면 확정
    drop_misses: int = 6                # 미확정 콘: 보여야 할 때 이만큼 안 보이면 버림
    drop_misses_confirmed: int = 30     # 확정 콘 (가림 때문에 크게)
    visible_range_mm: float = 1800.0    # 이 안에 있는 앞쪽 콘은 '보여야 정상'으로 친다
    # --- 카메라 확인 (주황색) ---
    cam_min_checks: int = 4             # 이만큼 확인한 뒤에야 거부할 수 있다
    cam_min_orange_frac: float = 0.25   # 확인한 프레임 중 주황으로 보인 비율이 이보다 낮으면 콘 아님
    # --- 경로 ---
    car_half_width_mm: float = 115.0    # LIMO 폭 약 220mm의 절반 + 조금
    clear_margin_mm: float = 40.0       # 차 옆면 ~ 콘 바닥 사이 목표 여유
    gate_min_mm: float = 300.0          # 좌우 콘 쌍(게이트)으로 볼 콘 간격 범위 (중심 간)
    gate_max_mm: float = 1000.0
    gate_max_dx_mm: float = 400.0       # 쌍의 앞뒤 어긋남 상한
    gate_max_slope: float = 0.8         # 쌍 방향이 옆 방향에서 이만큼(|dx|/|dy|) 이상 기울면 쌍 아님
    corridor_width_mm: float = 500.0    # 쌍을 하나도 못 찾았을 때 가정하는 통로 폭 (콘 중심 간)
    single_tie_side: str = 'right'      # 가운데 놓인 콘 하나: 'right'면 콘의 오른쪽으로 지나간다
    single_tie_mm: float = 40.0
    zone_half_width_mm: float = 650.0   # 구간 좌표계에서 y_lane ± 이 안의 콘만 쓴다
    zone_x_min_mm: float = -300.0
    zone_x_max_mm: float = 3300.0
    pre_gate_mm: float = 250.0          # 첫 게이트 이만큼 앞에 같은 높이 점을 넣어 똑바로 들어가게
    exit_blend_mm: float = 350.0        # 마지막 콘 뒤 이 거리에서 차로 중심(y_lane)으로 돌아온다
    path_step_mm: float = 40.0
    # --- 추종 ---
    lookahead_mm: float = 260.0         # 뒤차축 -> 목표점. 시뮬레이션 220~300이 좋음 (길면 콘 쪽으로 질러 감)
    # 목표점으로 가는 원호가 콘에 이보다 가까우면(차 중심 ~ 콘 중심 = 반폭 + 바닥 반지름 + 이 값)
    # 목표점을 당겨서 다시 본다. Pure Pursuit가 코너를 안쪽으로 질러 콘을 치는 것을 막는다
    arc_margin_mm: float = 15.0
    lookahead_scales: Tuple[float, ...] = (1.0, 0.8, 0.65, 0.5)
    kappa_max: float = 2.1              # 명령 한계 (브리지 축거 0.2 기준 조향 0.40 rad)
    kappa_rate_max: float = 0.5         # 제어 주기당 곡률 변화 한계 (20Hz면 0.2초에 끝까지)
    latency_sec: float = 0.35           # 명령 -> 실제 회전 지연 (11/02 측정 0.30~0.40)
    speed: float = 0.15                 # m/s
    min_speed: float = 0.10
    curve_slowdown: float = 0.35        # |kappa| = kappa_max일 때 35% 감속
    front_axle_mm: float = 200.0        # 뒤차축 -> 앞차축
    # --- 종료 ---
    done_margin_mm: float = 150.0       # 마지막 콘 x + 이 거리를 뒤차축이 지나면 끝
    done_min_progress_mm: float = 1200.0
    done_max_progress_mm: float = 2700.0  # 콘을 하나도 못 봐도 진입점에서 이만큼 가면 끝

    @property
    def min_pass_mm(self) -> float:
        """차 중심 ~ 콘 중심 최소 거리."""
        return self.car_half_width_mm + self.cone_base_radius_mm + self.clear_margin_mm

    @classmethod
    def from_dict(cls, d: dict) -> 'ConeParams':
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})

    def as_dict(self) -> dict:
        return asdict(self)


# ---------------------------------------------------------------------------
# 위치 (월드 좌표 = 오도메트리 시작점 기준, yaw는 IMU)
# ---------------------------------------------------------------------------
@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def to_world(pose: Pose2D, pts) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.stack([pose.x + c * pts[:, 0] - s * pts[:, 1],
                     pose.y + s * pts[:, 0] + c * pts[:, 1]], axis=1)


def to_vehicle(pose: Pose2D, pts) -> np.ndarray:
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    dx, dy = pts[:, 0] - pose.x, pts[:, 1] - pose.y
    return np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1)


def advance(pose: Pose2D, dist_mm: float, kappa: float) -> Pose2D:
    """곡률 kappa(1/m)로 dist_mm 만큼 간 위치 (지연 보상용 예측)."""
    k = kappa / 1000.0
    if abs(k * dist_mm) < 1e-6:
        return Pose2D(pose.x + dist_mm * math.cos(pose.yaw),
                      pose.y + dist_mm * math.sin(pose.yaw), pose.yaw)
    yaw1 = pose.yaw + k * dist_mm
    return Pose2D(pose.x + (math.sin(yaw1) - math.sin(pose.yaw)) / k,
                  pose.y - (math.cos(yaw1) - math.cos(pose.yaw)) / k, wrap(yaw1))


class PoseIntegrator:
    """오도메트리 이동 거리 + IMU yaw로 위치를 적분한다.

    /odom의 yaw는 중간 조향에서 약 12% 틀려서(11/02 캘리브레이션) 위치도 같이 휜다.
    거리만 /odom에서 가져오고 방향은 IMU로 다시 적분한다 (tools/path_recorder의 xf, yf와 같은 생각).
    """

    def __init__(self, odom_scale: float = 1.0):
        self.odom_scale = odom_scale
        self.pose = Pose2D()
        self.yaw_imu: Optional[float] = None
        self._last_xy = None
        self._yaw_at_last = None
        self.dist_mm = 0.0

    @property
    def ready(self) -> bool:
        return self.yaw_imu is not None and self._last_xy is not None

    def on_imu(self, yaw: float) -> None:
        self.yaw_imu = yaw
        self.pose.yaw = yaw

    def on_odom(self, x_m: float, y_m: float, v: float = 0.0) -> None:
        if self._last_xy is None or self.yaw_imu is None:
            self._last_xy = (x_m, y_m)
            self._yaw_at_last = self.yaw_imu
            return
        ds = 1000.0 * math.hypot(x_m - self._last_xy[0], y_m - self._last_xy[1]) * self.odom_scale
        if v < -0.01:
            ds = -ds
        y0 = self._yaw_at_last if self._yaw_at_last is not None else self.yaw_imu
        ym = y0 + 0.5 * wrap(self.yaw_imu - y0)
        self.pose.x += ds * math.cos(ym)
        self.pose.y += ds * math.sin(ym)
        self.pose.yaw = self.yaw_imu
        self.dist_mm += abs(ds)
        self._last_xy = (x_m, y_m)
        self._yaw_at_last = self.yaw_imu


# ---------------------------------------------------------------------------
# 1) 콘 후보
# ---------------------------------------------------------------------------
def cone_candidates(points: np.ndarray, p: ConeParams,
                    lidar_xy: Tuple[float, float] = (0.0, 0.0)) -> List[dict]:
    """차량 좌표 점 (n, 2) -> 콘 중심 추정 목록 [{'x', 'y', 'size', 'n', 'range'}].

    라이다는 콘의 앞면만 보므로, 가장 가까운 점에서 광선 방향으로 단면 반지름만큼 뒤를 중심으로 본다.
    """
    out = []
    if points is None or len(points) == 0:
        return out
    lx, ly = lidar_xy
    for g in cluster_groups(points, p.cluster_gap_mm, p.cone_min_pts, origin=lidar_xy):
        size = float(np.hypot(*(g.max(axis=0) - g.min(axis=0))))
        if size > p.cone_max_size_mm:
            continue
        rel = g - np.array([lx, ly])
        r = np.hypot(rel[:, 0], rel[:, 1])
        i = int(np.argmin(r))
        rng = float(r[i])
        if rng > p.cone_max_range_mm or rng < p.cone_min_range_mm:
            continue
        u = rel[i] / rng
        c = g[i] + u * p.cone_scan_radius_mm
        out.append({'x': float(c[0]), 'y': float(c[1]), 'size': size, 'n': int(len(g)),
                    'range': rng})
    return out


# ---------------------------------------------------------------------------
# 2) 추적
# ---------------------------------------------------------------------------
@dataclass
class ConeTrack:
    id: int
    x: float                  # 월드 좌표 (mm)
    y: float
    hits: int = 1
    misses: int = 0
    confirmed: bool = False
    cam_checks: int = 0
    cam_orange: int = 0
    rejected: bool = False    # 카메라가 콘이 아니라고 판단

    @property
    def ok(self) -> bool:
        return self.confirmed and not self.rejected

    def add_camera(self, orange: bool, p: ConeParams) -> None:
        self.cam_checks += 1
        self.cam_orange += int(bool(orange))
        self.rejected = (self.cam_checks >= p.cam_min_checks and
                         self.cam_orange < p.cam_min_orange_frac * self.cam_checks)


class ConeTracker:
    def __init__(self, p: ConeParams):
        self.p = p
        self.tracks: List[ConeTrack] = []
        self._next_id = 0

    def reset(self) -> None:
        self.tracks = []
        self._next_id = 0

    def update(self, dets_world: np.ndarray, pose: Pose2D,
               lidar_xy: Tuple[float, float] = (0.0, 0.0)) -> None:
        p = self.p
        dets = np.asarray(dets_world, dtype=np.float64).reshape(-1, 2)
        pairs = []
        for ti, t in enumerate(self.tracks):
            if len(dets) == 0:
                break
            d = np.hypot(dets[:, 0] - t.x, dets[:, 1] - t.y)
            for di in np.nonzero(d <= p.assoc_gate_mm)[0]:
                pairs.append((float(d[di]), ti, int(di)))
        pairs.sort()
        used_t, used_d = set(), set()
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t = self.tracks[ti]
            w = 1.0 / min(t.hits + 1, 10)        # 평균, 오래된 콘은 천천히만 움직임
            t.x += w * (dets[di, 0] - t.x)
            t.y += w * (dets[di, 1] - t.y)
            t.hits += 1
            t.misses = 0
            t.confirmed = t.confirmed or t.hits >= p.confirm_hits
        n_before = len(self.tracks)
        for di in range(len(dets)):
            if di not in used_d:
                self.tracks.append(ConeTrack(self._next_id, float(dets[di, 0]),
                                             float(dets[di, 1])))
                self._next_id += 1
        # 보여야 하는데 안 보인 트랙 (이번에 새로 생긴 트랙은 제외)
        # 시야 가장자리·옆·뒤 콘은 안 보여도 지우지 않는다 -> 지나간 콘은 기억으로 유지
        unmatched = [ti for ti in range(n_before) if ti not in used_t]
        if unmatched:
            pv = to_vehicle(pose, [[t.x, t.y] for t in self.tracks])
            rel = pv - np.array(lidar_xy)
            rng = np.hypot(rel[:, 0], rel[:, 1])
            brg = np.arctan2(rel[:, 1], rel[:, 0])
            vis_half = math.radians(min(90.0, p.lidar_fov_deg / 2.0 - 15.0))
            for ti in unmatched:
                if (rng[ti] > p.visible_range_mm or rng[ti] < p.cone_min_range_mm
                        or abs(brg[ti]) > vis_half):
                    continue
                if self._occluded(ti, rng, brg):
                    continue
                self.tracks[ti].misses += 1
        keep = []
        for t in self.tracks:
            lim = p.drop_misses_confirmed if t.confirmed else p.drop_misses
            if t.misses < lim:
                keep.append(t)
        self.tracks = self._merge(keep)

    def _occluded(self, ti: int, rng: np.ndarray, brg: np.ndarray) -> bool:
        """더 가까운 콘이 같은 방향을 가리면 안 보여도 정상."""
        r = self.p.cone_base_radius_mm
        for tj, t in enumerate(self.tracks):
            if tj == ti or rng[tj] >= rng[ti]:
                continue
            half = math.atan2(1.5 * r, max(rng[tj], 1.0))
            if abs(wrap(brg[ti] - brg[tj])) < half:
                return True
        return False

    def _merge(self, tracks: List[ConeTrack]) -> List[ConeTrack]:
        out: List[ConeTrack] = []
        for t in sorted(tracks, key=lambda t: -t.hits):
            if any(math.hypot(t.x - o.x, t.y - o.y) < self.p.merge_mm for o in out):
                continue
            out.append(t)
        return out

    def ok_tracks(self) -> List[ConeTrack]:
        return [t for t in self.tracks if t.ok]


# ---------------------------------------------------------------------------
# 3) 구간 좌표계
# ---------------------------------------------------------------------------
@dataclass
class Corridor:
    ox: float                 # 원점 = 진입 순간 뒤차축 (월드)
    oy: float
    axis: float               # 구간 직선 방향 (월드 yaw)
    y_lane: float = 0.0       # 차로 중심 (구간 좌표 y)
    source: str = ''          # axis를 어디서 얻었나: 'route' | 'lane' | 'yaw'

    def to_c(self, pts_w) -> np.ndarray:
        return to_vehicle(Pose2D(self.ox, self.oy, self.axis), pts_w)

    def to_w(self, pts_c) -> np.ndarray:
        return to_world(Pose2D(self.ox, self.oy, self.axis), pts_c)

    def pose_c(self, pose: Pose2D) -> Pose2D:
        q = self.to_c([[pose.x, pose.y]])[0]
        return Pose2D(float(q[0]), float(q[1]), wrap(pose.yaw - self.axis))


class YawOffsetEstimator:
    """IMU yaw와 도면 방향의 차이(offset = IMU - 도면)를 직선 구간에서 배운다.

    차선 노드가 본 차로 방향(heading_rad) + IMU yaw = 차로의 IMU 기준 방향.
    도면 경로 지도가 알려 주는 그 자리 차로 방향과 빼면 offset. IMU 드리프트가 0°/분이라
    한 번 배우면 라바콘 구간 방향(도면상 북쪽)을 IMU 기준으로 정확히 안다.
    """

    def __init__(self, maxlen: int = 400, min_samples: int = 30):
        self.buf: deque = deque(maxlen=maxlen)
        self.min_samples = min_samples

    def add(self, imu_yaw: float, lane_heading: float, route_heading: float) -> None:
        self.buf.append(wrap(imu_yaw + lane_heading - route_heading))

    def offset(self) -> Optional[float]:
        if len(self.buf) < self.min_samples:
            return None
        a = np.array(self.buf)
        ref = a[0]
        rel = np.array([wrap(v - ref) for v in a])
        return wrap(ref + float(np.median(rel)))

    def spread(self) -> Optional[float]:
        if len(self.buf) < self.min_samples:
            return None
        off = self.offset()
        return float(np.median(np.abs([wrap(v - off) for v in self.buf])))


# ---------------------------------------------------------------------------
# 4) 경로
# ---------------------------------------------------------------------------
@dataclass
class Plan:
    path: np.ndarray                      # (m, 2) 구간 좌표, 진행 순
    gates: List[Tuple[int, int]] = field(default_factory=list)   # (왼쪽 콘, 오른쪽 콘) 인덱스
    singles: List[Tuple[int, str]] = field(default_factory=list)  # (콘, 'L'|'R' = 그 콘이 왼/오른쪽 경계)
    waypoints: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    x_last: Optional[float] = None
    width_mm: float = 0.0
    cones: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))   # 계획에 쓴 콘 (구간 좌표)


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-9), 0.0, 1.0))
    return float(np.hypot(*(a + ab * t - p)))


def find_gates(cones: np.ndarray, p: ConeParams) -> List[Tuple[int, int, float]]:
    """좌/우 콘 쌍. 반환 [(왼쪽 i, 오른쪽 j, 폭)]. 한 콘은 최대 두 쌍까지."""
    n = len(cones)
    cand = []
    for i in range(n):
        for j in range(i + 1, n):
            dx = cones[j, 0] - cones[i, 0]
            dy = cones[j, 1] - cones[i, 1]
            lat = abs(dy)
            if not (p.gate_min_mm <= lat <= p.gate_max_mm):
                continue
            if abs(dx) > p.gate_max_dx_mm or abs(dx) > p.gate_max_slope * lat:
                continue
            a, b = cones[i], cones[j]
            blocked = any(_seg_dist(cones[k], a, b) < p.car_half_width_mm + p.cone_base_radius_mm
                          for k in range(n) if k not in (i, j))
            if blocked:
                continue
            left, right = (i, j) if dy < 0 else (j, i)
            cand.append((abs(dx) + 0.3 * abs(lat - p.corridor_width_mm), left, right, lat))
    cand.sort()
    use: Dict[int, int] = {}
    out = []
    for _, li, ri, lat in cand:
        if use.get(li, 0) >= 2 or use.get(ri, 0) >= 2:
            continue
        use[li] = use.get(li, 0) + 1
        use[ri] = use.get(ri, 0) + 1
        out.append((li, ri, lat))
    return _drop_outlier_gates(cones, out)


def _drop_outlier_gates(cones: np.ndarray, gates: List[Tuple[int, int, float]],
                        width_tol: float = 0.4, mid_tol: float = 0.5):
    """다른 게이트들과 어긋나는 게이트를 뺀다 (게이트가 3개 이상일 때만 판단).

    예: 콘 줄 바깥 기둥이 같은 쪽 콘과 짝지어지면 폭이 다르거나 중점이 줄에서 크게 벗어난다.
    """
    gates = list(gates)
    while len(gates) >= 3:
        w_med = float(np.median([w for _, _, w in gates]))
        mids = np.array([(cones[a] + cones[b]) / 2.0 for a, b, _ in gates])
        worst, worst_err = None, 0.0
        for k, (_, _, w) in enumerate(gates):
            others = np.delete(mids, k, axis=0)
            order = np.argsort(others[:, 0])
            ref = float(np.interp(mids[k, 0], others[order, 0], others[order, 1]))
            err = max(abs(mids[k, 1] - ref) / (mid_tol * w_med),
                      abs(w - w_med) / (width_tol * w_med))
            if err > 1.0 and err > worst_err:
                worst, worst_err = k, err
        if worst is None:
            break
        gates.pop(worst)
    return gates


def _merge_points(pts: List[Tuple[float, float]], dx: float = 80.0,
                  dy: float = 100.0) -> np.ndarray:
    pts = sorted(pts)
    out: List[List[float]] = []
    for x, y in pts:
        if out and x - out[-1][0] < dx and abs(y - out[-1][1]) < dy:
            n = out[-1][2]
            out[-1] = [(out[-1][0] * n + x) / (n + 1), (out[-1][1] * n + y) / (n + 1), n + 1]
        else:
            out.append([x, y, 1])
    return np.array([[a, b] for a, b, _ in out]).reshape(-1, 2)


def _densify(pts: np.ndarray, step: float) -> np.ndarray:
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        d = float(np.hypot(*(b - a)))
        n = max(1, int(math.ceil(d / step)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * (k / n))
    return np.array(out)


def _smooth(path: np.ndarray, passes: int) -> np.ndarray:
    q = path.copy()
    for _ in range(passes):
        q[1:-1] = 0.25 * q[:-2] + 0.5 * q[1:-1] + 0.25 * q[2:]
    return q


def _repel(path: np.ndarray, cones: np.ndarray, need: float, iters: int,
           fixed_head: int = 2) -> np.ndarray:
    """콘에 need보다 가까운 경로 점을 밀어낸다. 양쪽 콘이 동시에 밀면 가운데에 머문다."""
    if len(cones) == 0:
        return path
    q = path.copy()
    for _ in range(iters):
        d = q[:, None, :] - cones[None, :, :]          # (m, n, 2)
        r = np.hypot(d[..., 0], d[..., 1])
        viol = np.clip(need - r, 0.0, None)
        r = np.where(r < 1e-6, 1e-6, r)
        push = (d / r[..., None] * viol[..., None]).sum(axis=1)
        push[:fixed_head] = 0.0
        q = q + 0.5 * push
    return q


def _boundary(pts: List[np.ndarray], x: float, reach: float = 450.0) -> Optional[float]:
    """같은 쪽 게이트 콘들을 이은 경계선의 x 위치 y (범위 밖 reach 넘으면 None)."""
    if not pts:
        return None
    a = np.array(sorted((float(q[0]), float(q[1])) for q in pts))
    if x < a[0, 0] - reach or x > a[-1, 0] + reach:
        return None
    return float(np.interp(x, a[:, 0], a[:, 1]))


def plan_path(cones: np.ndarray, y_lane: float, p: ConeParams,
              y_start: float = 0.0, side_hint: Optional[Dict[int, str]] = None) -> Plan:
    """구간 좌표 콘들 (n, 2) -> 경로.

    y_start: 진입 순간 차 위치 (구간 좌표에서는 원점이라 0). 경로는 (−200, y_start)에서 시작한다.
    side_hint: {콘 인덱스: 'L'|'R'} 지난 계획에서 정한 짝 없는 콘의 방향. 프레임마다 방향이
               뒤집히면 경로가 콘을 가로지르므로 한 번 정하면 유지한다.
    짝 없는 콘 처리
      - 게이트가 있으면: 게이트 중점을 이은 선보다 왼쪽이면 왼쪽 경계('L'), 오른쪽이면 'R'
        단 게이트 콘 줄보다 바깥에 있는 것(기둥·구경꾼 등)은 경로에 영향 없음 (무시)
      - 게이트가 없으면: 직전 경로 점에서 덜 움직이는 쪽으로 비켜 간다 (슬라럼이면 한쪽으로 쭉)
    """
    cones = np.asarray(cones, dtype=np.float64).reshape(-1, 2)
    side_hint = side_hint or {}
    gates = find_gates(cones, p)
    in_gate = {i for a, b, _ in gates for i in (a, b)}
    mids = _merge_points([tuple((cones[a] + cones[b]) / 2.0) for a, b, _ in gates])
    width = float(np.median([w for _, _, w in gates])) if gates else p.corridor_width_mm
    half = max(float(np.clip(width / 2.0, 150.0, 500.0)), p.min_pass_mm)
    lefts = [cones[a] for a, _, _ in gates]
    rights = [cones[r] for _, r, _ in gates]
    y_lim = p.zone_half_width_mm - p.min_pass_mm      # 경로가 y_lane에서 벗어나도 되는 한계

    def ref_y(x: float) -> float:
        if len(mids):
            return float(np.interp(x, mids[:, 0], mids[:, 1]))
        return y_lane

    events = [(float(m[0]), 'mid', m) for m in mids]
    events += [(float(cones[i, 0]), 'single', i) for i in range(len(cones)) if i not in in_gate]
    events.sort(key=lambda e: e[0])
    wps: List[Tuple[float, float]] = []
    singles: List[Tuple[int, str]] = []
    prev_y = y_start
    for x, kind, item in events:
        if kind == 'mid':
            wps.append((float(item[0]), float(item[1])))
            prev_y = float(item[1])
            continue
        i = item
        y = float(cones[i, 1])
        cand = {'L': y - half, 'R': y + half}      # 그 콘이 왼쪽 경계면 콘 오른쪽(y - half)으로
        side = side_hint.get(i)
        if side not in ('L', 'R'):
            r = ref_y(x)
            if len(mids) and abs(y - r) > p.single_tie_mm:
                side = 'L' if y > r else 'R'
            else:
                dl, dr = abs(cand['L'] - prev_y), abs(cand['R'] - prev_y)
                if abs(dl - dr) < p.single_tie_mm:
                    side = 'L' if p.single_tie_side == 'right' else 'R'
                else:
                    side = 'L' if dl < dr else 'R'
            other = 'R' if side == 'L' else 'L'
            if abs(cand[side] - y_lane) > y_lim and abs(cand[other] - y_lane) <= y_lim:
                side = other
        singles.append((i, side))
        if len(mids):
            lb, rb = _boundary(lefts, x), _boundary(rights, x)
            if side == 'L' and lb is not None and y > lb + 60.0:
                continue                            # 왼쪽 콘 줄 바깥: 경로에 영향 없음
            if side == 'R' and rb is not None and y < rb - 60.0:
                continue
        wps.append((x, cand[side]))
        prev_y = cand[side]
    wps_arr = _merge_points(wps)

    pts: List[Tuple[float, float]] = [(-200.0, y_start), (0.0, y_start)]
    if len(wps_arr):
        x1, y1 = wps_arr[0]
        if x1 - p.pre_gate_mm > 150.0:
            pts.append((x1 - p.pre_gate_mm, y1))
        pts.extend(tuple(w) for w in wps_arr if w[0] > 50.0)
        x_end = max(float(wps_arr[-1, 0]), float(cones[:, 0].max())) + p.exit_blend_mm
    else:
        x_end = 600.0
    pts.append((x_end, y_lane))
    pts.append((x_end + 1500.0, y_lane))
    path = _densify(np.array(pts), p.path_step_mm)
    path = _smooth(path, 4)
    path = _repel(path, cones, p.min_pass_mm, iters=6)
    path = _smooth(path, 1)
    path = _repel(path, cones, p.min_pass_mm, iters=2)
    return Plan(path=path, gates=[(a, b) for a, b, _ in gates], singles=singles,
                waypoints=wps_arr, x_last=float(cones[:, 0].max()) if len(cones) else None,
                width_mm=width, cones=cones)


# ---------------------------------------------------------------------------
# 5) 추종
# ---------------------------------------------------------------------------
def pursue(path: np.ndarray, pose: Pose2D, lookahead: float) -> Tuple[float, np.ndarray, int]:
    """경로 위 Pure Pursuit. 반환 (kappa 1/m 왼쪽 +, 목표점 (같은 좌표), 가장 가까운 점 인덱스)."""
    pos = np.array([pose.x, pose.y])
    d = np.hypot(path[:, 0] - pos[0], path[:, 1] - pos[1])
    i = int(np.argmin(d))
    j = i
    while j < len(path) - 1 and d[j] < lookahead:
        j += 1
    tgt = path[j]
    v = to_vehicle(pose, [tgt])[0]
    d2 = float(v[0] ** 2 + v[1] ** 2)
    kappa = 0.0 if d2 < 1.0 else 2000.0 * float(v[1]) / d2
    return kappa, tgt, i


def arc_clearance(pose: Pose2D, kappa: float, length: float, cones: np.ndarray,
                  n: int = 8) -> float:
    """pose에서 곡률 kappa로 length만큼 가는 동안 차 중심과 콘 중심 사이 최소 거리."""
    if len(cones) == 0:
        return math.inf
    pts = np.array([[q.x, q.y] for q in (advance(pose, length * (k / n), kappa)
                                         for k in range(1, n + 1))])
    d = np.hypot(pts[:, None, 0] - cones[None, :, 0], pts[:, None, 1] - cones[None, :, 1])
    return float(d.min())


# ---------------------------------------------------------------------------
# 전체
# ---------------------------------------------------------------------------
class ConeFollower:
    """라바콘 구간 처리 묶음. 노드는 on_scan(), start(), control()만 부른다."""

    def __init__(self, p: Optional[ConeParams] = None):
        self.p = p or ConeParams()
        self.tracker = ConeTracker(self.p)
        self.reset()

    def reset(self) -> None:
        self.tracker.reset()
        self.corridor: Optional[Corridor] = None
        self.plan: Optional[Plan] = None
        self.zone_idx: List[int] = []        # plan의 콘 인덱스 -> tracker.tracks 인덱스
        self.sides: Dict[int, str] = {}      # 트랙 id -> 짝 없는 콘을 지나갈 방향 (한 번 정하면 유지)
        self.kappa_cmd = 0.0
        self.v_cmd = 0.0
        self.done = False
        self.done_reason = ''
        self.progress_mm = 0.0
        self.lateral_mm = 0.0
        self.target_c: Optional[np.ndarray] = None
        self.ever_cones = 0
        self.arc_clear_mm = math.inf
        self.lookahead_scale = 1.0

    # --- 입력 ---------------------------------------------------------------
    def on_scan(self, points_vehicle: np.ndarray, pose: Pose2D,
                lidar_xy: Tuple[float, float] = (0.0, 0.0)) -> List[dict]:
        cands = cone_candidates(points_vehicle, self.p, lidar_xy)
        dets = to_world(pose, [[c['x'], c['y']] for c in cands]) if cands else np.zeros((0, 2))
        self.tracker.update(dets, pose, lidar_xy)
        if self.corridor is not None:
            self.replan()
        return cands

    def start(self, pose: Pose2D, axis: float, lane_point_vehicle=None,
              source: str = '', kappa0: float = 0.0, v0: float = 0.0) -> Corridor:
        """라바콘 모드 진입.

        lane_point_vehicle: 차선 노드가 본 차로 중심점 (차량 좌표, mm). 없으면 지금 차 위치를 차로 중심으로.
        kappa0, v0: 넘겨받는 순간 차선 주행이 내던 곡률·속도. 코너를 돌며 들어오므로 0에서 시작하면
                    조향이 갑자기 풀려 바깥으로 밀린다 (시뮬레이션에서 첫 콘 여유가 크게 줄었음).
        """
        cor = Corridor(pose.x, pose.y, axis, 0.0, source)
        if lane_point_vehicle is not None:
            q = cor.to_c(to_world(pose, [lane_point_vehicle]))[0]
            cor.y_lane = float(q[1])
        self.corridor = cor
        self.kappa_cmd = float(np.clip(kappa0, -self.p.kappa_max, self.p.kappa_max))
        self.v_cmd = float(v0)
        self.replan()
        return cor

    # --- 경로 ---------------------------------------------------------------
    def zone_cones(self) -> Tuple[np.ndarray, List[int]]:
        cor, p = self.corridor, self.p
        idx = [i for i, t in enumerate(self.tracker.tracks) if t.ok]
        if not idx or cor is None:
            return np.zeros((0, 2)), []
        pc = cor.to_c([[self.tracker.tracks[i].x, self.tracker.tracks[i].y] for i in idx])
        m = ((pc[:, 0] >= p.zone_x_min_mm) & (pc[:, 0] <= p.zone_x_max_mm)
             & (np.abs(pc[:, 1] - cor.y_lane) <= p.zone_half_width_mm))
        return pc[m], [i for i, k in zip(idx, m) if k]

    def replan(self) -> None:
        cones, self.zone_idx = self.zone_cones()
        self.ever_cones = max(self.ever_cones, len(cones))
        ids = [self.tracker.tracks[i].id for i in self.zone_idx]
        hint = {k: self.sides[tid] for k, tid in enumerate(ids) if tid in self.sides}
        self.plan = plan_path(cones, self.corridor.y_lane, self.p, side_hint=hint)
        for k, side in self.plan.singles:
            self.sides.setdefault(ids[k], side)

    # --- 제어 ---------------------------------------------------------------
    def control(self, pose: Pose2D) -> Tuple[float, float]:
        """(v m/s, kappa 1/m). start() 전이면 (0, 0)."""
        p = self.p
        if self.corridor is None or self.plan is None:
            return 0.0, 0.0
        cor = self.corridor
        pc = cor.pose_c(pose)
        self.progress_mm, self.lateral_mm = pc.x, pc.y
        pred = advance(pc, max(self.v_cmd, p.min_speed) * 1000.0 * p.latency_sec, self.kappa_cmd)
        need = p.car_half_width_mm + p.cone_base_radius_mm + p.arc_margin_mm
        best = None
        for sc in p.lookahead_scales:
            k_try, tgt_try, _ = pursue(self.plan.path, pred, p.lookahead_mm * sc)
            clr = arc_clearance(pred, k_try, p.lookahead_mm * sc, self.plan.cones)
            if best is None or clr > best[0] + 1e-6:
                best = (clr, k_try, tgt_try, sc)
            if clr >= need:
                best = (clr, k_try, tgt_try, sc)
                break
        self.arc_clear_mm, k_raw, tgt, self.lookahead_scale = best
        self.target_c = tgt
        k = self.kappa_cmd + float(np.clip(k_raw - self.kappa_cmd, -p.kappa_rate_max,
                                           p.kappa_rate_max))
        self.kappa_cmd = float(np.clip(k, -p.kappa_max, p.kappa_max))
        v = p.speed * (1.0 - p.curve_slowdown * min(1.0, abs(self.kappa_cmd) / p.kappa_max))
        self.v_cmd = max(v, p.min_speed)
        self._check_done(pc.x)
        return self.v_cmd, self.kappa_cmd

    def _check_done(self, x_car: float) -> None:
        if self.done:
            return
        p, xl = self.p, (self.plan.x_last if self.plan else None)
        if xl is not None and x_car >= xl + p.done_margin_mm and x_car >= p.done_min_progress_mm:
            self.done, self.done_reason = True, f'passed last cone x={xl:.0f}'
        elif x_car >= p.done_max_progress_mm:
            self.done, self.done_reason = True, f'max progress {x_car:.0f}'

    # --- 보고 ---------------------------------------------------------------
    def cone_ahead_mm(self, pose: Pose2D, half_width: float = 180.0) -> Optional[float]:
        """차가 그대로 가면 부딪힐 폭(차 중심 ± half_width) 안 확정 콘까지 앞바퀴 기준 거리."""
        ok = self.tracker.ok_tracks()
        if not ok:
            return None
        v = to_vehicle(pose, [[t.x, t.y] for t in ok])
        m = (v[:, 0] > self.p.front_axle_mm - 50.0) & (np.abs(v[:, 1]) < half_width)
        if not m.any():
            return None
        return float(v[m, 0].min() - self.p.front_axle_mm - self.p.cone_base_radius_mm)

    def nearest_ahead_mm(self, pose: Pose2D, half_width: Optional[float] = None) -> Optional[float]:
        """앞바퀴 앞, 차 중심 ± half_width(기본 구간 반폭) 안 확정 콘 중 가장 가까운 것까지 거리.
        양쪽 통로라 차 앞을 막지 않는 콘도 센다 (M8 끝 판단: '라바콘이 더 안 보임')."""
        ok = self.tracker.ok_tracks()
        if not ok:
            return None
        hw = self.p.zone_half_width_mm if half_width is None else half_width
        v = to_vehicle(pose, [[t.x, t.y] for t in ok])
        m = (v[:, 0] > self.p.front_axle_mm) & (np.abs(v[:, 1]) < hw)
        if not m.any():
            return None
        return float(v[m, 0].min() - self.p.front_axle_mm)

    def state_dict(self) -> dict:
        tr = self.tracker.tracks
        d = {'tracks': len(tr), 'confirmed': sum(t.confirmed for t in tr),
             'rejected': sum(t.rejected for t in tr), 'done': self.done,
             'done_reason': self.done_reason, 'corridor': self.corridor is not None}
        if self.corridor is not None and self.plan is not None:
            d.update({'axis_source': self.corridor.source,
                      'y_lane_mm': round(self.corridor.y_lane),
                      'zone_cones': len(self.zone_idx), 'gates': len(self.plan.gates),
                      'singles': len(self.plan.singles),
                      'x_last_mm': None if self.plan.x_last is None else round(self.plan.x_last),
                      'progress_mm': round(self.progress_mm),
                      'lateral_mm': round(self.lateral_mm),
                      'kappa': round(self.kappa_cmd, 3), 'v': round(self.v_cmd, 3)})
        return d


# ---------------------------------------------------------------------------
# 카메라 확인 (BEV 호모그래피를 거꾸로 써서 콘 바닥 위치를 이미지에 찍는다)
# ---------------------------------------------------------------------------
class GroundProjector:
    """차량 좌표 바닥점 (x 전방, y 왼쪽, mm) -> 카메라 이미지 px.

    차선 노드의 BEV yaml(bev_limo.yaml)을 그대로 쓴다. 바닥 평면이면 BEV 범위 밖이어도 맞다.
    """

    def __init__(self, bev: dict):
        H = np.array(bev['homography'], dtype=np.float64).reshape(3, 3)
        self.Hinv = np.linalg.inv(H)
        self.w = int(bev.get('image_width', 640))
        self.h = int(bev.get('image_height', 480))
        self.bev_h = int(bev.get('bev_h', 400))
        self.mmx = float(bev.get('mm_per_px_x', 4.0))
        self.mmy = float(bev.get('mm_per_px_y', 4.0))
        self.col0 = float(bev.get('vehicle_col', 200.0))
        self.bottom = float(bev.get('ref_to_bev_bottom_mm', 300.0))
        self.calibrated = bool(bev.get('calibrated', False))

    @classmethod
    def from_yaml(cls, path: str) -> 'GroundProjector':
        import yaml
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f)['bev'])

    def project(self, x_fwd: float, y_left: float) -> Optional[Tuple[float, float]]:
        col = self.col0 + (-y_left) / self.mmx
        row = (self.bev_h - 1) - (x_fwd - self.bottom) / self.mmy
        q = self.Hinv @ np.array([col, row, 1.0])
        # 호모그래피 스케일 부호는 캘리브레이션마다 다를 수 있어 부호로 판단하지 않는다.
        # 차 뒤쪽 점은 이상한 곳으로 가므로 부르는 쪽에서 앞쪽(x >= 300mm)만 넘길 것
        if abs(q[2]) < 1e-12:
            return None
        return float(q[0] / q[2]), float(q[1] / q[2])

    def cone_box(self, x: float, y: float, base_r: float,
                 aspect: float = 1.6) -> Optional[Tuple[int, int, int, int]]:
        """콘 바닥 앞 가장자리 ~ 위쪽 영역 (u0, v0, u1, v1). 화면 밖이면 None."""
        bot = self.project(x - base_r, y)
        lft = self.project(x, y + base_r)
        rgt = self.project(x, y - base_r)
        if bot is None or lft is None or rgt is None:
            return None
        wpx = abs(rgt[0] - lft[0])
        if wpx < 6.0:
            return None
        u0, u1 = bot[0] - wpx / 2.0, bot[0] + wpx / 2.0
        v1 = bot[1]
        v0 = v1 - aspect * wpx
        if u0 < 0 or u1 > self.w - 1 or v0 < 0 or v1 > self.h - 1:
            return None
        return int(u0), int(v0), int(math.ceil(u1)), int(math.ceil(v1))


def orange_mask(hsv: np.ndarray, h_max: int = 18, h_wrap_min: int = 170, s_min: int = 110,
                v_min: int = 70) -> np.ndarray:
    """주황(빨강 쪽 포함) 마스크. 차선 노랑은 H 15~40이라 h_max를 18 넘게 올리지 말 것."""
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return ((h <= h_max) | (h >= h_wrap_min)) & (s >= s_min) & (v >= v_min)


def orange_ratio(hsv: np.ndarray, box: Tuple[int, int, int, int], **kw) -> float:
    u0, v0, u1, v1 = box
    crop = hsv[v0:v1 + 1, u0:u1 + 1]
    if crop.size == 0:
        return 0.0
    return float(orange_mask(crop, **kw).mean())
