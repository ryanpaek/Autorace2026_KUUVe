"""도면 경로 지도 + 위치 추정 (경로를 따라 몇 mm 왔는지 = s).

라이다 지도 대신, 도면에서 뽑은 차로 중심선(track_route.yaml)을 "외워" 두고
  1) 오도메트리 주행 거리로 s를 계속 늘리고
  2) 차선 노드가 흰 가로선을 보면(stoplines_mm) 도면의 그 선 위치로 s를 다시 맞춘다.
좌우 위치는 차선 인식이 맡으므로 여기서는 경로 방향 1차원 위치만 다룬다.

RouteMap    : 경로 기하 (s -> xy, 방향, 곡률), 랜드마크·구간 경계의 s
RouteTracker: 주행 중 s 추정 (오도메트리 + 가로선 보정)
"""

import bisect
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class _Seg:
    kind: str                 # 'line' | 'arc'
    s0: float
    length: float
    p0: Tuple[float, float]
    p1: Tuple[float, float] = (0.0, 0.0)
    center: Tuple[float, float] = (0.0, 0.0)
    radius: float = 0.0
    a0: float = 0.0           # rad
    a1: float = 0.0

    def point(self, ds: float) -> Tuple[float, float]:
        if self.kind == 'line':
            t = ds / self.length if self.length else 0.0
            return (self.p0[0] + (self.p1[0] - self.p0[0]) * t,
                    self.p0[1] + (self.p1[1] - self.p0[1]) * t)
        a = self.a0 + (self.a1 - self.a0) * (ds / self.length)
        return (self.center[0] + self.radius * math.cos(a),
                self.center[1] + self.radius * math.sin(a))

    def heading(self, ds: float) -> float:
        if self.kind == 'line':
            return math.atan2(self.p1[1] - self.p0[1], self.p1[0] - self.p0[0])
        a = self.a0 + (self.a1 - self.a0) * (ds / self.length)
        turn = 1.0 if self.a1 > self.a0 else -1.0          # 반시계(왼쪽) +1
        return _wrap(a + turn * math.pi / 2)

    @property
    def curvature(self) -> float:
        if self.kind == 'line':
            return 0.0
        return (1.0 if self.a1 > self.a0 else -1.0) / self.radius * 1000.0   # 1/m, 왼쪽 +

    def project(self, xy) -> Tuple[float, float]:
        """(이 구간 안의 ds, 거리) - 구간 밖이면 끝점으로 자른다."""
        if self.kind == 'line':
            dx, dy = self.p1[0] - self.p0[0], self.p1[1] - self.p0[1]
            t = ((xy[0] - self.p0[0]) * dx + (xy[1] - self.p0[1]) * dy) / (dx * dx + dy * dy)
            t = min(1.0, max(0.0, t))
            q = (self.p0[0] + dx * t, self.p0[1] + dy * t)
            return t * self.length, math.dist(q, xy)
        a = math.atan2(xy[1] - self.center[1], xy[0] - self.center[0])
        lo, hi = sorted((self.a0, self.a1))
        a = _unwrap_near(a, (lo + hi) / 2)
        a = min(hi, max(lo, a))
        ds = (a - self.a0) / (self.a1 - self.a0) * self.length
        return ds, math.dist(self.point(ds), xy)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _unwrap_near(a: float, ref: float) -> float:
    while a - ref > math.pi:
        a -= 2 * math.pi
    while a - ref < -math.pi:
        a += 2 * math.pi
    return a


class RouteMap:
    def __init__(self, data: dict):
        self.segs: List[_Seg] = []
        cur = tuple(map(float, data['start_xy']))
        s = 0.0
        for d in data['segments']:
            if d['type'] == 'line':
                p1 = tuple(map(float, d['to']))
                seg = _Seg('line', s, math.dist(cur, p1), cur, p1=p1)
            elif d['type'] == 'arc':
                a0, a1 = math.radians(d['start_deg']), math.radians(d['end_deg'])
                r = float(d['radius'])
                c = tuple(map(float, d['center']))
                seg = _Seg('arc', s, abs(a1 - a0) * r, cur, center=c, radius=r, a0=a0, a1=a1)
                start = seg.point(0.0)
                if math.dist(start, cur) > 30.0:
                    raise ValueError(f'arc does not start at previous end: {start} vs {cur}')
            else:
                raise ValueError(f'unknown segment type {d["type"]}')
            self.segs.append(seg)
            s += seg.length
            cur = seg.point(seg.length)
        self.length = s
        self._s0 = [g.s0 for g in self.segs]
        self.landmarks: List[dict] = []
        for lm in data.get('landmarks', []):
            self.landmarks.append({'name': lm['name'], 's': self.s_of(lm['xy']),
                                   'width': float(lm.get('width', 40.0)),
                                   # crosswalk: 뒤에 횡단보도 막대가 있는 정지선 (그런 선으로만 맞춘다)
                                   'kind': str(lm.get('kind', 'line'))})
        self.landmarks.sort(key=lambda m: m['s'])
        self.points: Dict[str, float] = {k: self.s_of(v)
                                         for k, v in data.get('points', {}).items()}
        for lm in self.landmarks:
            self.points[lm['name']] = lm['s']
        # 3차선·갈림길 (11/11). 없으면 어디서나 차로 하나
        self.three = None
        tl = data.get('three_lane')
        if tl:
            isl = [tuple(map(float, q)) for q in tl['island']]
            ss = [self.s_of(q) for q in isl]
            self.three = {
                'spacing': float(tl.get('spacing_mm', 520.0)),
                'in': tuple(sorted(self.s_of(q) for q in tl['taper_in'])),
                'out': tuple(sorted(self.s_of(q) for q in tl['taper_out'])),
                'island': (min(ss), max(ss)),
                'island_xy': isl,
            }

    @classmethod
    def from_yaml(cls, path: str) -> 'RouteMap':
        import yaml
        with open(path, encoding='utf-8') as f:
            return cls(yaml.safe_load(f))

    def _seg(self, s: float) -> _Seg:
        i = max(0, min(len(self.segs) - 1, bisect.bisect_right(self._s0, s) - 1))
        return self.segs[i]

    def xy(self, s: float) -> Tuple[float, float]:
        g = self._seg(s)
        return g.point(min(max(s - g.s0, 0.0), g.length))

    def heading(self, s: float) -> float:
        """경로 접선 방향 (도면 기준, 동쪽 0, 반시계 +)."""
        g = self._seg(s)
        return g.heading(min(max(s - g.s0, 0.0), g.length))

    def curvature(self, s: float) -> float:
        return self._seg(s).curvature

    def max_abs_curvature(self, s_from: float, s_to: float) -> float:
        k = 0.0
        for g in self.segs:
            if g.s0 < s_to and g.s0 + g.length > s_from:
                k = max(k, abs(g.curvature))
        return k

    def widen(self, s: float) -> float:
        """3차선 벌어짐 정도: 1차로 0 ~ 3차로 1 (taper에서 선형)."""
        t = self.three
        if t is None:
            return 0.0
        (a0, a1), (b0, b1) = t['in'], t['out']
        if s <= a0 or s >= b1:
            return 0.0
        if s < a1:
            return (s - a0) / (a1 - a0)
        if s > b0:
            return (b1 - s) / (b1 - b0)
        return 1.0

    def in_taper(self, s: float) -> bool:
        w = self.widen(s)
        return 0.0 < w < 1.0

    def on_island(self, s: float) -> bool:
        t = self.three
        return t is not None and t['island'][0] <= s <= t['island'][1]

    def lane_centers(self, s: float) -> List[float]:
        """이 위치에서 차가 있을 수 있는 차로 중심의 좌우 위치 d (경로 중심선 기준, 왼쪽 +).
        보통 [0], 3차선 [+520, 0, -520], 섬 옆 [+520(A), -520(B)], taper에서는 간격이 줄어든다."""
        w = self.widen(s)
        if w <= 0.0:
            return [0.0]
        sp = self.three['spacing'] * w
        if self.on_island(s):
            return [sp, -sp]
        return [sp, 0.0, -sp]

    def s_of(self, xy) -> float:
        """도면 좌표를 경로에 투영한 s (가장 가까운 구간)."""
        best = None
        for g in self.segs:
            ds, dist = g.project(xy)
            if best is None or dist < best[1]:
                best = (g.s0 + ds, dist)
        return best[0]


class RouteTracker:
    """오도메트리로 s를 늘리고, 흰 가로선을 보면 도면 값으로 다시 맞춘다."""

    def __init__(self, route: RouteMap, gate_mm: float = 250.0, gate_per_m: float = 60.0,
                 gate_max_mm: float = 800.0, max_range_mm: float = 900.0,
                 gain: float = 0.6, odom_scale: float = 1.0, ordered_gate_mm: float = 0.0,
                 same_gate_mm: float = 200.0, crosswalk_gate_mm: float = 400.0,
                 lost_misses: int = 0):
        self.route = route
        # 예측 위치와 도면 선이 gate 안이어야 같은 선으로 본다.
        # 마지막 보정 뒤 많이 달렸을수록 오도메트리 오차가 커지므로 m당 gate_per_m씩 넓힌다
        # (기본 6%: LIMO 바퀴 미끄러짐 여유). 이웃 선끼리 가장 가까운 간격(약 540)보다 커지지 않게 상한.
        self.gate_mm = gate_mm
        self.gate_per_m = gate_per_m
        self.gate_max_mm = gate_max_mm
        self.max_range_mm = max_range_mm  # 너무 멀리 본 선은 거리 오차가 커서 안 씀
        self.gain = gain                  # 한 번에 고치는 비율 (여러 프레임에 걸쳐 수렴)
        self.odom_scale = odom_scale
        # 순서 짝짓기 (11/06): 선은 도면 순서대로 나타나므로, 마지막으로 맞춘 선 다음 선을
        # 이 범위(mm) 안이면 받아들인다. 지그재그를 따라 흔들리며 달리면 odom 거리가 도면보다
        # 최대 약 18% 길어져(11/02 실측 80cm) 일반 게이트로는 엉뚱한 선과 짝지어진다. 0이면 끔
        self.ordered_gate_mm = ordered_gate_mm
        self.same_gate_mm = same_gate_mm      # 같은 선을 다시 볼 때 허용 오차
        # 횡단보도 정지선(kind=crosswalk) 앞 400~600mm에 '천천히' 글자가 있어 순서 짝짓기 범위(1200)로는
        # 글자가 정지선이 됐다 (11/09). 글자는 정지선보다 앞(가까운 쪽)에 있으므로 "도면 선이 본 선보다
        # 이만큼 넘게 뒤"인 짝은 버린다. 반대쪽(지그재그 odom 과대, 본 선이 예측보다 가까움)은 그대로 허용
        self.crosswalk_gate_mm = crosswalk_gate_mm
        # 길 잃음 (11/10): 마지막으로 맞춘 선 뒤로 도면 선을 이만큼 연속으로 못 보고 지나치면 lost.
        # lost면 mission_node가 FSM에 경로 위치를 안 준다 (표지·카메라 선·구간 거리로만 판단). 0이면 끔
        self.lost_misses = int(lost_misses)
        self.missed = 0                   # 마지막으로 맞춘 선 뒤로 못 보고 지나친 선 수
        self.last_idx: Optional[int] = None
        self.s = 0.0
        self._dist_ref = None
        self._since_fix = 0.0
        self.last_fix: Optional[dict] = None
        self.fixes = 0

    def reset(self, s: float, dist_mm: float) -> None:
        self.s = float(s)
        self._dist_ref = dist_mm
        self._since_fix = 0.0
        # 출발 위치보다 앞(뒤쪽)에 있는 마지막 선 = 이미 지난 선
        passed = [i for i, m in enumerate(self.route.landmarks) if m['s'] < s - 50.0]
        self.last_idx = passed[-1] if passed else -1
        self.missed = 0

    @property
    def lost(self) -> bool:
        return self.lost_misses > 0 and self.missed >= self.lost_misses

    def _count_missed(self) -> int:
        """마지막으로 맞춘 선 다음부터, 지금 위치가 '선 + 게이트'를 넘은(= 보였어야 할) 선 수."""
        lms = self.route.landmarks
        i0 = (self.last_idx if self.last_idx is not None else -1) + 1
        n = 0
        for lm in lms[i0:]:
            if lm['s'] + self.gate < self.s:
                n += 1
            else:
                break
        return n

    @property
    def gate(self) -> float:
        return min(self.gate_max_mm, self.gate_mm + self.gate_per_m * self._since_fix / 1000.0)

    def update(self, dist_mm: float, stoplines_mm: Optional[List[float]] = None,
               crosswalk_mm: Optional[List[float]] = None) -> float:
        """crosswalk_mm: 그중 횡단보도 막대가 뒤따르는 선 (lane 노드 crosswalk_lines_mm).
        주면 kind=crosswalk 도면 선(정지선)은 이 선들로만 맞춘다. None이면 예전처럼 아무 선이나."""
        if self._dist_ref is None:
            self._dist_ref = dist_mm
        step = (dist_mm - self._dist_ref) * self.odom_scale
        self.s += step
        self._since_fix += abs(step)
        self._dist_ref = dist_mm
        self.missed = self._count_missed()
        # 이 프레임의 선 중 도면 선과 가장 잘 맞는 것 하나만 쓴다.
        # (11/09: 가장 가까운 선부터 쓰면 정지선 앞 '천천히' 글자가 순서 짝짓기로 정지선 자리를 먼저 차지했다)
        best = None
        for d in sorted(stoplines_mm or []):
            if d < -50.0 or d > self.max_range_mm:
                continue
            is_xw = None if crosswalk_mm is None else any(abs(d - c) < 1.0 for c in crosswalk_mm)
            lm = self.match(self.s + d, is_xw)
            if lm is not None:
                err = lm['s'] - (self.s + d)
                if best is None or abs(err) < abs(best[2]):
                    best = (d, lm, err)
        if best is not None:
            d, lm, err = best
            new_line = self.route.landmarks.index(lm) != self.last_idx
            # 순서 짝짓기로 새 선을 처음 맞추면 한 번에 고친다 (오차는 odom 쪽이 원인).
            # 남겨 두면 다음 프레임에 이웃 선(횡단보도 끝선 등)과 짝지어질 수 있다
            g = 1.0 if (self.ordered_gate_mm > 0.0 and new_line) else self.gain
            self.s += g * err
            self.fixes += 1
            if abs(err) * (1.0 - self.gain) < 100.0:
                self._since_fix = 0.0      # 남은 오차가 작으면 게이트를 다시 좁힌다
            self.last_fix = {'name': lm['name'], 'err_mm': round(err), 'd_mm': round(d)}
            self.last_idx = self.route.landmarks.index(lm)
            self.missed = self._count_missed()
        return self.s

    def match(self, s_line: float, is_xw: Optional[bool] = None) -> Optional[dict]:
        lms = self.route.landmarks

        def ok(lm):
            # 정지선은 막대가 뒤따르는 선만, 도면 예측 가까이에서만 (글자·마름모 표시를 정지선으로 맞추지 않게, 11/09)
            if lm.get('kind') == 'crosswalk':
                if is_xw is False:
                    return False
                if self.crosswalk_gate_mm > 0.0 and lm['s'] - s_line > self.crosswalk_gate_mm:
                    return False
            return True

        if self.ordered_gate_mm > 0.0 and self.last_idx is not None:
            i = self.last_idx
            if 0 <= i < len(lms) and abs(lms[i]['s'] - s_line) <= self.same_gate_mm and ok(lms[i]):
                return lms[i]                       # 방금 맞춘 선을 다시 봄
            # 다음 선 (하나 놓쳤으면 그다음). 11/10: 이미 못 보고 지나친 선(missed)이 있으면 그만큼 더 뒤까지 본다.
            # 예전엔 i+1, i+2만 봐서 선 2개를 놓치면 다시는 못 맞췄다 (한 번 길을 잃으면 끝까지 틀림)
            for j in range(i + 1, i + 3 + self.missed):
                g = self.ordered_gate_mm
                if j >= i + 2 and 0 <= j - 1 < len(lms) and lms[j - 1]['s'] > s_line:
                    # 건너뛸 선이 본 선보다 앞(아직 안 지남)이면 놓친 게 아니다 -> 일반 게이트로만
                    # (11/09: 정지선으로 못 맞춘 글자가 건너뛰기로 횡단보도 끝선이 됐다)
                    g = self.gate
                if 0 <= j < len(lms) and abs(lms[j]['s'] - s_line) <= g and ok(lms[j]):
                    return lms[j]
            return None
        best = None
        for lm in self.route.landmarks:
            if not ok(lm):
                continue
            e = abs(lm['s'] - s_line)
            if e <= self.gate and (best is None or e < abs(best['s'] - s_line)):
                best = lm
        return best

    def state(self) -> dict:
        r = self.route
        nxt = next((m for m in r.landmarks if m['s'] > self.s), None)
        return {
            's_mm': round(self.s),
            'heading': round(r.heading(self.s), 4),
            'kappa': round(r.curvature(self.s), 3),
            'kappa_ahead': round(r.max_abs_curvature(self.s, self.s + 1000.0), 3),
            'next': nxt['name'] if nxt else None,
            'next_in_mm': round(nxt['s'] - self.s) if nxt else None,
            'gate_mm': round(self.gate),
            'last_line': (self.route.landmarks[self.last_idx]['name']
                          if self.last_idx is not None and self.last_idx >= 0 else None),
            'fixes': self.fixes,
            'last_fix': self.last_fix,
            'missed': self.missed,
            'lost': self.lost,
        }


def motion_sign(dx_m: float, dy_m: float, odom_yaw: Optional[float], twist_x: float,
                cmd_v: Optional[float]) -> Tuple[int, str]:
    """이번 오도메트리 걸음이 전진(+1)인지 후진(-1)인지와 그 근거 (11/11).

    근거 셋 중 하나라도 후진이면 후진:
      'odom_v'  : /odom twist.linear.x < 0
      'odom_xy' : /odom 위치 변화가 /odom 방향과 반대 (드라이버가 속도 부호를 안 줘도 위치가 뒤로 가면 잡힘)
      'cmd'     : 최근 /cmd_vel 속도 < 0 (드라이버가 후진에도 위치를 앞으로 쌓는 경우 대비)
    11/10까지는 twist.linear.x만 봤는데 LIMO가 후진 때 음수를 주는지 실측 전이었다.
    """
    if twist_x < -0.005:
        return -1, 'odom_v'
    if odom_yaw is not None and math.hypot(dx_m, dy_m) > 1e-4:
        if dx_m * math.cos(odom_yaw) + dy_m * math.sin(odom_yaw) < 0.0:
            return -1, 'odom_xy'
    if cmd_v is not None and cmd_v < -0.02:
        return -1, 'cmd'
    return 1, ''


class MapPose:
    """도면 위 차 자세: 경로 위치 s(RouteTracker) + 좌우 위치 d + 방향 (11/06).

    - 방향: IMU yaw + 보정값. 보정값은 직선 구간에서 두 선이 다 보일 때
      "도면 경로 방향 = IMU yaw + 차선 방향"으로 맞춘다 (IMU 드리프트 거의 0, 11/02 실측).
    - 좌우 d (경로 중심선 기준 왼쪽 +, mm): 차선이 잘 보이면 차선 노드 offset으로 맞추고,
      놓치면 진행 거리 x sin(차 방향 - 경로 방향)으로 적분한다.
    차선을 놓쳤을 때 "도면상 차로 가운데가 지금 차 기준 어디인지"를 알려 주는 게 목적.
    """

    def __init__(self, route: RouteMap, front_axle_mm: float = 200.0,
                 valid_mm: float = 3000.0, alpha: float = 0.2,
                 jump_gate_mm: float = 200.0, jump_frames: int = 10):
        self.route = route
        # 11/11: 차선 노드 offset은 '카메라가 보는 차로'의 중심 기준이다. 차가 옆 차로에 있거나
        # 차로를 벗어나 엉뚱한 선 두 개를 잡으면 d가 가운데(0)로 튀었다 -> 
        #   (1) 도면상 차로 중심 후보(3차선·갈림길 A/B) 중 지금 추정에 가장 가까운 차로로 맞추고
        #   (2) 그래도 추정과 jump_gate_mm 넘게 다르면 jump_frames 프레임 연속일 때만 받아들인다
        self.jump_gate_mm = jump_gate_mm
        self.jump_frames = jump_frames
        self._disagree = 0
        self.lane_fix = ''        # 마지막 차선 보정 결과 (디버그)
        self.lane_center = 0.0    # 지금 차가 있다고 본 차로 중심 d
        self.front_axle_mm = front_axle_mm      # 뒷바퀴 축(차선 노드 기준점) -> 앞바퀴 (s 기준)
        self.valid_mm = valid_mm                # 마지막 좌우 보정 뒤 이 거리까지만 믿는다
        self.alpha = alpha
        self.yaw_off: Optional[float] = None
        self.d = 0.0
        self._d_fix_dist: Optional[float] = None
        self._dist = None

    def align(self, s: float, imu_yaw: Optional[float], dist_mm: float) -> bool:
        """차를 경로 방향으로 차로 가운데에 놓았다고 보고 방향·좌우를 맞춘다 (11/11).

        출발선·연습 시작점처럼 사람이 차를 반듯이 놓는 곳에서만 부른다.
        직선에서 두 선을 다 보기 전에도 방향이 있어야 advance()가 경로 방향 투영을 할 수 있다.
        """
        if imu_yaw is None:
            return False
        self.yaw_off = _wrap(self.route.heading(s) - imu_yaw)
        self.d = 0.0
        self._d_fix_dist = dist_mm
        return True

    def heading_error(self, s: float, imu_yaw: Optional[float]) -> Optional[float]:
        """차 방향 - 경로 방향 (rad, 왼쪽 +). 방향을 아직 모르면 None."""
        if self.yaw_off is None or imu_yaw is None:
            return None
        return _wrap(imu_yaw + self.yaw_off - self.route.heading(s))

    def advance(self, step_mm: float, s: float, imu_yaw: Optional[float],
                max_turn_rad: float = 1.2) -> float:
        """오도메트리 한 걸음(후진이면 음수)을 경로 방향 성분 ds와 좌우 성분으로 나눈다 (11/11).

        - 차가 경로와 비스듬하면(지그재그·차로 변경·주차 꺾기) 바퀴가 굴러간 거리 전부가 경로 진행이 아니다:
          ds = step * cos(방향 차) / (1 - 곡률 * d),  d += step * sin(방향 차)
          (예전엔 step 전부를 s에 더해 지그재그에서 odom이 도면보다 최대 18% 길어졌다, 11/02)
        - 후진이면 step < 0이라 s도 d도 뒤로 간다.
        - 방향을 모르거나(정렬 전) 방향 차가 max_turn_rad보다 크면(IMU·정렬이 틀렸을 가능성) step 그대로.
        반환: s에 더할 거리 (mm)
        """
        e = self.heading_error(s, imu_yaw)
        if e is None or abs(e) > max_turn_rad:
            return step_mm
        k = self.route.curvature(s) / 1000.0                # 1/mm, 왼쪽 +
        denom = max(0.5, 1.0 - k * self.d)                  # 곡선 안쪽(d가 중심 쪽)이면 같은 거리에 s가 더 감
        self.d += step_mm * math.sin(e)
        return step_mm * math.cos(e) / denom

    def predict(self, dist_mm: float, s: float, imu_yaw: Optional[float]) -> None:
        if self._dist is not None and self.yaw_off is not None and imu_yaw is not None:
            ds = dist_mm - self._dist
            self.d += ds * math.sin(_wrap(imu_yaw + self.yaw_off - self.route.heading(s)))
        self._dist = dist_mm

    def update_lane(self, s: float, imu_yaw: Optional[float], lane: dict,
                    dist_mm: float) -> bool:
        """차선 상태로 보정. 직선 구간 + 두 선 + 다항식 모델일 때만 (원호 모델 offset은 부정확)."""
        if imu_yaw is None or not lane:
            return False
        if (lane.get('status') != 'BOTH' or lane.get('low_conf') or lane.get('model') != 'poly'
                or lane.get('heading_rad') is None or lane.get('offset_mm') is None):
            return False
        if abs(self.route.curvature(s)) > 1e-6 or abs(self.route.curvature(s + 600.0)) > 1e-6:
            return False
        if self.route.in_taper(s) or self.route.in_taper(s + 600.0):
            return False                        # 차로가 벌어지거나 모이는 곳: 선 방향이 경로와 다름
        hd = float(lane['heading_rad'])
        if abs(hd) > 0.3:
            return False
        local = -float(lane['offset_mm'])       # offset = 차 기준 차로 중심 (왼쪽 +) -> 그 차로 안 차의 위치
        centers = self.route.lane_centers(s)
        lid = lane.get('lane_id')
        if len(centers) == 3 and lid in (1, 0, -1):
            c = centers[(1, 0, -1).index(lid)]   # 선 종류로 본 차로 번호 (+1 왼쪽 = A쪽)
            how = f'lane_id {lid}'
        else:
            c = min(centers, key=lambda cc: abs(cc + local - self.d))
            how = 'nearest lane' if len(centers) > 1 else 'lane'
        new_d = c + local
        if self.yaw_off is not None and self._d_fix_dist is not None \
                and abs(new_d - self.d) > self.jump_gate_mm:
            self._disagree += 1
            if self._disagree < self.jump_frames:
                self.lane_fix = f'rejected: {new_d - self.d:+.0f}mm jump ({self._disagree}/{self.jump_frames})'
                return False
            how += ', accepted after repeats'
        self._disagree = 0
        off = _wrap(self.route.heading(s) - (imu_yaw + hd))
        if self.yaw_off is None:
            self.yaw_off = off
        else:
            self.yaw_off = _wrap(self.yaw_off + self.alpha * _wrap(off - self.yaw_off))
        self.d = new_d
        self.lane_center = c
        self.lane_fix = f'ok ({how}, lane d {c:+.0f})'
        self._d_fix_dist = dist_mm
        return True

    def valid(self, dist_mm: float) -> bool:
        return (self.yaw_off is not None and self._d_fix_dist is not None
                and dist_mm - self._d_fix_dist <= self.valid_mm)

    def points_ahead(self, s: float, imu_yaw: float, n: int = 14,
                     step_mm: float = 100.0) -> List[Tuple[float, float]]:
        """경로 중심선 점 (차선 노드 좌표: x 오른쪽+, y 전방+, 뒷바퀴 축 기준 mm)."""
        h = self.route.heading(s)
        fx, fy = self.route.xy(s)
        fx, fy = fx - self.d * math.sin(h), fy + self.d * math.cos(h)   # 왼쪽 법선 = (-sin, cos)
        psi = imu_yaw + self.yaw_off
        c, sn = math.cos(psi), math.sin(psi)
        out = []
        for i in range(n):
            si = s + i * step_mm
            qx, qy = self.route.xy(si)
            # 지금 차가 있는 차로의 중심을 따라간다 (3차선·갈림길에서 경로 중심선 = 가운데 차로가 아님, 11/11)
            ci = min(self.route.lane_centers(si), key=lambda cc: abs(cc - self.lane_center))
            if ci:
                hi = self.route.heading(si)
                qx, qy = qx - ci * math.sin(hi), qy + ci * math.cos(hi)
            dx, dy = qx - fx, qy - fy
            fwd = dx * c + dy * sn
            right = dx * sn - dy * c
            out.append((round(right, 1), round(fwd + self.front_axle_mm, 1)))
        return out
