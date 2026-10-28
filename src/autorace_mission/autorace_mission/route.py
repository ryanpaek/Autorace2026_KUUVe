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
                                   'width': float(lm.get('width', 40.0))})
        self.landmarks.sort(key=lambda m: m['s'])
        self.points: Dict[str, float] = {k: self.s_of(v)
                                         for k, v in data.get('points', {}).items()}
        for lm in self.landmarks:
            self.points[lm['name']] = lm['s']

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
                 gain: float = 0.6, odom_scale: float = 1.0):
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
        self.s = 0.0
        self._dist_ref = None
        self._since_fix = 0.0
        self.last_fix: Optional[dict] = None
        self.fixes = 0

    def reset(self, s: float, dist_mm: float) -> None:
        self.s = float(s)
        self._dist_ref = dist_mm
        self._since_fix = 0.0

    @property
    def gate(self) -> float:
        return min(self.gate_max_mm, self.gate_mm + self.gate_per_m * self._since_fix / 1000.0)

    def update(self, dist_mm: float, stoplines_mm: Optional[List[float]] = None) -> float:
        if self._dist_ref is None:
            self._dist_ref = dist_mm
        step = (dist_mm - self._dist_ref) * self.odom_scale
        self.s += step
        self._since_fix += abs(step)
        self._dist_ref = dist_mm
        for d in sorted(stoplines_mm or []):
            if d < -50.0 or d > self.max_range_mm:
                continue
            lm = self.match(self.s + d)
            if lm is not None:
                err = lm['s'] - (self.s + d)
                self.s += self.gain * err
                self.fixes += 1
                if abs(err) * (1.0 - self.gain) < 100.0:
                    self._since_fix = 0.0      # 남은 오차가 작으면 게이트를 다시 좁힌다
                self.last_fix = {'name': lm['name'], 'err_mm': round(err), 'd_mm': round(d)}
                break                     # 가장 가까운 선 하나만
        return self.s

    def match(self, s_line: float) -> Optional[dict]:
        best = None
        for lm in self.route.landmarks:
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
            'fixes': self.fixes,
            'last_fix': self.last_fix,
        }
