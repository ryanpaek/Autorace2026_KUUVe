"""2D 라이다 처리 함수 (ROS 의존성 없음).

좌표: base_link 기준 x = 전방 +, y = 왼쪽 + (ROS 규약), 단위 mm.
라이다 장착 위치·방향은 파라미터로 받는다. LIMO 기본 tf는 실제와 다를 수 있으니
첫날 차 앞 정면에 물체를 두고 각도가 0 근처로 나오는지 꼭 확인할 것.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

import numpy as np


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float, laser_x_mm: float = 0.0,
                   laser_y_mm: float = 0.0, laser_yaw: float = 0.0) -> np.ndarray:
    """LaserScan 값 -> (n, 2) [x, y] mm. 범위 밖·inf·nan은 버린다."""
    r = np.asarray(ranges, dtype=np.float64)
    a = angle_min + angle_increment * np.arange(r.size) + laser_yaw
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[ok] * 1000.0, a[ok]
    return np.stack([laser_x_mm + r * np.cos(a), laser_y_mm + r * np.sin(a)], axis=1)


def roi(points: np.ndarray, x_min: float, x_max: float, y_min: float,
        y_max: float) -> np.ndarray:
    if points.size == 0:
        return points.reshape(0, 2)
    m = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max)
         & (points[:, 1] >= y_min) & (points[:, 1] <= y_max))
    return points[m]


def cluster_points(points: np.ndarray, gap_mm: float = 60.0, min_pts: int = 3) -> List[dict]:
    """각도 순으로 정렬된 점을 이웃 간격으로 끊어 묶는다 (스캔 순서 기반, 가볍다).

    반환: [{'x', 'y' (중심), 'n', 'width' (y 방향 폭), 'depth' (x 방향 폭), 'near_x'}]
    """
    if len(points) == 0:
        return []
    ang = np.arctan2(points[:, 1], points[:, 0])
    pts = points[np.argsort(ang)]
    d = np.hypot(*np.diff(pts, axis=0).T) if len(pts) > 1 else np.array([])
    breaks = np.nonzero(d > gap_mm)[0] + 1
    out = []
    for grp in np.split(pts, breaks):
        if len(grp) < min_pts:
            continue
        out.append({'x': float(grp[:, 0].mean()), 'y': float(grp[:, 1].mean()),
                    'n': int(len(grp)),
                    'width': float(np.ptp(grp[:, 1])), 'depth': float(np.ptp(grp[:, 0])),
                    'near_x': float(grp[:, 0].min())})
    return out


def lane_occupancy(clusters: List[dict], lane_centers_y: Sequence[float],
                   half_width: float, x_max: float) -> List[Optional[float]]:
    """차로마다 가장 가까운 장애물의 전방 거리 (없으면 None).

    lane_centers_y: 각 차로 중심의 y (mm, 왼쪽 +). 3차선이면 예: [500, 0, -500]
    """
    out: List[Optional[float]] = []
    for cy in lane_centers_y:
        xs = [c['near_x'] for c in clusters
              if abs(c['y'] - cy) <= half_width and 0.0 < c['near_x'] <= x_max]
        out.append(min(xs) if xs else None)
    return out


def choose_lane(occupancy: List[Optional[float]], current: int,
                min_clear_mm: float = 900.0) -> int:
    """지금 차로가 막혔으면 가장 멀리까지 비어 있는 차로를 고른다 (가까운 차로 우선)."""
    def clear(i):
        return math.inf if occupancy[i] is None else occupancy[i]
    if clear(current) >= min_clear_mm:
        return current
    order = sorted(range(len(occupancy)), key=lambda i: (-min(clear(i), 1e9), abs(i - current)))
    return order[0]


def cone_gate_target(clusters: List[dict], x_max: float = 1200.0,
                     max_gap: float = 700.0) -> Optional[tuple]:
    """가장 가까운 왼쪽 콘과 오른쪽 콘 사이 중점 (x, y). 한쪽만 있으면 그 반대쪽으로 비켜 간다."""
    near = [c for c in clusters if 0.0 < c['x'] <= x_max]
    left = [c for c in near if c['y'] > 0]
    right = [c for c in near if c['y'] <= 0]
    lc = min(left, key=lambda c: c['x']) if left else None
    rc = min(right, key=lambda c: c['x']) if right else None
    if lc and rc and abs(lc['y'] - rc['y']) <= max_gap:
        return ((lc['x'] + rc['x']) / 2.0, (lc['y'] + rc['y']) / 2.0)
    if lc:
        return (lc['x'], lc['y'] - 250.0)
    if rc:
        return (rc['x'], rc['y'] + 250.0)
    return None


def pursuit_kappa(target: tuple) -> float:
    """기준점에서 목표점(x 전방, y 왼쪽, mm)을 지나는 원의 곡률 (1/m, 왼쪽 +)."""
    x, y = target
    d2 = x * x + y * y
    return 0.0 if d2 < 1.0 else 2000.0 * y / d2
