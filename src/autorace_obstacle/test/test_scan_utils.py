"""라이다 함수 테스트 (`python3 -m pytest src/autorace_obstacle/test`)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_obstacle.scan_utils import (choose_lane, cluster_points,  # noqa: E402
                                          cone_gate_target, lane_occupancy,
                                          pursuit_kappa, scan_to_points)


def fake_scan(objects, n=720, rmax=8.0):
    """objects: [(x_mm, y_mm, radius_mm)] 원기둥들의 가상 스캔."""
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    ranges = np.full(n, np.inf)
    for i, a in enumerate(angles):
        d = np.array([math.cos(a), math.sin(a)])
        for x, y, r in objects:
            c = np.array([x, y]) / 1000.0
            b = d @ c
            disc = b * b - (c @ c - (r / 1000.0) ** 2)
            if disc >= 0 and b > 0:
                ranges[i] = min(ranges[i], b - math.sqrt(disc))
    return ranges, -math.pi, 2 * math.pi / n


def test_scan_to_points_front_object():
    ranges, amin, inc = fake_scan([(1000, 0, 50)])
    pts = scan_to_points(ranges, amin, inc, 0.05, 8.0)
    assert len(pts) > 0
    assert pts[:, 0].min() == pytest.approx(950, abs=15)


def test_clusters_and_lane_occupancy():
    ranges, amin, inc = fake_scan([(800, 0, 60), (1500, 500, 60)])
    pts = scan_to_points(ranges, amin, inc, 0.05, 8.0)
    cl = cluster_points(pts)
    assert len(cl) == 2
    occ = lane_occupancy(cl, [500, 0, -500], 230, 2000)
    assert occ[1] == pytest.approx(740, abs=20)
    assert occ[0] == pytest.approx(1440, abs=20)
    assert occ[2] is None
    assert choose_lane(occ, current=1) == 2     # 오른쪽 차로가 비어 있음


def test_cone_gate_midpoint_and_kappa():
    ranges, amin, inc = fake_scan([(700, 250, 40), (750, -230, 40)])
    cl = cluster_points(scan_to_points(ranges, amin, inc, 0.05, 8.0))
    tgt = cone_gate_target(cl)
    assert tgt[1] == pytest.approx(10, abs=25)
    assert pursuit_kappa((1000, 100)) > 0       # 왼쪽 목표면 왼쪽(+)으로
