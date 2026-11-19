"""m7_waypoint_core (11/19 실험, m7_impl:=waypoint) 오프라인 시뮬로 확인."""
import math

import numpy as np
import pytest

from limo_obstacle_avoid.m7_waypoint_core import M7Waypoint, WaypointParams, fit_box
from limo_obstacle_avoid.m7_waypoint_sim import run


def test_fit_box_rear_face_gives_heading_and_center():
    # 가로 정차 LIMO(차로 방향 0.22, 폭 0.32) 뒷면만 보임, 차가 5° 틀어짐 (맞추기 함수만 확인; 실차 기본값은 세로 0.32 x 0.22)
    th = math.radians(5.0)
    cx, cy = 1.2, 0.05
    pts = []
    for i in range(16):
        b = -0.16 + i * 0.02
        a = -0.11
        x = cx + a * math.cos(-th) - b * math.sin(-th)
        y = cy + a * math.sin(-th) + b * math.cos(-th)
        pts.append((x, y, math.atan2(y, x)))
    r = fit_box(pts, 0.0, 0.22, 0.32, 0.10)
    assert r is not None
    assert abs(r[0] - cx) < 0.03 and abs(r[1] - cy) < 0.03
    assert abs(math.degrees(r[2]) + 5.0) < 2.0
    assert r[3]                      # 뒷면(32 cm)만으로 방향을 잼


def test_m2_only_mode_finishes_immediately():
    p = WaypointParams()
    p.m2_test_start = True
    c = M7Waypoint(p)
    c.start()
    assert c.state == 'DONE'


@pytest.mark.parametrize('seed', [0, 3, 14])
def test_race1_layout_straight_entry_passes_without_reverse(seed):
    r = run(seed, 'straight', {'reverse_max': 0})
    assert r['feasible']
    assert not r['contact'] and not r['depart']
    assert r['success'], r


@pytest.mark.parametrize('seed', [1, 4])
def test_infeasible_layout_stops_without_contact(seed):
    r = run(seed, 'straight', {'reverse_max': 0}, t_max=40.0)
    assert not r['feasible']
    assert not r['contact']


def test_walking_person_in_lane3_does_not_block_forever():
    # 11/19 13번 판: 3차로에 사람이 5 s 지나감 -> 기다렸다가(옮길 자리 남김) 지나감
    r = run(0, 'straight', {'reverse_max': 0}, ped=True)
    assert r['feasible'] and not r['contact'] and not r['depart']
    assert r['success'], r


def test_lane_core_yellow_fit_ignores_far_points():
    from autorace_lane.lane_core import _yellow_fit_lat
    # 오른쪽 노란선이 차 오른쪽 0.6 m에 있고 차가 20° 그쪽으로 틀어짐 -> 앞으로 갈수록 옆 거리가 줄어듦
    a = math.radians(20.0)
    ys = np.arange(600.0, 2100.0, 20.0)
    xs = 600.0 / math.cos(a) - ys * math.tan(a)        # x 오른쪽 + (mm)
    pts = np.stack([xs, ys], axis=1)
    lat = _yellow_fit_lat(pts, 550.0)
    assert lat == pytest.approx(600.0 / math.cos(a) - 550.0 * math.tan(a), abs=1.0)   # 약 438 mm
    assert float(np.percentile(xs, 15)) < 150.0                                         # 예전 방식은 '붙음'으로 봄


def _core_running():
    p = WaypointParams()
    p.section_length = 6.0
    c = M7Waypoint(p)
    c.start()
    return c


def _wall_scan(r_front):
    """앞이 r_front까지 비어 있는 스캔 (각도 -110..110°, 1°)."""
    pts = []
    for i in range(-110, 111):
        a = math.radians(i)
        r = min(r_front / max(math.cos(a), 1e-3), 3.0) if abs(i) < 80 else 3.0
        if r < 3.0:
            pts.append((0.10 + r * math.cos(a), r * math.sin(a), a))
    return pts


def test_ghost_memory_forgotten_when_lidar_sees_through():
    # 11/19 13번 판: 지나간 사람 자리가 기억에 남아 '길 없음'이 이어짐 -> 그 자리를 라이다가 뚫고 보면 지움
    c = _core_running()
    c.mem.append({'x': 1.2, 'y': 0.0, 'n': 5, 'seen': 0.0, 'prior': False})
    pts = _wall_scan(2.5)
    for i in range(c.p.forget_n + 1):
        c.on_scan(1.0 + 0.1 * i, pts, math.radians(-110), math.radians(110))
    assert not any(abs(m['x'] - 1.2) < 0.2 and abs(m['y']) < 0.2 for m in c.mem)
    assert c.n_forgot >= 1


def test_memory_kept_when_occluded():
    # 앞 0.8 m에 다른 물체(벽)가 가리면 그 뒤 기억은 그대로
    c = _core_running()
    c.mem.append({'x': 1.6, 'y': 0.0, 'n': 5, 'seen': 0.0, 'prior': False})
    pts = _wall_scan(0.8)
    for i in range(c.p.forget_n + 3):
        c.on_scan(1.0 + 0.1 * i, pts, math.radians(-110), math.radians(110))
    assert any(abs(m['x'] - 1.6) < 0.05 for m in c.mem)


def test_heading_needs_consistent_window():
    # 한 바퀴씩 ±8° 튀는 방향 값은 바로 반영하지 않음 (중앙값·퍼짐으로만)
    c = _core_running()
    psi0 = c.pose()[2]
    th = math.radians(5.0)
    for i, j in enumerate([8, -8, 8, -8, 8, -8, 8, -8]):
        a = math.radians(j)
        pts = []
        for k in range(14):
            b = -0.11 + k * 0.017
            x = 1.3 + -0.16 * math.cos(a) - b * math.sin(a)
            y = 0.0 + -0.16 * math.sin(a) + b * math.cos(a)
            pts.append((x, y, math.atan2(y, x - 0.10)))
        pts.sort(key=lambda q: q[2])
        c.on_scan(1.0 + 0.1 * i, pts, math.radians(-110), math.radians(110))
    assert abs(math.degrees(c.pose()[2] - psi0)) < 3.0
