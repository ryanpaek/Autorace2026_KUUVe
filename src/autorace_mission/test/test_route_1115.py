"""11/15: 1114 bag 위치 분석 반영 - 선 보정 조건, 코너 맞추기, 횡단보도 선 짝짓기, 곡선 d 보정, 조향 배율."""
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.ackermann_bridge import twist_to_ackermann
from autorace_mission.route import CornerAnchor, MapPose, RouteMap, RouteTracker, line_fix_allowed

ROUTE = RouteMap.from_yaml(str(Path(__file__).resolve().parents[2] / 'autorace_bringup' / 'config' / 'track_route.yaml'))
LM = {m['name']: m['s'] for m in ROUTE.landmarks}


def test_line_fix_allowed():
    assert line_fix_allowed(None, 'M7_OBSTACLE', None)                 # 방향 모르면 예전처럼
    assert line_fix_allowed(math.radians(5), 'M1_TRAFFIC', 'R')
    assert not line_fix_allowed(math.radians(25), 'M1_TRAFFIC', 'BOTH')   # 비스듬히 본 선
    assert not line_fix_allowed(math.radians(12), 'M7_OBSTACLE', 'BOTH')  # M7은 10°
    assert not line_fix_allowed(math.radians(3), 'M7_OBSTACLE', 'L')      # M7은 두 선
    assert line_fix_allowed(math.radians(3), 'M7_OBSTACLE', 'BOTH')
    assert not line_fix_allowed(0.0, 'M8_CONES', 'BOTH')                  # 라바콘 사이는 안 씀


def _drive_corner(anchor, s_est_offset, corner_idx):
    s0, s1, h_in, h_out = anchor.corners[corner_idx]
    out = None
    dist = 0.0
    n = 120
    for i in range(n + 1):           # 진입 직선 1 m -> 코너 -> 출구 직선 0.5 m (실제 s), 추정은 offset만큼 틀림
        s_true = s0 - 1000.0 + (s1 - s0 + 1500.0) * i / n
        dist = s_true
        k = min(1.0, max(0.0, (s_true - s0) / (s1 - s0)))
        h = h_in + math.atan2(math.sin(h_out - h_in), math.cos(h_out - h_in)) * k
        r = anchor.update(dist, s_true + s_est_offset, h)
        if r is not None and out is None:
            out = (r, s_true)
    return out


def test_corner_anchor_fixes_lag_once_per_corner():
    ca = CornerAnchor(ROUTE)
    assert len(ca.corners) == 4
    res = _drive_corner(ca, -1800.0, 3)          # 신호등 앞 코너, s 1.8 m 뒤처짐 (1114_03/04)
    assert res is not None
    s_new, s_true = res
    assert abs(s_new - s_true) < 150.0
    assert _drive_corner(ca, -1800.0, 3) is None   # 같은 코너는 한 번만


def test_corner_anchor_ignores_small_error_and_far_corner():
    ca = CornerAnchor(ROUTE)
    assert _drive_corner(ca, -50.0, 0) is None      # 작은 오차는 그대로
    ca = CornerAnchor(ROUTE)
    assert _drive_corner(ca, -5000.0, 1) is None    # window(3 m) 밖이면 안 맞춤


def test_corner_anchor_needs_entry_heading():
    ca = CornerAnchor(ROUTE)
    s0, s1, h_in, h_out = ca.corners[2]
    for i in range(50):                             # 출구 방향만 계속 (진입 방향 이력 없음)
        assert ca.update(i * 20.0, s1 - 1500.0, h_out) is None


def test_crosswalk_line_does_not_match_plain_landmark():
    tr = RouteTracker(ROUTE, ordered_gate_mm=1200.0)
    tr.reset(LM['m1_pre_line'] - 2500.0, 0.0)
    # 1114_03/04: s가 뒤처진 채 정지선(막대가 뒤따름)을 본 상황. 앞 선(m1_pre_line)과 짝지으면 안 됨
    d = 900.0
    s_line = LM['m1_pre_line'] + 100.0
    tr.s = s_line - d
    before = tr.s
    tr.update(0.0, [d], [d])
    assert tr.last_fix is None or tr.last_fix['name'] != 'm1_pre_line'
    tr2 = RouteTracker(ROUTE, ordered_gate_mm=1200.0, xw_only_crosswalk=False)
    tr2.reset(LM['m1_pre_line'] - 2500.0, 0.0)
    tr2.s = before
    tr2.update(0.0, [d], [d])
    assert tr2.last_fix and tr2.last_fix['name'] == 'm1_pre_line'      # 예전 동작


def test_mappose_curve_fix_blends_d():
    mp = MapPose(ROUTE)
    s0, s1 = ROUTE.segs[1].s0, ROUTE.segs[1].s0 + ROUTE.segs[1].length      # 첫 코너
    s = (s0 + s1) / 2.0
    yaw = ROUTE.heading(s)
    mp.align(s, yaw, 0.0)
    mp.d = 150.0
    lane = {'status': 'BOTH', 'model': 'poly', 'low_conf': False, 'heading_rad': 0.0, 'offset_mm': 0.0}
    assert mp.update_lane(s, yaw, lane, 10.0)
    assert 90.0 < mp.d < 120.0                      # 150 -> 0 쪽으로 30%만
    off0 = mp.yaw_off
    mp.update_lane(s, yaw + 0.1, dict(lane, heading_rad=0.0), 20.0)
    assert mp.yaw_off == off0                        # 곡선에선 방향 보정값 안 바꿈
    mp2 = MapPose(ROUTE, curve_fix=False)
    mp2.align(s, yaw, 0.0)
    assert not mp2.update_lane(s, yaw, lane, 10.0)


def test_steer_gain():
    v, d1 = twist_to_ackermann(0.2, 0.2 * 1.5, steer_gain=1.0)
    v, d2 = twist_to_ackermann(0.2, 0.2 * 1.5, steer_gain=1.2)
    assert abs(d2 - 1.2 * d1) < 1e-9
    v, d3 = twist_to_ackermann(0.2, 0.2 * 3.0, steer_gain=1.2)
    assert d3 == 0.40                                # 최대 조향은 그대로
