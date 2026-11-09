"""도면 경로 지도·위치 추정 테스트."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.route import RouteMap, RouteTracker  # noqa: E402

ROUTE_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup',
                          'config', 'track_route.yaml')


@pytest.fixture(scope='module')
def route():
    return RouteMap.from_yaml(ROUTE_YAML)


def test_route_is_continuous_and_closed_where_expected(route):
    # 출발 체커에서 안쪽 상단 차로 끝까지 약 32 m
    assert 31000 < route.length < 33000
    for a, b in zip(route.segs, route.segs[1:]):
        end = a.point(a.length)
        assert math.dist(end, b.point(0.0)) < 1.0


def test_track_order_matches_rules(route):
    p = route.points
    order = ['m5_slow_line', 'm6_zigzag_line', 'm6_zigzag_end', 'm3_stop_line',
             'm7_start_line', 'm7_island_tip', 'm2_merge_end', 'm8_start_line',
             'm8_end_line', 'm1_stop_line', 'm4_parking_from']
    s = [p[k] for k in order]
    assert s == sorted(s)


def test_known_distances(route):
    p = route.points
    assert p['m6_zigzag_end'] - p['m6_zigzag_line'] == pytest.approx(1717, abs=5)  # 1677 + 선 40
    assert p['m3_stop_line'] - p['m6_zigzag_end'] == pytest.approx(266, abs=5)
    assert p['m8_end_line'] - p['m8_start_line'] == pytest.approx(2100, abs=5)


def test_headings_and_curvature(route):
    assert route.heading(1000) == pytest.approx(0.0)                 # 동쪽
    assert route.heading(10000) == pytest.approx(-math.pi / 2)       # 우측 남쪽
    assert abs(route.heading(15000)) == pytest.approx(math.pi)       # 하단 서쪽
    assert route.heading(22500) == pytest.approx(math.pi / 2)        # 좌측 북쪽
    assert route.curvature(8000) < 0                                 # 시계방향 = 오른쪽 회전
    assert route.curvature(5000) == 0.0
    assert route.max_abs_curvature(7000, 8000) > 1.0


def test_s_of_roundtrip(route):
    for s in (500.0, 8300.0, 12000.0, 16000.0, 21000.0, 23000.0, 30000.0):
        assert route.s_of(route.xy(s)) == pytest.approx(s, abs=1.0)


def test_tracker_odometry_only(route):
    t = RouteTracker(route)
    t.reset(0.0, dist_mm=100.0)
    assert t.update(1100.0) == pytest.approx(1000.0)


def test_tracker_snaps_after_long_drift(route):
    # 오도메트리가 실제보다 6% 짧게 잰다: 실제 9000 mm -> 8460 (540 mm 뒤처짐)
    t = RouteTracker(route, gain=1.0)
    t.reset(0.0, dist_mm=0.0)
    t.update(8460.0)
    assert t.gate > 540.0                     # 오래 달려서 게이트가 넓어졌다
    true_s = 9000.0
    d = route.points['m6_zigzag_line'] - true_s   # 카메라: 앞바퀴 -> 지그재그 시작선
    t.update(8460.0, [d])
    assert t.s == pytest.approx(true_s, abs=1.0)
    assert t.last_fix['name'] == 'm6_zigzag_line'
    assert t.gate == pytest.approx(t.gate_mm)  # 맞춘 뒤 다시 좁아진다


def test_gate_is_capped(route):
    t = RouteTracker(route)
    t.reset(0.0, 0.0)
    t.update(30000.0)
    assert t.gate == t.gate_max_mm


def test_tracker_converges_with_partial_gain(route):
    t = RouteTracker(route, gain=0.6)
    t.reset(13000.0, 0.0)                     # 실제는 13100 (100 mm 뒤처짐)
    line = route.points['m7_start_line']
    for _ in range(6):
        t.update(0.0, [line - 13100.0])
    assert t.s == pytest.approx(13100.0, abs=2.0)


def test_tracker_ignores_unknown_line(route):
    t = RouteTracker(route)
    t.reset(5000.0, 0.0)                      # 상단 직선 한가운데: 근처에 도면 선 없음
    t.update(0.0, [400.0])
    assert t.fixes == 0 and t.s == 5000.0


def test_state_reports_next_landmark(route):
    t = RouteTracker(route)
    t.reset(9000.0, 0.0)
    st = t.state()
    assert st['next'] == 'm6_zigzag_line'
    assert st['next_in_mm'] == pytest.approx(route.points["m6_zigzag_line"] - 9000.0, abs=5)


def test_ordered_matching_survives_zigzag_overrun(route):
    """지그재그에서 odom이 80cm 더 늘어나도 정지선을 순서대로 맞춘다 (11/06)."""
    lm = {m['name']: m['s'] for m in route.landmarks}
    for ordered, expect in ((1200.0, 'm3_stop_line'), (0.0, None)):
        tr = RouteTracker(route, ordered_gate_mm=ordered)
        tr.reset(lm['m6_zigzag_line'] + 50.0, 0.0)
        # 실제로는 정지선 600mm 앞에 왔는데 odom은 800mm 더 왔다고 본다
        true_s = lm['m3_stop_line'] - 600.0
        dist = (true_s - tr.s) + 800.0
        tr.update(dist, [600.0])
        if expect is None:
            assert tr.last_fix is None or tr.last_fix['name'] != 'm3_stop_line'
        else:
            assert tr.last_fix['name'] == expect
            for _ in range(10):                       # 같은 선을 계속 보면 수렴
                tr.update(dist, [600.0])
            assert tr.s == pytest.approx(true_s, abs=30.0)
            # 다음 선(횡단보도 끝선)도 순서대로
            tr.update(dist + (lm['m3_crosswalk_end'] - 300.0 - true_s), [300.0])
            assert tr.last_fix['name'] == 'm3_crosswalk_end'


def test_map_pose_points_and_drift(route):
    from autorace_mission.route import MapPose
    mp = MapPose(route)
    s = 3000.0                                   # 상단 직선 (동쪽, heading 0)
    imu_yaw = 0.7                                # IMU 기준이 도면과 0.7 rad 다르다고 하자
    lane = {'status': 'BOTH', 'low_conf': False, 'model': 'poly', 'heading_rad': 0.0,
            'offset_mm': 100.0}                 # 차로 중심이 차 왼쪽 100mm -> 차는 중심 오른쪽 100
    assert mp.update_lane(s, imu_yaw, lane, 0.0)
    assert mp.yaw_off == pytest.approx(-0.7)
    pts = mp.points_ahead(s, imu_yaw)
    # 중심선은 차 왼쪽 100mm, 앞으로 쭉
    assert pts[5][0] == pytest.approx(-100.0, abs=1.0)
    assert pts[5][1] == pytest.approx(200.0 + 500.0, abs=1.0)
    # 차가 오른쪽으로 0.1 rad 틀어진 채 1m 가면 d가 약 -100 더 (오른쪽으로)
    mp.predict(0.0, s, imu_yaw)
    mp.predict(1000.0, s + 1000.0, imu_yaw - 0.1)
    assert mp.d == pytest.approx(-100.0 - 1000.0 * math.sin(0.1), abs=2.0)
    assert mp.valid(1000.0) and not mp.valid(5000.0)
    # 코너 앞: 중심선 점이 오른쪽으로 휜다
    s2 = 7800.0
    mp2 = MapPose(route)
    mp2.update_lane(7000.0, 0.0, dict(lane, offset_mm=0.0), 0.0)
    pts2 = mp2.points_ahead(s2, 0.0)
    assert pts2[-1][0] > 100.0


def test_text_before_stop_line_does_not_snap(route):
    """정지선 550mm 앞 '천천히' 글자를 정지선·끝선으로 맞추지 않고, 진짜 정지선에서 맞춘다 (11/09)."""
    lm = {m['name']: m['s'] for m in route.landmarks}
    tr = RouteTracker(route, ordered_gate_mm=1200.0)
    tr.reset(lm['m6_zigzag_line'] + 50.0, 0.0)
    tr.last_idx = [m['name'] for m in route.landmarks].index('m6_zigzag_line')
    text_s = lm['m3_stop_line'] - 550.0
    dist = (text_s - 700.0) - tr.s                    # 글자가 700mm 앞에 보이는 위치까지 odom 정확히
    tr.update(dist, [700.0], [700.0])                 # 글자가 막대 판정까지 받은 최악의 경우
    assert tr.last_fix is None or tr.last_fix['name'] not in ('m3_stop_line', 'm3_crosswalk_end')
    s_before = tr.s
    dist2 = dist + (lm['m3_stop_line'] - 700.0) - s_before
    tr.update(dist2, [700.0 - 550.0, 700.0], [700.0])  # 글자(150) + 정지선(700, 막대)
    assert tr.last_fix['name'] == 'm3_stop_line'
    assert tr.s == pytest.approx(lm['m3_stop_line'] - 700.0, abs=30.0)


def test_plain_line_never_snaps_to_crosswalk_landmark(route):
    lm = {m['name']: m['s'] for m in route.landmarks}
    tr = RouteTracker(route)
    tr.reset(lm['m3_stop_line'] - 600.0, 0.0)
    tr.update(0.0, [600.0], [])                       # 막대 없는 선
    assert tr.last_fix is None
    tr.update(0.0, [600.0], [600.0])
    assert tr.last_fix['name'] == 'm3_stop_line'
