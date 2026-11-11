"""11/10 위치 회복 테스트: 길 잃음(lost) 판단, 선 여러 개 놓친 뒤 다시 맞추기, 후진, 카메라 선 위치 다시 잡기."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.fsm import Inputs, MissionFSM  # noqa: E402
from autorace_mission.route import RouteMap, RouteTracker  # noqa: E402

ROUTE_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup',
                          'config', 'track_route.yaml')


@pytest.fixture(scope='module')
def route():
    return RouteMap.from_yaml(ROUTE_YAML)


def _overrun(route, lost_misses):
    """횡단보도 끝선 뒤에서 출발, odom이 8% 길게 재면서 m7 시작선·m8 시작선을 못 보고 지나감."""
    lm = route.points
    tr = RouteTracker(route, ordered_gate_mm=1200.0, lost_misses=lost_misses)
    tr.reset(12000.0, 0.0)
    true_s = lm['m8_end_line'] - 600.0
    dist = (true_s - 12000.0) * 1.08
    tr.update(dist)
    return tr, true_s, dist


def test_lost_after_two_missed_lines(route):
    tr, _, _ = _overrun(route, lost_misses=2)
    assert tr.missed == 2
    assert tr.lost and tr.state()['lost'] is True


def test_lost_off_by_default(route):
    tr, _, _ = _overrun(route, lost_misses=0)
    assert tr.missed == 2 and not tr.lost


def test_recovers_after_two_missed_lines(route):
    """예전엔 다음 선 2개(i+1, i+2)만 봐서 둘 다 놓치면 영영 못 맞췄다."""
    tr, true_s, dist = _overrun(route, lost_misses=2)
    tr.update(dist, [600.0])                  # 카메라: m8 끝선이 600mm 앞
    assert tr.last_fix['name'] == 'm8_end_line'
    assert tr.s == pytest.approx(true_s, abs=1.0)
    assert tr.missed == 0 and not tr.lost


def test_reverse_distance_moves_s_back(route):
    """mission_node가 후진 거리를 빼서 주면 s도 뒤로 간다 (route_signed_dist)."""
    tr = RouteTracker(route)
    tr.reset(5000.0, 0.0)
    assert tr.update(1000.0) == pytest.approx(6000.0)
    assert tr.update(400.0) == pytest.approx(5400.0)   # 600mm 후진


def test_camera_line_crossing_requests_anchor(route):
    """경로 위치가 없어도(lost) 카메라로 본 지그재그 시작선을 지나면 그 선으로 위치 다시 잡기 요청."""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'route_points': route.points})
    fsm.step(Inputs(t=0.0, dist_mm=0.0, lane={'stoplines_mm': [400.0]}))
    assert fsm.pop_anchor() is None                    # 아직 안 지남
    fsm.step(Inputs(t=1.0, dist_mm=450.0, lane={'stoplines_mm': []}))
    a = fsm.pop_anchor()
    assert a is not None and a[0] == 'm6_zigzag_line'
    assert a[1] == pytest.approx(-50.0)                # 선을 50mm 지남
    assert fsm.sub == 'zigzag'
    fsm.step(Inputs(t=2.0, dist_mm=500.0, lane={'stoplines_mm': []}))
    assert fsm.pop_anchor() is None                    # 같은 선은 한 번만


def test_route_only_crossing_does_not_anchor(route):
    """도면 위치로만 판단한 통과는 위치 다시 잡기를 하지 않는다 (틀린 위치를 스스로 굳히지 않게)."""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'route_points': route.points})
    s = route.points['m6_zigzag_line'] + 10.0
    fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=s))
    assert fsm.sub == 'zigzag'
    assert fsm.pop_anchor() is None
