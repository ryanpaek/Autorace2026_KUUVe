"""/mission/stage: 지금 몇 번 미션인지, 확실하지 않으면 null (11/15)"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from autorace_mission.route import RouteMap  # noqa: E402
from autorace_mission.stage import mission_no, stage_info  # noqa: E402

ROUTE = RouteMap.from_yaml(os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup', 'config',
                                        'track_route.yaml'))
P = ROUTE.points


def test_mission_numbers():
    assert mission_no('M7_OBSTACLE') == 7 and mission_no('M2_EXIT') == 2 and mission_no('M4_PARKING') == 4
    assert mission_no('WAIT_START') is None and mission_no('FINISHED') is None and mission_no(None) is None


def test_sure_when_fsm_and_map_agree():
    s = P['m7_island_tip'] - 1500.0                      # 3차선 한가운데
    st = stage_info('M7_OBSTACLE', s, P)
    assert st['mission'] == 7 and st['order'] == 4 and st['sure']


def test_null_when_unsure():
    s = P['m7_island_tip'] - 1500.0
    st = stage_info('M8_CONES', s, P)                    # 미션은 라바콘인데 도면은 3차선 -> 모름
    assert st['mission'] is None and st['fsm_mission'] == 8 and st['map_mission'] == 7 and not st['sure']
    st = stage_info('M7_OBSTACLE', None, P)              # 도면 위치 없음(길 잃음) -> 모름
    assert st['mission'] is None and st['fsm_mission'] == 7
    st = stage_info('WAIT_START', 0.0, P)
    assert st['mission'] is None


def test_boundary_tolerance():
    s = P['m7_island_tip'] + 300.0                       # 섬 끝 30 cm 지남: FSM은 아직 M7일 수 있음
    assert stage_info('M7_OBSTACLE', s, P)['mission'] == 7
    assert stage_info('M2_FORK', s, P)['mission'] == 2
    assert stage_info('M7_OBSTACLE', P['m7_island_tip'] + 2000.0, P)['mission'] is None   # 2 m 넘게 다르면 모름
