"""도면 경로 위치(route_s)로 한 바퀴 돌려 보는 FSM 시뮬레이션."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.fsm import Inputs, MissionFSM  # noqa: E402
from autorace_mission.route import RouteMap  # noqa: E402

ROUTE_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup',
                          'config', 'track_route.yaml')


def run_lap(dt=0.05):
    route = RouteMap.from_yaml(ROUTE_YAML)
    fsm = MissionFSM({'route_points': route.points, 'vote_window': 3, 'vote_need': 2})
    t, s = 0.0, 0.0
    entered = {}
    stops = {}
    inp = Inputs(t=t, dist_mm=s, route_s=s, start=True)
    for _ in range(40000):
        det = {}
        if fsm.state == 'M7_OBSTACLE' or fsm.state == 'M2_FORK':
            det['fork_b'] = 0.9
        if fsm.state == 'M1_TRAFFIC':
            det['green' if fsm.sub == 'stopped' and t - fsm.sub_t > 2.0 else 'red'] = 0.9
        parking = {'done': fsm.state == 'M4_PARKING' and s > route.points['m4_parking_from']}
        inp = Inputs(t=t, dist_mm=s, route_s=s, detections=det, new_detection_frame=True,
                     parking=parking, start=inp.start and fsm.state == 'WAIT_START')
        cmd = fsm.step(inp)
        entered.setdefault(fsm.state, s)
        if cmd.source == 'stop' and fsm.state in ('M3_PEDESTRIAN', 'M1_TRAFFIC'):
            stops.setdefault(fsm.state, s)
        v = 0.0 if cmd.source == 'stop' else cmd.lane.get('speed', 0.0)
        s += v * 1000.0 * dt
        t += dt
        if fsm.state == 'FINISHED':
            break
    return route, fsm, entered, stops


def test_full_lap_by_route():
    route, fsm, entered, stops = run_lap()
    p = route.points
    assert fsm.state == 'FINISHED', fsm.log
    # 각 상태가 도면 위치 근처에서 시작한다
    assert entered['M6_ZIGZAG'] < p['m6_zigzag_line']
    assert entered['M3_PEDESTRIAN'] == pytest.approx(p['m6_zigzag_end'], abs=20)
    # 출발 후 leave_mm(600)과 횡단보도 끝선 둘 다 지나야 넘어간다
    assert p['m3_crosswalk_end'] <= entered['M7_OBSTACLE'] <= p['m3_crosswalk_end'] + 100
    assert entered['M2_FORK'] == pytest.approx(p['m7_island_tip'] - 300, abs=20)
    assert entered['M8_CONES'] == pytest.approx(p['m2_merge_end'], abs=20)
    assert entered['M1_TRAFFIC'] == pytest.approx(p['m8_end_line'] + 100, abs=20)
    # 카메라 정지선 없이도 도면 위치로 정지선 앞에서 선다 (brake 30 mm 앞)
    assert stops['M3_PEDESTRIAN'] == pytest.approx(p['m3_stop_line'] - 30, abs=15)
    assert stops['M1_TRAFFIC'] == pytest.approx(p['m1_stop_line'] - 30, abs=15)
    assert fsm.fork_choice == 'B'


def test_zigzag_mode_only_inside_zigzag():
    route = RouteMap.from_yaml(ROUTE_YAML)
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'route_points': route.points})
    zl, ze = route.points['m6_zigzag_line'], route.points['m6_zigzag_end']
    assert fsm.step(Inputs(t=0, dist_mm=0, route_s=zl - 500)).lane['mode'] == 'normal'
    assert fsm.step(Inputs(t=1, dist_mm=500, route_s=zl)).lane['mode'] == 'zigzag'
    assert fsm.step(Inputs(t=2, dist_mm=1500, route_s=zl + 1000)).lane['mode'] == 'zigzag'
    fsm.step(Inputs(t=3, dist_mm=2300, route_s=ze + 1))
    assert fsm.state == 'M3_PEDESTRIAN'


def test_without_route_falls_back_to_distance():
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'zigzag_start_mm': 100.0,
                      'zigzag_len_mm': 300.0, 'route_points': {'m6_zigzag_line': 9292.0}})
    fsm.step(Inputs(t=0, dist_mm=0))
    assert fsm.step(Inputs(t=1, dist_mm=150)).lane['mode'] == 'zigzag'   # route_s 없음 -> 거리
