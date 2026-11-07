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


def run_m7_external(done_at_mm=300.0, fork_sign=True, active_from=0.0, dt=0.05):
    """M7을 limo_obstacle_avoid로: 시작선부터 m7_active, 섬 끝 + done_at_mm에서 m7_done."""
    route = RouteMap.from_yaml(ROUTE_YAML)
    p = route.points
    start = next(m['s'] for m in route.landmarks if m['name'] == 'm7_start_line')
    fsm = MissionFSM({'route_points': p, 'vote_window': 3, 'vote_need': 2,
                      'start_state': 'M7_OBSTACLE', 'm7_external_avoid': True})
    t, s = 0.0, p['m3_crosswalk_end']
    sources, entered = [], {}
    for _ in range(20000):
        det = {'fork_b': 0.9} if fork_sign and fsm.state in ('M7_OBSTACLE', 'M2_FORK') else {}
        end = p['m7_island_tip'] + (done_at_mm if done_at_mm is not None else 1e9)
        obstacle = {'m7_active': start + active_from <= s < end,
                    'm7_done': done_at_mm is not None and s >= p['m7_island_tip'] + done_at_mm,
                    'm7_exit': 'A'}
        cmd = fsm.step(Inputs(t=t, dist_mm=s - p['m3_crosswalk_end'], route_s=s, detections=det,
                              new_detection_frame=True, obstacle=obstacle))
        entered.setdefault(fsm.state, s)
        if fsm.state == 'M7_OBSTACLE':
            sources.append((s, cmd.source))
        s += cmd.lane.get('speed', 0.0) * 1000.0 * dt
        t += dt
        if fsm.state == 'M8_CONES':
            break
    return p, start, fsm, sources, entered


def test_m7_external_waits_for_avoid_done():
    p, start, fsm, sources, entered = run_m7_external(done_at_mm=300.0)
    # 예전처럼 섬 끝 300 mm 앞이 아니라, 회피 노드가 끝났을 때 M2로
    assert entered['M2_FORK'] == pytest.approx(p['m7_island_tip'] + 300, abs=20)
    # 코너·시작선 전에는 차선, 회피 노드가 켜진 뒤에는 obstacle
    assert all(src == 'lane' for s, src in sources if s < start)
    assert all(src == 'obstacle' for s, src in sources if s >= start + 20)
    assert fsm.fork_choice == 'B'                      # 표지 투표가 회피 차로(A)보다 우선


def test_m7_external_forced_end_if_avoid_never_finishes():
    p, start, fsm, sources, entered = run_m7_external(done_at_mm=None)
    assert entered['M2_FORK'] == pytest.approx(p['m7_island_tip'] + 500, abs=20)
    assert 'forced' in ' '.join(fsm.log)


def test_m2_uses_avoid_exit_lane_when_sign_unread():
    p, start, fsm, sources, entered = run_m7_external(fork_sign=False)
    assert fsm.fork_choice == 'A'                      # 회피가 끝난 차로 그대로 (섬을 건너지 않게)
    assert entered['M8_CONES'] == pytest.approx(p['m2_merge_end'], abs=20)


def test_m7_external_off_keeps_old_behaviour():
    route, fsm, entered, stops = run_lap()
    assert entered['M2_FORK'] == pytest.approx(route.points['m7_island_tip'] - 300, abs=20)


def test_m7_external_hold_stops():
    """회피 노드가 도중에 멈추면(m7_hold) 차선 주행으로 넘기지 않고 정지."""
    fsm = MissionFSM({'start_state': 'M7_OBSTACLE', 'm7_external_avoid': True,
                      'route_points': {'m7_island_tip': 17973.0}})
    run = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=14000.0, obstacle={'m7_active': True}))
    assert run.source == 'obstacle'
    held = fsm.step(Inputs(t=0.1, dist_mm=20.0, route_s=14020.0,
                           obstacle={'m7_active': False, 'm7_hold': True}))
    assert held.source == 'stop' and fsm.state == 'M7_OBSTACLE'
    lost = fsm.step(Inputs(t=0.2, dist_mm=20.0, route_s=14020.0, obstacle={'m7_active': False}))
    assert lost.source == 'lane'                   # hold가 없으면 예전대로 (코너 구간 등)
