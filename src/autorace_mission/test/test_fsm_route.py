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
    # 11/14: 섬 끝 + 200 mm -> M2_EXIT(차선 인식 모드), 라바콘 시작선(도면 위치 - cones_enter_before) -> M8
    assert entered['M2_EXIT'] == pytest.approx(p['m2_island_end'] + 200, abs=20)
    assert p['m8_start_line'] - 400 <= entered['M8_CONES'] <= p['m8_start_line'] + 20
    assert entered['M1_TRAFFIC'] == pytest.approx(p['m8_end_line'] + 100, abs=20)
    # 카메라 정지선 없이도 도면 위치로 정지선 앞에서 선다 (목표 150 mm + brake 30 mm 앞, 11/15 저녁)
    # M3(25 cm 목표): 지그재그 끝(M3 시작) ~ 정지선이 도면상 266 mm뿐이라 M3에 들어오자마자 섬
    assert p['m6_zigzag_end'] - 20 <= stops['M3_PEDESTRIAN'] <= p['m3_stop_line'] - 240
    assert stops['M1_TRAFFIC'] == pytest.approx(p['m1_stop_line'] - 180, abs=15)
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


def test_cones_by_route_positions():
    route = RouteMap.from_yaml(ROUTE_YAML)
    p = route.points
    fsm = MissionFSM({'start_state': 'M8_CONES', 'route_points': p, 'cones_source_ready': True})
    s0 = p['m2_merge_end']
    s = s0
    subs = {}
    for k in range(4000):
        inp = Inputs(t=k * 0.05, dist_mm=s - s0, route_s=s,
                     lane={'status': 'BOTH'} if s > p['m8_end_line'] + 200 else {'status': 'NONE'})
        fsm.step(inp)
        subs.setdefault((fsm.state, fsm.sub), s)
        if fsm.state == 'M1_TRAFFIC':
            break
        s += 0.15 * 1000 * 0.05
    assert subs[('M8_CONES', 'cones')] == pytest.approx(p['m8_start_line'], abs=10)   # 시작선 통과 순간 (11/09)
    assert subs[('M8_CONES', 'handover')] == pytest.approx(p['m8_end_line'] + 150, abs=10)
    assert fsm.state == 'M1_TRAFFIC'


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
    assert fsm.fork_choice == 'A'                      # 11/11: 표지 B여도 이미 A 차로(섬 옆)에서 끝남 -> 차로를 따름


def test_m7_external_forced_end_if_avoid_never_finishes():
    p, start, fsm, sources, entered = run_m7_external(done_at_mm=None)
    assert entered['M2_FORK'] == pytest.approx(p['m7_island_tip'] + 500, abs=20)
    assert 'forced' in ' '.join(fsm.log)


def test_m2_uses_avoid_exit_lane_when_sign_unread():
    p, start, fsm, sources, entered = run_m7_external(fork_sign=False)
    assert fsm.fork_choice == 'A'                      # 회피가 끝난 차로 그대로 (섬을 건너지 않게)
    # 11/14: 섬 끝 + 200 mm -> M2_EXIT(차선 인식 모드), 라바콘 시작선(도면 위치 - cones_enter_before) -> M8
    assert entered['M2_EXIT'] == pytest.approx(p['m2_island_end'] + 200, abs=20)
    assert p['m8_start_line'] - 400 <= entered['M8_CONES'] <= p['m8_start_line'] + 20


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


def _m2_fsm(**kw):
    return MissionFSM({'start_state': 'M2_FORK', 'route_points': {'m7_island_tip': 17973.0, 'm2_merge_end': 20272.0},
                       **kw})


def test_m2_old_option_sign_wins_and_logs_conflict(old_m2):
    """fork_follow_m7_exit false(예전): 표지 A인데 B 차로에서 끝남 -> 표지 쪽 left_only."""
    fsm = _m2_fsm(fork_follow_m7_exit=False)
    fsm.fork_choice = 'A'
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=18100.0, obstacle={'m7_exit': 'B'}))
    assert fsm.fork_choice == 'A' and cmd.lane.get('mode') == 'left_only'
    assert any('fork conflict' in l and 'follow sign' in l for l in fsm.log)


def test_m2_default_follows_lane_on_conflict(old_m2):
    """기본(11/11): 표지 A인데 회피가 B 차로에서 끝남 -> 이미 섬 옆이라 B 차로 바깥선(right_only)을 따름."""
    fsm = _m2_fsm()
    fsm.fork_choice = 'A'
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=18100.0, obstacle={'m7_exit': 'B'}))
    assert fsm.fork_choice == 'B' and cmd.lane.get('mode') == 'right_only'
    assert any('fork conflict' in l and 'follow lane B' in l for l in fsm.log)


def test_m2_agreeing_sign_and_lane_unchanged(old_m2):
    fsm = _m2_fsm()
    fsm.fork_choice = 'A'
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=18100.0, obstacle={'m7_exit': 'A'}))
    assert fsm.fork_choice == 'A' and cmd.lane.get('mode') == 'left_only'
    assert not any('fork conflict' in l for l in fsm.log)


def test_m2_normal_mode_inside_branch(old_m2):
    """11/16: 섬 앞 끝 + 300 mm를 지나면 갈래 안은 양쪽 선 보통 차선 모드 (한쪽 선 모드는 선을 자주 놓침)."""
    fsm = _m2_fsm()
    fsm.fork_choice = 'A'
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=18100.0, obstacle={'m7_exit': 'A'}))
    assert cmd.lane.get('mode') == 'left_only'
    cmd = fsm.step(Inputs(t=1.0, dist_mm=300.0, route_s=18400.0, obstacle={'m7_exit': 'A'}))
    assert cmd.lane.get('mode') == 'normal' and cmd.lane.get('lane_width_mm') == fsm.p['fork_lane_width_mm']
    off = _m2_fsm(fork_normal=False)
    off.fork_choice = 'A'
    cmd = off.step(Inputs(t=0.0, dist_mm=0.0, route_s=18400.0, obstacle={'m7_exit': 'A'}))
    assert cmd.lane.get('mode') == 'left_only'


def _m2_end_dist(to_tip):
    """도면 위치 없이(길 잃음) M2를 끝내는 주행 거리 (M2 시작부터)."""
    fsm = MissionFSM({'start_state': 'M2_FORK'})
    fsm.fork_choice = 'A'
    ob = {'m7_exit': 'A'} if to_tip is None else {'m7_exit': 'A', 'm7_to_tip_mm': to_tip}
    d = 0.0
    while fsm.state == 'M2_FORK' and d < 6000.0:
        fsm.step(Inputs(t=d / 200.0, dist_mm=d, obstacle=ob))
        d += 10.0
    return d


def test_m2_end_distance_follows_island_tip_without_route():
    """표지로 일찍 꺼 섬 1 m 앞에서 M2가 시작되면 섬 옆에서 normal로 돌아가지 않게 1 m 더 감."""
    base = _m2_end_dist(None)
    assert base == pytest.approx(2300.0, abs=20)
    assert _m2_end_dist(1000.0) == pytest.approx(3300.0, abs=20)
    assert _m2_end_dist(-400.0) == pytest.approx(1900.0, abs=20)
    assert _m2_end_dist(5000.0) == pytest.approx(3800.0, abs=20)      # 최대 1.5 m


@pytest.fixture
def old_m2(monkeypatch):
    """11/18 이전 M2 동작(한쪽 선 모드·폭 460·합류 S자) 테스트: m2_plain_lane 끔."""
    from autorace_mission import fsm as fsm_mod
    monkeypatch.setitem(fsm_mod.DEFAULT_PARAMS, 'm2_plain_lane', False)


def test_m2_plain_lane_after_replay_is_normal_lane():
    """11/18: CSV 재생 뒤 M2는 일반 차선만 (한쪽 선 모드·폭 460·직선 방향 붙잡기·합류 S자 없음), 끝 판정은 그대로."""
    fsm = _m2_fsm(m2_island_ref=False)        # 11/19부터 기본은 섬 기준 (test_m2_island_ref_*)
    fsm.fork_choice = 'B'
    for i, s in enumerate((18100.0, 18400.0, 19500.0)):
        cmd = fsm.step(Inputs(t=float(i), dist_mm=s - 18100.0, route_s=s, yaw=0.0, obstacle={'m7_exit': 'B'}))
        assert cmd.lane.get('mode') == 'normal'
        assert 'lane_width_mm' not in cmd.lane and 'hold_axis_deg' not in cmd.lane
    assert getattr(fsm, 'm2m', None) is None
    d = 0.0
    while fsm.state == 'M2_FORK' and d < 6000.0:
        fsm.step(Inputs(t=d / 200.0, dist_mm=d, yaw=0.0, obstacle={'m7_exit': 'B'}))
        d += 10.0
    assert fsm.state == 'M2_EXIT'
    cmd = fsm.step(Inputs(t=99.0, dist_mm=d, yaw=0.0, obstacle={'m7_exit': 'B'}))
    assert cmd.lane.get('mode') == 'normal' and getattr(fsm, 'm2m', None) is None


def test_m2_island_ref_after_replay():
    """11/19: CSV 재생 뒤 M2 = 섬 기준. B면 섬 오른쪽 변을 왼쪽 선으로(left_only), A면 right_only, 폭 520, 끝나면 M2_EXIT 일반 차선"""
    for choice, mode in (('B', 'left_only'), ('A', 'right_only')):
        fsm = _m2_fsm()
        fsm.fork_choice = choice
        cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=18100.0, yaw=0.0, obstacle={'m7_exit': choice}))
        assert cmd.lane['mode'] == mode and cmd.lane['island_ref'] is True
        assert cmd.lane['lane_width_mm'] == pytest.approx(520.0)
        d = 0.0
        while fsm.state == 'M2_FORK' and d < 6000.0:
            fsm.step(Inputs(t=d / 200.0, dist_mm=d, yaw=0.0, obstacle={'m7_exit': choice}))
            d += 10.0
        assert fsm.state == 'M2_EXIT'
        cmd = fsm.step(Inputs(t=99.0, dist_mm=d, yaw=0.0, obstacle={'m7_exit': choice}))
        assert cmd.lane.get('mode') == 'normal' and 'island_ref' not in cmd.lane


def test_m2_end_uses_distance_when_route_lags():
    """11/19: 1119_09 도면 위치가 3 m 뒤처져 M2를 50초 끌었음 -> M2 시작부터 거리로 끝 (섬 1745 + 200)"""
    fsm = _m2_fsm(route_points={'m7_island_tip': 17973.0, 'm2_island_end': 19718.0, 'm2_merge_end': 20272.0})
    fsm.fork_choice = 'B'
    lag = 3000.0
    d = 0.0
    while fsm.state == 'M2_FORK' and d < 6000.0:
        fsm.step(Inputs(t=d / 200.0, dist_mm=d, route_s=17973.0 + d - lag, yaw=0.0, obstacle={'m7_exit': 'B'}))
        d += 10.0
    assert fsm.state == 'M2_EXIT'
    assert 1900.0 <= d <= 2000.0
    assert any('distance (route s off)' in m for m in fsm.log)


def test_m2_end_keeps_route_when_close():
    fsm = _m2_fsm(route_points={'m7_island_tip': 17973.0, 'm2_island_end': 19718.0, 'm2_merge_end': 20272.0})
    fsm.fork_choice = 'B'
    d = 0.0
    while fsm.state == 'M2_FORK' and d < 6000.0:
        fsm.step(Inputs(t=d / 200.0, dist_mm=d, route_s=17973.0 + d + 300.0, yaw=0.0, obstacle={'m7_exit': 'B'}))
        d += 10.0
    assert 1600.0 <= d <= 1700.0                      # 도면이 300 mm 앞서 있음 -> 도면대로 (700 안)
    assert not any('distance (route s off)' in m for m in fsm.log)


def test_m7_practice_start_sets_axis_for_m2_hold():
    """11/20 yubin: M7부터 연습 시작이면 M7 첫 yaw = 직선 방향 -> M2 섬 기준이 놓쳤을 때 그 방향 (재생 끝 방향 아님)"""
    import math as _m
    fsm = MissionFSM({'start_state': 'M7_OBSTACLE'})
    fsm.step(Inputs(t=0.1, dist_mm=0.0, yaw=_m.radians(10.0)))
    assert fsm.axis_yaw == pytest.approx(_m.radians(10.0))
    fsm.fork_choice = 'A'
    fsm._goto('M2_FORK', Inputs(t=5.0, dist_mm=4000.0, yaw=_m.radians(31.0)), 'obstacle zone done')
    cmd = fsm.step(Inputs(t=5.1, dist_mm=4010.0, yaw=_m.radians(31.0), obstacle={'m7_exit': 'A'}))
    assert cmd.lane['island_ref'] is True and cmd.lane['hold_axis_deg'] == pytest.approx(10.0)
