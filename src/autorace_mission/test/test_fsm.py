"""미션 FSM 오프라인 테스트 (`python3 -m pytest src/autorace_mission/test`)."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.fsm import Inputs, MissionFSM, StopApproach, Vote  # noqa: E402


class Sim:
    """일정 속도로 달리는 가짜 차. cmd.source가 stop이면 멈춘다."""

    def __init__(self, fsm, dt=0.05):
        self.fsm, self.dt, self.t, self.dist = fsm, dt, 0.0, 0.0
        self.cmd = None

    def tick(self, **kw):
        speed = 0.0
        if self.cmd is not None and self.cmd.source != 'stop':
            speed = self.cmd.lane.get('speed', 0.0)
        self.t += self.dt
        self.dist += speed * 1000.0 * self.dt
        kw.setdefault('new_detection_frame', True)
        self.cmd = self.fsm.step(Inputs(t=self.t, dist_mm=self.dist, **kw))
        return self.cmd


def test_vote_majority_and_same_frame_guard():
    v = Vote(window=4, need=3)
    assert not v.push(True, t=1)
    assert not v.push(True, t=1)        # 같은 프레임 중복 무시
    assert not v.push(True, t=2)
    assert v.push(True, t=3)


def test_start_then_bump_slowdown():
    fsm = MissionFSM({'bump_slow_mm': 300.0})
    sim = Sim(fsm)
    assert sim.tick().source == 'stop'
    sim.tick(start=True)
    assert fsm.state == 'M5_BUMP'
    for _ in range(6):
        cmd = sim.tick(detections={'slow': 0.9})
    assert fsm.sub == 'slow'
    assert cmd.lane['speed'] == pytest.approx(fsm.p['slow_speed'])
    for _ in range(100):
        sim.tick()
        if fsm.state != 'M5_BUMP':
            break
    assert fsm.state == 'M6_ZIGZAG'


def test_zigzag_sets_lane_mode():
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'zigzag_start_mm': 100.0,
                      'zigzag_len_mm': 300.0})
    sim = Sim(fsm)
    modes = set()
    for _ in range(200):
        modes.add(sim.tick().lane['mode'])
        if fsm.state != 'M6_ZIGZAG':
            break
    assert 'zigzag' in modes
    assert fsm.state == 'M3_PEDESTRIAN'


def test_pedestrian_stop_precision_and_wait():
    """정지선을 보다가 사라져도 오도메트리로 이어서 목표 위치에 선다."""
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'stop_brake_mm': 0.0})
    sim = Sim(fsm, dt=0.02)
    line_at = 800.0                      # 처음 볼 때 앞바퀴에서 800mm (누적 거리 기준 위치)
    stopped_at = None
    for i in range(2000):
        remaining = line_at - sim.dist
        lane = {'crosswalk_lines_mm': [remaining] if remaining > 120 else [],
                'stoplines_mm': [remaining] if remaining > 120 else []}
        cmd = sim.tick(detections={'pedestrian': 0.9}, lane=lane)
        if fsm.sub == 'stopped' and stopped_at is None:
            stopped_at = sim.dist
            t_stop = sim.t
        if fsm.sub == 'go':
            break
    assert stopped_at is not None
    assert line_at - stopped_at == pytest.approx(250.0, abs=10)  # 11/15 저녁: 횡단보도(M3)는 25cm 목표
    assert sim.t - t_stop >= 3.0
    assert cmd.source == 'lane'


def test_pedestrian_arms_on_crosswalk_without_sign():
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN'})
    sim = Sim(fsm)
    sim.tick(lane={'crosswalk_lines_mm': [900.0]})
    assert fsm.sub == 'approach'


def test_start_line_without_bars_is_ignored():
    """막대 없는 시작선(stoplines_mm에만 있음)에서는 서지 않는다."""
    s = StopApproach(0.0, 0.0, 400.0)
    inp = Inputs(t=0.0, dist_mm=0.0, lane={'stoplines_mm': [5.0], 'crosswalk_lines_mm': []})
    assert not s.should_stop(inp)


def _approach(s, line_at, fake_bottom=None, route_line=None, route_err=0.0, step=5.0):
    """line_at(누적 거리)에 정지선. 앞바퀴 350mm 안은 카메라가 못 본다.
    fake_bottom: 선이 사각지대로 들어간 뒤 BEV 아래 끝에 계속 보이는 가짜 선 거리 (횡단보도 블록)."""
    s.route_line_s = route_line
    dist = 0.0
    while dist < line_at + 1000.0:
        rem = line_at - dist
        lines = [rem] if 350.0 < rem < 1500.0 else []
        if fake_bottom is not None and -540.0 < rem <= 350.0:
            lines = [fake_bottom]
        inp = Inputs(t=0.0, dist_mm=dist, lane={'crosswalk_lines_mm': lines, 'stoplines_mm': lines})
        inp.route_s = None if route_line is None else dist + route_err
        if s.should_stop(inp):
            return line_at - dist           # 멈출 때 남은 거리 (+면 선 앞)
        dist += step
    return None


def test_stop_ignores_crosswalk_blocks_after_freeze():
    """정지선이 사각지대로 들어간 뒤 블록이 348mm에 계속 보여도 정지선에서 선다 (11/09 끝선 정지)."""
    old = _approach(StopApproach(0.0, 0.0, 400.0), 1200.0, fake_bottom=348.0)
    assert old is None or old < -300.0         # 예전: 블록을 다 지나서야 (또는 못 섬)
    new = _approach(StopApproach(0.0, 0.0, 400.0, freeze_mm=450.0), 1200.0, fake_bottom=348.0)
    assert new == pytest.approx(0.0, abs=10.0)


def test_stop_route_gate_skips_crosswalk_end_line():
    """정지선이 이미 사각지대일 때 보이는 끝선(540mm 뒤)을 정지선으로 잡지 않고 도면 위치로 선다."""
    s = StopApproach(0.0, 0.0, 400.0, freeze_mm=450.0, route_gate_mm=300.0)
    s.route_line_s = 300.0
    for dist in range(0, 400, 5):
        rem = 300.0 - dist
        inp = Inputs(t=0.0, dist_mm=float(dist),
                     lane={'crosswalk_lines_mm': [rem + 540.0], 'stoplines_mm': [rem + 540.0]})
        inp.route_s = float(dist)
        if s.should_stop(inp):
            break
    assert s.how == 'route'
    assert 300.0 - dist == pytest.approx(0.0, abs=10.0)


def test_fork_choice_remembered_from_obstacle_zone(old_m2):
    fsm = MissionFSM({'start_state': 'M7_OBSTACLE', 'm7_len_mm': 300.0,
                      'fork_len_mm': 300.0})
    sim = Sim(fsm)
    for _ in range(8):
        sim.tick(detections={'fork_b': 0.8})
    assert fsm.fork_choice == 'B'
    modes = []
    for _ in range(300):
        modes.append(sim.tick().lane['mode'])
        if fsm.state == 'M8_CONES':
            break
    assert 'right_only' in modes


def test_traffic_light_red_then_green():
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'stop_brake_mm': 0.0})
    sim = Sim(fsm, dt=0.02)
    line_at = 600.0
    phase = 'red'
    for i in range(3000):
        remaining = line_at - sim.dist
        lane = {'crosswalk_lines_mm': [remaining] if remaining > 120 else []}
        sim.tick(detections={phase: 0.9}, lane=lane)
        if fsm.sub == 'stopped' and phase == 'red' and sim.t > 6.0:
            phase = 'green'
        if fsm.state == 'M4_PARKING':
            break
    assert fsm.state == 'M4_PARKING'
    log = ' | '.join(fsm.log)
    assert 'passed traffic light' in log


def test_state_distance_limit_prevents_stall():
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'state_max_mm': 500.0})
    sim = Sim(fsm)
    for _ in range(400):
        sim.tick()
        if fsm.state != 'M3_PEDESTRIAN':
            break
    assert fsm.state == 'M7_OBSTACLE'


def test_cones_approach_cones_handover_then_traffic():
    """M8: 접근(차선) -> cones(obstacle) -> done -> handover(obstacle, 차선 normal) -> 차선 잡히면 M1."""
    fsm = MissionFSM({'start_state': 'M8_CONES', 'cones_source_ready': True,
                      'cones_approach_mm': 300.0})
    sim = Sim(fsm)
    cmd = sim.tick()
    assert fsm.sub == '' and cmd.source == 'lane'
    for _ in range(200):
        cmd = sim.tick()
        if fsm.sub == 'cones':
            break
    assert cmd.source == 'obstacle' and cmd.lane['mode'] == 'heading'
    for _ in range(20):
        cmd = sim.tick(obstacle={'cones_done': False})
    assert fsm.sub == 'cones'
    sim.tick(obstacle={'cones_done': True})
    assert fsm.sub == 'handover'
    cmd = sim.tick(obstacle={'cones_done': True}, lane={'status': 'NONE'})
    assert cmd.source == 'obstacle' and cmd.lane['mode'] == 'normal'   # 차선 노드 초기화
    for _ in range(fsm.p['cones_handover_frames']):
        cmd = sim.tick(lane={'status': 'BOTH', 'low_conf': False})
    assert fsm.state == 'M1_TRAFFIC' and cmd.source == 'lane'


def test_cones_enter_early_when_cone_close():
    fsm = MissionFSM({'start_state': 'M8_CONES', 'cones_source_ready': True,
                      'cones_approach_mm': 2000.0})
    sim = Sim(fsm)
    for _ in range(300):
        sim.tick(obstacle={'cone_ahead_mm': 400.0})
        if fsm.sub == 'cones':
            break
    assert fsm.sub == 'cones'
    assert sim.dist >= 2000.0 - 50                          # 11/14: 400mm 앞 콘으로는 일찍 안 들어가고 차선 인식 유지 (11/14 밤 원복: 기준 150)

    fsm2 = MissionFSM({'start_state': 'M8_CONES', 'cones_source_ready': True,
                       'cones_approach_mm': 2000.0})
    sim2 = Sim(fsm2)
    for _ in range(300):
        sim2.tick(obstacle={'cone_ahead_mm': 120.0})
        if fsm2.sub == 'cones':
            break
    assert fsm2.sub == 'cones'
    assert 1500.0 - 50 <= sim2.dist < 2000.0                # 라바콘 직전(150mm 이내)에 들어오면 전환 (11/14 밤 원복)


def test_cones_handover_times_out_without_lane():
    fsm = MissionFSM({'start_state': 'M8_CONES', 'cones_source_ready': True,
                      'cones_approach_mm': 100.0, 'cones_zone_mm': 300.0})
    sim = Sim(fsm)
    for _ in range(1000):
        sim.tick(lane={'status': 'NONE'})
        if fsm.state != 'M8_CONES':
            break
    assert fsm.state == 'M1_TRAFFIC'
    assert any('distance' in s for s in fsm.log)


def test_odom_jump_does_not_skip_states():
    """체크섬 오류 등으로 주행거리가 갑자기 수 km 튀어도 상태가 넘어가지 않는다 (11/09 FINISHED 사고)."""
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN'})
    sim = Sim(fsm)
    for _ in range(20):
        sim.tick()
    sim.dist += 7_500_000.0
    for _ in range(40):
        sim.tick()
    assert fsm.state == 'M3_PEDESTRIAN'
    assert any('odom jump' in m for m in fsm.log)


# ---------------------------------------------------------------- 11/09 트리거 정리
def _drive(fsm, n, step_mm=10.0, t0=0.0, d0=0.0, **kw):
    """step_mm씩 n틱 달린다. kw 값이 함수면 (i, dist)로 불러 그 틱 입력을 만든다."""
    d = d0
    for i in range(n):
        args = {k: (v(i, d) if callable(v) else v) for k, v in kw.items()}
        args.setdefault('new_detection_frame', True)
        fsm.step(Inputs(t=t0 + i * 0.05, dist_mm=d, **args))
        d += step_mm
    return d


def test_bump_slow_ends_after_imu_bump():
    """서행 표지 하나만 봐도 서행, IMU로 방지턱을 느끼고 bump_after_mm 더 가면 서행 끝."""
    fsm = MissionFSM({'start_state': 'M5_BUMP'})
    d = _drive(fsm, 10, detections={'slow': 0.9})
    assert fsm.sub == 'slow'
    d = _drive(fsm, 20, d0=d, tilt_rate=0.05)
    assert fsm.state == 'M5_BUMP'
    d = _drive(fsm, 1, d0=d, tilt_rate=0.6)
    d = _drive(fsm, 25, d0=d, tilt_rate=0.05)          # 250mm: 아직
    assert fsm.state == 'M5_BUMP'
    _drive(fsm, 10, d0=d, tilt_rate=0.05)
    assert fsm.state == 'M6_ZIGZAG'
    assert any('bump passed (imu)' in m for m in fsm.log)


def test_bump_slow_fallback_distance_without_imu():
    fsm = MissionFSM({'start_state': 'M5_BUMP', 'bump_slow_mm': 500.0})
    d = _drive(fsm, 10, detections={'bump': 0.9})
    _drive(fsm, 60, d0=d)
    assert fsm.state == 'M6_ZIGZAG'
    assert any('FALLBACK' in m for m in fsm.log)


def test_zigzag_starts_when_front_wheel_crosses_white_line():
    """지그재그 시작 흰 선: 카메라로 본 거리만큼 더 가야 서행 시작 (보자마자가 아니라 지나는 순간)."""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG'})
    line_at = 1000.0
    lanef = lambda i, d: {'stoplines_mm': [line_at - d]} if 400.0 < line_at - d < 1800.0 else {}
    d = 0.0
    while fsm.sub == '' and d < 3000.0:
        d = _drive(fsm, 1, d0=d, lane=lanef)
    assert fsm.sub == 'zigzag'
    assert d == pytest.approx(line_at, abs=25)
    assert fsm.line.how == 'camera'


def test_slow_kept_until_crosswalk_stop_then_base_speed():
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN'})
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0))
    assert cmd.lane['speed'] == fsm.p['slow_speed']
    assert fsm.p['slow_speed'] == 0.20


def test_cones_end_when_no_cone_visible():
    """라바콘 모드: 콘을 봤다가 앞 800mm 안에 0.5초 동안 하나도 없으면 handover."""
    fsm = MissionFSM({'start_state': 'M8_CONES', 'cones_source_ready': True})
    fsm._sub('cones', Inputs(t=0.0, dist_mm=0.0))
    d = _drive(fsm, 60, obstacle={'cone_any_ahead_mm': 300.0})     # 600mm: 콘 계속 보임
    assert fsm.sub == 'cones'
    d = _drive(fsm, 5, d0=d, t0=3.0, obstacle={'cone_any_ahead_mm': None})   # 0.2초
    assert fsm.sub == 'cones'
    _drive(fsm, 10, d0=d, t0=3.25, obstacle={'cone_any_ahead_mm': 1500.0})
    assert fsm.sub == 'handover'
    assert any('no cones ahead' in m for m in fsm.log)


def test_cones_not_ended_before_any_cone_seen():
    fsm = MissionFSM({'start_state': 'M8_CONES', 'cones_source_ready': True})
    fsm._sub('cones', Inputs(t=0.0, dist_mm=0.0))
    _drive(fsm, 80, obstacle={'cone_any_ahead_mm': None})
    assert fsm.sub == 'cones'


def test_parking_armed_by_sign_seen_at_traffic_light():
    """주차 표지는 신호등 구간에서 봐도 기억했다가 M4에서 주차 노드를 무장시킨다 (경로 지도 있을 때)."""
    pts = {'m4_pre_line': 5000.0}
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'route_points': pts})
    _drive(fsm, 10, detections={'parking': 0.9, 'red': 0.9}, route_s=100.0)
    assert fsm.parking_sign and fsm.state == 'M1_TRAFFIC'
    fsm._goto('M4_PARKING', Inputs(t=1.0, dist_mm=100.0))
    cmd = fsm.step(Inputs(t=1.05, dist_mm=110.0, route_s=1000.0))
    assert fsm.parking_armed and cmd.source == 'parking'
    assert cmd.lane['speed'] == fsm.p['slow_speed']


def test_parking_armed_fallback_near_pre_line():
    pts = {'m4_pre_line': 5000.0}
    fsm = MissionFSM({'start_state': 'M4_PARKING', 'route_points': pts})
    fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=3000.0))
    assert not fsm.parking_armed
    fsm.step(Inputs(t=0.05, dist_mm=10.0, route_s=4300.0))
    assert fsm.parking_armed


def test_command_path_keeps_route_fallbacks():
    """명령 경로(use_class_commands)에서도 표지를 놓치면 도면 위치로 서행·주차를 켠다 (11/10, #23 뒤)."""
    fsm = MissionFSM({'start_state': 'M5_BUMP', 'route_points': {'m5_slow_line': 2000.0}})
    fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=500.0, confirmed_classes={}))
    assert fsm.sub == ''
    fsm.step(Inputs(t=0.05, dist_mm=10.0, route_s=1750.0, confirmed_classes={}))
    assert fsm.sub == 'slow'

    fsm = MissionFSM({'start_state': 'M4_PARKING', 'route_points': {'m4_pre_line': 5000.0}})
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=3000.0, confirmed_classes={}))
    assert cmd.source == 'lane' and not fsm.parking_armed
    cmd = fsm.step(Inputs(t=0.05, dist_mm=10.0, route_s=4300.0, confirmed_classes={}))
    assert cmd.source == 'parking' and fsm.parking_armed


def test_command_path_parking_command_arms_at_once():
    fsm = MissionFSM({'start_state': 'M4_PARKING', 'route_points': {'m4_pre_line': 5000.0}})
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=3000.0, confirmed_classes={'parking': 1.0}))
    assert cmd.source == 'parking' and fsm.parking_sign


# --- 11/11: 라바콘 코너 지킴 (M2 끝 -> 좌하단 코너 90° -> 시작선) --------------------------------
def _m8_from_m2(turn_deg):
    import math
    fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'm8_corner_turn_deg': turn_deg})
    sim = Sim(fsm)
    fsm._goto('M8_CONES', Inputs(t=0.0, dist_mm=0.0, yaw=math.pi), 'fork B done')   # 서쪽으로 가다 코너
    entered = None
    for _ in range(600):
        d = sim.dist
        # 609 mm 직선 -> R640 90° 우회전(1005 mm) -> 북쪽 직선
        turn = min(max(d - 609.0, 0.0) / 1005.0, 1.0) * (math.pi / 2)
        sim.tick(yaw=math.pi - turn, obstacle={'cone_ahead_mm': 120.0})   # 코너에서도 앞에 콘이 보임
        if fsm.sub == 'cones':
            entered = d
            break
    return fsm, entered


def test_cones_not_entered_inside_corner():
    """예전: 코너 안(콘이 앞에 보임 + 진입 후 거리)에서 라바콘 모드 -> 콘 쪽으로 곧장(지름길)."""
    _, old = _m8_from_m2(0.0)
    assert old is not None and old < 609.0 + 1005.0                 # 코너 안에서 들어감 (재현)
    fsm, new = _m8_from_m2(75.0)
    assert new is not None and new >= 609.0 + 1005.0 * (75.0 / 90.0)   # 코너를 거의 다 돈 뒤 (11/14 밤 원복)
    assert any('M8 corner done' in m for m in fsm.log)


def test_cones_corner_guard_off_for_practice_start():
    """연습으로 바로 M8 (set_state) 이면 코너 지킴 안 함."""
    fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True})
    fsm._goto('M8_CONES', Inputs(t=0.0, dist_mm=0.0, yaw=0.0), 'manual')
    assert not fsm.m8_corner.active


def test_parking_direct_hands_over_at_m4_start():
    """11/12 m4_direct: 표지도 없고 도면상 앞 선에서 멀어도 M4 들어오자마자 주차 노드에 넘긴다."""
    fsm = MissionFSM({'start_state': 'M4_PARKING', 'route_points': {'m4_pre_line': 5000.0}, 'm4_direct': True})
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=3000.0, confirmed_classes={}))
    assert cmd.source == 'parking' and fsm.parking_armed
    assert cmd.lane['speed'] == fsm.p['slow_speed']


def test_m7_corner_preview_from_crosswalk_only():
    """11/12 밤: 횡단보도에서 넘어온 M7은 코너를 IMU로 다 돌 때까지 R640 오른쪽 곡률을 미리보기로 낸다. 연습 set_state M7은 안 냄."""
    import math as _m
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN'})
    inp = Inputs(t=0.0, dist_mm=1000.0, yaw=0.0)
    fsm._goto('M7_OBSTACLE', inp, 'left crosswalk')
    k = fsm.m7_corner_k(1000.0, 0.0)                     # 진입 직후: 330 mm 뒤 코너 (+ 차선 노드 미리보기 300 mm 만큼 밈)
    assert k[0] == 0.0 and k[5] == 0.0 and k[7] == pytest.approx(-1.5625, abs=1e-3)
    k = fsm.m7_corner_k(1600.0, _m.radians(-60.0))       # 돌기 시작한 뒤에는 다 돌 때까지 전부 코너 곡률 (60~70°에서 끊기던 것)
    assert all(v == pytest.approx(-1.5625, abs=1e-3) for v in k)
    assert fsm.m7_corner_k(2200.0, _m.radians(-86.0)) is None    # 다 돎 -> 끝
    manual = MissionFSM({'start_state': 'M3_PEDESTRIAN'})
    manual._goto('M7_OBSTACLE', inp, 'manual')
    assert manual.m7_corner_k(1000.0, 0.0) is None


def test_m7_corner_uses_plain_lane_config_until_turned():
    """11/12 밤 실차: 코너 동안은 3차선 설정(흰 선도 차선, 폭 520) 대신 보통 차선 설정. 다 돌면 3차선 설정."""
    import math as _m
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'm7_external_avoid': True})
    fsm._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, yaw=0.0), 'left crosswalk')
    cmd = fsm.step(Inputs(t=0.1, dist_mm=50.0, yaw=0.0, obstacle={}))
    assert 'lane_color' not in cmd.lane and 'lane_width_mm' not in cmd.lane
    fsm.m7_corner_k(1500.0, _m.radians(-88.0))                   # 다 돎 -> 미리보기 끝
    cmd = fsm.step(Inputs(t=0.2, dist_mm=1500.0, yaw=_m.radians(-88.0), obstacle={}))
    assert cmd.lane.get('lane_color') == 'both' and cmd.lane.get('lane_width_mm') == 520.0


def test_m8_corner_preview_after_m2():
    """11/12 밤 실차: M2 끝 뒤 좌하단 코너에서 차선을 놓치고 멋대로 감 -> M2에서 넘어오면 같은 IMU 코너 미리보기 (연습 M8은 아님)."""
    import math as _m
    fsm = MissionFSM({'start_state': 'M2_FORK'})
    fsm._goto('M8_CONES', Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork B done')
    k = fsm.m7_corner_k(5000.0, 0.0)                     # 진입 직후: 650 mm 뒤 코너 (+300 밈)
    kc = -1.5625 * fsm.p['m8_corner_k_gain']             # 11/13: 1.15 -> 1.0 (R557은 안쪽 선을 밟음)
    assert fsm.p['m8_corner_k_gain'] == 1.0
    assert k[0] == 0.0 and k[9] == 0.0 and k[10] == pytest.approx(kc, abs=1e-3)
    assert all(v == pytest.approx(kc, abs=1e-3) for v in fsm.m7_corner_k(5900.0, _m.radians(-30.0)))
    assert fsm.m7_corner_k(6900.0, _m.radians(-86.0)) is None
    practice = MissionFSM({'start_state': 'M2_FORK'})
    practice._goto('M8_CONES', Inputs(t=0.0, dist_mm=0.0, yaw=0.0), 'manual')
    assert practice.m7_corner_k(0.0, 0.0) is None


def _m8_after_fork(choice, ready, **kw):
    fsm = MissionFSM(dict({'start_state': 'M2_FORK', 'cones_source_ready': ready}, **kw))
    fsm.fork_choice = choice
    fsm._goto('M8_CONES', Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), f'fork {choice} done')
    return fsm


def test_m8_keeps_one_line_through_merge_and_corner():
    """11/12 밤 실차: M2 끝(합류)에서 normal로 바꾸자 선을 놓쳐 차선 무시 + 바로 뒤 코너에서 직진.
    M2에서 넘어오면 합류 ~ 코너 앞: 갈림길 바깥선(A=left_only, B=right_only), 양쪽 선이 보여도 유지.
    코너 600 mm 앞부터 다 돌 때까지: 안쪽 선(오른쪽 코너 = right_only). 그동안 0.15 m/s. 다 돌면 normal"""
    import math as _m
    for ready in (False, True):
        for choice, mode in (('A', 'left_only'), ('B', 'right_only')):
            fsm = _m8_after_fork(choice, ready)
            cmd = fsm.step(Inputs(t=0.1, dist_mm=5050.0, yaw=0.0, lane={'status': 'L'}))
            assert cmd.lane['mode'] == mode and cmd.lane['speed'] == pytest.approx(0.15)
            for i in range(5):                                       # 양쪽 선이 보여도 코너 전이면 그대로
                cmd = fsm.step(Inputs(t=0.2 + 0.1 * i, dist_mm=5100.0 + 20 * i, yaw=0.0, lane={'status': 'BOTH'}))
            assert cmd.lane['mode'] == mode
            cmd = fsm.step(Inputs(t=1.0, dist_mm=5400.0, yaw=0.0, lane={'status': 'BOTH'}))   # 코너 600 mm 앞
            assert cmd.lane['mode'] == 'right_only'                  # 코너 안쪽 선
            cmd = fsm.step(Inputs(t=2.0, dist_mm=6000.0, yaw=_m.radians(-45.0), lane={'status': 'R'}))
            assert cmd.lane['mode'] == 'right_only' and cmd.lane['speed'] == pytest.approx(0.15)
            fsm.step(Inputs(t=3.0, dist_mm=6700.0, yaw=_m.radians(-88.0), lane={'status': 'R'}))   # 다 돎
            cmd = fsm.step(Inputs(t=3.1, dist_mm=6720.0, yaw=_m.radians(-89.0), lane={'status': 'BOTH'}))
            assert cmd.lane['mode'] == 'normal'
            assert any('M8 lane normal (corner done' in x for x in fsm.log)


def test_m8_one_line_safety_distance_and_manual():
    fsm = _m8_after_fork('A', False, m8_corner_preview=False)    # 코너 미리보기가 없으면 거리 상한(2.5 m)으로 끝
    assert fsm.step(Inputs(t=0.1, dist_mm=7000.0, yaw=0.0, lane={'status': 'NONE'})).lane['mode'] == 'left_only'
    assert fsm.step(Inputs(t=0.2, dist_mm=7520.0, yaw=0.0, lane={'status': 'NONE'})).lane['mode'] == 'normal'
    manual = MissionFSM({'start_state': 'M2_FORK'})
    manual.fork_choice = 'A'
    manual._goto('M8_CONES', Inputs(t=0.0, dist_mm=0.0, yaw=0.0), 'manual')
    assert manual.step(Inputs(t=0.1, dist_mm=10.0, yaw=0.0, lane={'status': 'L'})).lane['mode'] == 'normal'


def test_fork_lane_cfg_keeps_away_from_island_and_searches(old_m2):
    """11/12 밤: M2 한쪽 선 모드는 폭 460(바깥선에서 230 mm, 섬 쪽 선에서 멀리) + 선 놓치면 찾기 + 직선 방향(M7 진입 방향 - 90°)"""
    import math as _m
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN'})
    fsm._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, yaw=_m.radians(-90.0)), 'crosswalk done')   # 남쪽으로 가다 코너
    fsm.fork_choice = 'B'
    fsm._goto('M2_FORK', Inputs(t=1.0, dist_mm=5000.0, yaw=_m.radians(180.0)), 'obstacle zone done')
    cmd = fsm.step(Inputs(t=1.1, dist_mm=5010.0, yaw=_m.radians(175.0), obstacle={'m7_exit': 'B'}))
    assert cmd.lane['mode'] == 'right_only' and cmd.lane['lane_width_mm'] == 460.0
    assert cmd.lane['lost_search'] is True
    assert abs(abs(cmd.lane['hold_axis_deg']) - 180.0) < 0.2          # 서쪽 (코너 진입 -90° 에서 오른쪽 90°)
    fsm._goto('M8_CONES', Inputs(t=5.0, dist_mm=5020.0, yaw=_m.radians(180.0)), 'fork B done')
    cmd = fsm.step(Inputs(t=5.1, dist_mm=5070.0, yaw=_m.radians(180.0), lane={'status': 'R'}))
    assert cmd.lane['mode'] == 'right_only' and cmd.lane['lane_width_mm'] == 460.0 and cmd.lane['lost_search'] is True
    cmd = fsm.step(Inputs(t=6.0, dist_mm=5420.0, yaw=_m.radians(180.0), lane={'status': 'R'}))   # 코너 안쪽 선 구간
    assert cmd.lane['mode'] == 'right_only' and 'lane_width_mm' not in cmd.lane and 'hold_axis_deg' not in cmd.lane


def _m1_from_m8(yaw0=0.0, route_s=None, **kw):
    import math as _m
    fsm = MissionFSM(dict({'start_state': 'M3_PEDESTRIAN', 'route_points': {'m1_corner_start': 24506.0}}, **kw))
    fsm._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, yaw=yaw0), 'crosswalk done')       # 3차선 방향 = yaw0 - 90°
    fsm._goto('M8_CONES', Inputs(t=1.0, dist_mm=5000.0, yaw=yaw0 - _m.pi / 2), 'fork B done')
    fsm._goto('M1_TRAFFIC', Inputs(t=2.0, dist_mm=9000.0, yaw=yaw0 - _m.pi, route_s=route_s), 'lane back (R, 120mm)')
    return fsm


def test_m1_corner_after_cones_slow_inner_outward_then_normal():
    """11/13 실차: 라바콘 뒤 좌상단 코너에서 0.3 m/s 보통 차선으로 직진하다 늦게 꺾어 안쪽 선을 밟음.
    M8에서 넘어오면 코너 미리보기 + 0.2 m/s + 안쪽(오른쪽) 선 모드 + 바깥쪽(왼쪽) 3 cm, 다 돌면 보통 차선."""
    import math as _m
    fsm = _m1_from_m8(route_s=24300.0, m1_force_corner=False)   # 11/15부터 기본은 강제 원호 (test_m1_forced_corner_*)
    assert fsm.m7c is not None and fsm.m7c[3] == pytest.approx(206.0)          # 도면: 코너까지 206 mm
    lane = {'status': 'BOTH'}
    cmd = fsm.step(Inputs(t=2.1, dist_mm=9010.0, yaw=-_m.pi, lane=lane))
    assert cmd.lane['speed'] == pytest.approx(0.20)
    assert cmd.lane['mode'] == 'right_only' and cmd.lane['target_offset_mm'] == pytest.approx(30.0)
    k = fsm.m7_corner_k(9010.0, -_m.pi)
    assert k[0] == 0.0 and any(v == pytest.approx(-1.5625, abs=1e-3) for v in k)
    cmd = fsm.step(Inputs(t=4.0, dist_mm=9600.0, yaw=-_m.pi - _m.radians(60), lane=lane))   # 돌고 있음
    assert cmd.lane['mode'] == 'right_only'
    cmd = fsm.step(Inputs(t=6.0, dist_mm=10200.0, yaw=-_m.pi - _m.radians(88), lane=lane))  # 다 돎
    assert cmd.lane['mode'] == 'normal' and cmd.lane['speed'] == pytest.approx(fsm.p['base_speed'])
    assert 'target_offset_mm' not in cmd.lane
    assert any('M1 corner done' in x for x in fsm.log)


def test_m1_corner_counts_turn_already_done_in_cones():
    """라바콘 끝에서 이미 30° 돈 채로 M1에 들어와도 기준은 라바콘 세로 직선: 55° 더 돌면 끝 (85° 넘게 더 돌지 않음)."""
    import math as _m
    fsm = _m1_from_m8(m1_force_corner=False)
    fsm.m7c = (fsm.m7c[0], 9000.0, 'M1_TRAFFIC', fsm.m7c[3])
    assert fsm.m7_corner_k(9100.0, -_m.pi - _m.radians(30)) is not None       # 30°: 아직
    assert fsm.m7_corner_k(9500.0, -_m.pi - _m.radians(86)) is None           # 세로 직선 기준 86°: 끝


def test_m1_corner_off_for_practice_and_without_preview():
    import math as _m
    manual = MissionFSM({'start_state': 'M8_CONES'})
    manual._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=0.0), 'manual')
    assert manual.m7c is None
    cmd = manual.step(Inputs(t=0.1, dist_mm=10.0, yaw=0.0, lane={'status': 'BOTH'}))
    assert cmd.lane['mode'] == 'normal' and cmd.lane['speed'] == pytest.approx(manual.p['base_speed'])
    off = _m1_from_m8(m1_corner_preview=False)
    assert off.m7c is None


def test_m8_corner_inner_mode_pushes_outward():
    import math as _m
    fsm = _m8_after_fork('B', True)
    fsm.step(Inputs(t=0.1, dist_mm=5050.0, yaw=0.0, lane={'status': 'L'}))
    cmd = fsm.step(Inputs(t=1.0, dist_mm=5400.0, yaw=0.0, lane={'status': 'BOTH'}))     # 코너 600 mm 앞 -> 안쪽 선
    assert cmd.lane['mode'] == 'right_only' and cmd.lane['target_offset_mm'] == pytest.approx(30.0)


def test_pedestrian_sign_seen_in_m6_arms_m3_stop():
    """11/13 실차: 보행자 표지가 M6 끝 무렵 17번 보였지만 M6은 표지를 안 받아 버려졌고, M3는 도면 위치로만 섰다.
    M6에서 확정된 보행자 표지를 기억해 M3 시작에 바로 정지 준비 (도면 위치가 없어도)."""
    fsm = MissionFSM({'start_state': 'M5_BUMP'})
    fsm._goto('M6_ZIGZAG', Inputs(t=0.0, dist_mm=0.0), 'bump done')
    fsm.step(Inputs(t=0.1, dist_mm=100.0, confirmed_classes={'pedestrian'}))
    assert fsm.ped_seen
    fsm._goto('M3_PEDESTRIAN', Inputs(t=1.0, dist_mm=2000.0), 'zigzag done')
    fsm.step(Inputs(t=1.1, dist_mm=2010.0, confirmed_classes=set()))          # 이제 표지는 안 보임
    assert fsm.sub == 'approach'
    assert any('pedestrian sign seen in M6' in x for x in fsm.log)


def test_m3_without_sign_or_route_keeps_going():
    fsm = MissionFSM({'start_state': 'M5_BUMP'})
    fsm._goto('M6_ZIGZAG', Inputs(t=0.0, dist_mm=0.0), 'bump done')
    fsm._goto('M3_PEDESTRIAN', Inputs(t=1.0, dist_mm=2000.0), 'zigzag done')
    fsm.step(Inputs(t=1.1, dist_mm=2010.0, confirmed_classes=set()))
    assert fsm.sub == ''


# ---------------------------------------------------------------- 11/14 M2_EXIT · 주차 결과 · 신호등 초록 통과
def test_m2_exit_lane_mode_only_with_corner_preview():
    """11/14: M2 끝(합류) -> M2_EXIT. 한쪽 선 모드 없이 normal(양쪽 노란 선) + 코너 곡률 미리보기, 코너 동안 0.15 m/s.
    라바콘 시작선(도면 위치)에 오면 M8_CONES로 넘기고 M8은 바로 cones (접근 단계 없음)."""
    import math as _m
    for choice in ('A', 'B'):
        fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9,
                          'route_points': {'m8_start_line': 7000.0}})
        fsm.fork_choice = choice
        fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), f'fork {choice} done')
        assert fsm.state == 'M2_EXIT'
        cmd = fsm.step(Inputs(t=0.1, dist_mm=5050.0, yaw=0.0, route_s=5050.0, lane={'status': 'L'}))
        assert cmd.source == 'lane' and cmd.lane['mode'] == 'normal' and cmd.lane['speed'] == pytest.approx(0.15)
        assert 'lane_width_mm' not in cmd.lane and 'lane_color' not in cmd.lane
        k = fsm.m7_corner_k(5050.0, 0.0)
        assert k is not None and min(k) < -1.0                     # 코너 미리보기 (오른쪽)
        cmd = fsm.step(Inputs(t=2.0, dist_mm=6000.0, yaw=_m.radians(-45.0), route_s=6000.0, lane={'status': 'R'}))
        assert cmd.lane['mode'] == 'normal'
        fsm.m7_corner_k(6700.0, _m.radians(-88.0))                # 다 돎 -> 미리보기 끝
        cmd = fsm.step(Inputs(t=3.0, dist_mm=6700.0, yaw=_m.radians(-88.0), route_s=6700.0, lane={'status': 'BOTH'}))
        assert fsm.state == 'M2_EXIT' and cmd.lane['speed'] == pytest.approx(fsm.p['slow_speed'])
        fsm.step(Inputs(t=4.0, dist_mm=6900.0, yaw=_m.radians(-90.0), route_s=6900.0, lane={'status': 'BOTH'}))
        assert fsm.state == 'M2_EXIT'                              # 시작선 100 mm 전
        fsm.step(Inputs(t=4.5, dist_mm=7010.0, yaw=_m.radians(-90.0), route_s=7010.0, lane={'status': 'BOTH'}))
        assert fsm.state == 'M8_CONES'
        cmd = fsm.step(Inputs(t=4.6, dist_mm=7020.0, yaw=_m.radians(-90.0), route_s=7020.0, lane={'status': 'BOTH'}))
        assert fsm.sub == 'cones' and cmd.source == 'obstacle'
        assert fsm.m7_corner_k(7020.0, _m.radians(-90.0)) is None


def test_m2_exit_corner_reference_is_lane_axis_not_taper_heading():
    """11/18 실차: M2_EXIT는 합류 테이퍼 비스듬한 중에 시작. 기준 방향을 그때 방향으로 잡으면 곧게 펴는 것만으로 '코너 시작'이 되어
    코너 앞에서 코너 곡률 전부 (A: 일찍 꺾어 안쪽, B: 왼쪽 비스듬을 오른쪽 코너가 덮음). 직선 방향 기준 + 코너 시작 전은 거리로만."""
    import math as _m
    for choice, taper_deg in (('A', -25.0), ('B', 25.0)):
        fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9})
        fsm.fork_choice = choice
        fsm.axis_yaw = 0.0                                         # 3차선 직선 방향
        fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=_m.radians(taper_deg)), f'fork {choice} done')
        assert fsm.state == 'M2_EXIT'
        start = fsm.m7c[3]
        k = fsm.m7_corner_k(5100.0, 0.0)                         # 테이퍼 끝나 곧게 폄 (코너는 아직 멂)
        assert k is not None and k[0] == 0.0                       # 예전: 25° '돎' -> 처음부터 코너 곡률
        cmd = fsm.step(Inputs(t=0.1, dist_mm=5100.0, yaw=0.0, lane={'status': 'BOTH'}))
        assert cmd.lane.get('route_corner_zero_lane') is True      # 코너 앞은 차선 그대로
        k = fsm.m7_corner_k(5000.0 + start, _m.radians(-10.0))   # 코너 시작에서 오른쪽으로 10° -> 코너 곡률 전부
        assert min(k) < -1.0 and k[0] < -1.0
        fsm.m7_corner_k(5000.0 + start + 900, _m.radians(-86.0))  # 직선 기준 86° -> 끝
        assert fsm.m7c is None
    old = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9, 'm2x_corner_axis': False})
    old.fork_choice, old.axis_yaw = 'A', 0.0
    old._next(Inputs(t=0.0, dist_mm=5000.0, yaw=_m.radians(-25.0)), 'fork A done')
    assert old.m7_corner_k(5100.0, 0.0)[0] < -1.0                  # 끄면 예전 (기준 = 들어올 때 방향)


def test_corner_preview_lookahead_in_m7_corner_and_m2_exit():
    """11/19: corner_preview_lookahead_mm는 M7 직전 코너(차선 주행)·M2_EXIT 동안만 차선 lookahead로. 0이면 안 보냄"""
    fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9,
                      'corner_preview_lookahead_mm': 500.0})
    fsm.fork_choice, fsm.axis_yaw = 'A', 0.0
    fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork A done')
    cmd = fsm.step(Inputs(t=0.1, dist_mm=5100.0, yaw=0.0, lane={'status': 'BOTH'}))
    assert fsm.state == 'M2_EXIT' and cmd.lane['lookahead_mm'] == 500.0
    off = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9})
    off.fork_choice, off.axis_yaw = 'A', 0.0
    off._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork A done')
    assert 'lookahead_mm' not in off.step(Inputs(t=0.1, dist_mm=5100.0, yaw=0.0, lane={'status': 'BOTH'})).lane
    m7 = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'corner_preview_lookahead_mm': 500.0, 'odom_jump_mm': 1e9,
                     'm7_external_avoid': True})          # 실차 launch (m7_avoid:=true)
    m7._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=1000.0, yaw=0.0), 'crosswalk done')
    assert m7.m7c is not None
    cmd = m7.step(Inputs(t=0.1, dist_mm=1050.0, yaw=0.0, lane={'status': 'BOTH'}))
    assert cmd.lane['lookahead_mm'] == 500.0


def _m2x(**kw):
    fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9, **kw})
    fsm.fork_choice, fsm.axis_yaw = 'B', 0.0
    fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork B done')
    assert fsm.state == 'M2_EXIT'
    return fsm


def test_m8_cones_on_when_camera_sees_tilted_start_line():
    """11/19: 코너를 30° 넘게 돈 뒤 카메라가 시작선을 2번 보면 바로 라바콘 모드 (지나가기 전). 그동안 가로선 기울기 60°까지 찾음"""
    import math as _m
    fsm = _m2x()
    cmd = fsm.step(Inputs(t=0.1, dist_mm=5100.0, yaw=_m.radians(-10.0), lane={'status': 'BOTH', 'stoplines_mm': [700.0]}))
    assert fsm.state == 'M2_EXIT'                                  # 10°: 아직 (가짜 선 거름)
    cmd = fsm.step(Inputs(t=0.2, dist_mm=5600.0, yaw=_m.radians(-40.0), lane={'status': 'BOTH'}))
    assert fsm.state == 'M2_EXIT' and cmd.lane.get('stopline_max_angle_deg') == 60.0
    fsm.step(Inputs(t=0.3, dist_mm=5650.0, yaw=_m.radians(-45.0), lane={'status': 'BOTH', 'stoplines_mm': [650.0]}))
    assert fsm.state == 'M2_EXIT'                                  # 한 번
    fsm.step(Inputs(t=0.4, dist_mm=5700.0, yaw=_m.radians(-50.0), lane={'status': 'BOTH', 'stoplines_mm': [600.0]}))
    assert fsm.state == 'M8_CONES'                                 # 두 번 -> 시작선 600 mm 앞에서 켬
    cmd = fsm.step(Inputs(t=0.5, dist_mm=5720.0, yaw=_m.radians(-52.0), lane={'status': 'BOTH'}))
    assert fsm.sub == 'cones' and cmd.source == 'obstacle'


def test_m8_cones_on_after_full_turn_or_cone_even_without_line():
    """11/19: 선을 놓쳐도 80° 넘게 돌고 200 mm 더 가면, 또는 70° 넘게 돌았고 차로 안 콘이 가까우면 바로 라바콘 모드"""
    import math as _m
    fsm = _m2x()
    fsm.step(Inputs(t=0.1, dist_mm=6000.0, yaw=_m.radians(-82.0), lane={'status': 'BOTH'}))
    assert fsm.state == 'M2_EXIT'
    cmd = fsm.step(Inputs(t=0.2, dist_mm=6100.0, yaw=_m.radians(-88.0), lane={'status': 'BOTH'}))
    assert fsm.state == 'M2_EXIT' and cmd.lane['speed'] <= 0.12 + 1e-9    # 코너 끝 천천히 (선이 잘 보이게)
    fsm.step(Inputs(t=0.3, dist_mm=6210.0, yaw=_m.radians(-90.0), lane={'status': 'BOTH'}))
    assert fsm.state == 'M8_CONES'
    c = _m2x()
    c.step(Inputs(t=0.1, dist_mm=6000.0, yaw=_m.radians(-72.0), lane={'status': 'BOTH'}, obstacle={'cone_ahead_mm': 140.0}))
    assert c.state == 'M8_CONES'
    off = _m2x(m8_line_trigger=False)
    off.step(Inputs(t=0.1, dist_mm=6000.0, yaw=_m.radians(-82.0), lane={'status': 'BOTH'}))
    off.step(Inputs(t=0.3, dist_mm=6300.0, yaw=_m.radians(-90.0), lane={'status': 'BOTH'}))
    assert off.state == 'M2_EXIT'                                  # 끄면 예전 규칙


def test_m2_exit_off_keeps_old_m8_approach():
    fsm = MissionFSM({'start_state': 'M2_FORK', 'm2_exit': False})
    fsm.fork_choice = 'A'
    fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork A done')
    fsm.step(Inputs(t=0.1, dist_mm=5010.0, yaw=0.0, lane={'status': 'L'}))
    assert fsm.state == 'M8_CONES' and fsm.sub == ''
    cmd = fsm.step(Inputs(t=0.2, dist_mm=5050.0, yaw=0.0, lane={'status': 'L'}))
    assert cmd.lane['mode'] == 'left_only'


def test_parking_failure_is_not_reported_as_parked():
    """11/14 (1114_01): 주차 노드 FAIL_NO_CAR(done=true)인데 'parked' + 띠링이었다."""
    for res, ok in (('PARKED', True), ('PARKED_NOT_ALIGNED', True), ('FAIL_NO_CAR', False), ('', False)):
        fsm = MissionFSM({'start_state': 'M4_PARKING', 'm4_direct': True})
        fsm.step(Inputs(t=0.0, dist_mm=0.0, parking={'done': False}))
        fsm.step(Inputs(t=1.0, dist_mm=100.0, parking={'done': True, 'result': res}))
        assert fsm.state == 'FINISHED'
        assert any(('parked (' in x) == ok and ('parking failed' in x) == (not ok) for x in fsm.log if 'FINISHED' in x)
        assert fsm.parking_result == (res or 'UNKNOWN')


def test_green_on_arrival_passes_without_stopping():
    """11/14: stop_on_green_arrival false -> 초록이 확정돼 있으면 정지선에서 안 섬. 아무것도 못 봤으면 서서 초록을 기다림."""
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'stop_on_green_arrival': False,
                      'route_points': {'m1_stop_line': 3000.0}})
    srcs, subs = [], []
    for i in range(60):
        d = 2000.0 + 40.0 * i
        cmd = fsm.step(Inputs(t=0.1 * i, dist_mm=d, route_s=d, confirmed_classes={'green': 1}))
        srcs.append(cmd.source)
        subs.append((d, fsm.sub))
    assert 'stop' not in srcs and fsm.state == 'M4_PARKING'
    # 11/15: 초록이 일찍 보여도 'go'는 정지 지점에 도착했을 때만 (그 전엔 approach)
    first_go = next(d for d, sb in subs if sb == 'go')
    assert first_go >= 3000.0 - fsm.p['stop_target_mm'] - fsm.p['stop_brake_mm'] - 50.0
    wait = MissionFSM({'start_state': 'M1_TRAFFIC', 'stop_on_green_arrival': False,
                       'route_points': {'m1_stop_line': 3000.0}})
    for i in range(60):
        d = min(2000.0 + 40.0 * i, 3000.0)
        cmd = wait.step(Inputs(t=0.1 * i, dist_mm=d, route_s=d, confirmed_classes={}))
    assert wait.sub == 'stopped' and cmd.source == 'stop'


def test_green_seen_early_then_red_still_stops():
    """11/15: 접근 중 초록을 봤어도 정지선 도착 때 초록이 아니면(빨강으로 바뀜) 선다."""
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'stop_on_green_arrival': False,
                      'route_points': {'m1_stop_line': 3000.0}})
    for i in range(80):
        d = min(1500.0 + 40.0 * i, 3000.0)
        cls = {'green': 1} if d < 2400.0 else {}
        cmd = fsm.step(Inputs(t=0.1 * i, dist_mm=d, route_s=d, confirmed_classes=cls))
        assert fsm.sub != 'go'
    assert fsm.sub == 'stopped' and cmd.source == 'stop'


def test_m3_stop_uses_camera_line_tracked_in_zigzag():
    """11/15: M6 지그재그 중 본 횡단보도 정지선을 M3 정지에 그대로 쓴다 (도면 위치가 틀려도 카메라 기준)."""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'm3_stop_target_mm': 30.0, 'stop_brake_mm': 30.0,
                      'route_points': {'m3_stop_line': 11365.0,
                                                                  'm6_zigzag_end': 11100.0,
                                                                  'm3_crosswalk_end': 11905.0}})
    fsm.sub = 'zigzag'
    true_line = 11365.0 + 200.0          # 도면 위치가 200 mm 틀림 (실제 선이 더 멀다)
    stop_at = None
    for i in range(400):
        d = 10000.0 + 20.0 * i            # 실제 앞바퀴 위치 = d, 도면 s도 d (도면 선만 400 mm 틀림)
        rem = true_line - d
        lane = {'crosswalk_lines_mm': [rem]} if 450.0 < rem < 1500.0 else {}
        cmd = fsm.step(Inputs(t=0.1 * i, dist_mm=d, route_s=d, lane=lane))
        if fsm.state == 'M3_PEDESTRIAN' and fsm.sub == 'stopped' and stop_at is None:
            stop_at = d
        if stop_at is not None:
            break
    assert stop_at is not None
    assert fsm.stop.how == 'camera'
    assert abs(true_line - stop_at) < 120.0      # 도면(200 mm 앞)이 아니라 카메라 선 앞에서 섬


def test_stop_relocks_to_route_consistent_line_after_false_line():
    """11/15: 처음 잡은 가짜 횡단보도 선(도면과 600 mm 다름)을 따라가다가 도면과 맞는 진짜 선이 보이면 갈아탄다."""
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'route_points': {'m1_stop_line': 3000.0},
                      'stop_target_mm': 30.0, 'stop_brake_mm': 30.0})
    stop_at = None
    for i in range(200):
        d = 1500.0 + 10.0 * i
        lane = {}
        if d < 1560.0:
            lane = {'crosswalk_lines_mm': [2400.0 - d]}       # 가짜 선 (s 2400)
        elif 2100.0 < d < 2550.0:
            lane = {'crosswalk_lines_mm': [3000.0 - d]}       # 진짜 정지선
        fsm.step(Inputs(t=0.1 * i, dist_mm=d, route_s=d, lane=lane, confirmed_classes={}))
        if fsm.sub == 'stopped':
            stop_at = d
            break
    assert stop_at is not None and abs(3000.0 - stop_at) < 120.0


def test_stop_now_if_stop_line_already_passed():
    """11/15 저녁 실차: 횡단보도 정지선을 늦게 잡거나 놓치면 끝선까지 가서 섬 -> 정지선을 지났다고 보이면 그 자리에서 섬"""
    from autorace_mission.fsm import StopApproach
    st = StopApproach(250.0, 30.0, 400.0)
    st.seen_d, st.seen_at = 600.0, 0.0                   # 600 mm 앞에서 봄
    assert not st.should_stop(Inputs(t=0.0, dist_mm=100.0, lane={}))       # 아직 500 mm
    assert st.should_stop(Inputs(t=0.0, dist_mm=400.0, lane={}))           # 200 mm: 목표 안
    st2 = StopApproach(250.0, 30.0, 400.0)
    st2.seen_d, st2.seen_at = 300.0, 0.0
    assert st2.should_stop(Inputs(t=0.0, dist_mm=900.0, lane={}))          # 놓친 채 정지선을 600 mm 지남 -> 바로 정지


def test_m7_corner_start_from_route_position():
    """11/15 저녁: 횡단보도에서 늦게 출발해도 M7 직전 코너 보완은 도면상 끝선 기준 (예전: M7 시작 + 330 mm 고정)"""
    import math as _m
    pts = {'m3_crosswalk_end': 10000.0}
    fsm = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'route_points': pts})
    fsm._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, yaw=_m.radians(-90.0), route_s=10400.0), 'left crosswalk')
    assert fsm.m7c[3] == 0.0                             # 끝선 400 mm 지나서 시작 -> 코너 보완 바로
    fsm2 = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'route_points': pts})
    fsm2._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, yaw=_m.radians(-90.0), route_s=9900.0), 'left crosswalk')
    assert fsm2.m7c[3] == 430.0                          # 끝선 100 mm 전 -> 330 + 100
    fsm3 = MissionFSM({'start_state': 'M3_PEDESTRIAN', 'route_points': pts})
    fsm3._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, yaw=_m.radians(-90.0)), 'left crosswalk')
    assert fsm3.m7c[3] == 330.0                          # 도면 위치 없으면 예전대로


def _zigzag_to_crosswalk(pretrack, zigzag_len):
    """지그재그 시작선 지난 뒤부터: 정지선은 지그재그 시작 + 1943 mm. 카메라는 앞바퀴 350~2000 mm 안의 선만 봄 (사각 350 mm)"""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'm3_pretrack': pretrack, 'zigzag_len_mm': zigzag_len,
                      'zigzag_start_mm': 0.0})
    sim = Sim(fsm)
    fsm._sub('zigzag', Inputs(t=0.0, dist_mm=0.0), 'start line (test)')   # 지그재그 시작선을 막 지남
    line = 1943.0
    stopped_at = None
    for _ in range(1500):
        rem = line - sim.dist
        lane = {'crosswalk_lines_mm': [rem] if 350.0 <= rem <= 2000.0 else []}
        cmd = sim.tick(lane=lane)
        if fsm.state == 'M3_PEDESTRIAN' and fsm.sub == 'stopped' and stopped_at is None:
            stopped_at = sim.dist
            break
    return fsm, line, stopped_at


def test_crosswalk_stop_line_pretracked_in_zigzag():
    """11/15 밤 실차: M7 전 횡단보도를 지나서 섬. M3가 시작할 때(정지선 266 mm 앞) 정지선은 이미 카메라 사각(350 mm)이라
    못 잡았다 -> 지그재그 중에 미리 재 두고 이어받음. 지그재그 끝 판단이 늦어도(거리 2100 mm) 정지선 앞에서 섬"""
    for zl in (1677.0, 2100.0):
        fsm, line, at = _zigzag_to_crosswalk(True, zl)
        assert at is not None, zl
        assert 150.0 <= line - at <= 300.0, (zl, line - at)          # 25 cm 목표 근처, 선 앞
    fsm, line, at = _zigzag_to_crosswalk(False, 2100.0)              # 예전: 지그재그가 늦게 끝나면 정지선을 지나서 섬
    assert at is None or line - at < 0.0


def _zigzag_blind(cap, cam_lines=False, line=1983.0, zz_len=1800.0):
    """11/14 밤 실차 '횡단보도를 다 지난 다음 섬': 카메라가 정지선을 못 주고(cam_lines False) 도면 위치도 없을 때.
    지그재그 시작선을 카메라로 지난 순간부터 달린 거리로 정지선 앞에서 서야 한다."""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'm3_zz_cap': cap, 'zigzag_len_mm': zz_len, 'zigzag_start_mm': 0.0})
    sim = Sim(fsm)
    fsm.line = StopApproach(0.0, 0.0, 400.0)
    fsm.line.how = 'camera'
    fsm._sub('zigzag', Inputs(t=0.0, dist_mm=0.0), 'start line (test)')
    fsm.zz_at, fsm.zz_how = (0.0, 'camera') if cap else (None, None)
    fsm.ped_seen = True                                   # 표지는 봄 (그래도 정지선 거리를 모르면 못 섬)
    for _ in range(2000):
        rem = line - sim.dist
        lane = {'crosswalk_lines_mm': [rem] if cam_lines and 350.0 <= rem <= 1200.0 else []}
        sim.tick(lane=lane)
        if fsm.state == 'M3_PEDESTRIAN' and fsm.sub == 'stopped':
            return line - sim.dist, fsm
        if sim.dist > line + 1500.0:
            break
    return None, fsm


def test_crosswalk_zigzag_distance_cap_stops_before_line():
    left, fsm = _zigzag_blind(cap=False)
    assert left is None                                   # 예전: 정지선 거리를 모르면 안 섬 (다 지나감)
    left, fsm = _zigzag_blind(cap=True)
    assert left is not None and 60.0 <= left <= 160.0, left   # 지그재그 시작선부터 거리로 정지선 약 10 cm 앞
    assert any('zigzag-line distance' in m for m in fsm.log)


def test_crosswalk_camera_stop_wins_over_cap():
    """카메라로 정지선을 잡으면 평소대로 m3_stop_target_mm(25 cm) 앞에서 서고, 안전장치는 안 씀."""
    left, fsm = _zigzag_blind(cap=True, cam_lines=True)
    assert left is not None and left >= 200.0
    assert not any('M3 stop by zigzag-line distance' in m for m in fsm.log)


# ---------------------------------------------------------------- 11/15 M2 합류 강제 S자
def _merge_sim(choice, axis_err_deg=0.0, delay_ticks=2, y0_err=0.0):
    """직선 좌표(진행 방향 +x = 도면 서쪽, 왼쪽 +y)에서 M2 -> 합류. 차선 노드 heading 모드 흉내:
    곡률 = -1.5 x 방향오차 / v (최소 회전반경 0.53 m로 제한), 명령은 delay_ticks 늦게.
    도면: 섬 왼쪽 끝 S_e, 섬은 S_e-253부터 좁아짐(반폭 260 -> 0), 바깥 선은 S_e+47부터 S_e+554까지 780 -> 250."""
    import math as _m
    S_e = 20000.0
    pts = {'m2_island_end': S_e, 'm2_merge_end': S_e + 554.0, 'm7_island_tip': S_e - 1745.0}
    fsm = MissionFSM({'start_state': 'M2_FORK', 'route_points': pts, 'm2_exit': True})
    fsm.fork_choice = choice
    side = 1.0 if choice == 'A' else -1.0
    x, y, yaw = S_e - 1500.0, side * 520.0 + y0_err, 0.0
    fsm.axis_yaw = _m.radians(axis_err_deg)        # 직선 방향을 이만큼 틀리게 앎
    dist, q, worst_edge, worst_island, log = 0.0, [], 9e9, 9e9, []
    for i in range(400):
        cmd = fsm.step(Inputs(t=i * 0.05, dist_mm=dist, route_s=x, yaw=yaw, confirmed_classes=set()))
        q.append(cmd.lane)
        lane = q.pop(0) if len(q) > delay_ticks else {'mode': 'left_only'}
        v = 180.0                                   # mm/s
        if lane.get('mode') == 'heading':
            k = -1.5 * _m.atan2(_m.sin(yaw - lane['yaw_ref']), _m.cos(yaw - lane['yaw_ref'])) / 0.18 / 1000.0
        else:
            k = (side * 520.0 - y) * 2.0 / 350.0 ** 2 if x < S_e else -y * 2.0 / 350.0 ** 2   # 차선 주행 흉내
        k = max(-1 / 530.0, min(1 / 530.0, k))
        ds = v * 0.05
        yaw += k * ds
        x += ds * _m.cos(yaw)
        y += ds * _m.sin(yaw)
        dist += ds
        # 차체 네 모서리 (뒤 160, 앞 200, 반폭 120 mm; 기준점 뒤차축)
        c, s_ = _m.cos(yaw), _m.sin(yaw)
        for a, b in ((200, 120), (200, -120), (-160, 120), (-160, -120)):
            cx, cy = x + a * c - b * s_, y + a * s_ + b * c
            u = cx - S_e
            edge = 780.0 if u < 47.0 else (250.0 if u > 554.0 else 780.0 - (u - 47.0) / 507.0 * 530.0)
            worst_edge = min(worst_edge, edge - abs(cy))
            if -253.0 <= u <= 0.0:
                worst_island = min(worst_island, side * cy - 260.0 * (-u) / 253.0)
        if fsm.state == 'M2_EXIT' and getattr(fsm, 'm2m', None) and fsm.m2m['done']:
            break
    return fsm, x - S_e, y, _m.degrees(yaw), worst_edge, worst_island


def test_m2_merge_forced_s_curve_a_and_b(old_m2):
    """A(왼쪽 차로)면 오른쪽으로, B면 왼쪽으로 강제 S자. 합류 차로 안(바깥 선·섬 안 밟음), 코너(합류 끝 + 609 mm) 전에 끝."""
    for ch in ('A', 'B'):
        for ax_err, y_err in ((0.0, 0.0), (3.0, 0.0), (-3.0, 0.0), (0.0, 30.0), (0.0, -30.0)):
            fsm, u_end, y_end, yaw_end, edge, isl = _merge_sim(ch, ax_err, y0_err=y_err)
            assert any('M2 merge start' in m for m in fsm.log), ch
            assert fsm.m2m['done'], (ch, ax_err, y_err)
            assert u_end < 554.0 + 609.0, (ch, u_end)                   # 좌하단 코너 전에 끝
            assert abs(y_end) < 110.0 + abs(y_err), (ch, ax_err, y_err, y_end)
            assert abs(yaw_end - ax_err) < 10.0, (ch, yaw_end)
            assert edge > 0.0, (ch, ax_err, y_err, edge)                 # 바깥 선 밖으로 안 나감 (차체)
            assert isl > 0.0, (ch, isl)                                  # 섬 안 밟음
        fsm, *_ = _merge_sim(ch)
        st = [m for m in fsm.log if 'M2 merge start' in m][0]
        assert ('오른쪽' in st) == (ch == 'A')


def test_m2_merge_off_keeps_lane_mode(old_m2):
    fsm = MissionFSM({'start_state': 'M2_FORK', 'm2_merge': False,
                      'route_points': {'m2_island_end': 20000.0, 'm2_merge_end': 20554.0}})
    fsm.fork_choice = 'A'
    fsm.axis_yaw = 0.0
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=19900.0, yaw=0.0, confirmed_classes=set()))
    assert cmd.lane['mode'] == 'left_only'


def test_m2_merge_starts_from_camera_offset(old_m2):
    """시작할 때 차선 노드 offset(차로 중심이 차 왼쪽 +)으로 옆 위치를 잡음."""
    fsm = MissionFSM({'start_state': 'M2_FORK', 'route_points': {'m2_island_end': 20000.0, 'm2_merge_end': 20554.0}})
    fsm.fork_choice, fsm.axis_yaw = 'A', 0.0
    fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=19600.0, yaw=0.0, confirmed_classes=set(),
                    lane={'status': 'L', 'offset_mm': 40.0}))
    assert fsm.m2m is not None and fsm.m2m['y'] == 480.0


def test_floor_letter_decides_fork_over_sign_and_m7_exit(old_m2):
    """11/15 실차: 표지를 못 보거나 잘못 보고 B로 들어감 -> 바닥 글자로 실제 갈림길을 알고 그쪽 모드·합류 방향."""
    fsm = MissionFSM({'start_state': 'M2_FORK', 'route_points': {'m2_island_end': 20000.0, 'm2_merge_end': 20554.0},
                      'floor_need_exit_agree': False})                  # 11/15 기본은 회피 차로와 같을 때만 (아래 시험)
    fsm.fork_choice, fsm.axis_yaw = 'A', 0.0                           # 표지는 A로 읽음
    cmd = None
    for i in range(6):
        cmd = fsm.step(Inputs(t=0.05 * i, dist_mm=10.0 * i, route_s=18500.0 + 10.0 * i, yaw=0.0,
                              confirmed_classes=set(), obstacle={'m7_exit': 'A'},
                              lane={'floor_letter': 'B' if i % 2 == 0 else None}))
    assert fsm.floor_fork == 'B' and fsm.fork_choice == 'B'
    assert cmd.lane['mode'] == 'right_only'
    assert any('floor letter B' in m for m in fsm.log)
    cmd = fsm.step(Inputs(t=1.0, dist_mm=100.0, route_s=19600.0, yaw=0.0, confirmed_classes=set(), lane={}))
    assert fsm.m2m['side'] == -1.0                                      # B -> 왼쪽으로 합류


def test_floor_letter_single_frame_not_enough():
    fsm = MissionFSM({'start_state': 'M2_FORK'})
    fsm.fork_choice = 'A'
    for i, ch in enumerate(['B', None, None, None, 'A', None]):
        fsm.step(Inputs(t=0.05 * i, dist_mm=10.0 * i, confirmed_classes=set(), lane={'floor_letter': ch}))
    assert getattr(fsm, 'floor_fork', None) is None and fsm.fork_choice == 'A'


def _m1_corner_sim(y0=0.0, ax_err_deg=0.0, delay_ticks=2, start_mm=280.0):
    """라바콘 끝(직선, 북쪽 = +x) -> 좌상단 R640 오른쪽 90° -> 동쪽 직선. 차선 노드 heading 모드 흉내 (최소 회전반경 0.53 m)."""
    import math as _m
    fsm = MissionFSM({'start_state': 'M8_CONES', 'm1_corner_start_mm': start_mm})
    fsm.axis_yaw = _m.radians(ax_err_deg) - fsm.p['m7_corner_dir'] * _m.pi / 2.0    # 3차선 방향 (라바콘 직선 = 이 + 오른쪽 90°)
    x, y, yaw, dist = 0.0, y0, 0.0, 0.0
    fsm._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=yaw, lane={}), 'cones done')
    q, worst = [], 0.0
    R, xs = 640.0, start_mm
    for i in range(400):
        cmd = fsm.step(Inputs(t=0.05 * i, dist_mm=dist, yaw=yaw, confirmed_classes=set(), lane={}))
        q.append(cmd.lane)
        lane = q.pop(0) if len(q) > delay_ticks else {'mode': 'normal'}
        if lane.get('mode') == 'heading':
            k = -1.5 * _m.atan2(_m.sin(yaw - lane['yaw_ref']), _m.cos(yaw - lane['yaw_ref'])) / 0.18 / 1000.0
        else:
            k = 0.0
        k = max(-1 / 530.0, min(1 / 530.0, k))
        ds = 9.0
        yaw += k * ds
        x += ds * _m.cos(yaw)
        y += ds * _m.sin(yaw)
        dist += ds
        # 진짜 경로(중심선)에서 옆 거리
        if x <= xs and y > -R * 0.5:
            off = y
        elif y >= -R:
            off = _m.hypot(x - xs, y + R) - R if x >= xs else y
        else:
            off = (xs + R) - x
        worst = max(worst, abs(off))
        if getattr(fsm, 'm1fc', None) and fsm.m1fc['done']:
            break
    return fsm, _m.degrees(yaw), worst


def test_m1_forced_corner_turns_90_on_path():
    """11/15 실차: 라바콘 뒤 커브에서 옆 출발 차로 선을 따라가 꼬임 -> 도면 원호로 강제 90°, 차로(반폭 250 - 차 반폭 120) 안."""
    for y0, ax in ((0.0, 0.0), (40.0, 0.0), (-40.0, 0.0), (0.0, 3.0), (0.0, -3.0)):
        fsm, yaw_end, worst = _m1_corner_sim(y0, ax)
        assert fsm.m1fc['done'], (y0, ax)
        assert -95.0 <= yaw_end <= -80.0, (y0, ax, yaw_end)            # 오른쪽으로 약 90°
        assert worst <= 130.0 + abs(y0) + 25.0, (y0, ax, worst)
        assert any('M1 forced corner done' in m for m in fsm.log)


def test_m1_forced_corner_off_uses_old_corner_help():
    fsm = MissionFSM({'start_state': 'M8_CONES', 'm1_force_corner': False})
    fsm.axis_yaw = 3.14159
    fsm._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=0.0, lane={}), 'cones done')
    cmd = fsm.step(Inputs(t=0.05, dist_mm=10.0, yaw=0.0, confirmed_classes=set(), lane={}))
    assert cmd.lane['mode'] != 'heading'


def test_floor_letter_ignored_when_it_disagrees_with_m7_lane(old_m2):
    """11/15 실차: 1차선(A)으로 섬을 지났는데 B로 가서 합류에서 탈선. 바닥 글자(구멍 수)를 잘못 읽어도 차가 실제로 있는
    차로(회피가 끝난 차로)를 따름. 회피 차로를 모르면 바닥 글자를 씀"""
    fsm = MissionFSM({'start_state': 'M2_FORK'})
    for i in range(6):
        cmd = fsm.step(Inputs(t=0.1 * i, dist_mm=10.0 * i, yaw=0.0, lane={'floor_letter': 'B'},
                              obstacle={'m7_exit': 'A'}))
    assert fsm.fork_choice == 'A' and cmd.lane['mode'] == 'left_only'
    assert any('floor letter B ignored' in m for m in fsm.log)
    fsm2 = MissionFSM({'start_state': 'M2_FORK'})
    for i in range(6):
        cmd = fsm2.step(Inputs(t=0.1 * i, dist_mm=10.0 * i, yaw=0.0, lane={'floor_letter': 'B'}, obstacle={}))
    assert fsm2.fork_choice == 'B'


def test_lane_first_overrides_forced_corner_until_lane_lost():
    """11/16 (1116_01): 라바콘 뒤 강제 원호 중에도 선이 보이면 차선 주행, 5프레임 연속 놓치면 원호(heading)."""
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC'})
    fsm.m1fc = {'axis': 0.0, 'last': 0.0, 'x': 0.0, 'y': 0.0, 'xs': 300.0, 's': 0.0, 'done': False}
    fsm.sub = ''
    seen = {'status': 'L', 'offset_mm': 0.0}
    lane = fsm._m1_lane(Inputs(t=0.0, dist_mm=10.0, yaw=0.0, lane=seen), 0.2)
    assert lane.get('mode') != 'heading'
    modes = [fsm._m1_lane(Inputs(t=0.1 * i, dist_mm=20.0 + 10 * i, yaw=0.0, lane={'status': 'NONE'}), 0.2).get('mode')
             for i in range(6)]
    assert modes[:4] == [None] * 4 or all(m != 'heading' for m in modes[:4])
    assert modes[-1] == 'heading'
    off = MissionFSM({'start_state': 'M1_TRAFFIC', 'lane_first': False})
    off.m1fc = dict(fsm.m1fc, done=False, s=0.0, last=0.0)
    assert off._m1_lane(Inputs(t=0.0, dist_mm=10.0, yaw=0.0, lane=seen), 0.2).get('mode') == 'heading'


def _m2_exit_camera_line(params, yaw_deg):
    """M2_EXIT: 도면 위치(route_s)는 1.5 m 뒤처짐(1114 분석). 카메라는 앞바퀴 600 mm 앞 흰 선을 보고 다가감."""
    import math as _m
    fsm = MissionFSM(dict({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9,
                           'route_points': {'m8_start_line': 7000.0}}, **params))
    fsm.fork_choice = 'A'
    fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork A done')
    assert fsm.state == 'M2_EXIT'
    line_at = 6400.0                                  # 실제 선 위치 (주행 거리)
    t, entered = 0.0, None
    for d in range(5000, 7200, 20):
        t += 0.05
        rem = line_at - d
        lane = {'status': 'BOTH', 'stoplines_mm': [rem] if 350.0 <= rem <= 900.0 else []}   # 350 mm 안은 카메라 사각
        fsm.step(Inputs(t=t, dist_mm=float(d), yaw=_m.radians(yaw_deg), route_s=float(d) - 1500.0, lane=lane))
        if fsm.state == 'M8_CONES':
            entered = d
            break
    return fsm, entered, line_at


def test_cones_start_by_camera_line_even_if_route_off():
    """11/16: 라바콘 시작선 = 카메라로 본 흰 가로선을 앞바퀴가 지난 순간. 도면 위치가 1.5 m 틀려도 (예전엔 gate로 버림)."""
    for yaw in (-90.0, -60.0):                         # 코너 다 돈 뒤 / 아직 도는 중 (곡선 돌자마자 선)
        fsm, entered, line_at = _m2_exit_camera_line({}, yaw)
        assert entered is not None and line_at <= entered <= line_at + 40, (yaw, entered)
        assert 'camera' in fsm.m2x_how
    _, old, line_at = _m2_exit_camera_line({'cones_line_camera_first': False}, -90.0)
    assert old is None or old > line_at + 400            # 예전: 카메라 선 버리고 도면/거리로 늦게


@pytest.fixture
def old_m2(monkeypatch):
    """11/18 이전 M2 동작(한쪽 선 모드·폭 460·합류 S자) 테스트: m2_plain_lane 끔."""
    from autorace_mission import fsm as fsm_mod
    monkeypatch.setitem(fsm_mod.DEFAULT_PARAMS, 'm2_plain_lane', False)


# ---------------------------------------------------------------- 11/19 라바콘 뒤 코너: 출발 차로로 나감
def _m1_forced_fsm(**kw):
    import math as _m
    fsm = MissionFSM(dict({'start_state': 'M8_CONES', 'm1_corner_start_mm': 280.0,
                           'route_points': {'m1_corner_start': 24506.0}}, **kw))
    fsm.axis_yaw = -fsm.p['m7_corner_dir'] * _m.pi / 2.0
    return fsm


def test_m1_corner_start_ignores_lagging_route():
    """1119_09: 도면 위치가 3 m 뒤처져 코너 시작을 3150 mm 앞으로 잡음 -> 라바콘 끝 기준 280 mm로"""
    fsm = _m1_forced_fsm()
    fsm._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=0.0, lane={}, route_s=24506.0 - 3150.0), 'cones done')
    assert fsm.m1fc['xs'] == pytest.approx(280.0)
    assert any('from route ignored' in m for m in fsm.log)
    ok = _m1_forced_fsm()
    ok._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=0.0, lane={}, route_s=24506.0 - 400.0), 'cones done')
    assert ok.m1fc['xs'] == pytest.approx(400.0)              # 도면과 120 mm 차이는 도면 그대로


def test_m1_corner_lane_underturn_uses_forced_arc():
    """1119_09: 코너 안에서 바깥 선만 보며 k -0.4~-0.8 (원호 -1.56) -> 강제 원호. 차선이 충분히 꺾으면 차선 주행"""
    fsm = _m1_forced_fsm()
    fsm._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=0.0, lane={}), 'cones done')
    weak = {'status': 'L', 'kappa': -0.5}
    cmd = None
    for i in range(10):                                        # 코너 시작 전 직선: 차선 주행
        cmd = fsm.step(Inputs(t=0.05 * i, dist_mm=10.0 * i, yaw=0.0, confirmed_classes=set(), lane=weak))
    assert cmd.lane['mode'] != 'heading'
    for i in range(10, 20):                                    # 원호 구간 (s >= 280 - 100)
        cmd = fsm.step(Inputs(t=0.05 * i, dist_mm=200.0 + 10.0 * i, yaw=0.0, confirmed_classes=set(), lane=weak))
    assert cmd.lane['mode'] == 'heading'
    assert any('turns too little' in m for m in fsm.log)
    good = _m1_forced_fsm()
    good._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=0.0, yaw=0.0, lane={}), 'cones done')
    for i in range(20):
        cmd = good.step(Inputs(t=0.05 * i, dist_mm=20.0 * i, yaw=0.0, confirmed_classes=set(),
                               lane={'status': 'BOTH', 'kappa': -1.5}))
    assert cmd.lane['mode'] != 'heading'


# ---------------------------------------------------------------- 11/19 빨간불이면 카메라 정지선 우선
@pytest.mark.parametrize('route_off', [17000.0, -3000.0, 0.0])
def test_m1_red_stops_at_camera_line_even_if_route_is_off(route_off):
    """1119_08·09: 빨간불 확정인데 도면 위치가 틀려(정지선 17~22 m 앞으로 계산) 카메라 정지선을 버리고 안 섬"""
    line_s = 27176.0
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'stop_brake_mm': 0.0,
                      'route_points': {'m1_stop_line': line_s}})
    sim = Sim(fsm, dt=0.02)
    line_at = 1800.0                                     # 진짜 정지선: 출발 1.8 m 앞
    s0 = line_s - line_at - route_off                    # 도면 위치 (route_off만큼 틀림)
    stopped_at = None
    for i in range(1500):
        remaining = line_at - sim.dist
        lane = {'crosswalk_lines_mm': [remaining] if 120 < remaining < 1900 else [],
                'stoplines_mm': [remaining] if 120 < remaining < 1900 else []}
        sim.tick(detections={'red': 0.9}, lane=lane, route_s=s0 + sim.dist)
        if fsm.sub == 'stopped' and stopped_at is None:
            stopped_at = remaining
        if stopped_at is not None and sim.t > 20.0:
            break
    assert stopped_at is not None, fsm.log[-5:]
    assert abs(stopped_at - fsm.p['stop_target_mm']) <= 60.0     # 목표(150) ±6 cm 안에서 정지 명령
    assert fsm.sub == 'stopped'
    if route_off:
        assert any('camera stop line first' in m for m in fsm.log)
def test_m2_only_replay_uses_avoid_node_then_follows_branch():
    """11/19: M7 없이 M2로 시작 -> obstacle_node가 M2만 재생을 켬. 그동안 회피 노드 명령(직진 차선 주행 아님), 끊기면 정지,
    끝나면 들어간 갈래(m7_exit)를 따름"""
    fsm = MissionFSM({'start_state': 'M2_FORK'})
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0, obstacle={'m2_replay_active': True}))
    assert cmd.source == 'obstacle' and 'M7 skipped' in cmd.note
    cmd = fsm.step(Inputs(t=0.1, dist_mm=10.0, obstacle={'m2_replay_hold': True}))
    assert cmd.source == 'stop'
    cmd = fsm.step(Inputs(t=5.0, dist_mm=1300.0, obstacle={'m2_replay_done': True, 'm7_exit': 'B'}))
    assert fsm.state == 'M2_FORK' and fsm.fork_choice == 'B' and cmd.source == 'lane'
