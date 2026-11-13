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
    assert line_at - stopped_at == pytest.approx(0.0, abs=10)   # ±5cm 판정 안쪽
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


def test_fork_choice_remembered_from_obstacle_zone():
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
    assert 1500.0 - 50 <= sim.dist < 2000.0                # 끝까지 안 기다리고 조금 일찍 (armed 이후)


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
        sim.tick(yaw=math.pi - turn, obstacle={'cone_ahead_mm': 400.0})   # 코너에서도 앞에 콘이 보임
        if fsm.sub == 'cones':
            entered = d
            break
    return fsm, entered


def test_cones_not_entered_inside_corner():
    """예전: 코너 안(콘이 앞에 보임 + 진입 후 거리)에서 라바콘 모드 -> 콘 쪽으로 곧장(지름길)."""
    _, old = _m8_from_m2(0.0)
    assert old is not None and old < 609.0 + 1005.0                 # 코너 안에서 들어감 (재현)
    fsm, new = _m8_from_m2(75.0)
    assert new is not None and new >= 609.0 + 1005.0 * (75.0 / 90.0)   # 코너를 거의 다 돈 뒤
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
    kc = -1.5625 * 1.15                                  # 11/12 밤: M2 뒤 코너는 1.15배 (더 안쪽으로)
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


def test_fork_lane_cfg_keeps_away_from_island_and_searches():
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
