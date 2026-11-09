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
