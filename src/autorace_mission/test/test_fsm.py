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
