"""11/18 1118_03: M2 출구 코너 위치 뒤처짐 대책"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.fsm import Inputs, MissionFSM  # noqa: E402


def test_m2_exit_lowers_route_corner_weight():
    fsm = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9})
    fsm.fork_choice = 'B'
    fsm.axis_yaw = 0.0
    fsm._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork B done')
    assert fsm.state == 'M2_EXIT'
    cmd = fsm.step(Inputs(t=0.1, dist_mm=5100.0, yaw=0.0, lane={'status': 'BOTH'}))
    assert cmd.lane.get('route_corner_weight') == pytest.approx(0.4)
    off = MissionFSM({'start_state': 'M2_FORK', 'cones_source_ready': True, 'odom_jump_mm': 1e9, 'm2x_corner_weight': 0.0})
    off.fork_choice, off.axis_yaw = 'B', 0.0
    off._next(Inputs(t=0.0, dist_mm=5000.0, yaw=0.0), 'fork B done')
    cmd = off.step(Inputs(t=0.1, dist_mm=5100.0, yaw=0.0, lane={'status': 'BOTH'}))
    assert 'route_corner_weight' not in cmd.lane


def test_m2_end_anchor_s():
    from autorace_mission.route import m2_end_anchor_s
    # 1118_03: 섬 꼭짓점 17973, 회피 x 5.03 / 4.80 -> 18203 (그때 s 17680, -523)
    assert m2_end_anchor_s(17973.0, 5.03, 4.80, 17680.0, 1500.0) == pytest.approx(18203.0)
    assert m2_end_anchor_s(17973.0, 5.03, 4.80, 16000.0, 1500.0) is None
