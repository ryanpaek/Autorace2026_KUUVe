"""디버그 화면 '왜?' 설명 (explain.py): 모든 상태에서 깨지지 않고, 핵심 근거·조건이 나온다."""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.explain import STATE_KO, explain  # noqa: E402
from autorace_mission.fsm import SEQUENCE  # noqa: E402

POINTS = {'m5_slow_line': 1500.0, 'm6_zigzag_line': 9000.0, 'm6_zigzag_end': 10700.0,
          'm3_stop_line': 11000.0, 'm3_crosswalk_end': 11540.0, 'm7_island_tip': 17970.0,
          'm2_merge_end': 20270.0, 'm8_start_line': 21400.0, 'm8_end_line': 23500.0,
          'm1_stop_line': 27500.0, 'm4_pre_line': 29900.0}


@pytest.mark.parametrize('state', SEQUENCE + ['LANE_ONLY'])
@pytest.mark.parametrize('sub', ['', 'approach', 'stopped', 'go', 'slow', 'zigzag', 'cones', 'handover', 'branch', 'armed'])
def test_every_state_and_sub_renders(state, sub):
    out = explain({'state': state, 'sub': sub, 'source': 'lane', 'route': {'s_mm': 5000}},
                  points=POINTS, sub_age=1.0, cmd=(0.2, 0.0))
    assert out['title'] and out['state'].startswith(STATE_KO[state])
    for c in out['next']:
        assert c['progress'] is None or 0.0 <= c['progress'] <= 1.0


def test_empty_inputs_do_not_crash():
    out = explain({})
    assert out['title'] == '-'


def test_m3_approach_shows_camera_and_map_distance():
    out = explain({'state': 'M3_PEDESTRIAN', 'sub': 'approach', 'source': 'lane',
                   'route': {'s_mm': POINTS['m3_stop_line'] - 600}},
                  lane={'stoplines_mm': [420.0, 1000.0]}, points=POINTS)
    texts = {c['text']: c['value'] for c in out['next']}
    assert texts['정지선까지 (카메라)'] == '420 mm'
    assert texts['정지선까지 (도면)'] == '600 mm'


def test_vote_progress_and_fallback_warning():
    out = explain({'state': 'M3_PEDESTRIAN', 'sub': '', 'source': 'lane', 'note': 'FALLBACK: x',
                   'route': {'s_mm': 9000, 'lost': True}},
                  commands={'confirmed_classes': [], 'vote': {'window': 10, 'need': 7, 'frames': 10,
                                                              'counts': {'pedestrian': 5}}},
                  points=POINTS)
    ped = out['next'][0]
    assert ped['progress'] == pytest.approx(5 / 7, abs=0.01) and not ped['met']
    assert any('FALLBACK' in w for w in out['warn'])
    assert any('길 잃음' in w for w in out['warn'])
    assert '도면 위치 모름' in out['next'][1]['value']      # 길 잃으면 도면 거리 안 씀


def test_m7_hold_warns():
    out = explain({'state': 'M7_OBSTACLE', 'source': 'stop'}, obstacle={'m7_hold': True})
    assert any('m7_hold' in w for w in out['warn'])


def test_manual_mode_control_text():
    out = explain({'state': 'M5_BUMP', 'manual': True}, cmd=(0.2, 0.0))
    assert '수동' in out['control']
