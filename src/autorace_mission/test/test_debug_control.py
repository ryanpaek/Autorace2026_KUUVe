"""디버그 화면 미션 조작 버튼 (debug_control.py)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.debug_control import CHOOSABLE, control_request, next_state  # noqa: E402


def req(body, enabled=True, pw='', cur='M7_OBSTACLE'):
    return control_request(body, enabled, pw, cur)


def test_set_state_resets_to_start_point_topic():
    ok, _, pubs = req({'action': 'set_state', 'state': 'M2_FORK'})
    assert ok and pubs == [('/mission/set_state', 'M2_FORK')]


def test_skip_and_next_keep_position():
    assert req({'action': 'skip', 'state': 'M4_PARKING'})[2] == [('/mission/skip', 'M4_PARKING')]
    assert req({'action': 'next'})[2] == [('/mission/skip', 'M2_FORK')]
    assert not req({'action': 'next'}, cur='FINISHED')[0]


def test_start_and_estop():
    assert req({'action': 'start'})[2] == [('/mission/start', None)]
    assert req({'action': 'estop_off'})[2] == [('/mission/estop', False)]


def test_estop_always_allowed_even_when_disabled_or_wrong_password():
    for kw in ({'enabled': False}, {'pw': 'kuuve'}):
        ok, _, pubs = req({'action': 'estop'}, **kw)
        assert ok and pubs == [('/mission/estop', True)]


def test_disabled_rejects_everything_else():
    for act in ('start', 'next', 'estop_off'):
        assert req({'action': act}, enabled=False) == (False, req({'action': act}, enabled=False)[1], [])


def test_password():
    assert not req({'action': 'start'}, pw='kuuve')[0]
    assert not req({'action': 'start', 'password': 'x'}, pw='kuuve')[0]
    assert req({'action': 'start', 'password': 'kuuve'}, pw='kuuve')[0]


def test_bad_requests():
    assert not req(None)[0]
    assert not req({'action': 'set_state', 'state': 'M9_SPACE'})[0]
    assert not req({'action': 'fly'})[0]
    assert 'WAIT_START' not in CHOOSABLE and 'M4_PARKING' in CHOOSABLE
    assert next_state('M1_TRAFFIC') == 'M4_PARKING'
