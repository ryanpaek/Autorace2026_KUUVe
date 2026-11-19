"""11/19 corner_fix: m1_lane_back_frames (4번 강제 원호 -> 차선 복귀 히스테리시스). 1 = 기존 동작."""
import types

from autorace_mission.fsm import MissionFSM


def _lf(back, seq, n_lost=5):
    s = types.SimpleNamespace(p={'lane_first': True, 'lane_first_lost_frames': n_lost})
    out = []
    for good in seq:
        inp = types.SimpleNamespace(lane={'status': 'BOTH' if good else 'NONE', 'low_conf': False})
        out.append(MissionFSM._lane_first(s, inp) if back is None else MissionFSM._lane_first(s, inp, back))
    return out


def test_lane_first_default_unchanged():
    seq = [0] * 6 + [1, 0, 1, 1]
    assert _lf(None, seq) == _lf(1, seq)
    assert _lf(1, seq)[6] is True


def test_lane_first_back_frames():
    r = _lf(3, [0] * 6 + [1, 0, 1, 1, 1])
    assert r[5] is False
    assert r[6] is False and r[8] is False
    assert r[10] is True
