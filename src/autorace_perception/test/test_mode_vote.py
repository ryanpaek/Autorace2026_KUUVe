"""미션별 투표 기준 (vote_need.<미션>) 테스트 (11/10)."""

import pytest

from autorace_perception.class_commands import MODE_CLASSES, ClassCommandRouter


def _feed(router, cls, hits, window=10):
    """window 프레임 중 앞쪽 hits 프레임만 cls를 본다. 마지막 출력의 confirmed_classes."""
    out = None
    for i in range(window):
        dets = [{'cls': cls, 'conf': 0.9}] if i < hits else []
        out = router.route({'stamp': float(i + 1), 'detections': dets})
    return out['confirmed_classes'], out['vote']['need']


def _router(per_mode):
    r = ClassCommandRouter()
    need = dict.fromkeys(MODE_CLASSES, 0)
    need.update(per_mode)
    r.configure(0.5, 'none', 10, 7, mode_need=need)
    return r


def test_default_uses_global_need():
    r = _router({})
    r.set_mode('M5_BUMP')
    confirmed, need = _feed(r, 'slow', 6)
    assert need == 7 and 'slow' not in confirmed


def test_looser_mission_confirms_earlier():
    r = _router({'M5_BUMP': 5})
    r.set_mode('M5_BUMP')
    confirmed, need = _feed(r, 'slow', 5)
    assert need == 5 and 'slow' in confirmed


def test_strict_mission_keeps_global():
    r = _router({'M5_BUMP': 5, 'M2_FORK': 7})
    r.set_mode('M2_FORK')
    confirmed, need = _feed(r, 'fork_a', 5)
    assert need == 7 and 'fork_a' not in confirmed


def test_need_follows_mode_change():
    r = _router({'M4_PARKING': 5})
    r.set_mode('M4_PARKING')
    assert r.need_for(r.mode) == 5
    r.set_mode('M1_TRAFFIC')
    assert r.need_for(r.mode) == 7


@pytest.mark.parametrize('bad', [-1, 11, 2.5])
def test_rejects_bad_values(bad):
    r = ClassCommandRouter()
    need = dict.fromkeys(MODE_CLASSES, 0)
    need['M5_BUMP'] = bad
    with pytest.raises(ValueError):
        r.configure(0.5, 'none', 10, 7, mode_need=need)
