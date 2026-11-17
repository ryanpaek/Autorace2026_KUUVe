"""가속도로 전진/후진 (조종기 후진: 드라이버가 위치·속도를 전진으로 낼 때)."""
import random

from autorace_mission.motion_dir import AccelDirection
from autorace_mission.route import motion_sign


def _drive(a, seq, axis=1, seed=0):
    random.seed(seed)
    t, v, x, out = 0.0, 0.0, 0.0, []
    for dur, vt, cmd in seq:
        d = 0
        for i in range(int(dur / 0.01)):
            t += 0.01
            acc = max(-1.0, min(1.0, (vt - v) / 0.3))
            v += acc * 0.01
            x += abs(v) * 0.01                       # 드라이버: 후진도 위치를 앞으로 쌓음
            a.on_imu(t, axis * acc + 0.03 + random.gauss(0, 0.08))
            if i % 2 == 0:
                d = a.on_odom(t, x, 0.0, cmd)
        out.append((vt, d))
    return out


SEQ = [(1, 0, 0), (2, 0.2, 0.2), (1, 0, 0), (2, 0.2, 0.2), (1, 0, 0), (2, 0.2, 0.2),
       (1, 0, 0.2), (2, -0.2, 0.2),                 # 우리 명령은 전진인데 조종기로 후진
       (1, 0, 0), (2, 0.2, 0.2), (1, 0, 0), (1.5, -0.25, None), (0.5, 0, None), (2, 0.2, 0.2)]


def test_learns_axis_and_detects_rc_reverse_both_axis_signs():
    for axis in (1, -1):
        for seed in range(10):
            out = _drive(AccelDirection(), SEQ, axis, seed)
            moving = [(vt, d) for vt, d in out if vt != 0]
            assert moving[0][1] == 0                         # 처음엔 아직 모름 (예전과 같음)
            for vt, d in moving[1:]:
                assert d == (1 if vt > 0 else -1)


def test_unknown_axis_never_says_reverse():
    a = AccelDirection()
    out = _drive(a, [(1, 0, None), (2, -0.2, None), (1, 0, None), (2, 0.2, None)])
    assert all(d == 0 for _, d in out)


def test_motion_sign_uses_accel_last():
    assert motion_sign(0.01, 0.0, 0.0, 0.2, None, -1) == (-1, 'imu_acc')
    assert motion_sign(0.01, 0.0, 0.0, 0.2, None, 0) == (1, '')
