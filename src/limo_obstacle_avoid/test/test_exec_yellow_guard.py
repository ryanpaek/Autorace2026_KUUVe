"""11/18 시험: 재생(EXEC) 중 노란선 안전장치. 노란선이 '보이기만' 하면 안 걸리고, 붙기 직전에만 걸리는지."""
import math

from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, default_change, mirror
from limo_obstacle_avoid.path_track import MAX_CURV

F = 0.575   # 차선 노드가 노란선 거리를 재는 앞 거리 (m, 제어 기준점 기준)


def _exec_left(arm=0.0):
    left = default_change(+1, 0.52)
    m = M7Replay(ReplayParams(start_lane=0, trigger_mm=900.0, eyg_arm_frac=arm), left, mirror(left))
    m.start()
    m.scan_t = 0.0
    m._begin_exec(+1, None, 0.0, 'test')
    assert m.state == 'EXEC'
    return m


def _lane(d_ref, psi, side=+1):
    """차 기준점에서 노란선까지 옆 거리 d_ref, 차 방향 psi(왼쪽 +)일 때 차선 노드가 보낼 값."""
    a = side * psi
    meas = 1000.0 * (d_ref - F * math.tan(a))
    key = 'yellow_l_mm' if side > 0 else 'yellow_r_mm'
    return {'status': 'R' if side > 0 else 'L', key: meas, 'yellow_f_mm': 1000.0 * F}


def _check(m, d_ref, psi_deg, t, side=+1):
    m.on_lane(_lane(d_ref, math.radians(psi_deg), side), t)
    return m._exec_yellow_check(t, math.radians(psi_deg))


def test_seeing_yellow_at_start_of_change_does_not_trip():
    """가운데 차로에서 왼쪽으로 15° 틀기 시작: 노란선이 보여도(옆 78 cm) 안 걸림"""
    m = _exec_left()
    for i in range(5):
        assert _check(m, 0.78, 15.0, 0.1 * i) is None
    assert m.eyg_last['clear_mm'] > 400


def test_mid_change_heading_toward_yellow_does_not_trip():
    """차로 변경 중간: 노란선 35 cm, 20° 노란선 쪽 -> 여유 남음, 안 걸림 (재생이 알아서 폄)"""
    m = _exec_left()
    for i in range(5):
        assert _check(m, 0.35, 20.0, 0.1 * i) is None
    assert 50 < m.eyg_last['clear_mm'] < 150


def test_trips_when_about_to_cross_yellow_after_two_frames():
    """1차선 중심(노란선 26 cm) 근처에서 아직 20° 노란선 쪽 -> 2프레임 연속이면 걸림"""
    m = _exec_left()
    assert _check(m, 0.27, 20.0, 0.0) is None          # 1프레임째
    assert _check(m, 0.27, 20.0, 0.05) == +1            # 2프레임째


def test_heading_away_never_trips():
    m = _exec_left()
    for i in range(5):
        assert _check(m, 0.15, -10.0, 0.1 * i) is None


def test_right_side_mirror():
    left = default_change(+1, 0.52)
    m = M7Replay(ReplayParams(start_lane=0, trigger_mm=900.0, eyg_arm_frac=0.0), left, mirror(left))
    m.start()
    m.scan_t = 0.0
    m._begin_exec(-1, None, 0.0, 'test')
    _check(m, 0.27, -20.0, 0.0, side=-1)
    assert _check(m, 0.27, -20.0, 0.05, side=-1) == -1


def test_arm_fraction_blocks_early_trip():
    """진행률이 eyg_arm_frac 전이면 안 걸림"""
    m = _exec_left(arm=0.3)
    for i in range(5):
        assert _check(m, 0.27, 20.0, 0.05 * i) is None


def test_stale_lane_status_ignored():
    m = _exec_left()
    m.on_lane(_lane(0.27, math.radians(20.0)), 0.0)
    assert m._exec_yellow_check(1.0, math.radians(20.0)) is None


def test_step_ends_replay_and_steers_away():
    """걸리면 재생 끝 -> 그 틱 노란선 반대로 최대 조향, 상태 CRUISE, 목표 차로로"""
    m = _exec_left()
    m._shift(ddpsi=math.radians(20.0))
    m.on_lane(_lane(0.27, math.radians(20.0)), 0.0)
    m.step(0.0)
    m.on_lane(_lane(0.27, math.radians(20.0)), 0.05)
    v, k = m.step(0.05)
    assert m.state == 'CRUISE' and m.lane == +1 and m.eyg_hits == 1
    assert abs(k + MAX_CURV) < 1e-9 and v >= 0
    assert any('노란선 안전장치' in e for e in m.events)
    assert m.status()['eyg_hits'] == 1


def test_off_switch():
    m = _exec_left()
    m.p.exec_yellow_guard = False
    for i in range(5):
        assert _check(m, 0.20, 25.0, 0.05 * i) is None
