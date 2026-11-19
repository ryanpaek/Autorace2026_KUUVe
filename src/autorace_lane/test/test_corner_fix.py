"""11/19 corner_fix: keep_kappa_on_heading_switch, corner_lock (기본 꺼짐 = 기존 동작)."""
from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams, corner_lock_kappa


def _follower(**kw):
    cfg = BevConfig.__new__(BevConfig)           # set_mode는 cfg를 안 씀
    return LaneFollower(cfg, LaneParams(**kw))


def _in_corner(f, k=-1.72):
    f.route_corner, f.route_k = True, [k] * 14
    return f


def test_switch_default_resets_kappa():
    f = _in_corner(_follower())
    f.kappa_cmd = 1.5
    f.set_mode('heading')
    assert f.kappa_cmd == 0.0


def test_switch_keep_normal_heading_in_corner():
    f = _in_corner(_follower(keep_kappa_on_heading_switch=True))
    f.kappa_cmd, f.in_corner_spd = 1.5, True
    f.set_mode('heading')
    assert f.kappa_cmd == 1.5 and f.in_corner_spd
    f.set_mode('normal')
    assert f.kappa_cmd == 1.5


def test_switch_keep_not_outside_corner():
    f = _follower(keep_kappa_on_heading_switch=True)
    f.route_corner, f.route_k = True, [0.0] * 14  # 미리보기는 켜졌지만 코너 전 (합류 S자 등)
    f.kappa_cmd = 1.5
    f.set_mode('heading')
    assert f.kappa_cmd == 0.0
    f = _follower(keep_kappa_on_heading_switch=True)   # 미리보기 꺼짐 (코너 끝)
    f.kappa_cmd = 1.5
    f.set_mode('heading')
    assert f.kappa_cmd == 0.0


def test_switch_keep_only_normal_heading():
    f = _in_corner(_follower(keep_kappa_on_heading_switch=True))
    f.kappa_cmd = 1.5
    f.set_mode('zigzag')
    assert f.kappa_cmd == 0.0


def test_corner_lock_flip_cannot_undo_turn():
    k_ff = -1.72                                  # 오른쪽 코너 (왼쪽 +)
    blended = 2.0 + 0.8 * (k_ff - 2.0)            # L/R 헷갈림 +2.0 -> 기존 섞기 -0.98
    assert abs(corner_lock_kappa(blended, k_ff, 0.3) - (k_ff + 0.3)) < 1e-9
    assert corner_lock_kappa(1.0, 1.72, 0.3) == 1.72 - 0.3


def test_corner_lock_keeps_harder_turn():
    assert corner_lock_kappa(-2.05, -1.72, 0.3) == -2.05   # 바깥 밀림 보정 (11/12 규칙) 그대로
    assert corner_lock_kappa(-1.60, -1.72, 0.3) == -1.60
