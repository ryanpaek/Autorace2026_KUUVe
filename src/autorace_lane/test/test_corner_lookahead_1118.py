"""11/18: 커브 lookahead, 코너 미리보기 중 선 놓침 + 위치 뒤처짐"""
import numpy as np

from autorace_lane.lane_core import LaneFollower, LaneParams


def _f(**kw):
    import os
    from autorace_lane.lane_core import BevConfig
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = BevConfig.from_yaml(os.path.join(here, '..', '..', 'autorace_bringup', 'config', 'bev_limo.yaml'))
    return LaneFollower(cfg, LaneParams(**kw)), cfg


def _blank(cfg):
    return np.zeros((cfg.bev_h, cfg.bev_w), np.uint8)


def test_lost_in_corner_preview_uses_corner_ahead_when_s_lags():
    f, cfg = _f(lost_hold_frames=30)
    f.route_step_mm = 100.0
    f.route_k = [0.0] * 9 + [-1.5625] * 5          # 도면상 코너는 0.9 m 뒤 (위치 뒤처짐)
    f.route_corner = True
    f.last_good_kappa = -1.3
    for _ in range(5):                               # 조향 변화율 제한 때문에 몇 프레임
        out = f.process_bev(_blank(cfg), _blank(cfg), yaw=0.0, speed=0.2)
    assert out.source == 'route' and out.kappa < -1.0


def test_lost_corner_window_off_keeps_old():
    f, cfg = _f(lost_corner_window=False)
    f.route_step_mm = 100.0
    f.route_k = [0.0] * 9 + [-1.5625] * 5
    f.route_corner = True
    for _ in range(5):
        out = f.process_bev(_blank(cfg), _blank(cfg), yaw=0.0, speed=0.2)
    assert abs(out.kappa) < 0.3


def test_corner_lookahead_only_in_corner():
    f, cfg = _f()
    f.route_step_mm = 100.0
    f.route_k = [0.0] * 14
    out = f.process_bev(_blank(cfg), _blank(cfg), yaw=0.0, speed=0.3)
    assert out.lookahead_mm == 650.0
    f.route_k = [-1.5625] * 14
    out = f.process_bev(_blank(cfg), _blank(cfg), yaw=0.0, speed=0.3)
    assert out.lookahead_mm == 700.0
