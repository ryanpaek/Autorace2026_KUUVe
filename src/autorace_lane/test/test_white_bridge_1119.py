"""11/19: 노출 과다로 노란 선이 흰색이 될 때 이어 붙이기 (white_bridge_mm), 선 놓쳤을 때 속도 상한 (lost_speed)"""
import cv2
import numpy as np

from autorace_lane.lane_core import LaneFollower, LaneParams
from synth import blank, make_cfg, straight_lane


def _follower(**kw):
    cfg = make_cfg()
    p = LaneParams(lane_color='yellow', lane_width_mm=530.0, **kw)
    return cfg, LaneFollower(cfg, p)


def test_bridge_keeps_lane_when_yellow_turns_white():
    cfg, lf = _follower(white_bridge_mm=100.0)
    lane = straight_lane(cfg, width_mm=530.0)
    none = blank(cfg)
    o = lf.process_bev(lane, none.copy(), speed=0.4)       # 노란 선 보임
    assert o.status == 'BOTH'
    for _ in range(10):                                   # 노란 마스크는 비고 흰 마스크에만 선
        o = lf.process_bev(none.copy(), lane.copy(), speed=0.4)
        assert o.status == 'BOTH'
        assert abs(o.offset_mm) < 40.0
        assert o.debug.get('white_bridge')


def test_bridge_off_loses_lane():
    cfg, lf = _follower(white_bridge_mm=0.0)
    lane = straight_lane(cfg, width_mm=530.0)
    none = blank(cfg)
    lf.process_bev(lane, none.copy(), speed=0.4)
    o = lf.process_bev(none.copy(), lane.copy(), speed=0.4)
    assert o.status == 'NONE'


def test_bridge_never_starts_new_line():
    cfg, lf = _follower(white_bridge_mm=100.0)
    lane = straight_lane(cfg, width_mm=530.0)
    none = blank(cfg)
    o = lf.process_bev(none.copy(), lane.copy(), speed=0.4)   # 처음부터 흰색만 (따라가던 선 없음)
    assert o.status == 'NONE'


def test_bridge_drops_wide_bright_floor():
    cfg, lf = _follower(white_bridge_mm=100.0)
    lane = straight_lane(cfg, width_mm=530.0)
    none = blank(cfg)
    lf.process_bev(lane, none.copy(), speed=0.4)
    floor = blank(cfg)
    floor[:, :] = 255                                      # 바닥 전체가 밝게 (가로로 넓은 덩어리)
    o = lf.process_bev(none.copy(), floor, speed=0.4)
    assert not o.debug.get('white_bridge')
    assert o.status == 'NONE'


def test_lost_speed_caps_hold():
    cfg, lf = _follower(lost_speed=0.15)
    lane = straight_lane(cfg, width_mm=530.0)
    none = blank(cfg)
    lf.process_bev(lane, none.copy(), yaw=0.0, speed=0.4)
    o = lf.process_bev(none.copy(), none.copy(), yaw=0.0, speed=0.4)
    assert o.source in ('hold', 'route', 'search')
    assert o.speed_scale * 0.4 <= 0.15 + 1e-6
