"""11/19: 갈림길 섬 기준 (island_ref). 1119_09: 차가 섬 오른쪽 변 위에 걸쳐 가장 가까운 선이 섬 왼쪽 변이 됨"""
import numpy as np

from autorace_lane.lane_core import LaneFollower, LaneParams
from synth import blank, draw_polyline_mm, make_cfg


def _scene(cfg, xs_mm):
    img = blank(cfg)
    ys = np.linspace(0, 3000, 50)
    for x in xs_mm:
        draw_polyline_mm(img, cfg, np.full_like(ys, x), ys)
    return img


def _lf(**kw):
    cfg = make_cfg()
    lf = LaneFollower(cfg, LaneParams(lane_color='yellow', lane_width_mm=520.0, island_ref=True, **kw))
    lf.mode = 'left_only'          # B
    return cfg, lf


def test_b_picks_island_right_edge_from_outer_line():
    # 섬 왼쪽 변 -300, 섬 오른쪽 변 +100 (차가 걸쳐 있음), B 바깥선 +620
    cfg, lf = _lf()
    o = lf.process_bev(_scene(cfg, [-300.0, 100.0, 620.0]), blank(cfg), speed=0.2)
    assert o.status == 'L'
    x0 = (o.debug['left'][0, 0] - cfg.vehicle_col) * cfg.mm_per_px_x
    assert abs(x0 - 100.0) < 30.0                     # 섬 오른쪽 변
    assert o.offset_mm < -150.0                       # 차로 가운데(+360)는 오른쪽


def test_b_letter_hint_picks_island_edge_without_outer_line():
    cfg, lf = _lf()
    lf._letter_seen = ('B', 360.0, 0)                 # B 글자 = B 차로 가운데 (+360)
    o = lf.process_bev(_scene(cfg, [-300.0, 100.0]), blank(cfg), speed=0.2)
    x0 = (o.debug['left'][0, 0] - cfg.vehicle_col) * cfg.mm_per_px_x
    assert abs(x0 - 100.0) < 30.0


def test_wrong_letter_is_not_used():
    cfg, lf = _lf()
    lf._letter_seen = ('A', 360.0, 0)                 # B 갈래인데 A 글자 -> 안 씀 (가장 가까운 선)
    o = lf.process_bev(_scene(cfg, [-300.0, 100.0]), blank(cfg), speed=0.2)
    x0 = (o.debug['left'][0, 0] - cfg.vehicle_col) * cfg.mm_per_px_x
    assert abs(x0 + 300.0) < 30.0


def test_island_ref_off_keeps_old_choice():
    cfg = make_cfg()
    lf = LaneFollower(cfg, LaneParams(lane_color='yellow', lane_width_mm=520.0, island_ref=False))
    lf.mode = 'left_only'
    o = lf.process_bev(_scene(cfg, [-300.0, 100.0, 620.0]), blank(cfg), speed=0.2)
    x0 = (o.debug['left'][0, 0] - cfg.vehicle_col) * cfg.mm_per_px_x
    assert abs(x0 + 300.0) < 30.0


def test_only_far_side_outer_line_is_not_island_edge():
    """11/20 yubin: A 끝 직선에서 섬 변이 화면 밖 -> 차 왼쪽 바깥선만 보임. 그걸 오른쪽 선으로 잡으면 왼쪽으로 나감 -> 선 없음"""
    for mode, x in (('right_only', -260.0), ('left_only', 260.0)):
        cfg, lf = _lf()
        lf.mode = mode
        o = lf.process_bev(_scene(cfg, [x]), blank(cfg), speed=0.2)
        assert o.status == 'NONE'
    cfg, lf = _lf()
    lf.mode = 'right_only'
    assert lf.process_bev(_scene(cfg, [260.0]), blank(cfg), speed=0.2).status == 'R'   # 맞는 쪽 선은 그대로


def test_outer_line_crossing_ahead_is_not_island_edge():
    """11/20 yubin sim_1 042914: A 재생 끝에 차가 바깥쪽으로 21° -> 왼쪽 바깥선이 차 앞 가운데를 지나 오른쪽으로 뻗음. 섬 변 아님"""
    import math
    cfg, lf = _lf()
    lf.mode = 'right_only'
    ys = np.linspace(0, 3000, 50)
    img = blank(cfg)
    t = math.tan(math.radians(21.0))
    draw_polyline_mm(img, cfg, -250.0 / math.cos(math.radians(21.0)) + ys * t, ys)
    assert lf.process_bev(img, blank(cfg), speed=0.2).status == 'NONE'
