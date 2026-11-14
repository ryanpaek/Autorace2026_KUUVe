"""선 종류(노란 실선 / 흰 점선) 판단과 3차선 차로 번호 (11/10)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from autorace_lane.lane_core import BevConfig, LaneParams, classify_line, lane_id_from_types  # noqa: E402

CFG = BevConfig.from_yaml(os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup',
                                       'config', 'bev_limo.yaml'))


def _masks(col, yellow, dashed):
    h, w = CFG.bev_h, CFG.bev_w
    lane, white = np.zeros((h, w), np.uint8), np.zeros((h, w), np.uint8)
    hw = max(1, int(10 / CFG.mm_per_px_x))
    period = int(200 / CFG.mm_per_px_y)
    for r in range(h):
        if dashed and (r // (period // 2)) % 2:
            continue
        lane[r, col - hw:col + hw + 1] = 255
        if not yellow:
            white[r, col - hw:col + hw + 1] = 255
    n = 10
    track = np.array([[col, h - (i + 0.5) * h / n] for i in range(n)], float)
    return lane, white, track


def test_yellow_solid_and_white_dashed():
    p = LaneParams()
    lane, white, tr = _masks(CFG.bev_w // 3, yellow=True, dashed=False)
    a = classify_line(tr, lane, white, CFG, p)
    assert a['color'] == 'yellow' and not a['dashed']
    lane, white, tr = _masks(2 * CFG.bev_w // 3, yellow=False, dashed=True)
    b = classify_line(tr, lane, white, CFG, p)
    assert b['color'] == 'white' and b['dashed']


def test_lane_id_rules():
    Y = {'color': 'yellow', 'dashed': False}
    W = {'color': 'white', 'dashed': True}
    assert lane_id_from_types(Y, W) == 1
    assert lane_id_from_types(W, W) == 0
    assert lane_id_from_types(W, Y) == -1
    assert lane_id_from_types(Y, Y) is None and lane_id_from_types(None, W) is None


def test_yellow_solid_guard_steers_strongly_away_from_yellow_boundary():
    """11/14: 3차선 M7(또는 일반 구간)에서 노란 실선에 가까워지면 강하게 안쪽으로 조향해 이탈을 막는다."""
    cfg = BevConfig()
    p = LaneParams(lane_width_mm=520.0, lane_color='both', yellow_solid_guard=True,
                   yellow_guard_margin_mm=200.0, yellow_guard_hard_mm=140.0)
    f = LaneFollower(cfg, p)
    img = np.zeros((cfg.bev_h, cfg.bev_w, 3), np.uint8)
    draw_line(img, cfg, -120.0, (0, 220, 240), dashed=False)
    draw_line(img, cfg, +400.0, (240, 240, 240), dashed=True)
    lane, white = color_masks(img, p)
    out = f.process_bev(lane, white, speed=0.2, raw_lane=lane)
    assert out.yellow_guard == 'L'
    assert out.kappa == pytest.approx(-p.kappa_max, abs=1e-3)

