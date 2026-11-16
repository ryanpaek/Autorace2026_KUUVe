"""11/15 바닥 갈림길 글자 A/B (흰 페인트) 구멍 수로 구분."""
import cv2
import numpy as np

from autorace_lane.lane_core import LaneParams, detect_floor_letter
from synth import _to_px, make_cfg, straight_lane


def _letter_img(cfg, ch, x_mm=0.0, y_mm=900.0, h_mm=300.0, lane=True):
    img = straight_lane(cfg, 0.0, 500.0) if lane else np.zeros((cfg.bev_h, cfg.bev_w), np.uint8)
    img = img.copy()
    scale = h_mm / cfg.mm_per_px_y / 22.0                     # HERSHEY_SIMPLEX 대문자 높이 약 22 px (scale 1)
    (tw, th), _ = cv2.getTextSize(ch, cv2.FONT_HERSHEY_SIMPLEX, scale, max(2, int(scale * 2.5)))
    cx, cy = _to_px(cfg, x_mm, y_mm)
    # BEV에서 글자가 진행 방향(위)으로 읽히게 (차에서 보면 아래가 가까움)
    cv2.putText(img, ch, (int(cx - tw / 2), int(cy + th / 2)), cv2.FONT_HERSHEY_SIMPLEX, scale, 255,
                max(2, int(scale * 2.5)))
    return img


def test_floor_letter_a_and_b():
    cfg, p = make_cfg(), LaneParams()
    for ch in ('A', 'B'):
        r = detect_floor_letter(_letter_img(cfg, ch), cfg, p)
        assert r is not None and r[0] == ch, (ch, r)
        assert 500.0 < r[1] < 900.0 and abs(r[2]) < 60.0


def test_floor_letter_ignores_lines_other_lane_and_cut():
    cfg, p = make_cfg(), LaneParams()
    assert detect_floor_letter(straight_lane(cfg, 0.0, 500.0), cfg, p) is None          # 차선만
    assert detect_floor_letter(_letter_img(cfg, 'B', x_mm=520.0), cfg, p) is None       # 옆 차로 글자
    assert detect_floor_letter(_letter_img(cfg, 'A', y_mm=cfg.row_to_y_mm(0)), cfg, p) is None   # 화면 위로 잘림
    assert detect_floor_letter(_letter_img(cfg, 'C'), cfg, p) is None                   # 구멍 없음
