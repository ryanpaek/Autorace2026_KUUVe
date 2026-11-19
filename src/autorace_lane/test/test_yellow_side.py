"""yellow_side (노란 선 옆 거리) 시험. 합성 BEV(호모그래피 = 단위행렬) 위에서 잰다."""

import numpy as np
import pytest

from autorace_lane.lane_core import BevConfig
from autorace_lane.yellow_side import (BandSampler, SideFilter, YellowSide,
                                       offset_from_sides, row_runs)
from synth import arc_lane, make_cfg, straight_lane

YELLOW_BGR = (0, 200, 230)     # HSV H~26, S 255, V 230 -> 노란색 임계 통과


def to_bgr(mask):
    img = np.zeros(mask.shape + (3,), np.uint8)
    img[mask > 0] = YELLOW_BGR
    return img


@pytest.fixture
def cfg():
    return make_cfg(ref_bottom=550.0)


def test_row_runs():
    row = np.array([0, 255, 255, 0, 0, 255, 0, 255, 255, 255], np.uint8)
    assert row_runs(row) == [(1, 3), (5, 6), (7, 10)]


def test_band_rows_cover_requested_range(cfg):
    s = BandSampler(cfg, 800.0, 1000.0, 2)
    assert s.ys_mm.min() >= 800.0 - 1e-6
    assert s.ys_mm.max() <= 1000.0 + 1e-6
    assert len(s.rows) >= 20


def test_identity_sampler_matches_warp(cfg):
    """remap 띠 = 전체 warp 결과의 같은 줄 (화소 단위로 같아야 함)."""
    img = to_bgr(straight_lane(cfg, center_x_mm=37.0, width_mm=515.0))
    s = BandSampler(cfg, 800.0, 1000.0, 2)
    full = cfg.warp(img)
    assert np.array_equal(s.sample(img), full[s.rows])


@pytest.mark.parametrize('shift', [0.0, 40.0, -60.0, 100.0])
def test_centered_and_shifted(cfg, shift):
    """차로 중심이 오른쪽으로 shift mm -> 차는 차로 중심의 왼쪽 (offset = -shift)."""
    ys = YellowSide(cfg)
    d = ys.measure(to_bgr(straight_lane(cfg, center_x_mm=shift, width_mm=515.0)))
    assert d['l_ok'] and d['r_ok']
    assert d['raw_l'] == pytest.approx(257.5 - shift, abs=6)
    assert d['raw_r'] == pytest.approx(257.5 + shift, abs=6)
    off, src = offset_from_sides(d['raw_l'], d['raw_r'])
    assert src == 'both'
    assert off == pytest.approx(-shift, abs=5)


def test_one_side_uses_half_width(cfg):
    ys = YellowSide(cfg)
    d = ys.measure(to_bgr(straight_lane(cfg, center_x_mm=50.0, width_mm=515.0, left=False)))
    assert not d['l_ok'] and d['r_ok']
    off, src = offset_from_sides(None, d['raw_r'])
    assert src == 'R'
    assert off == pytest.approx(-50.0, abs=6)


def test_nearest_line_wins_in_three_lanes(cfg):
    """선이 여러 개면 차에서 가장 가까운 선을 잡는다 (3차선)."""
    m = straight_lane(cfg, 0.0, 515.0) | straight_lane(cfg, 0.0, 1545.0)
    d = YellowSide(cfg).measure(to_bgr(m))
    assert d['raw_l'] == pytest.approx(257.5, abs=6)
    assert d['raw_r'] == pytest.approx(257.5, abs=6)


def test_wide_blob_rejected(cfg):
    """정지선처럼 넓은 가로 덩어리는 선으로 안 친다."""
    m = np.zeros((cfg.bev_h, cfg.bev_w), np.uint8)
    m[:, 100:200] = 255             # 400 mm 폭
    d = YellowSide(cfg).measure(to_bgr(m))
    assert d['n_l'] == 0 and d['n_r'] == 0


def test_arc_near_band_still_close_to_half_width(cfg):
    """코너 입구(차가 차로 중심, 접선 방향): 가까운 띠에서는 반폭 근처로 잰다."""
    d = YellowSide(cfg).measure(to_bgr(arc_lane(cfg, 640.0, 'left', 515.0)))
    # 800~1000 mm 앞이면 R640 원호가 이미 휘어 있어 안쪽 선은 멀어지고(오른쪽으로 밀림) 바깥은 가까워짐
    assert d['l_ok'] or d['r_ok']


def test_filter_median_and_jump():
    f = SideFilter(window_s=0.5, max_jump_mm=50.0)
    for i, v in enumerate([250, 252, 248, 251]):
        f.update(i * 0.1, v, True)
    assert f.value() == pytest.approx(250.5, abs=1)
    f.update(0.4, 400, True)         # 튐 -> 버림
    assert f.value() == pytest.approx(250.5, abs=2)
    assert f.rejected == 1


def test_filter_accepts_real_change_after_repeats():
    f = SideFilter(window_s=1.0, max_jump_mm=50.0, reset_after=3)
    for i in range(4):
        f.update(i * 0.1, 250, True)
    for i in range(3):
        f.update(0.4 + i * 0.1, 350, True)
    f.update(0.8, 350, True)
    assert f.value() == pytest.approx(350, abs=1)


def test_filter_out_of_range_and_timeout():
    f = SideFilter(window_s=0.5, lo_mm=120, hi_mm=450)
    f.update(0.0, 50, True)          # 범위 밖
    f.update(0.1, 500, True)
    assert f.value() is None
    f.update(0.2, 250, True)
    f.update(0.3, 250, True)
    assert f.value() == 250
    f.update(2.0, None, False)       # 오래 못 보면 비워짐
    assert f.value() is None


def test_step_offset_over_time(cfg):
    ys = YellowSide(cfg)
    img = to_bgr(straight_lane(cfg, center_x_mm=-40.0, width_mm=515.0))
    d = None
    for i in range(5):
        d = ys.step(i * 0.1, img)
    assert d['src'] == 'both'
    assert d['offset_mm'] == pytest.approx(40.0, abs=5)
    assert d['offset_one_mm'] is None


def test_step_one_side_goes_to_separate_field(cfg):
    ys = YellowSide(cfg)
    img = to_bgr(straight_lane(cfg, center_x_mm=0.0, width_mm=515.0, left=False))
    d = None
    for i in range(3):
        d = ys.step(i * 0.1, img)
    assert d['src'] == 'R'
    assert d['offset_mm'] is None
    assert d['offset_one_mm'] == pytest.approx(0.0, abs=6)


def test_real_bev_yaml_loads():
    """실제 bev_limo.yaml로 띠 표가 만들어지고, 띠 위치가 영상 안쪽인지."""
    import os
    here = os.path.dirname(__file__)
    path = os.path.join(here, '..', '..', 'autorace_bringup', 'config', 'bev_limo.yaml')
    cfg = BevConfig.from_yaml(path)
    s = BandSampler(cfg)
    assert s.ys_mm.min() >= 800.0 - 1e-6
    # 띠 안 모든 줄에서 차로 반폭(257.5)+밀림 80 mm까지 영상 안에 보여야 함
    for i in range(len(s.rows)):
        vis = (s.map_x[i] >= 0) & (s.map_x[i] < cfg.image_width) & \
              (s.map_y[i] >= 0) & (s.map_y[i] < cfg.image_height)
        xs = s.x_mm[vis]
        assert xs.min() < -337.0 and xs.max() > 337.0


def test_550_band_cannot_see_lines():
    """근거 기록: 550 mm 앞은 옆으로 +-257 mm 선이 화각 밖 (그래서 띠를 800~1000으로)."""
    import os
    here = os.path.dirname(__file__)
    cfg = BevConfig.from_yaml(os.path.join(here, '..', '..', 'autorace_bringup', 'config', 'bev_limo.yaml'))
    s = BandSampler(cfg, 550.0, 560.0, 1)
    vis = (s.map_x[0] >= 0) & (s.map_x[0] < cfg.image_width)
    assert s.x_mm[vis].min() > -257.5


def test_pale_bridge_only_near_previous_line(cfg):
    """조명에 날아간 옅은 노랑: 직전 선 둘레에서만 이어 받고, 처음부터 옅으면 안 잡는다."""
    ys = YellowSide(cfg)
    strict = to_bgr(straight_lane(cfg, 0.0, 515.0))
    pale_img = strict.copy()
    m = straight_lane(cfg, 0.0, 515.0, right=False)
    pale_img[m > 0] = (190, 235, 245)      # 왼쪽 선만 옅은 노랑 (S~57, V 245)
    fresh = YellowSide(cfg).measure(pale_img)
    assert not fresh['l_ok']                # 처음부터는 안 잡음 (새 선 안 만듦)
    for i in range(3):
        ys.step(i * 0.1, strict)
    d = ys.step(0.3, pale_img)
    assert d['l_ok'] and d['n_bridge'] >= 5
    assert d['raw_l'] == pytest.approx(257.5, abs=6)


def test_pale_far_from_previous_ignored(cfg):
    ys = YellowSide(cfg)
    for i in range(3):
        ys.step(i * 0.1, to_bgr(straight_lane(cfg, 0.0, 515.0)))
    img = to_bgr(straight_lane(cfg, 0.0, 515.0, left=False))
    m = straight_lane(cfg, -150.0, 515.0, right=False)   # 150 mm 떨어진 곳의 옅은 노랑
    img[m > 0] = (190, 235, 245)
    d = ys.measure(img)
    assert not d['l_ok']
