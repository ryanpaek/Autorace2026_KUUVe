"""lane_core 오프라인 테스트 (ROS 없이 `python3 -m pytest src/autorace_lane/test`)."""

import cv2
import numpy as np
import pytest

import synth
from autorace_lane.lane_core import (BevConfig, LaneFollower, LaneParams,
                                     detect_stoplines)


def run(f, bev, frames=12, yaw=None, speed=0.3, white=None):
    out = None
    for _ in range(frames):
        out = f.process_bev(bev.copy(), white if white is not None else bev.copy(),
                            yaw=yaw, speed=speed)
    return out


@pytest.fixture
def cfg():
    return synth.make_cfg()


def test_straight_centered(cfg):
    f = LaneFollower(cfg)
    out = run(f, synth.straight_lane(cfg))
    assert out.status == 'BOTH'
    assert out.source == 'lane'
    assert abs(out.kappa) < 0.05
    assert abs(out.offset_mm) < 15


def test_lane_center_left_turns_left(cfg):
    """차로 중심이 왼쪽에 있으면 (차가 오른쪽으로 치우침) 왼쪽으로 돌아야 한다."""
    f = LaneFollower(cfg)
    out = run(f, synth.straight_lane(cfg, center_x_mm=-100.0))
    assert out.status == 'BOTH'
    assert out.offset_mm == pytest.approx(100.0, abs=15)
    assert out.kappa > 0.2          # ROS 규약: 왼쪽 +


@pytest.mark.parametrize('radius,tol', [(900.0, 0.15), (650.0, 0.25)])
@pytest.mark.parametrize('turn,sign', [('left', 1.0), ('right', -1.0)])
def test_arc_curvature(cfg, turn, sign, radius, tol):
    """원호 위에 접해 있으면 Pure Pursuit 곡률은 1/R에 가까워야 한다.

    R650은 스트레스 케이스: 안쪽 선이 BEV 밖으로 금방 빠져서 한쪽 선만 보일 수 있다.
    """
    # synth.arc_lane은 동심원 차로라 '넓어지는 코너' 보정(corner_outer_extra_mm)을 끄고 본다.
    # 실제 트랙 코너(안쪽 R600 / 바깥 R650, 비동심)는 test_tight_curve.py가 맡는다.
    f = LaneFollower(cfg, LaneParams(lookahead_mm=600.0, corner_outer_extra_mm=0.0))
    out = run(f, synth.arc_lane(cfg, radius, turn=turn), frames=20)
    assert out.status != 'NONE'
    assert out.source == 'lane'
    assert sign * out.kappa == pytest.approx(1000.0 / radius, rel=tol)
    assert sign * out.path_kappa > 0.5


def test_single_left_line_extrapolates(cfg):
    f = LaneFollower(cfg)
    out = run(f, synth.straight_lane(cfg, right=False))
    assert out.status == 'L'
    assert abs(out.offset_mm) < 30
    assert abs(out.kappa) < 0.1


def test_single_line_on_right_side(cfg):
    f = LaneFollower(cfg)
    out = run(f, synth.straight_lane(cfg, left=False))
    assert out.status == 'R'
    assert abs(out.offset_mm) < 30


def test_zigzag_symmetric_goes_straight(cfg):
    """거울상 지그재그는 행별 중점이 일직선이라 곧게 가야 한다."""
    p = LaneParams(lane_width_mm=570.0)
    f = LaneFollower(cfg, p)
    f.set_mode('zigzag')
    out = run(f, synth.zigzag_lane(cfg))
    assert out.status == 'BOTH'
    assert abs(out.offset_mm) < 30
    assert abs(out.kappa) < 0.15


def test_zigzag_one_side_only(cfg):
    p = LaneParams(lane_width_mm=570.0)
    f = LaneFollower(cfg, p)
    f.set_mode('zigzag')
    out = run(f, synth.zigzag_lane(cfg, left=False))
    assert out.status == 'R'
    assert abs(out.offset_mm) < 60
    assert abs(out.kappa) < 0.3


def test_stopline_distance(cfg):
    lane = synth.straight_lane(cfg)
    white = synth.add_stopline(synth.straight_lane(cfg), cfg, front_dist_mm=400.0)
    f = LaneFollower(cfg)
    out = run(f, lane, white=white)
    assert out.stoplines_mm, 'stop line not found'
    assert out.stoplines_mm[0] == pytest.approx(400.0, abs=10)


def test_stopline_does_not_break_lane(cfg):
    """정지선이 차선 마스크에 섞여 있어도 (흰 차선 트랙) 곧게 가야 한다."""
    bev = synth.add_stopline(synth.straight_lane(cfg), cfg, front_dist_mm=350.0)
    f = LaneFollower(cfg)
    out = run(f, bev)
    assert out.status == 'BOTH'
    assert abs(out.kappa) < 0.05
    assert out.stoplines_mm and out.stoplines_mm[0] == pytest.approx(350.0, abs=10)


def test_crosswalk_bars_are_not_stoplines(cfg):
    """진행 방향과 나란한 횡단보도 막대는 정지선으로 잡히면 안 된다."""
    white = synth.straight_lane(cfg)
    for x in (-150, -50, 50, 150):            # 폭 40mm, 길이 100mm 막대 4개
        r0 = int(cfg.y_mm_to_row(900))
        r1 = int(cfg.y_mm_to_row(800))
        c0 = int(cfg.vehicle_col + (x - 20) / 4.0)
        white[r0:r1, c0:c0 + 10] = 255
    assert detect_stoplines(white, cfg.vehicle_col, cfg, LaneParams()) == []


def test_center_diamond_marking_ignored(cfg):
    """차로 한가운데 '천천히' 마름모 표시가 있어도 차선 쌍을 뺏기지 않아야 한다."""
    bev = synth.straight_lane(cfg)
    ys = np.array([500, 600, 700, 600, 500]) + 200.0
    xs = np.array([0, 60, 0, -60, 0], dtype=float)
    synth.draw_polyline_mm(bev, cfg, xs, ys)
    f = LaneFollower(cfg)
    out = run(f, bev)
    assert out.status == 'BOTH'
    assert abs(out.kappa) < 0.08
    assert abs(out.offset_mm) < 20


def test_speckle_noise(cfg):
    rng = np.random.default_rng(0)
    bev = synth.straight_lane(cfg, center_x_mm=40.0)
    noise = rng.random(bev.shape) < 0.01
    bev[noise] = 255
    f = LaneFollower(cfg)
    out = run(f, bev)
    assert out.status == 'BOTH'
    assert out.offset_mm == pytest.approx(-40.0, abs=20)


def _add_bars(img, cfg, y0_mm, length_mm=100.0, width_mm=40.0):
    """y0_mm(기준점 전방)부터 진행 방향과 나란한 막대 4개."""
    r0 = int(round(cfg.y_mm_to_row(y0_mm + length_mm)))
    r1 = int(round(cfg.y_mm_to_row(y0_mm)))
    for x in (-150, -50, 50, 150):
        c0 = int(round(cfg.vehicle_col + (x - width_mm / 2) / cfg.mm_per_px_x))
        img[r0:r1, c0:c0 + int(width_mm / cfg.mm_per_px_x)] = 255
    return img


def test_crosswalk_line_vs_start_line(cfg):
    """막대가 뒤따르는 가로선만 횡단보도 정지선으로 표시돼야 한다."""
    white = synth.straight_lane(cfg)
    synth.add_stopline(white, cfg, front_dist_mm=250.0)          # 미션 시작선 (막대 없음)
    synth.add_stopline(white, cfg, front_dist_mm=700.0)          # 정지선
    # 정지선(앞바퀴 700mm = 기준점 900mm, 두께 20mm) 바로 뒤 10mm 띄우고 막대
    _add_bars(white, cfg, y0_mm=900.0 + 20.0 + 10.0)
    f = LaneFollower(cfg)
    out = run(f, synth.straight_lane(cfg), white=white)
    assert [round(d) for d in out.stoplines_mm] == pytest.approx([250, 700], abs=10)
    assert len(out.crosswalk_lines_mm) == 1
    assert out.crosswalk_lines_mm[0] == pytest.approx(700.0, abs=10)


def test_target_offset_shifts_left(cfg):
    """target_offset_mm(+왼쪽)를 주면 차로 중심에서 그만큼 왼쪽을 겨냥한다."""
    f = LaneFollower(cfg)
    run(f, synth.straight_lane(cfg), frames=5)
    f.p.target_offset_mm = 200.0
    out = run(f, synth.straight_lane(cfg), frames=12)
    assert out.source == 'lane'
    assert out.kappa > 0.3
    assert out.target_mm[0] == pytest.approx(-200.0, abs=15)


def test_lost_holds_then_stops(cfg):
    p = LaneParams(lost_hold_frames=5)
    f = LaneFollower(cfg, p)
    run(f, synth.straight_lane(cfg), frames=5, yaw=0.0)
    empty = synth.blank(cfg)
    sources = [f.process_bev(empty, empty, yaw=0.0).source for _ in range(7)]
    assert sources[:5] == ['hold'] * 5
    assert sources[-1] == 'stop'


def test_heading_mode_corrects_yaw(cfg):
    """yaw가 기준보다 왼쪽(+)으로 틀어져 있으면 오른쪽(-)으로 돌아야 한다."""
    f = LaneFollower(cfg)
    f.set_mode('heading', yaw_ref=0.0)
    out = run(f, synth.blank(cfg), yaw=0.2)
    assert out.source == 'heading'
    assert out.kappa < -0.3


def test_fork_left_only_follows_left_line(cfg):
    f = LaneFollower(cfg)
    f.set_mode('left_only')
    bev = synth.straight_lane(cfg)
    # 오른쪽 선이 오른쪽으로 벌어지는 갈림길: 왼쪽 선만 따라가야 한다
    ys = np.linspace(0, 3000, 50)
    synth.draw_polyline_mm(bev, cfg, 265 + 0.4 * np.clip(ys - 900, 0, None), ys)
    out = run(f, bev)
    assert out.status == 'L'
    assert abs(out.offset_mm) < 30


def test_bev_yaml_roundtrip(tmp_path, cfg):
    path = tmp_path / 'bev.yaml'
    cfg.to_yaml(str(path), note='test')
    back = BevConfig.from_yaml(str(path))
    assert np.allclose(back.H, cfg.H)
    assert back.bev_w == cfg.bev_w and back.calibrated


def test_full_pipeline_through_camera(cfg):
    """사각형 캘리브레이션 -> 원근 이미지 -> process() 전체 경로."""
    Hg = synth.camera_homography(cfg)
    # 바닥 사각형(500x600, 기준점에서 400mm 앞)의 이미지 좌표로 캘리브레이션
    corners = [(-250, 400), (250, 400), (250, 1000), (-250, 1000)]
    img_pts = [synth.project_ground(Hg, cfg, x, y) for x, y in corners]
    cal = BevConfig.from_rectangle(img_pts, 500, 600, 400, (640, 480),
                                   ref_to_bev_bottom_mm=cfg.ref_to_bev_bottom_mm)
    # 캘리브레이션 결과가 실제 카메라 모델과 같은 그리드를 주는지
    pt = synth.project_ground(Hg, cfg, 100.0, 700.0)
    g = cal.H @ np.array([pt[0], pt[1], 1.0])
    assert g[0] / g[2] == pytest.approx(cfg.vehicle_col + 25.0, abs=1.0)
    assert g[1] / g[2] == pytest.approx(cfg.y_mm_to_row(700.0), abs=1.0)
    # 카메라 이미지를 합성: 회색 바닥 + 흰 차선
    bev_img = np.full((cfg.bev_h, cfg.bev_w, 3), 60, np.uint8)
    bev_img[synth.straight_lane(cfg, center_x_mm=-60.0) > 0] = (235, 235, 235)
    cam = cv2.warpPerspective(bev_img, Hg, (640, 480), flags=cv2.INTER_LINEAR)
    f = LaneFollower(cal, LaneParams(lane_color='white'))
    out = None
    for _ in range(12):
        out = f.process(cam, speed=0.3)
    assert out.status == 'BOTH'
    assert out.offset_mm == pytest.approx(60.0, abs=20)
    assert out.kappa > 0.05


def test_mask_fn_replaces_hsv(cfg):
    """분할 모델 자리(mask_fn)에 넣은 마스크가 HSV 대신 쓰인다."""
    calls = []

    def fake_model(bgr):
        calls.append(bgr.shape)
        m = synth.straight_lane(cfg, center_x_mm=-80.0)
        return m, m
    f = LaneFollower(cfg, mask_fn=fake_model)
    blank_bgr = np.zeros((cfg.image_height, cfg.image_width, 3), np.uint8)
    out = None
    for _ in range(12):
        out = f.process(blank_bgr)
    assert calls and out.status == 'BOTH'
    assert out.offset_mm == pytest.approx(80.0, abs=15)
