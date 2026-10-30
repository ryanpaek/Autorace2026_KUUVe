"""급곡선(차로 중심 반경 625mm) 폐루프 시험.

10/30 실차 캘리브레이션(bev_limo.yaml)과 같은 카메라 시야로 트랙을 그려,
차선 노드 출력 곡률로 차를 움직이며 곡선을 끝까지 도는지 본다.
이 곡선에서는 안쪽 선이 카메라에 안 들어오고 바깥 선만 짧고 비스듬히 보인다.
원호 모델 이전 코드는 곡선 진입 직후 반대로 꺾어 차선을 놓쳤다.
"""

import math

import cv2
import numpy as np
import pytest

from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams

RC = 625.0          # 차로 중심 반경 (도면: 안쪽 R600, 바깥 R650 선 사이)
HALF = 265.0        # 차로 폭 절반
LINE_HALF = 12.0    # 선 두께 절반 (mm)


def _cfg():
    pts = [(0, 401), (594, 392), (425, 263), (182, 266)]
    return BevConfig.from_rectangle(pts, 500, 600, 650, (640, 480), ref_to_bev_bottom_mm=550)


class Track:
    """북쪽 직선 -> 오른쪽 90도 곡선(중심 (RC, 0)) -> 동쪽 직선."""

    def __init__(self, cfg):
        self.cfg = cfg
        rows, cols = np.mgrid[0:cfg.bev_h, 0:cfg.bev_w]
        self.gx = cfg.col_to_x_mm(cols)
        self.gy = cfg.row_to_y_mm(rows)

    def render(self, x, y, hd, mirror=False):
        c, s = math.cos(hd), math.sin(hd)
        X = x + self.gx * c + self.gy * s
        Y = y - self.gx * s + self.gy * c
        img = np.zeros(self.gx.shape, np.uint8)
        r = np.hypot(X - RC, Y)
        ang = np.arctan2(Y, RC - X)
        for rr in (RC + HALF, RC - HALF):
            on = (np.abs(r - rr) < LINE_HALF) & (ang >= 0) & (ang <= math.pi / 2)
            on |= (np.abs((RC - X) - rr) < LINE_HALF) & (Y < 0)
            on |= (np.abs(Y - rr) < LINE_HALF) & (X > RC)
            img[on] = 255
        # 카메라 시야 밖은 지운다
        cam = cv2.warpPerspective(img, np.linalg.inv(self.cfg.H), (640, 480),
                                  flags=cv2.INTER_NEAREST)
        bev = self.cfg.warp(cam)
        return np.ascontiguousarray(np.fliplr(bev)) if mirror else bev

    @staticmethod
    def lateral_error(x, y):
        if y < 0:
            return x
        if x > RC:
            return RC - y
        return RC - math.hypot(x - RC, y)


def drive(mirror=False, v=0.25, x0=0.0, params=None):
    cfg = _cfg()
    tr = Track(cfg)
    f = LaneFollower(cfg, LaneParams(lane_color='yellow', **(params or {})))
    x, y, hd, dt = x0, -1200.0, 0.0, 1.0 / 25.0
    errs = []
    sign = 1.0 if mirror else -1.0     # 반전 장면이면 왼쪽 곡선이므로 곡률 부호도 반전
    for _ in range(500):
        bev = tr.render(x, y, hd, mirror)
        out = f.process_bev(bev.copy(), bev.copy(), speed=v)
        if out.source == 'stop':
            return False, errs
        hd += sign * out.kappa * v * dt
        x += v * 1000.0 * dt * math.sin(hd)
        y += v * 1000.0 * dt * math.cos(hd)
        errs.append(Track.lateral_error(x, y))
        if x > RC + 800.0:
            return True, errs
    return False, errs


@pytest.mark.parametrize('mirror', [False, True], ids=['right_curve', 'left_curve'])
def test_tight_curve_completes(mirror):
    ok, errs = drive(mirror=mirror)
    assert ok, f'곡선 도중 정지 (최대 횡오차 {max(map(abs, errs)):.0f}mm)'
    assert max(map(abs, errs)) < 150.0


def test_old_behavior_fails():
    """원호 모델·덩어리 검출·좌우 유지가 없으면 실차처럼 곡선에서 놓친다 (회귀 기준)."""
    ok, _ = drive(params=dict(circle_fit=False, min_component_px=10 ** 9, side_track_mm=0.0))
    assert not ok
