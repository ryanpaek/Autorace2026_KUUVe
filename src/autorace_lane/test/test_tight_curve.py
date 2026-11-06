"""급곡선 폐루프 시험 (모의 맵 코너: 안쪽 선 R600, 바깥 선 R650, 동심원 아님).

10/30 실차 캘리브레이션(bev_limo.yaml)과 같은 카메라 시야로 트랙을 그려,
차선 노드 출력 곡률로 차를 움직이며 곡선을 끝까지 도는지 본다.
곡선에서는 한쪽 선만 짧고 비스듬히 보이는 때가 많다.
원호 모델 이전 코드는 곡선 진입 직후 반대로 꺾어 차선을 놓쳤다.
"""

import math

import cv2
import numpy as np
import pytest

from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams

HALF = 257.5        # 차로 폭 절반 (모의 맵 실측: 선 중심 간 515mm)
R_IN = 600.0        # 안쪽 선 코너 반경 (규정·도면)
R_OUT = 650.0       # 바깥 선 코너 반경. 동심원이 아니라 코너 가운데서 차로가 약 730mm로 넓어진다
LINE_HALF = 12.0    # 선 두께 절반 (mm)
END_X = HALF + R_IN + 800.0


def _cfg():
    pts = [(0, 401), (594, 392), (425, 263), (182, 266)]
    return BevConfig.from_rectangle(pts, 500, 600, 650, (640, 480), ref_to_bev_bottom_mm=550)


def _dist_inner(X, Y):
    """오른쪽 곡선의 안쪽(오른쪽) 선까지 거리: 북쪽 직선 x=HALF -> R_IN 코너 -> 동쪽 직선 y=R_IN."""
    cx, cy = HALF + R_IN, 0.0
    d_arc = np.abs(np.hypot(X - cx, Y - cy) - R_IN)
    in_arc = (X <= cx) & (Y >= cy)
    d = np.where(Y < cy, np.abs(X - HALF), np.where(X > cx, np.abs(Y - R_IN), d_arc))
    return np.where(in_arc, d_arc, d)


def _dist_outer(X, Y):
    """바깥(왼쪽) 선: 북쪽 직선 x=-HALF -> R_OUT 코너 -> 동쪽 직선 y=R_IN+2*HALF."""
    top = R_IN + 2 * HALF
    cx, cy = -HALF + R_OUT, top - R_OUT
    d_arc = np.abs(np.hypot(X - cx, Y - cy) - R_OUT)
    in_arc = (X <= cx) & (Y >= cy)
    d = np.where(Y < cy, np.abs(X + HALF), np.where(X > cx, np.abs(Y - top), d_arc))
    return np.where(in_arc, d_arc, d)


class Track:
    """북쪽 직선 -> 오른쪽 90도 코너 -> 동쪽 직선. 두 선은 각자 R600/R650으로 모서리를 깎았다."""

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
        on = (_dist_inner(X, Y) < LINE_HALF) | (_dist_outer(X, Y) < LINE_HALF)
        img[on] = 255
        # 카메라 시야 밖은 지운다
        cam = cv2.warpPerspective(img, np.linalg.inv(self.cfg.H), (640, 480),
                                  flags=cv2.INTER_NEAREST)
        bev = self.cfg.warp(cam)
        return np.ascontiguousarray(np.fliplr(bev)) if mirror else bev

    @staticmethod
    def lateral_error(x, y):
        """차로 가운데에서 오른쪽(안쪽)으로 벗어난 거리 (+)."""
        X, Y = np.array([x]), np.array([y])
        return float((_dist_outer(X, Y) - _dist_inner(X, Y))[0] / 2.0)


def drive(mirror=False, v=0.25, x0=0.0, params=None, delay_frames=0, blackout=None,
          route_k=None):
    """delay_frames: 명령이 바퀴에 반영되기까지의 지연 (프레임, 1프레임 = 40ms).

    blackout: (시작 프레임, 프레임 수) 동안 카메라에 선이 안 보이게 한다 (조명·반사로 놓친 상황).
    route_k: 함수 (x, y) -> 앞바퀴 기준 앞쪽 곡률 리스트. 도면 미리보기 시험용.
    """
    from collections import deque
    q = deque([0.0] * delay_frames)
    cfg = _cfg()
    tr = Track(cfg)
    f = LaneFollower(cfg, LaneParams(lane_color='yellow', lane_width_mm=2 * HALF, **(params or {})))
    x, y, hd, dt = x0, -1200.0, 0.0, 1.0 / 25.0
    errs = []
    sign = 1.0 if mirror else -1.0     # 반전 장면이면 왼쪽 곡선이므로 곡률 부호도 반전
    dist = 0.0
    for i in range(500):
        bev = tr.render(x, y, hd, mirror)
        if blackout and blackout[0] <= i < blackout[0] + blackout[1]:
            bev[:] = 0
        # 차가 보는 세계 기준 위치 (반전 장면이면 세계도 좌우 반전)
        pose = ((-x if mirror else x), y, (math.pi / 2 + hd) if mirror else (math.pi / 2 - hd))
        f.route_k = route_k(x, y) if route_k else None
        out = f.process_bev(bev.copy(), bev.copy(), speed=v, yaw=pose[2], pose=pose,
                            dist_mm=dist)
        dist += v * 1000.0 * dt
        if out.source == 'stop':
            return False, errs
        q.append(out.kappa)
        hd += sign * q.popleft() * v * dt
        x += v * 1000.0 * dt * math.sin(hd)
        y += v * 1000.0 * dt * math.cos(hd)
        errs.append(Track.lateral_error(x, y))
        if x > END_X:
            return True, errs
    return False, errs


@pytest.mark.parametrize('line_half', [12.0, 25.0], ids=['thin', 'thick'])
@pytest.mark.parametrize('mirror', [False, True], ids=['right_curve', 'left_curve'])
def test_tight_curve_completes(mirror, line_half, monkeypatch):
    # 실차 BEV에서는 20mm 테이프가 번짐 때문에 40~60mm로 보인다. 두꺼운 선은 히스토그램 봉우리가
    # 둘로 갈려 같은 선을 좌우 쌍으로 착각하던 문제(10/30 curve_v2)를 재현한다.
    monkeypatch.setattr(__import__(__name__), 'LINE_HALF', line_half)
    ok, errs = drive(mirror=mirror)
    assert ok, f'곡선 도중 정지 (최대 횡오차 {max(map(abs, errs)):.0f}mm)'
    assert max(map(abs, errs)) < 150.0


def test_old_behavior_fails():
    """원호 모델·덩어리 검출·좌우 유지가 없으면 실차처럼 곡선에서 놓친다 (회귀 기준)."""
    ok, _ = drive(params=dict(circle_fit=False, min_component_px=10 ** 9, side_track_mm=0.0))
    assert not ok


@pytest.mark.parametrize('which', ['inner', 'outer'])
def test_single_line_corner_center(which):
    """한쪽 선만 보일 때 중앙선 원은 선 원을 차로 쪽으로 반폭 평행 이동한 것 (반경 유지)."""
    from autorace_lane.lane_core import circle_centerline
    if which == 'inner':      # 오른쪽 선 = 안쪽
        c, r, side = np.array([HALF + R_IN, 0.0]), R_IN, 'R'
    else:                     # 왼쪽 선 = 바깥
        top = R_IN + 2 * HALF
        c, r, side = np.array([-HALF + R_OUT, top - R_OUT]), R_OUT, 'L'
    ang = np.radians(np.linspace(150.0, 110.0, 60))      # 북쪽에서 동쪽으로 도는 앞부분
    pts = np.stack([c[0] + r * np.cos(ang), c[1] + r * np.sin(ang)], axis=1)
    circ = circle_centerline(pts[::6], side, HALF, LaneParams(), fit_pts=pts)
    assert circ is not None
    assert circ[2] == pytest.approx(r, abs=15.0)
    assert circ[4] == 1                                  # 오른쪽 곡선
    # 중심이 차로 쪽으로 반폭만큼 옮겨졌는지
    assert math.hypot(circ[0] - c[0], circ[1] - c[1]) == pytest.approx(HALF, abs=25.0)
    # 안쪽 선이면 중심이 바깥(왼쪽 위)으로, 바깥 선이면 안쪽(오른쪽 아래)으로 옮겨진다
    moved_out = circ[0] < c[0]
    assert moved_out == (which == 'inner')


def _blackout_start():
    """코너 진입 직후(안쪽 선이 시야에서 빠지기 시작할 때) 프레임 번호."""
    return 70


@pytest.mark.parametrize('mirror', [False, True], ids=['right_curve', 'left_curve'])
def test_memory_survives_corner_blackout(mirror):
    """코너 한가운데서 0.6초 동안 선이 안 보여도 기억한 중앙선으로 계속 돈다."""
    bo = (_blackout_start(), 15)
    ok_m, errs_m = drive(mirror=mirror, blackout=bo, params=dict(memory_mm=600.0))
    assert ok_m, f'기억 켜고도 정지 (최대 횡오차 {max(map(abs, errs_m)):.0f}mm)'
    assert max(map(abs, errs_m)) < 200.0


def test_memory_better_than_hold_in_blackout():
    bo = (_blackout_start(), 15)
    _, errs_h = drive(blackout=bo)
    _, errs_m = drive(blackout=bo, params=dict(memory_mm=600.0))
    assert max(map(abs, errs_m)) <= max(map(abs, errs_h)) + 5.0


@pytest.mark.parametrize('mirror', [False, True], ids=['right_curve', 'left_curve'])
def test_lost_inner_arc_completes(mirror):
    """사라진 안쪽 선 원 이어 쓰기(11/06): 켜도 코너를 돌고 안쪽으로 파고들지 않는다."""
    ok, errs = drive(mirror=mirror, params=dict(lost_arc_mm=1000.0), delay_frames=5)
    assert ok
    assert max(errs) < 150.0 and min(errs) > -150.0
