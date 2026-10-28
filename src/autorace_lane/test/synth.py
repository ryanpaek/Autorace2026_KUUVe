"""테스트용 합성 트랙 그림 (BEV 그리드 위에 직접 그린다)."""

import math

import cv2
import numpy as np

from autorace_lane.lane_core import BevConfig

LINE_MM = 20.0


def make_cfg(bev_w=400, bev_h=400, mm=4.0, ref_bottom=300.0) -> BevConfig:
    return BevConfig(H=np.eye(3), image_width=bev_w, image_height=bev_h,
                     bev_w=bev_w, bev_h=bev_h, mm_per_px_x=mm, mm_per_px_y=mm,
                     vehicle_col=bev_w / 2.0, ref_to_bev_bottom_mm=ref_bottom,
                     ref_to_front_axle_mm=200.0, calibrated=True)


def blank(cfg):
    return np.zeros((cfg.bev_h, cfg.bev_w), np.uint8)


def _to_px(cfg, x_mm, y_mm):
    return (cfg.vehicle_col + x_mm / cfg.mm_per_px_x, cfg.y_mm_to_row(y_mm))


def draw_polyline_mm(img, cfg, xs_mm, ys_mm, thick_mm=LINE_MM, value=255):
    pts = np.array([_to_px(cfg, x, y) for x, y in zip(xs_mm, ys_mm)])
    pts = np.round(pts * 16).astype(np.int32)  # 서브픽셀 (shift=4)
    t = max(1, int(round(thick_mm / cfg.mm_per_px_x)))
    cv2.polylines(img, [pts], False, value, t, lineType=cv2.LINE_8, shift=4)


def straight_lane(cfg, center_x_mm=0.0, width_mm=530.0, slope=0.0,
                  left=True, right=True):
    img = blank(cfg)
    ys = np.linspace(0, 3000, 50)
    c = center_x_mm + slope * ys
    if left:
        draw_polyline_mm(img, cfg, c - width_mm / 2, ys)
    if right:
        draw_polyline_mm(img, cfg, c + width_mm / 2, ys)
    return img


def arc_lane(cfg, radius_mm, turn='left', width_mm=530.0):
    """기준점에서 차량 진행 방향으로 접하며 시작하는 원호 차로."""
    img = blank(cfg)
    s = -1.0 if turn == 'left' else 1.0     # 원의 중심 x
    th = np.linspace(0, math.pi / 2, 120)
    for r in (radius_mm - width_mm / 2, radius_mm + width_mm / 2):
        xs = s * radius_mm - s * r * np.cos(th)
        ys = r * np.sin(th)
        draw_polyline_mm(img, cfg, xs, ys)
    return img


def zigzag_lane(cfg, mean_width_mm=570.0, amp_mm=60.0, period_mm=240.0,
                left=True, right=True, phase_mm=0.0):
    """좌우가 거울상인 지그재그 (폭이 mean±2*amp 사이로 변한다)."""
    img = blank(cfg)
    ys = np.arange(0, 3000, period_mm / 2.0) + phase_mm
    tri = np.where((np.arange(len(ys)) % 2) == 0, amp_mm, -amp_mm)
    if left:
        draw_polyline_mm(img, cfg, -mean_width_mm / 2 + tri, ys)
    if right:
        draw_polyline_mm(img, cfg, mean_width_mm / 2 - tri, ys)
    return img


def add_stopline(img, cfg, front_dist_mm, center_x_mm=0.0, width_mm=530.0,
                 thick_mm=LINE_MM):
    """앞바퀴에서 front_dist_mm 앞(가까운 가장자리)에 가로선을 긋는다."""
    y0 = front_dist_mm + cfg.ref_to_front_axle_mm
    r_near = cfg.y_mm_to_row(y0)
    r_far = cfg.y_mm_to_row(y0 + thick_mm)
    c0 = int(round(cfg.vehicle_col + (center_x_mm - width_mm / 2) / cfg.mm_per_px_x))
    c1 = int(round(cfg.vehicle_col + (center_x_mm + width_mm / 2) / cfg.mm_per_px_x))
    img[int(round(r_far)) + 1:int(round(r_near)) + 1, c0:c1 + 1] = 255
    return img


# --- 원근 카메라 모델 (전체 파이프라인 테스트용) ------------------------------
def camera_homography(cfg, h_mm=170.0, pitch_deg=30.0, cam_y_mm=100.0,
                      f=450.0, size=(640, 480)):
    """BEV 그리드 px -> 카메라 이미지 px 호모그래피."""
    w, hgt = size
    K = np.array([[f, 0, w / 2.0], [0, f, hgt / 2.0], [0, 0, 1.0]])
    s, c = math.sin(math.radians(pitch_deg)), math.cos(math.radians(pitch_deg))
    # 지면 (X 오른쪽, Y 전방, 1) -> 카메라 좌표 (x 오른쪽, y 아래, z 전방)
    M = np.array([[1.0, 0, 0],
                  [0, -s, s * cam_y_mm + h_mm * c],
                  [0, c, -c * cam_y_mm + h_mm * s]])
    mm = cfg.mm_per_px_x
    G = np.array([[mm, 0, -cfg.vehicle_col * mm],
                  [0, -mm, cfg.ref_to_bev_bottom_mm + (cfg.bev_h - 1) * mm],
                  [0, 0, 1.0]])
    Hg = K @ M @ G
    return Hg / Hg[2, 2]


def project_ground(Hg_grid2img, cfg, x_mm, y_mm):
    col, row = _to_px(cfg, x_mm, y_mm)
    v = Hg_grid2img @ np.array([col, row, 1.0])
    return v[:2] / v[2]
