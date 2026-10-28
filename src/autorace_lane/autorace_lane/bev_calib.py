"""BEV 캘리브레이션 도구.

바닥에 크기를 아는 직사각형(예: 주차 칸 500x600mm, 또는 테이프로 만든 사각형)을
차 앞에 놓고 카메라 사진을 한 장 찍은 뒤, 네 꼭짓점을 지정하면 bev yaml을 만든다.

예)
  # 1) 사진 한 장 저장 (perception 패키지의 수집기)
  ros2 run autorace_perception image_collector --ros-args -p every_n:=1 -p max_images:=1
  # 2) 꼭짓점을 클릭해서 캘리브레이션 (화면이 있는 곳에서 실행)
  ros2 run autorace_lane bev_calib --image ~/autorace_images/img_00000.png \
      --rect 500 600 --near 450 --out ~/autorace_ws/src/autorace_bringup/config/bev_limo.yaml
  # 화면이 없으면 꼭짓점 픽셀을 직접 준다 (가까운-왼, 가까운-오, 먼-오, 먼-왼 순)
  ... --points 210,400 430,402 395,300 245,298

측정할 것
  --near : 제어 기준점(애커만이면 뒤차축 중심)에서 사각형의 가까운 변까지 전방 거리(mm)
  --lateral : 사각형 중심이 차량 중심선보다 오른쪽에 있으면 + (mm). 가운데 맞추면 0
  --front-axle : 제어 기준점 -> 앞차축 거리. 뒤차축 기준이면 휠베이스 200
결과 확인
  같은 폴더에 *_preview.png가 생긴다. 사각형이 반듯한 직사각형으로 보여야 한다.
"""

import argparse
import os
import sys

import cv2
import numpy as np

from autorace_lane.lane_core import BevConfig

ORDER = ('near-left', 'near-right', 'far-right', 'far-left')


def click_points(img):
    pts = []
    win = 'click 4 corners: near-left, near-right, far-right, far-left (r=reset, q=quit)'

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        view = img.copy()
        for i, p in enumerate(pts):
            cv2.circle(view, p, 5, (0, 0, 255), -1)
            cv2.putText(view, ORDER[i], (p[0] + 6, p[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if len(pts) < 4:
            cv2.putText(view, f'next: {ORDER[len(pts)]}', (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.polylines(view, [np.array(pts, np.int32)], True, (0, 255, 0), 1)
            cv2.putText(view, 'Enter: accept / r: reset', (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(win, view)
        key = cv2.waitKey(30) & 0xFF
        if key == ord('r'):
            pts.clear()
        elif key == ord('q'):
            sys.exit('aborted')
        elif len(pts) == 4 and key in (13, 32):   # Enter/Space로 확정
            break
    cv2.destroyAllWindows()
    return pts


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', required=True)
    ap.add_argument('--rect', nargs=2, type=float, metavar=('W_MM', 'L_MM'), required=True,
                    help='사각형 좌우 폭, 전후 길이 (mm)')
    ap.add_argument('--near', type=float, required=True,
                    help='기준점 -> 사각형 가까운 변 전방 거리 (mm)')
    ap.add_argument('--lateral', type=float, default=0.0)
    ap.add_argument('--points', nargs=4, default=None,
                    help='x,y 4개 (near-left near-right far-right far-left)')
    ap.add_argument('--resize', nargs=2, type=int, default=None, metavar=('W', 'H'),
                    help='노드가 쓸 처리 해상도 (기본: 사진 크기 그대로)')
    ap.add_argument('--mm-per-px', type=float, default=4.0)
    ap.add_argument('--bev', nargs=2, type=int, default=(400, 400), metavar=('W', 'H'))
    ap.add_argument('--bottom', type=float, default=None,
                    help='BEV 맨 아래 행의 전방 거리 (기본: near - 150)')
    ap.add_argument('--front-axle', type=float, default=200.0)
    ap.add_argument('--out', required=True)
    a = ap.parse_args(argv)

    img = cv2.imread(os.path.expanduser(a.image))
    if img is None:
        sys.exit(f'cannot read image: {a.image}')
    if a.resize:
        img = cv2.resize(img, tuple(a.resize), interpolation=cv2.INTER_AREA)
    if a.points:
        pts = [tuple(float(v) for v in s.split(',')) for s in a.points]
    else:
        pts = click_points(img)

    cfg = BevConfig.from_rectangle(
        pts, a.rect[0], a.rect[1], a.near, (img.shape[1], img.shape[0]),
        lateral_offset_mm=a.lateral, bev_w=a.bev[0], bev_h=a.bev[1],
        mm_per_px=a.mm_per_px, ref_to_bev_bottom_mm=a.bottom,
        ref_to_front_axle_mm=a.front_axle)
    out = os.path.expanduser(a.out)
    note = (f'rect {a.rect[0]:.0f}x{a.rect[1]:.0f}mm near {a.near:.0f}mm '
            f'lateral {a.lateral:.0f}mm points {pts}')
    cfg.to_yaml(out, note=note)

    preview = cfg.warp(img)
    for x_mm in (-a.rect[0] / 2 + a.lateral, a.rect[0] / 2 + a.lateral):
        c = int(round(cfg.vehicle_col + x_mm / cfg.mm_per_px_x))
        cv2.line(preview, (c, 0), (c, cfg.bev_h - 1), (0, 255, 0), 1)
    for y_mm in (a.near, a.near + a.rect[1]):
        r = int(round(cfg.y_mm_to_row(y_mm)))
        cv2.line(preview, (0, r), (cfg.bev_w - 1, r), (0, 255, 0), 1)
    prev_path = os.path.splitext(out)[0] + '_preview.png'
    cv2.imwrite(prev_path, preview)
    print(f'wrote {out}\npreview {prev_path} (초록 선 = 사각형이 있어야 할 자리)')


if __name__ == '__main__':
    main()
