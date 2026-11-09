"""라바콘 디버그 그림 (위에서 본 그림, 차 기준). /obstacle/debug/compressed로 나간다.

  주황 원 = 확정 콘, 회색 원 = 아직 확인 중, 빨간 X = 카메라가 콘 아니라고 한 것
  하늘색 선 = 좌우 쌍(게이트), 초록 = 계획 경로, 분홍 점 = 지금 목표점
  노란 점선 = 차로 중심(y_lane), 흰 사각형 = 차(뒤차축 기준)
"""

from __future__ import annotations

import cv2
import numpy as np

from autorace_obstacle.cone_core import ConeFollower, Pose2D, to_vehicle

W, H, MM_PER_PX = 360, 480, 6.0
CAR_U, CAR_V = W // 2, H - 70


def _px(xv: float, yv: float):
    """차량 좌표(x 전방, y 왼쪽) -> 그림 px."""
    return int(round(CAR_U - yv / MM_PER_PX)), int(round(CAR_V - xv / MM_PER_PX))


def draw(fol: ConeFollower, pose: Pose2D, header: str = '') -> np.ndarray:
    img = np.full((H, W, 3), 40, np.uint8)
    for d in range(0, 3000, 500):                     # 0.5m 격자
        cv2.line(img, (0, _px(d, 0)[1]), (W - 1, _px(d, 0)[1]), (60, 60, 60), 1)
    cor = fol.corridor
    if cor is not None:
        # 구간 좌표 -> 월드 -> 차량
        xs = np.arange(-500.0, 3500.0, 100.0)
        lane = to_vehicle(pose, cor.to_w(np.stack([xs, np.full_like(xs, cor.y_lane)], 1)))
        for a, b in zip(lane[:-1:2], lane[1::2]):
            cv2.line(img, _px(*a), _px(*b), (0, 200, 220), 1)
        if fol.plan is not None:
            pv = to_vehicle(pose, cor.to_w(fol.plan.path))
            cv2.polylines(img, [np.array([_px(*q) for q in pv], np.int32)], False,
                          (80, 220, 80), 2)
            cz = to_vehicle(pose, cor.to_w(fol.plan.cones)) if len(fol.plan.cones) else []
            for a, b in fol.plan.gates:
                cv2.line(img, _px(*cz[a]), _px(*cz[b]), (220, 200, 60), 1)
            for i, side in fol.plan.singles:
                u, v = _px(*cz[i])
                cv2.putText(img, side, (u + 6, v + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                            (220, 200, 60), 1)
        if fol.target_c is not None:
            tv = to_vehicle(pose, cor.to_w([fol.target_c]))[0]
            cv2.circle(img, _px(*tv), 5, (200, 80, 220), -1)
    tr = fol.tracker.tracks
    if tr:
        tv = to_vehicle(pose, [[t.x, t.y] for t in tr])
        r = max(2, int(round(fol.p.cone_base_radius_mm / MM_PER_PX)))
        for t, q in zip(tr, tv):
            u, v = _px(*q)
            if t.rejected:
                cv2.line(img, (u - r, v - r), (u + r, v + r), (60, 60, 230), 2)
                cv2.line(img, (u - r, v + r), (u + r, v - r), (60, 60, 230), 2)
            elif t.confirmed:
                cv2.circle(img, (u, v), r, (0, 140, 255), -1)
            else:
                cv2.circle(img, (u, v), r, (150, 150, 150), 1)
    # 차 (뒤차축 기준 -61 ~ +261, 폭 220)
    box = [(-61, 110), (261, 110), (261, -110), (-61, -110)]
    cv2.polylines(img, [np.array([_px(*b) for b in box], np.int32)], True, (230, 230, 230), 1)
    cv2.circle(img, (CAR_U, CAR_V), 2, (230, 230, 230), -1)
    st = fol.state_dict()
    lines = [header,
             f"cones {st['confirmed']}/{st['tracks']} rej {st['rejected']}  "
             f"done {int(st['done'])}"]
    if st.get('corridor'):
        lines.append(f"x {st['progress_mm']} y {st['lateral_mm']}  gates {st['gates']} "
                     f"single {st['singles']}")
        lines.append(f"k {st['kappa']:+.2f} v {st['v']:.2f} axis {st['axis_source']}"
                     f" la x{fol.lookahead_scale:.2f}")
    for i, s in enumerate(lines):
        if s:
            cv2.putText(img, s, (6, 16 + 15 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255), 1)
    return img
