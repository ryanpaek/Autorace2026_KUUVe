"""카메라 노출 제어 계산부 (ROS 없음, 테스트용으로 분리). 11/19

배경 (11/19 bag 05·06 프레임 분석):
  - 카메라 자동 노출이 화면 전체(검은 벽 포함)로 밝기를 맞춰서, 벽이 화면을 채우면 노출이 올라감.
    노란선 R·G는 거의 모든 프레임에서 이미 255, B까지 차면 채도 S가 0이 되어 노란색 판정이 사라짐.
  - bag에는 노출 값이 안 남아 절대값은 모름. 대신 바닥(회색 매트)은 포화되지 않아 밝기 지표로 쓸 수 있음.
  - 노란선 B / 바닥 V 비율: 중앙 1.31, 상위 10% 1.77 (05·06, 44프레임).
    바닥 V 85면 노란선 B 약 150 -> S 약 105 (yellow_s_min 80 위). 바닥 V 120이면 S 약 43 (탈락).
그래서 '바닥 밝기 = target_floor_v'가 되도록 노출을 직접 맞춘다 (화면 아래쪽만 보는 자동 노출).
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ExposureParams:
    target_floor_v: float = 85.0   # 바닥 V 목표 (위 분석). 노란색 여유를 더 원하면 낮추고, 흰선이 약하면 올림
    deadband_v: float = 8.0        # |바닥 V - 목표| 가 이 안이면 그대로
    gamma: float = 2.2             # 영상 값 ~ 노출^(1/gamma) 가정 (sRGB 근사, 실제 곡선은 모름 -> kp로 천천히)
    kp: float = 0.6                # 한 번에 고칠 비율 (1.0 = 한 번에 다 고침)
    max_up: float = 1.3            # 한 번에 노출 최대 1.3배
    max_down: float = 0.6          # 한 번에 노출 최소 0.6배
    min_exposure: int = 1
    max_exposure: int = 0          # 0 = 시작 때 자동 노출 값의 1.5배
    roi_top: float = 0.55          # 화면 높이의 이 비율 아래만 (바닥)
    roi_side: float = 0.0          # 좌우 가장자리 잘라내는 비율 (0 = 안 자름)


def floor_v(bgr: np.ndarray, p: ExposureParams) -> float:
    """화면 아래쪽(바닥) 밝기 V 중앙값. 선은 면적이 작아 중앙값에 거의 영향 없음"""
    h, w = bgr.shape[:2]
    x0 = int(w * p.roi_side)
    roi = bgr[int(h * p.roi_top):, x0:w - x0:4][::4]     # 4칸씩 건너 샘플 (CPU 절약)
    v = roi.max(axis=2)                                  # HSV의 V = max(B, G, R)
    return float(np.median(v))


class ExposureController:
    def __init__(self, p: ExposureParams, start_exposure: int):
        self.p = p
        self.max_exp = p.max_exposure if p.max_exposure > 0 else max(p.min_exposure + 1, int(start_exposure * 1.5))

    def next_exposure(self, fv: float, exp: int):
        """지금 바닥 밝기 fv, 지금 노출 exp -> 새 노출 (바꿀 필요 없으면 None)"""
        p = self.p
        if fv <= 0 or abs(fv - p.target_floor_v) <= p.deadband_v:
            return None
        ratio = (p.target_floor_v / fv) ** p.gamma            # 목표 밝기까지 필요한 노출 배율
        ratio = math.exp(p.kp * math.log(ratio))             # 천천히
        ratio = min(p.max_up, max(p.max_down, ratio))
        new = int(round(exp * ratio))
        if new == exp:                                       # 정수 반올림으로 안 움직이면 1칸
            new = exp + (1 if ratio > 1 else -1)
        new = min(self.max_exp, max(p.min_exposure, new))
        return None if new == exp else new
