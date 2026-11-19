"""카메라 노출 제어 계산부 (ROS 없음, 테스트용으로 분리). 11/19

배경 (11/19 bag 05·06 프레임 + /perception/detections 분석):
  - 카메라 자동 노출이 화면 전체로 밝기를 맞춤. 1번 코너 앞에서 천장 조명·전광판이 화면에서 빠지고
    검은 벽이 차면 노출을 올림 -> 노란선 B 채널까지 255 -> 채도 S 0 -> 노란색 판정 사라짐.
    (05: 1000~1140 프레임 노란선 B 190~214 -> 1160 이후 255, 바닥 V 125 -> 166)
  - 흰선·신호등 불빛·표지판 하이라이트는 원래부터 255 근처 (흰선 B 98% = 255, 신호등 bbox 13~27% 포화).
    노란선만 '포화 직전의 B 채널'이 남아 있어서 여기만 지키면 됨.
  - 화면을 통째로 어둡게(바닥 V 85) 하면 YOLO 검출 프레임 밝기가 중앙 0.64배(최소 0.41배)까지 떨어짐.
    노란선 B만 지키면 중앙 0.91배(최소 0.81배). -> 노란선 B 기준, 필요한 만큼만 낮춘다.
제어:
  - 노란선이 보이면: 노란선 B 상위 10%가 yellow_b_max 를 넘으면 노출 내림, yellow_b_min 보다 낮으면 올림
  - 노란선이 안 보이면(포화로 사라졌거나 화면에 없음): 바닥 V가 floor_v_max 넘으면 내림, floor_v_min 밑이면 올림
  - 그 사이는 그대로 (= 카메라 자동 노출이 고른 밝기 유지)
  - 측광은 화면 아래쪽만 -> 벽·천장이 들어와도 노출이 안 흔들림
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class ExposureParams:
    yellow_b_max: float = 160.0    # 노란선 B 상위10% 이 위면 내림. R·G 포화 시 S = 255 - B -> 95 (yellow_s_min 80 여유 15)
    yellow_b_min: float = 120.0    # 이 밑이면 올림 (원래 밝기 쪽으로 복귀)
    floor_v_max: float = 110.0     # 노란선이 안 보일 때만: 바닥 V 이 위면 내림 (05 실패 구간 141~166)
    floor_v_min: float = 60.0      # 노란선이 안 보일 때만: 바닥 V 이 밑이면 올림
    yellow_h: tuple = (15, 40)     # 노란선 후보 (lane params와 같은 H, S는 낮춰서 포화 직전도 잡음)
    yellow_s_min: int = 40
    yellow_min_px: int = 150       # 노란선 후보 픽셀이 이보다 적으면 '안 보임'
    gamma: float = 2.2             # 영상 값 ~ 노출^(1/gamma) 가정 (sRGB 근사, 실제 곡선 모름 -> kp로 천천히)
    kp: float = 0.6
    max_up: float = 1.25
    max_down: float = 0.7
    min_exposure: int = 1
    max_exposure: int = 0          # 0 = 시작 때 자동 노출 값의 1.5배
    roi_top: float = 0.55          # 화면 높이의 이 비율 아래만 (바닥)


@dataclass
class Meas:
    floor_v: float
    yellow_b: float = None         # 노란선 B 상위 10% (안 보이면 None)
    yellow_px: int = 0


def measure(bgr: np.ndarray, p: ExposureParams) -> Meas:
    h = bgr.shape[0]
    roi = bgr[int(h * p.roi_top):][::2, ::2]              # 2칸씩 (CPU 절약)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    v = hsv[..., 2]
    fv = float(np.median(v))
    y = ((hsv[..., 0] >= p.yellow_h[0]) & (hsv[..., 0] <= p.yellow_h[1])
         & (hsv[..., 1] >= p.yellow_s_min) & (v >= fv + 40))
    n = int(y.sum()) * 4                                   # 2칸씩 샘플 -> 원래 픽셀 수로 환산
    if n < p.yellow_min_px:
        return Meas(fv, None, n)
    return Meas(fv, float(np.percentile(roi[..., 0][y], 90)), n)


def floor_v(bgr: np.ndarray, p: ExposureParams) -> float:
    return measure(bgr, p).floor_v


class ExposureController:
    def __init__(self, p: ExposureParams, start_exposure: int):
        self.p = p
        self.max_exp = p.max_exposure if p.max_exposure > 0 else max(p.min_exposure + 1, int(start_exposure * 1.5))

    def _ratio(self, m: Meas):
        """필요한 영상 밝기 배율 (None = 그대로)"""
        p = self.p
        if m.yellow_b is not None:
            if m.yellow_b > p.yellow_b_max:
                return p.yellow_b_max / m.yellow_b
            if m.yellow_b < p.yellow_b_min and m.floor_v < p.floor_v_max:
                return p.yellow_b_min / max(m.yellow_b, 1.0)
            return None
        if m.floor_v > p.floor_v_max:
            return p.floor_v_max / m.floor_v
        if 0 < m.floor_v < p.floor_v_min:
            return p.floor_v_min / m.floor_v
        return None

    def next_exposure(self, m: Meas, exp: int):
        """지금 측정 m, 지금 노출 exp -> 새 노출 (바꿀 필요 없으면 None)"""
        p = self.p
        r = self._ratio(m)
        if r is None:
            return None
        ratio = math.exp(p.kp * p.gamma * math.log(r))     # 영상 배율 -> 노출 배율, 천천히
        ratio = min(p.max_up, max(p.max_down, ratio))
        new = int(round(exp * ratio))
        if new == exp:
            new = exp + (1 if ratio > 1 else -1)
        new = min(self.max_exp, max(p.min_exposure, new))
        return None if new == exp else new
