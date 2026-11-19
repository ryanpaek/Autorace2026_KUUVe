"""카메라 노출 고정값 계산 (ROS 없음, 테스트용으로 분리). 11/19

배경 (11/19 bag 05·06·08):
  - 1번 코너 앞에서 천장 조명·전광판이 화면에서 빠지고 검은 벽이 차면 카메라 자동 노출이 노출을 올림
    -> 노란선 B 채널까지 255 -> 채도 0 -> 노란색 판정 사라짐 (05: 노란선 B 190~214 -> 255, 바닥 V 125 -> 166).
  - bag에 노출 값 로그가 없어 절대값은 모름. CPU는 주행 중 6코어 평균 77%, p95 85% (/system/usage)
    -> 주행 중 영상을 보며 계속 조정하지 않고, 출발 전에 한 번 계산해 고정.
계산:
  출발선에서 자동 노출이 고른 값 e0, 그때 바닥 밝기 V0를 읽고
  고정 노출 = e0 * (목표 바닥 V / V0)^gamma   (영상 값 ~ 노출^(1/gamma), gamma 2.2 가정)
  노출이 고정되면 바닥 밝기가 코스 내내 거의 목표값 근처 (홀 조명이 바닥에 고르게 비춘다고 가정).
단계 (05·06 프레임 99장, 노란선 B/바닥 V 비율 중앙 1.47 / 상위10% 1.88로 추정.
      '통과' = 노란선 B <= 175, 즉 R·G 포화 시 S >= 80 (yellow_s_min). 검출 = 신호등·표지판 검출 프레임 40장):
  level 0: 끔 (자동 노출 그대로)
  level 1: 바닥 V 115  노란선 픽셀 절반 이상 통과 83% / 거의 전부 63% 프레임  | 검출 프레임 밝기 x0.86
  level 2: 바닥 V 100                         96% / 82%              | x0.75
  level 3: 바닥 V  90                         99% / 95%              | x0.68
  level 4: 바닥 V  80                        100% / 98%              | x0.60
  (참고: 자동 노출 그대로일 때 출발선 바닥 V 119~132, 주행 중 중앙 137)
"""
from dataclasses import dataclass

import numpy as np

LEVEL_FLOOR_V = {1: 115.0, 2: 100.0, 3: 90.0, 4: 80.0}


@dataclass
class ExposureParams:
    level: int = 2                 # 0 = 끔, 1~4 = 위 표 (숫자가 클수록 어두움, 노란선 여유 큼)
    target_floor_v: float = 0.0    # 0이 아니면 level 대신 이 바닥 V를 목표로
    fixed_exposure: int = 0        # 0이 아니면 계산 없이 이 노출 값을 그대로 씀 (현장에서 값을 정했을 때)
    gamma: float = 2.2
    min_exposure: int = 1
    max_ratio: float = 1.5         # 자동 노출 값보다 최대 1.5배까지만 밝게
    roi_top: float = 0.55          # 화면 높이의 이 비율 아래만 (바닥)

    def target(self):
        if self.target_floor_v > 0:
            return self.target_floor_v
        return LEVEL_FLOOR_V.get(int(self.level))


def floor_v(bgr: np.ndarray, p: ExposureParams) -> float:
    """화면 아래쪽(바닥) 밝기 V 중앙값. 선은 면적이 작아 중앙값에 거의 영향 없음"""
    h = bgr.shape[0]
    roi = bgr[int(h * p.roi_top):][::4, ::4]
    return float(np.median(roi.max(axis=2)))


def fixed_exposure(p: ExposureParams, e0: int, v0: float):
    """출발선 자동 노출 값 e0, 그때 바닥 V v0 -> 고정할 노출 (None = 손대지 않음)"""
    if p.fixed_exposure > 0:
        return int(p.fixed_exposure)
    t = p.target()
    if t is None or e0 <= 0 or v0 <= 0:
        return None
    ratio = min(p.max_ratio, (t / v0) ** p.gamma)
    return max(p.min_exposure, int(round(e0 * ratio)))
