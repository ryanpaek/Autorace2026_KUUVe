"""exposure_core: 출발 전 노출 고정값 계산 (11/19)"""
import numpy as np

from autorace_bringup.exposure_core import LEVEL_FLOOR_V, ExposureParams, fixed_exposure, floor_v


def scene(light, exposure, gamma=2.2):
    """간단한 카메라: 값 = 255 * clip(radiance * exposure)^(1/gamma). 위쪽 벽, 아래 바닥 + 노란선"""
    img = np.zeros((240, 320, 3))
    img[:] = 0.18 * light
    img[:100] = 0.01
    img[150:, 100:120] = np.array([0.25, 0.90, 0.95]) * light
    return (np.clip(img * exposure, 0, 1) ** (1 / gamma) * 255).astype(np.uint8)


def test_level_reaches_target_floor():
    for lv, t in LEVEL_FLOOR_V.items():
        p = ExposureParams(level=lv)
        e0, light = 1000, 1.5
        v0 = floor_v(scene(light, e0 / 1000), p)
        e = fixed_exposure(p, e0, v0)
        assert abs(floor_v(scene(light, e / 1000), p) - t) <= 3


def test_higher_level_is_darker():
    es = [fixed_exposure(ExposureParams(level=lv), 1000, 130.0) for lv in (1, 2, 3, 4)]
    assert es == sorted(es, reverse=True)


def test_level0_off_and_fixed_value():
    assert fixed_exposure(ExposureParams(level=0), 1000, 130.0) is None
    assert fixed_exposure(ExposureParams(level=0, fixed_exposure=321), 1000, 130.0) == 321


def test_dark_start_capped():
    p = ExposureParams(level=1)
    assert fixed_exposure(p, 1000, 40.0) <= 1500
