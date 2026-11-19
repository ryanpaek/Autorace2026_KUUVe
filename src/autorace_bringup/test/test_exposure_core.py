"""exposure_core: 바닥 밝기로 노출 맞추기 (11/19)"""
import numpy as np

from autorace_bringup.exposure_core import ExposureController, ExposureParams, floor_v


def scene(radiance_floor, radiance_line, exposure, gamma=2.2):
    """간단한 카메라: 값 = 255 * clip(radiance * exposure)^(1/gamma). 바닥 회색 + 노란선 (B 낮음)"""
    img = np.zeros((240, 320, 3))
    img[:] = radiance_floor
    img[150:, 140:160] = radiance_line          # 노란선 (BGR, 채널별 반사율)
    v = np.clip(img * exposure, 0, 1) ** (1 / gamma) * 255
    return v.astype(np.uint8)


def run(start_exp, light, steps=40):
    p = ExposureParams()
    c = ExposureController(p, start_exp)
    exp = start_exp
    floor = np.array([0.18, 0.18, 0.18]) * light
    line = np.array([0.10, 0.90, 0.95]) * light
    for _ in range(steps):
        img = scene(floor, line, exp / 1000.0)
        new = c.next_exposure(floor_v(img, p), exp)
        if new is not None:
            exp = new
    return exp, floor_v(scene(floor, line, exp / 1000.0), p), scene(floor, line, exp / 1000.0)


def test_bright_scene_comes_down_to_target():
    exp, fv, img = run(start_exp=1000, light=4.0)       # 지금처럼 너무 밝음 (바닥 V 255 근처)
    assert abs(fv - 85) <= 8
    b, g, r = img[200, 150].astype(int)
    assert max(r, g) < 255 and b < 170                   # 노란선이 포화 안 됨


def test_dark_scene_goes_up_but_capped():
    p = ExposureParams()
    exp, fv, _ = run(start_exp=100, light=0.2)
    assert exp <= ExposureController(p, 100).max_exp     # 시작 값 1.5배 넘게 안 올림


def test_deadband_no_change():
    p = ExposureParams()
    c = ExposureController(p, 100)
    assert c.next_exposure(p.target_floor_v + 3, 100) is None


def test_step_limited():
    p = ExposureParams()
    c = ExposureController(p, 1000)
    assert c.next_exposure(250.0, 1000) >= int(1000 * p.max_down)
