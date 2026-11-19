"""exposure_core: 노란선 B 채널 기준 노출 (11/19)"""
import numpy as np

from autorace_bringup.exposure_core import ExposureController, ExposureParams, Meas, measure


def scene(light, exposure, gamma=2.2):
    """간단한 카메라: 값 = 255 * clip(radiance * exposure)^(1/gamma).
    바닥 회색, 노란선(B 반사 낮음), 흰선, 위쪽 검은 벽 (측광에 안 들어가야 함)"""
    img = np.zeros((240, 320, 3))
    img[:] = np.array([0.18, 0.18, 0.18]) * light
    img[:100] = 0.01                                          # 벽
    img[150:, 100:120] = np.array([0.25, 0.90, 0.95]) * light  # 노란선 (BGR)
    img[150:, 220:240] = np.array([0.95, 0.95, 0.95]) * light  # 흰선
    v = np.clip(img * exposure, 0, 1) ** (1 / gamma) * 255
    return v.astype(np.uint8)


def run(start_exp, light, steps=40):
    p = ExposureParams()
    c = ExposureController(p, start_exp)
    exp = start_exp
    for _ in range(steps):
        new = c.next_exposure(measure(scene(light, exp / 1000.0), p), exp)
        if new is not None:
            exp = new
    return exp, scene(light, exp / 1000.0)


def test_saturated_yellow_recovers():
    # 너무 밝아 노란선이 흰색 (B 255) -> 노란선 B 160 근처, 채도 회복
    exp, img = run(1000, light=4.0)
    b, g, r = img[200, 110].astype(int)
    s = (max(b, g, r) - min(b, g, r)) / max(b, g, r) * 255
    assert b <= 165 and s >= 80


def test_ok_scene_untouched():
    # 노란선 B가 120~160 사이면 노출 그대로 (다른 색 안 건드림)
    p = ExposureParams()
    c = ExposureController(p, 100)
    assert c.next_exposure(Meas(floor_v=90, yellow_b=140, yellow_px=3000), 100) is None


def test_no_yellow_uses_floor_ceiling():
    p = ExposureParams()
    c = ExposureController(p, 100)
    assert c.next_exposure(Meas(floor_v=160, yellow_b=None), 100) < 100     # 너무 밝음 -> 내림
    assert c.next_exposure(Meas(floor_v=90, yellow_b=None), 100) is None     # 적당 -> 그대로


def test_dark_goes_up_but_capped():
    p = ExposureParams()
    exp, _ = run(100, light=0.2)
    assert exp <= ExposureController(p, 100).max_exp


def test_step_limited():
    p = ExposureParams()
    c = ExposureController(p, 1000)
    assert c.next_exposure(Meas(floor_v=200, yellow_b=255, yellow_px=5000), 1000) >= int(1000 * p.max_down)
