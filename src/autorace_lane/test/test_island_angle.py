"""11/20 yubin: 섬 기준 모드에서 섬 변이 아닌 선(크게 기운 선, 차 반대편 선)은 안 씀."""
import numpy as np

from autorace_lane.lane_core import track_fit


class _Cfg:
    mm_per_px_x = 5.0
    mm_per_px_y = 5.0
    vehicle_col = 50.0
    bev_h = 101
    ref_to_bev_bottom_mm = 300.0


def test_track_fit_angle_and_side():
    rows = np.arange(100, 0, -10, dtype=float)
    straight = np.stack([np.full(len(rows), 100.0), rows], axis=1)          # 오른쪽 250 mm, 곧은 선
    ang, off = track_fit(straight, _Cfg())
    assert abs(ang) < 1e-6 and abs(off - 250.0) < 1e-6
    diag = np.stack([50.0 + (100 - rows), rows], axis=1)                      # 앞으로 갈수록 오른쪽 = 45°
    ang, off = track_fit(diag, _Cfg())
    assert abs(ang - 45.0) < 1e-6
    assert off < -150.0          # 맨 아래(차 앞 300 mm)에서 가운데 -> 차 기준점에서는 왼쪽 (차 앞을 가로지르는 선)
    few = straight.copy()
    few[2:, 0] = np.nan
    assert track_fit(few, _Cfg()) is None and track_fit(None, _Cfg()) is None
