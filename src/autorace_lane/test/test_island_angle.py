"""11/20 yubin: 섬 기준 모드에서 크게 기운 선(섬 앞 사선)은 섬 변으로 안 씀."""
import math

import numpy as np

from autorace_lane.lane_core import track_angle_deg


class _Cfg:
    mm_per_px_x = 5.0
    mm_per_px_y = 5.0


def test_track_angle_deg():
    rows = np.arange(100, 0, -10, dtype=float)
    straight = np.stack([np.full(len(rows), 50.0), rows], axis=1)
    assert abs(track_angle_deg(straight, _Cfg())) < 1e-6
    diag = np.stack([50.0 + (100 - rows), rows], axis=1)          # 앞으로 갈수록 오른쪽 = 45°
    assert abs(track_angle_deg(diag, _Cfg()) - 45.0) < 1e-6
    few = straight.copy()
    few[2:, 0] = np.nan
    assert track_angle_deg(few, _Cfg()) is None and track_angle_deg(None, _Cfg()) is None
