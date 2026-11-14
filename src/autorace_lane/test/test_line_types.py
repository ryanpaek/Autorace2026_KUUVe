"""선 종류(노란 실선 / 흰 점선) 판단과 3차선 차로 번호 (11/10)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from autorace_lane.lane_core import BevConfig, LaneParams, classify_line, lane_id_from_types  # noqa: E402

CFG = BevConfig.from_yaml(os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup',
                                       'config', 'bev_limo.yaml'))


def _masks(col, yellow, dashed):
    h, w = CFG.bev_h, CFG.bev_w
    lane, white = np.zeros((h, w), np.uint8), np.zeros((h, w), np.uint8)
    hw = max(1, int(10 / CFG.mm_per_px_x))
    period = int(200 / CFG.mm_per_px_y)
    for r in range(h):
        if dashed and (r // (period // 2)) % 2:
            continue
        lane[r, col - hw:col + hw + 1] = 255
        if not yellow:
            white[r, col - hw:col + hw + 1] = 255
    n = 10
    track = np.array([[col, h - (i + 0.5) * h / n] for i in range(n)], float)
    return lane, white, track


def test_yellow_solid_and_white_dashed():
    p = LaneParams()
    lane, white, tr = _masks(CFG.bev_w // 3, yellow=True, dashed=False)
    a = classify_line(tr, lane, white, CFG, p)
    assert a['color'] == 'yellow' and not a['dashed']
    lane, white, tr = _masks(2 * CFG.bev_w // 3, yellow=False, dashed=True)
    b = classify_line(tr, lane, white, CFG, p)
    assert b['color'] == 'white' and b['dashed']


def test_lane_id_rules():
    Y = {'color': 'yellow', 'dashed': False}
    W = {'color': 'white', 'dashed': True}
    assert lane_id_from_types(Y, W) == 1
    assert lane_id_from_types(W, W) == 0
    assert lane_id_from_types(W, Y) == -1
    assert lane_id_from_types(Y, Y) is None and lane_id_from_types(None, W) is None


def _scene(left_mm, right_mm, yellow_side='L'):
    """노란 실선 하나 + 흰 점선 하나 (차 중심 기준 옆 위치 mm). yellow_side 쪽이 노란 실선"""
    h, w = CFG.bev_h, CFG.bev_w
    lane, white = np.zeros((h, w), np.uint8), np.zeros((h, w), np.uint8)
    hw = max(1, int(10 / CFG.mm_per_px_x))
    period = int(200 / CFG.mm_per_px_y)
    cols = {'L': int(round(CFG.vehicle_col + left_mm / CFG.mm_per_px_x)),
            'R': int(round(CFG.vehicle_col + right_mm / CFG.mm_per_px_x))}
    for side, c in cols.items():
        for r in range(h):
            if side != yellow_side and (r // (period // 2)) % 2:
                continue                                  # 흰 선은 점선
            lane[r, c - hw:c + hw + 1] = 255
            if side != yellow_side:
                white[r, c - hw:c + hw + 1] = 255
    return lane, white


def _run_scene(left_mm, right_mm, yellow_side='L', frames=6, **kw):
    from autorace_lane.lane_core import LaneFollower
    p = LaneParams(lane_width_mm=520.0, lane_color='both', **kw)
    f = LaneFollower(CFG, p)
    lane, white = _scene(left_mm, right_mm, yellow_side)
    out = None
    for _ in range(frames):
        out = f.process_bev(lane.copy(), white.copy(), yaw=0.0, speed=0.2)
    return out, p


def test_yellow_solid_guard_steers_strongly_away_from_yellow_boundary():
    """11/14: 3차선 M7(또는 일반 구간)에서 노란 실선에 가까워지면 강하게 안쪽으로 조향해 이탈을 막는다.
    (11/15: 처음 올라온 시험은 없는 함수를 써서 한 번도 돌지 않았음 -> 다시 씀)"""
    out, p = _run_scene(-260.0, 260.0)                    # 차로 가운데: 지킴 안 함
    assert out.yellow_guard is None and abs(out.kappa) < 0.2
    out, p = _run_scene(-190.0, 330.0)                    # 왼쪽 노란 실선 19 cm: 안쪽(오른쪽)으로
    assert out.yellow_guard == 'L' and out.kappa < -1.0
    out, p = _run_scene(-120.0, 400.0, frames=1)          # 12 cm: 변화율 제한 없이 첫 프레임부터 최대 조향
    assert out.yellow_guard == 'L' and abs(out.kappa + p.kappa_max) < 1e-3
    out, p = _run_scene(-400.0, 120.0, yellow_side='R', frames=1)   # 오른쪽 노란 실선도 같게 (3차로)
    assert out.yellow_guard == 'R' and abs(out.kappa - p.kappa_max) < 1e-3
    out, p = _run_scene(-120.0, 400.0, frames=3, yellow_solid_guard=False)   # 끄면 예전대로
    assert out.yellow_guard is None and out.kappa > -p.kappa_max + 0.3



def test_yellow_guard_off_on_normal_two_yellow_lines():
    """11/15 실차: 일반 구간(양쪽 노란 실선)에서 지킴이 직선 흔들림을 만듦 -> 양쪽이 다 노란 실선이면 지킴 안 함.
    한쪽만 노란 실선(M7 바깥 차로)이면 그대로 지킴"""
    def both_yellow(left_mm, right_mm, **kw):
        from autorace_lane.lane_core import LaneFollower
        p = LaneParams(lane_width_mm=520.0, lane_color='yellow', **kw)
        f = LaneFollower(CFG, p)
        lane, _ = _scene(left_mm, right_mm)
        lane2, _ = _scene(right_mm, left_mm)
        lane = np.maximum(lane, lane2)                    # 두 선 모두 노란 실선 (흰 마스크 없음)
        white = np.zeros_like(lane)
        out = None
        for _ in range(3):
            out = f.process_bev(lane.copy(), white.copy(), yaw=0.0, speed=0.2)
        return out, p
    out, p = both_yellow(-150.0, 370.0)                   # 왼쪽 노란 실선 15 cm인데 오른쪽도 노란 실선
    assert out.yellow_guard is None and out.kappa > -p.kappa_max + 0.5
    out, p = both_yellow(-150.0, 370.0, yellow_guard_one_side_only=False)   # 예전(11/14)이면 최대 조향
    assert out.yellow_guard == 'L'
    out, p = _run_scene(-150.0, 370.0, frames=3)           # 오른쪽이 흰 점선(M7 1차로)이면 지킴
    assert out.yellow_guard == 'L'
