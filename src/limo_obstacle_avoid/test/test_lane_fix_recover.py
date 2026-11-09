"""11/09: 카메라 차선 보정(lane_fix)과 멈춤 탈출(recover) 시험 (ROS 없이)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))

from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner  # noqa: E402


def running(y0=0.0, psi0=0.0):
    pl = AvoidPlanner(AvoidParams())
    pl.start(0.0, 0.0, 0.0, 0.0)
    pl.pose.y, pl.pose.psi = y0, psi0
    return pl


def lane(off_mm, h_deg=0.0, status='BOTH', low_conf=False):
    return {'status': status, 'offset_mm': off_mm, 'heading_rad': math.radians(h_deg), 'low_conf': low_conf}


def test_lane_fix_pulls_y_to_camera():
    """추정은 차로 중앙(y=0)인데 카메라는 차로 중심이 5 cm 오른쪽(offset -50) = 차가 5 cm 왼쪽 → y가 +0.05로."""
    pl = running(0.0)
    for _ in range(60):
        assert pl.lane_fix(lane(-50.0))
    assert abs(pl.pose.y - 0.05) < 0.005
    assert pl.status()['lane_fix_n'] == 60


def test_lane_fix_uses_nearest_lane_center():
    """오른쪽 차로(-0.52)에 있고 카메라상 차로 중앙 → y가 -0.52 쪽으로."""
    pl = running(-0.47)
    for _ in range(60):
        pl.lane_fix(lane(0.0))
    assert abs(pl.pose.y - (-0.52)) < 0.005


def test_lane_fix_ignored_when_unreliable():
    for bad in (lane(-50, status='L'), lane(-50, h_deg=15.0), lane(-50, low_conf=True),
                {'status': 'BOTH', 'offset_mm': None, 'heading_rad': 0.0}):
        pl = running(0.0)
        assert not pl.lane_fix(bad)
        assert pl.pose.y == 0.0


def test_lane_fix_ignored_when_other_lane():
    """카메라 결과가 지금 추정과 15 cm 넘게 다르면 다른 차로를 본 것 → 안 씀."""
    pl = running(0.0)
    assert not pl.lane_fix(lane(-200.0))
    assert pl.pose.y == 0.0


def test_lane_fix_heading():
    """차가 차로보다 2° 왼쪽을 보고 있다(차로 방향 -2°) → psi 추정 0 이 +2° 쪽으로."""
    pl = running(0.0, 0.0)
    for _ in range(80):
        pl.lane_fix(lane(0.0, h_deg=-2.0))
    assert abs(math.degrees(pl.pose.psi) - 2.0) < 0.2


def _run_close_obstacle(recover):
    import stress_sim as ss
    p = AvoidParams()
    p.recover = recover
    # 켜는 순간 가운데 차로 장애물이 라이다 앞 0.6 m (차로를 바꾸다 모서리에 걸려 비상정지되는 거리)
    return ss.run([(0.6 + p.lidar_x, 0)], p, 4.38, seed=1, **ss.CONDITIONS[0][1])


def test_recover_gets_unstuck():
    stuck = _run_close_obstacle(False)
    assert not stuck['done'] and not stuck['collided']          # 탈출 없으면 장애물 앞에 멈춘 채 끝
    ok = _run_close_obstacle(True)
    assert ok['done'] and not ok['collided']                    # 후진하며 꺾어 다시 잡고 통과


def test_recover_side_toward_target_lane():
    """오른쪽 차로로 옮기던 중(목표 -1, 아직 가운데 근처)이면 머리를 오른쪽(-1)으로 돌린다."""
    pl = running(-0.04)
    pl.target = -1
    assert pl._recover_side(pl.obstacles(True)) == -1
