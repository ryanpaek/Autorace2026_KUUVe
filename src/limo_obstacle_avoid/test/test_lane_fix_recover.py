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


# ---------------------------------------------------------------- 11/10 추가
def test_lane_id_moves_pose_to_camera_lane():
    """가운데 차로라고 알고 시작했는데 카메라 선 종류가 계속 1차선(왼쪽 노란 실선) → 옆 위치를 1차선으로 옮김."""
    pl = running(0.0)
    for _ in range(4):
        pl.lane_fix({**lane(0.0), 'lane_id': 1}, 0.0)
    assert pl.nearest_lane(pl.pose.y) == 0                       # 아직 4번: 안 옮김
    pl.lane_fix({**lane(0.0), 'lane_id': 1}, 0.0)
    assert abs(pl.pose.y - 0.52) < 0.01 and pl.target == 1
    assert pl.status()['lane_id_fix_n'] == 1


def test_lane_id_ignored_while_changing_lane():
    pl = running(0.0)
    pl.target = -1                                               # 오른쪽으로 옮기는 중
    for _ in range(8):
        pl.lane_fix({**lane(0.0), 'lane_id': 1}, 0.0)
    assert pl.lane_id_fix_n == 0


def test_small_and_huge_clusters_ignored():
    pl = running(0.0)
    pole = [(1.0, 0.02 * i) for i in range(3)]                   # 4cm 기둥
    cross_wall = [(1.5, -0.5 + 0.02 * i) for i in range(50)]      # 앞을 가로지르는 1m 벽
    for _ in range(4):
        pl.update_scan(pole + [None] + cross_wall)
    assert not [o for o in pl.obs if not o.virtual]


def test_lane_keep_blends_lane_node_kappa():
    pl = running(0.0)
    pl.lane_fix({**lane(0.0), 'kappa': 0.5}, 10.0)
    w_blend = pl.control(0.2, 10.1, in_lane=True)
    assert pl.blend_used and abs(w_blend - 0.2 * 0.25) < 1e-6    # 회피 곡률 0 + 차선 0.5 x 0.5
    pl.control(0.2, 11.0, in_lane=True)                         # 차선 값이 오래됨 -> 안 섞음
    assert not pl.blend_used


def _run_layout(layout, **kw):
    import stress_sim as ss
    p = AvoidParams()
    return ss.run(layout, p, 4.38, seed=1, **{**ss.CONDITIONS[0][1], **kw})


def test_rollback_finds_other_lane():
    """가운데·왼쪽이 붙어서 막혀 있으면 온 길로 물러났다가 비어 있는 오른쪽 차로로 간다."""
    from limo_obstacle_avoid.avoid_core import AvoidParams as P
    lx = P().lidar_x
    r = _run_layout([(0.6 + lx, 0), (0.75 + lx, 1)])
    assert r['done'] and not r['collided']


def _run_capture(layout, island, **over):
    """stress_sim으로 돌리고 플래너(이벤트 기록)도 돌려준다."""
    import stress_sim as ss
    p = AvoidParams()
    for k, v in over.items():
        setattr(p, k, v)
    cap = {}
    orig = AvoidPlanner.__init__

    def init(self, *a, **k):
        orig(self, *a, **k)
        cap['pl'] = self
    AvoidPlanner.__init__ = init
    try:
        r = ss.run(layout, p, island, seed=0)
    finally:
        AvoidPlanner.__init__ = orig
    return r, cap['pl']


# 지나갈 길이 없는 배치 (줄 간격이 좁아 앞줄을 지나서는 차로를 못 바꿈, 11/10 시뮬레이션)
_DEAD_END = [(0.9, 0), (2.256, 1), (3.222, -1), (3.222, 1), (4.148, 0)]


def test_rollback_not_limited_to_three():
    """11/10 실차: 3번 후진하면 그 뒤로 멈춰 버림 -> 기본은 횟수 제한 없음."""
    _, pl = _run_capture(_DEAD_END, 5.27)
    assert AvoidParams().recover_max == 0
    assert pl.rec_n > 3 or pl.rb is not None and pl.rec_n == 3


def test_rollback_goes_further_when_blocked_at_same_spot():
    """같은 자리에서 또 막히면 매번 더 멀리 후진한다 (1.5 -> 2.5 -> ...)."""
    _, pl = _run_capture(_DEAD_END, 5.27)
    starts = [e for _, e in pl.events if '온 길 되돌아가기' in e]
    assert len(starts) >= 2
    assert '최대 1.5m' in starts[0] and '최대 2.5m' in starts[1] and '같은 자리 2번째' in starts[1]


def test_rollback_limit_still_works_when_set():
    _, pl = _run_capture(_DEAD_END, 5.27, recover_max=3)
    assert pl.rec_n == 3
