"""11/09: 카메라 차선 보정(lane_fix)과 멈춤 탈출(recover) 시험 (ROS 없이)."""
import math

import pytest
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


def test_close_obstacle_passed_without_reverse():
    """11/12 밤: 후진 끔(기본). 켜자마자 0.6 m 앞 장애물도 끝까지 꺾어 빠져 통과 (예전엔 후진해야 통과)."""
    ok = _run_close_obstacle(False)
    assert ok['done'] and not ok['collided']
    old = _run_close_obstacle(True)                             # 예전 후진 방식도 여전히 통과 (켜면)
    assert old['done'] and not old['collided']


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
    pl.p.lane_keep_blend = 0.5
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


# ---------------------------------------------------------------- 11/10 오후 (M7 3~5단계)
def test_lane_keep_default_is_exactly_team_lane_driving():
    """기본(1.0): 차로 안 직진 조향 = 팀 차선 주행과 같은 v x kappa."""
    pl = running(0.0)
    pl.lane_fix({**lane(0.0), 'kappa': 0.37}, 10.0)
    w = pl.control(0.2, 10.1, in_lane=True)
    assert pl.blend_used and abs(w - 0.2 * 0.37) < 1e-9


def test_single_lidar_point_is_not_emergency():
    pl = running(0.0)
    pl.update_scan([(0.30, 0.0)])                                # 범퍼 앞 10cm 점 하나 (튀는 값)
    assert not pl.emergency
    pl.update_scan([(0.30, 0.0), (0.30, 0.01)])                  # 두 점 이상이면 비상
    assert pl.emergency


def test_swing_after_trail_used_up_instead_of_standing():
    """trail을 다 써도(여기선 0번) 서 있지 않고 최대 조향 후진으로 방향을 다시 잡아 통과."""
    import stress_sim as ss
    for sw, want in ((0, False), (6, True)):
        p = AvoidParams()
        p.recover, p.squeeze = True, False                     # 예전 후진 탈출 기능 자체 시험 (기본은 꺼짐)
        p.recover_max, p.recover_swing_max = 0, sw
        r = ss.run([(0.6 + p.lidar_x, 0)], p, 4.38, seed=1)
        assert r['done'] is want and not r['collided']


def test_tight_pair_no_wrong_heading_alignment():
    """앞뒤 15cm로 붙은 두 대로 방향을 재면 6° 틀어져 가장자리를 넘었다 -> 옆 위치만 보정."""
    import stress_sim as ss
    p = AvoidParams()
    r = ss.run([(0.6 + p.lidar_x, 0), (0.75 + p.lidar_x, 1)], p, 4.38, seed=1)
    assert r['ok'] and r['off_edge'] <= 0


# ---------------------------------------------------------------- 11/10 밤: 정지·후진은 바로 앞만, 후진 0.3 m
def _with_obs(pl, *boxes):
    from limo_obstacle_avoid.avoid_core import Obstacle
    for b in boxes:
        o = Obstacle(*b)
        o.hits = pl.p.confirm_hits
        pl.obs.append(o)
    return pl.obstacles(True)


def test_side_obstacle_does_not_stop_or_reverse():
    """왼쪽 차로로 옮기는 중, 그 차로 장애물이 차 '옆'에 있음 -> 예전엔 막힘(정지·후진), 이제 앞길이 비면 그대로 간다."""
    pl = running(0.0)
    pl.target = 1
    obs = _with_obs(pl, (-0.10, 0.25, 0.41, 0.63))
    assert pl.free_ahead(1, obs) < pl.p.stop_dist                  # 예전 판단이면 막힘
    v, w = pl.step(10.0)
    assert not pl.blocked and v > 0 and pl.rb is None and pl.rec is None


def test_obstacle_right_in_front_turns_hard_no_reverse():
    """앞범퍼 25 cm 앞 내 차로 장애물: 막힘으로 보고, 후진하지 않고 끝까지 꺾어 빠진다 (11/12 밤)."""
    pl = running(0.0)
    _with_obs(pl, (0.20 + 0.25, 0.80, -0.11, 0.11))
    v, w = pl.step(10.0)
    assert pl.blocked and 0.0 < pl.hit_dist <= 0.30
    assert v > 0 and pl.squeezing and abs(w) == pytest.approx(v / pl.p.min_turn_radius)
    for t in range(40):
        v, _ = pl.step(10.0 + 0.05 * t)
        assert v >= 0.0                                          # 후진 없음


def test_obstacle_in_front_beyond_0p3_does_not_stop():
    pl = running(0.0)
    _with_obs(pl, (0.20 + 0.45, 1.00, -0.11, 0.11))                # 45 cm 앞: 차로는 바꾸되 서지는 않음
    v, _ = pl.step(10.0)
    assert not pl.blocked and v > 0


def test_rollback_at_most_0p3m():
    import stress_sim as ss
    from limo_obstacle_avoid import sim_common as sc
    back = {'d': 0.0}
    orig = sc.SimCar.step

    def step(self, v, w, dt, **k):
        if v < 0:
            back['d'] += -v * dt
        orig(self, v, w, dt, **k)
    sc.SimCar.step = step
    try:
        p = AvoidParams()
        p.recover_swing_max = 0
        ss.run([(0.6 + p.lidar_x, 0), (0.75 + p.lidar_x, 1)], p, 4.38, seed=1)
    finally:
        sc.SimCar.step = orig
    assert back['d'] <= p.recover_max * (p.rollback_max_dist + 0.05)


# ---------------------------------------------------------------- 11/10 밤 2: 1->2차선 중 3차선 장애물
def _diag(y=0.10, deg=-40.0, band=True):
    pl = running(y, math.radians(deg))
    pl.target = 0                                                  # 1차선에서 2차선으로 들어오는 중
    pl.p.emerg_lane_band = band
    return pl


def test_lane3_points_ignored_for_emergency_while_moving_1_to_2():
    """오른쪽으로 비스듬히(-40°) 2차선에 들어올 때 범퍼 앞 박스에 3차선 장애물 점이 들어옴 -> 예전엔 비상, 이제 무시."""
    pts = [(0.45, -0.10), (0.46, -0.11), (0.47, -0.10)]            # 지도 y 약 -0.32 (3차선)
    old = _diag(band=False)
    old.update_scan(pts)
    assert old.emergency
    new = _diag()
    new.update_scan(pts)
    assert not new.emergency


def test_points_in_target_lane_still_emergency():
    pts = [(0.30, 0.0), (0.31, 0.01), (0.32, 0.0)]                 # 지도 y 약 -0.10 (2차선, 가려는 차로)
    pl = _diag()
    pl.update_scan(pts)
    assert pl.emergency


def test_sharper_steering_after_reverse_until_new_lane():
    pl = running(0.45)                                              # 목표 차로까지 7 cm (최대 조향에 안 걸리게)
    pl.sharp = True
    pl.target = 1
    w_sharp = pl.control(0.2, None, False)
    pl.sharp = False
    w_norm = pl.control(0.2, None, False)
    assert w_sharp > w_norm > 0                                     # 후진 뒤엔 더 크게 꺾음
    pl.sharp = True
    pl.pose.y = 0.52                                                # 새 차로 도착
    pl.step(10.0)
    assert pl.sharp is False


def _mid_change(section=4.3, block_next=False, psi_deg=-20.0, **kw):
    """1차선(+1)에서 2차선(0)으로 옮기는 도중(y 0.35), 2차선 0.7 m 앞에 장애물이 막 보임."""
    from limo_obstacle_avoid.avoid_core import Obstacle
    p = AvoidParams(section_length=section, **kw)
    pl = AvoidPlanner(p)
    pl.start(0.0, 0.0, 0.0, 0.0)
    pl.pose.x, pl.pose.y, pl.pose.psi = 1.0, 0.35, math.radians(psi_deg)
    pl.target, pl.last_switch = 0, 0.9
    obs = [Obstacle(1.9, 2.22, -0.11, 0.11)]
    if block_next:
        obs.append(Obstacle(2.0, 2.32, -0.63, -0.41))
    for o in obs:
        o.hits = p.confirm_hits
    pl.obs = obs
    pl.step(1.0)
    return pl


def test_continue_to_next_lane_in_one_go():
    pl = _mid_change()
    assert pl.target == -1
    assert any('이어서' in e for _, e in pl.events)


def test_continue_needs_safety_conditions():
    assert _mid_change(section=2.4).target != -1            # 섬까지 1.7 m 안
    assert _mid_change(block_next=True).target != -1         # 다음 차로도 막힘
    assert _mid_change(psi_deg=-45.0).target != -1           # 이미 옮기는 쪽으로 40° 넘게 틀어짐
    assert _mid_change(cont_change=False).target != -1       # 끄면 예전대로


def test_lane_fix_half_while_changing_and_up_to_20deg():
    """11/12: 옮기는 중(목표 차로 중심 밖)이어도 양쪽 선이 보이면 반만 반영. 20° 넘으면 안 씀."""
    a = running(y0=0.0)
    a.lane_fix(lane(-60.0), 0.0)                  # 같은 차로, 똑바로 -> 전부
    b = running(y0=0.0)
    b.target = 1                                  # 1차선으로 옮기는 중
    b.lane_fix(lane(-60.0), 0.0)
    assert b.pose.y == pytest.approx(0.5 * a.pose.y, rel=0.05) and b.pose.y > 0
    c = running(y0=0.0)
    assert c.lane_fix(lane(95.0, h_deg=15.0), 0.0)         # 15° (차 옆 위치 약 +5 cm): 반만
    d = running(y0=0.0)
    assert not d.lane_fix(lane(-60.0, h_deg=25.0), 0.0)    # 25°: 안 씀


def test_edge_guard_turns_inward_near_outer_line():
    """1차선(왼쪽 끝)에서 바깥(왼쪽)으로 25° 향해 있으면 오른쪽으로 끝까지 꺾음."""
    pl = running(y0=0.60, psi0=math.radians(25.0))
    pl.target = 1
    w = pl.control(0.3, None, False)
    assert w < 0 and pl.edge_guarded
    ok = running(y0=0.52, psi0=0.0)
    ok.target = 1
    ok.control(0.3, None, True)
    assert not ok.edge_guarded


def test_choose_ignores_obstacle_being_passed_beside():
    """1차선에서 2차선 장애물 옆을 지나는 중, 1차선 앞 0.9 m 장애물 -> 2차선을 고름 (옆 장애물은 '앞'이 아님)."""
    from limo_obstacle_avoid.avoid_core import Obstacle
    p = AvoidParams()
    pl = AvoidPlanner(p)
    pl.start(0.0, 0.0, 0.0, 0.0)
    pl.pose.x, pl.pose.y, pl.pose.psi = 1.0, 0.52, 0.0
    pl.target = 1
    obs = [Obstacle(0.75, 1.07, -0.11, 0.11), Obstacle(2.1, 2.42, 0.41, 0.63), Obstacle(0.75, 1.07, -0.63, -0.41)]
    for o in obs:
        o.hits = p.confirm_hits
    assert pl.choose(1, obs) == 0
    p.clear_rollout = False
    assert pl.choose(1, obs) is None                 # 예전: 옆 장애물 때문에 못 고름


def test_done_waits_until_straight():
    """섬 끝 + done_margin을 지나도 차가 8° 넘게 비스듬하면 아직 안 끝냄 (차선 노드가 비스듬한 채 받지 않게)."""
    pl = running(0.52, math.radians(15.0))
    pl.target = 1
    pl.pose.x = pl.p.section_length + pl.p.done_margin + 0.05
    pl.step(10.0)
    assert pl.state == 'RUN'
    ok = running(0.52, math.radians(3.0))
    ok.target = 1
    ok.pose.x = ok.p.section_length + ok.p.done_margin + 0.05
    ok.step(10.0)
    assert ok.state == 'DONE'


def test_lane_keep_blend_single_line():
    """11/12 밤: 차로 안에서 한쪽 선(R)만 보여도 차선 노드 곡률을 0.7 섞는다 (예전엔 BOTH일 때만)."""
    pl = running(0.0)
    pl.lane_last, pl.lane_t = {'status': 'R', 'kappa': -1.0}, 0.0
    w = pl.control(0.2, 0.05, True)
    assert pl.blend_used and w == pytest.approx(0.2 * 0.7 * -1.0, abs=0.02)


def test_no_flip_back_to_lane_just_left():
    """11/12 밤 실차: -1 차로에서 0으로 옮기자마자 0.4초 뒤 다시 -1로 (반복). 방금 떠난 차로로 1.5초 안에는 안 돌아감."""
    from limo_obstacle_avoid.avoid_core import Obstacle
    p = AvoidParams()
    pl = AvoidPlanner(p)
    pl.start(0.0, 0.0, 0.0, 0.0)
    pl.pose.x, pl.pose.y = 1.0, -0.50
    pl.target, pl.flip_from, pl.last_switch = 0, -1, 9.8        # 방금 -1 -> 0 으로 바꿈
    o = Obstacle(2.0, 2.32, -0.11, 0.11)                        # 0 차로 앞 0.8 m (막혔다고 보지만 정말 붙은 건 아님)
    o2 = Obstacle(1.6, 1.92, 0.41, 0.63)                        # +1 차로도 막힘 (이어서 +1로 가는 것도 안 됨)
    o.hits = o2.hits = p.confirm_hits
    pl.obs = [o, o2]
    pl.step(10.0)
    assert pl.target == 0                                       # 예전: 바로 -1로 되돌아감
    pl.step(11.5)
    assert pl.target == -1                                      # 1.5초 지나면 허용
