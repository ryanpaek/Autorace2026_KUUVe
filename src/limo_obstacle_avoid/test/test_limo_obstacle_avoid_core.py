"""ROS 없이 도는 회피 로직 시험.  실행: python3 -m pytest -q test  (패키지 폴더에서)"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))

from limo_obstacle_avoid import sim_common as sc
from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, Obstacle, scan_to_base, parse_exit
import stress_sim


def test_parse_exit():
    assert parse_exit('A') == 1 and parse_exit('b') == -1 and parse_exit('auto') is None


def test_scan_to_base_flip_and_offset():
    p = AvoidParams()
    p.lidar_x = 0.1
    pts = scan_to_base([1.0], math.pi / 2, 0.01, p)          # 왼쪽 1m
    assert abs(pts[0][0] - 0.1) < 1e-9 and abs(pts[0][1] - 1.0) < 1e-9
    p.scan_flip = True
    pts = scan_to_base([1.0], math.pi / 2, 0.01, p)
    assert abs(pts[0][1] + 1.0) < 1e-9
    p.scan_flip, p.scan_yaw_offset_deg = False, 180.0
    pts = scan_to_base([1.0], 0.0, 0.01, p)                 # 뒤로 장착 → 앞 물체가 뒤로 보임
    assert pts[0][0] < 0
    assert scan_to_base([float('inf'), float('nan'), 0.01], 0.0, 0.1, AvoidParams()) == [None, None, None]


def test_choose_prefers_free_lane():
    p = AvoidParams()
    p.use_island = False
    pl = AvoidPlanner(p)
    pl.start(0, 0, 0)
    obs = [Obstacle(0.8, 1.1, -0.11, 0.11), Obstacle(0.8, 1.1, 0.44, 0.66)]   # 가운데, 왼쪽 막힘
    for o in obs:
        o.hits = 5
    assert pl.choose(0, obs) == -1
    obs.append(Obstacle(0.8, 1.1, -0.66, -0.44))
    obs[-1].hits = 5
    assert pl.choose(0, obs) is None                              # 다 막히면 None (정지)


def test_transform_keeps_relations():
    p = AvoidParams()
    pl = AvoidPlanner(p)
    pl.start(0, 0, 0)
    pl.pose.x, pl.pose.y = 1.0, 0.1
    o = Obstacle(2.0, 2.3, -0.1, 0.1)
    pl.obs = [o]
    w0 = o.ymax - o.ymin
    d0 = math.hypot(2.15 - 1.0, 0.0 - 0.1)
    pl._apply_transform(math.radians(3), 0.05)
    cx, cy = (o.xmin + o.xmax) / 2, (o.ymin + o.ymax) / 2
    assert abs((o.ymax - o.ymin) - w0) < 1e-12                    # 크기 그대로
    assert abs(math.hypot(cx - pl.pose.x, cy - pl.pose.y) - d0) < 1e-9   # 차와 장애물 거리 그대로


def _run_named(layout, exit_lane='auto', **kw):
    p = AvoidParams()
    p.exit_lane = exit_lane
    return stress_sim.run(sc.LAYOUTS[layout], p, 4.3, **kw)


def test_builtin_layouts_pass():
    for lay in ('single', 'zigzag6', 'mixed7', 'diag3'):
        r = _run_named(lay)
        assert r['ok'], (lay, r)


def test_exit_preference_both_sides():
    for ex in ('A', 'B'):
        r = _run_named('mixed7', ex)
        assert r['ok'], (ex, r)


def test_heading_error_recovered_by_alignment():
    r_on = _run_named('mixed7', head_err_deg=4, dy0=0.05, delay=3)
    assert r_on['ok'], r_on
    assert r_on['off_edge'] <= 0


def test_alignment_converges():
    """출발 방향 4° 틀어진 채, 서로 다른 거리의 두 줄 정차 LIMO 를 보면 정렬 보정이 약 4° 를 찾아야 함
    (한 줄만 보이면 방향과 옆 오프셋을 구분할 수 없음 → 주행하며 다음 줄이 보일 때 수렴)"""
    import random
    p = AvoidParams()
    p.section_length = 6.0
    rects = sc.layout_rects([(1.0, 0), (1.0, 1), (2.1, -1), (2.1, 1)], p.lane_width)
    segs = sc.rects_to_segments(rects)
    pl = AvoidPlanner(p)
    yaw = math.radians(4)
    pl.start(0, 0, yaw)
    rnd = random.Random(0)
    for _ in range(40):
        rng = sc.raycast(0.1 * math.cos(yaw), 0.1 * math.sin(yaw), yaw, -math.pi, 2 * math.pi / 360, 360,
                         segs, noise=0.005, rnd=rnd)
        pl.update_scan(scan_to_base(rng, -math.pi, 2 * math.pi / 360, p))
    assert abs(math.degrees(pl.align_th) - 4.0) < 1.0, math.degrees(pl.align_th)
    assert abs(math.degrees(pl.pose.psi) - 4.0) < 1.0     # 내 방향이 차선 기준 실제 방향(+4°)으로 고쳐짐


# ---------- 갈림길(미션 2) 출구 차선 이동 ----------
def test_shift_distance_grows_with_lanes():
    from limo_obstacle_avoid.fork_exit import shift_distance
    p = AvoidParams()
    c1, s1 = shift_distance(p, 1)
    c2, s2 = shift_distance(p, 2)
    assert 0 < c1 < s1 and 0 < c2 < s2 and c2 > c1 and s2 > s1
    assert shift_distance(p, -2) == shift_distance(p, 2)


def _planner_at(x, target, exit_dir, obs=()):
    p = AvoidParams()
    p.section_length = 4.38
    pl = AvoidPlanner(p)
    pl.start(0, 0, 0)
    pl.pose.x, pl.pose.y = x, target * p.lane_width
    pl.target = target
    pl.set_exit(exit_dir)
    pl.obs = []
    for (x0, lane) in obs:
        o = Obstacle(x0, x0 + 0.32, lane * p.lane_width - 0.11, lane * p.lane_width + 0.11)
        o.hits = 5
        pl.obs.append(o)
    return pl


def test_exit_plan_decisions():
    # 1차선(+1)에서 B(-1): 2차선 이동. 섬까지 충분하면 go
    pl = _planner_at(1.5, 1, 'B')
    d, info = pl.exit_plan(pl.obstacles(True))
    assert d == 'go' and info['need_lanes'] == -2
    # 가운데 차선 앞에 차가 있으면 wait
    pl = _planner_at(1.5, 1, 'B', obs=[(2.2, 0)])
    assert pl.exit_plan(pl.obstacles(True))[0] == 'wait'
    # 출구 차선에 섬 전까지 차가 있으면 wait
    pl = _planner_at(1.0, 1, 'B', obs=[(3.6, -1)])
    assert pl.exit_plan(pl.obstacles(True))[0] == 'wait'
    # 섬 1m 전: 2차선은 too_late, 1차선(가운데에서)도 판단
    pl = _planner_at(4.38 - 1.0 - 0.2, 1, 'B')
    assert pl.exit_plan(pl.obstacles(True))[0] == 'too_late'
    pl = _planner_at(4.38 - 1.3 - 0.2, 0, 'A')
    assert pl.exit_plan(pl.obstacles(True))[0] == 'go'
    # 이미 출구 차선이면 keep
    pl = _planner_at(2.0, -1, 'B')
    assert pl.exit_plan(pl.obstacles(True))[0] == 'keep'


def test_two_lane_exit_in_simulation():
    """1차선으로 밀려난 뒤 B 를 알면 섬 앞 끝 전에 3차선(B쪽)으로 들어감, 늦으면 무리하지 않음"""
    import fork_sim
    lay = [(0.9, 0), (0.9, -1), (1.9, 0)]
    r = fork_sim.run(lay, AvoidParams(), 4.38, 'B', None)
    assert r['ok'] and not r['collided'], r
    r = fork_sim.run(lay, AvoidParams(), 4.38, 'B', 1.0)            # 섬 1m 전에야 앎: 11/12 밤부터 가운데에서 기다려 1차로만 옮김 -> B
    assert r['ok'] and r['right_side'] and not r['collided'], r
    p = AvoidParams()
    p.center_before_island = False                                  # 예전(옆 차로에서 기다림)이면 불가, 충돌·멈춤 없이 통과
    p.center_force = False
    r = fork_sim.run(lay, p, 4.38, 'B', 1.0)
    assert not r['right_side'] and not r['collided'] and r['done']


def test_side_wall_is_not_an_obstacle():
    """왼쪽 차로 바깥에 붙은 벽(앞뒤로 길게 이어진 점)은 장애물로 안 잡고, 리모(0.32m)는 잡는다 (11/10 실차: n_obs 12, 왼쪽 막힘)."""
    p = AvoidParams()
    p.use_island = False
    pl = AvoidPlanner(p)
    pl.start(0, 0, 0)
    wall = [(0.3 + 0.02 * i, 0.72) for i in range(80)]            # 0.3~1.9 m 앞, 왼쪽 0.72 m에 이어진 벽
    limo = [(1.0 + 0.02 * i, 0.0) for i in range(17)] + [(1.0, -0.1 + 0.02 * i) for i in range(11)]
    for _ in range(5):
        pl.update_scan(wall + [None] + limo)
    real = [o for o in pl.obs if not o.virtual]
    assert len(real) == 1 and abs(0.5 * (real[0].ymin + real[0].ymax)) < 0.15
    assert pl.free_ahead(1, pl.obstacles()) > 2.0                 # 왼쪽 차로는 비어 있음


# ── 11/12 밤: 가운데 우선 + 두 차로 한 번에 줄임 + 가짜 장애물 지우기 ──
def _pl_center():
    p = AvoidParams()
    p.use_island = False
    pl = AvoidPlanner(p)
    pl.start(0.0, 0.0, 0.0, 0.0)
    return pl


def test_choose_prefers_one_lane_move_to_center():
    """1차로에서 막힘: 가운데 앞 1.6 m, 3차로 앞 3 m -> 예전엔 3차로(두 칸), 이제 가운데(한 칸)"""
    pl = _pl_center()
    pl.pose.y = 0.52
    pl.target = 1
    obs = [Obstacle(1.1, 1.43, 0.42, 0.62), Obstacle(1.8, 2.13, -0.1, 0.1)]
    assert pl.choose(1, obs) == 0


def test_returns_to_center_after_passing():
    """3차로에 있고 가운데 앞이 비면 가운데로 돌아옴"""
    pl = _pl_center()
    pl.pose.y = -0.52
    pl.target = -1
    pl.last_switch = -10.0
    pl.step(5.0)
    assert pl.target == 0


def test_no_return_to_center_near_island():
    """섬 앞 2.6 m 안에서는 가운데로 안 돌아옴 (출구·표지 판단 구간)"""
    pl = _pl_center()
    pl.pose.y = -0.52
    pl.pose.x = pl.p.section_length - 2.0
    pl.exit_pref = -1                           # 출구를 알면 (모르면 표지 보려고 가운데로 모으는 기존 동작)
    pl.target = -1
    pl.last_switch = -10.0
    pl.step(5.0)
    assert pl.target == -1


def _scan_through(dist_far):
    """정면 ±10° 빔이 dist_far 에서 반사 (그 앞 1.0 m 에는 아무것도 없음)"""
    from limo_obstacle_avoid.avoid_core import scan_to_base
    import math as _m
    p = AvoidParams()
    n = 21
    a0 = _m.radians(-10)
    ai = _m.radians(1)
    return scan_to_base([dist_far] * n, a0, ai, p)


def test_forget_ghost_when_beams_pass_through():
    """가운데 1.0 m 앞에 확정된 장애물이 있었는데 빔이 그 너머 2.5 m 에서 반사 -> 몇 스캔 뒤 지움"""
    pl = _pl_center()
    g = Obstacle(1.0, 1.33, -0.11, 0.11)
    g.hits = 5
    pl.obs = [g]
    for _ in range(pl.p.forget_scans):
        pl.update_scan(_scan_through(2.5))
    assert pl.obs == []


def test_forget_also_with_no_return():
    """반사 없음(inf)도 뚫고 지나간 것으로 봄"""
    pl = _pl_center()
    g = Obstacle(1.0, 1.33, -0.11, 0.11)
    g.hits = 5
    pl.obs = [g]
    for _ in range(pl.p.forget_scans):
        pl.update_scan(_scan_through(float('inf')))
    assert pl.obs == []


def test_keep_obstacle_that_is_still_seen():
    """빔이 장애물 앞면에서 반사되면 안 지움"""
    pl = _pl_center()
    g = Obstacle(1.0, 1.33, -0.11, 0.11)
    g.hits = 5
    pl.obs = [g]
    for _ in range(10):
        pl.update_scan(_scan_through(0.92))     # 라이다(0.10) + 0.92 = 1.02 -> 앞면
    assert len(pl.obs) == 1


def test_keep_obstacle_hidden_behind_nearer_one():
    """앞에 다른 물체가 가려서 빔이 못 가면 안 지움 (가려진 것은 판단 안 함)"""
    pl = _pl_center()
    g = Obstacle(1.6, 1.93, -0.11, 0.11)
    g.hits = 5
    pl.obs = [g]
    for _ in range(10):
        pl.update_scan(_scan_through(0.6))
    assert any(abs(o.xmin - 1.6) < 0.01 for o in pl.obs)


def _confirmed(o):
    o.hits = 5
    return o


def test_return_to_left_lane_right_away_if_it_is_clear():
    """11/12 밤: 2->1차로로 옮기자마자 1차로 0.9 m 앞에 장애물. 떠난 2차로가 이미 비었으면 1.5초 안 기다리고 바로 돌아감"""
    for clear_ok, want in ((True, 0), (False, 1)):
        pl = _pl_center()
        pl.p.flip_ok_if_clear = clear_ok
        pl.p.side_wait = False
        pl.pose.y = 0.52
        pl.target = 1
        pl.flip_from = 0
        pl.last_switch = 9.5                       # 0.5초 전에 옮김
        pl.obs = [_confirmed(Obstacle(1.1, 1.43, 0.42, 0.62))]
        pl.step(10.0)
        assert pl.target == want


def test_side_wait_goes_straight_then_changes():
    """옮기기 시작했는데 가려는 차로 바로 옆(앞범퍼 근처)에 장애물이 걸리면 원래 차로로 0.3 m 직진 뒤 다시 옮김"""
    pl = _pl_center()
    pl.pose.y = 0.0
    pl.target = 1
    pl.last_switch = 9.9
    pl.obs = [_confirmed(Obstacle(0.15, 0.48, 0.42, 0.62))]     # 1차로, 차 옆~앞범퍼
    v, w = pl.step(10.0)
    assert any('직진 뒤' in e for _, e in pl.events)
    assert v > 0 and abs(w) < 0.15                              # 꺾지 않고 거의 똑바로
    assert pl.target == 1                                       # 목표는 그대로 (지나가면 다시 옮김)
    pl.pose.x += 0.35                                           # 0.3 m 지나고 장애물도 지남
    pl.obs = []
    v, w = pl.step(10.5)
    assert w > 0.2                                              # 다시 1차로 쪽으로 꺾음


def test_side_wait_off_when_obstacle_is_ahead_not_beside():
    """가려는 차로 장애물이 옆이 아니라 앞이면 기다려도 소용없음 -> side_wait 안 함"""
    pl = _pl_center()
    pl.pose.y = 0.0
    pl.target = 1
    pl.last_switch = 9.9
    pl.obs = [_confirmed(Obstacle(0.9, 1.23, 0.42, 0.62))]
    pl.step(10.0)
    assert not any('직진 뒤' in e for _, e in pl.events)


def test_merge_keeps_box_limo_sized():
    """11/12 밤 실차: 같은 LIMO를 위치가 몇 cm씩 흔들린 채 여러 번 보면 상자가 합집합으로 0.45 m까지 커지고,
    넘으면 따로 새 장애물(겹친 상자 3개). 이제 0.30 m 넘으면 최근 관측 중심으로 줄임 -> 하나, 0.30 m 이하"""
    pl = _pl_center()
    for i in range(12):
        dy = 0.02 * i                                      # 위치 추정이 조금씩 밀림 (총 22 cm)
        pl._merge([(1.0, 1.33, -0.11 + dy, 0.11 + dy)])
    real = [o for o in pl.obs if not o.virtual]
    assert len(real) == 1
    o = real[0]
    assert o.ymax - o.ymin <= 0.30 + 1e-9
    assert o.ymin <= 0.11 and o.ymax >= 0.33 - 1e-9       # 최근 관측은 안에


def _stuck_beside(raw_ymax):
    """실차 로그 재현: 1차로(+1)에 있는데 2차로 LIMO 상자가 0.44까지 커져 옆에서 막힘"""
    pl = _pl_center()
    pl.pose.x, pl.pose.y = 1.83, 0.52
    pl.target = 1
    pl.last_switch = 0.0
    g = Obstacle(1.84, 2.17, -0.01, 0.44)
    g.hits = 20
    pl.obs = [g]
    pl.last_clusters = [(1.84, 2.17, -0.11, raw_ymax)]
    return pl


def test_creep_forward_when_only_map_blocks():
    """지도 상자 때문에 막혔지만 이번 라이다로는 똑바로 0.3 m 앞이 비면 천천히 전진 (예전: 6초 넘게 정지)"""
    pl = _stuck_beside(0.11)
    v, w = pl.step(10.0)
    assert v > 0 and w == 0.0
    assert any('천천히 전진' in e for _, e in pl.events)
    pl.p.creep = False
    pl2 = _stuck_beside(0.11)
    pl2.p.creep = False
    assert pl2.step(10.0)[0] == 0.0                         # 끄면 예전처럼 정지


def test_no_creep_when_lidar_also_blocks():
    """이번 라이다로도 앞이 막혀 있으면(진짜 장애물) 전진 안 함"""
    pl = _stuck_beside(0.44)
    v, _ = pl.step(10.0)
    assert v == 0.0


# ── 11/12 밤: M7 시작 약 4000 mm(섬 앞 2.07 m)에서 일단 2차선, 옆·앞 장애물이면 0.4 m 더 가서 ──
def _pl_force(y_lane, rem):
    p = AvoidParams()
    p.center_before_island = False                  # 기존 '섬까지 다 비어야' 모으기는 빼고 이것만 봄
    pl = AvoidPlanner(p)
    pl.start(0.0, 0.0, 0.0, 0.0)
    pl.pose.y = 0.52 * y_lane
    pl.target = y_lane
    pl.last_switch = -10.0
    pl.pose.x = p.section_length - rem - p.robot_front
    return pl


def test_center_force_at_4000():
    pl = _pl_force(1, 2.0)
    pl.obs = [_confirmed(Obstacle(pl.pose.x + 1.5, pl.pose.x + 1.83, -0.11, 0.11))]   # 2차로 1.3 m 앞 (섬 전)
    pl.step(10.0)
    assert pl.target == 0
    assert any('일단 2차선' in e for _, e in pl.events)


def test_center_force_not_before_4000_or_when_exit_known():
    pl = _pl_force(-1, 2.3)                          # 아직 4000 mm 전
    pl.step(10.0)
    assert pl.target == -1
    pl = _pl_force(-1, 2.0)
    pl.exit_pref = -1                                # 표지를 이미 앎 -> 출구 차로 그대로
    pl.step(10.0)
    assert pl.target == -1


def test_center_force_waits_04_when_obstacle_beside():
    pl = _pl_force(1, 2.05)
    x0 = pl.pose.x
    pl.obs = [_confirmed(Obstacle(x0 - 0.1, x0 + 0.23, -0.11, 0.11))]   # 2차로 바로 옆
    pl.p.use_island = False
    pl.step(10.0)
    assert pl.target == 1
    assert any('0.4m 더 가서' in e for _, e in pl.events)
    pl.obs = []                                      # 지나감
    pl.pose.x = x0 + 0.2
    pl.step(10.5)
    assert pl.target == 1                            # 아직 0.4 m 안 감
    pl.pose.x = x0 + 0.41
    pl.step(11.0)
    assert pl.target == 0                            # 0.4 m 더 간 뒤 2차선으로


# ── 11/12 밤: 두 칸(1<->3) 바로 건너기 보류 / 오래 막히면 양쪽으로 찾기 ──
def _one_blocked_mid_beside(front_gap):
    """1차로(+1)에 있고 앞 front_gap 에 1차로 장애물, 2차로 장애물은 바로 옆(지나가는 중), 3차로는 빔"""
    pl = _pl_center()
    pl.pose.x, pl.pose.y = 1.0, 0.52
    pl.target = 1
    xf = pl.pose.x + pl.p.robot_front
    obs = [_confirmed(Obstacle(xf + front_gap, xf + front_gap + 0.33, 0.41, 0.63)),
           _confirmed(Obstacle(pl.pose.x + 0.10, pl.pose.x + 0.43, -0.11, 0.11))]
    return pl, obs


def test_two_lane_jump_deferred_when_middle_frees_up_soon():
    """실차: 2차로를 타야 할 때 1 -> 3차로로 두 칸을 바로 감. 앞이 0.6 m 넘게 남았으면 보류(조금 더 가서 2차로)"""
    pl, obs = _one_blocked_mid_beside(1.0)
    pl.p.two_lane_wait = False
    assert pl.choose(1, obs) == -1                      # 예전: 3차로(두 칸)
    pl.p.two_lane_wait = True
    assert pl.choose(1, obs) is None                    # 이제: 보류 (직진 계속)
    assert '보류' in pl.lane_why[-1]
    pl.p.two_lane_wait_free = 1.2                       # 앞 남은 거리(1.0)가 기준보다 짧으면 급함 -> 두 칸 허용
    assert pl.choose(1, obs) == -1


def test_search_both_sides_when_stuck_long():
    """멈춘 채 끝내지 않기: 1초 넘게 막혀 있고 똑바로는 라이다로도 막혔으면, 양쪽으로 꺾은 길 중 빈 쪽으로 천천히 전진"""
    pl = _pl_center()
    pl.pose.x, pl.pose.y = 1.0, 0.0
    pl.target = 0
    pl.last_switch = 9.9                                # 차로 다시 고르기는 아직 (hold_time)
    g = _confirmed(Obstacle(1.45, 1.78, -0.30, -0.08))  # 앞 0.25 m, 오른쪽에 걸침
    pl.obs = [g]
    pl.last_clusters = [(1.45, 1.78, -0.30, -0.08)]
    pl.p.squeeze = False                                # 지도 기준 끝까지 꺾기는 빼고 찾기만 봄
    v, w = pl.step(10.0)
    assert v == 0.0                                     # 처음엔 정지 (똑바로는 막힘)
    v, w = pl.step(11.2)
    assert v > 0 and w > 0                              # 1초 뒤: 빈 왼쪽으로 꺾어 전진
    assert any('꺾어' in e and '천천히 전진' in e for _, e in pl.events)
    pl.p.creep_search = False
    pl.creep = None
    pl.creep_n = 0
    assert pl.step(11.3)[0] == 0.0                      # 끄면 예전처럼 정지


def test_search_does_not_turn_past_30deg():
    pl = _pl_center()
    pl.pose.x, pl.pose.y, pl.pose.psi = 1.0, 0.0, math.radians(35.0)
    pl.target = 1
    pl.block_t0 = 0.0
    pl.blocked = True
    c, s_ = math.cos(pl.pose.psi), math.sin(pl.pose.psi)
    bx, by = 1.0 + 0.35 * c, 0.35 * s_                  # 차 바로 앞(진행 방향)에 상자
    pl.last_clusters = [(bx - 0.05, bx + 0.25, by - 0.15, by + 0.15)]
    out = pl._creep_step(5.0)
    assert out is None or out[1] <= 0                   # 이미 왼쪽 35° -> 왼쪽으로 더는 안 꺾음
