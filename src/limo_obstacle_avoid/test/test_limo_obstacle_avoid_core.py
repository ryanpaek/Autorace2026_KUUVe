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
    r = fork_sim.run(lay, AvoidParams(), 4.38, 'B', 1.0)            # 섬 1m 전에야 앎 → 불가, 충돌·멈춤 없이 통과
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
