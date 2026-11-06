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
