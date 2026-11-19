"""11/19 실차 '앞·옆 다 막혔다고 보고 영원히 서 있음(실제로는 빠질 틈 있음)': 녹화 경로가 정해진 자리에서 양쪽 다 닿으면
막힘을 연속 3스캔 확인한 뒤 다른 길(조금 더 가서·물러나서, 녹화 / 만든 S자)을 찾아 간다."""
import math
import os

from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, load_change

PATHS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'paths')


def _m(lane=0, **kw):
    pl, pr = os.path.join(PATHS, 'm7_left.csv'), os.path.join(PATHS, 'm7_right.csv')
    L, _ = load_change(pl, pr, +1, 0.52)
    R, _ = load_change(pr, pl, -1, 0.52)
    p = ReplayParams(start_lane=lane, trigger_mm=540.0, backup_if_mm=460.0, backup_to_mm=500.0, **kw)
    m = M7Replay(p, L, R)
    m.start()
    m.scan_t = 0.0
    return m


def _limo(gap, yc):
    """라이다(차 기준 +0.10 m)에서 gap 앞에 정차 LIMO(폭 0.23, 길이 0.32). yc = 차 기준 옆."""
    x0 = 0.10 + gap
    return [(x0 + 0.02 * i, yc - 0.115 + 0.023 * j) for i in range(16) for j in range(11)]


# 2차선 54 cm 앞 + 1차선(왼쪽) 40 cm 앞 + 3차선(오른쪽) 바로 옆: 정해진 자리 녹화 경로는 양쪽 다 닿음
SCENE = _limo(0.54, 0.0) + _limo(0.30, 0.52) + _limo(0.0, -0.52)
WALL = _limo(0.54, 0.0) + _limo(0.45, 0.52) + _limo(0.45, -0.52)


def _feed(m, pts, n, t0=1.0):
    out = []
    for i in range(n):
        m.on_scan_points(t0 + 0.1 * i, pts)
        out.append(m.step(t0 + 0.1 * i))
    return out


def test_scene_is_blocked_for_fixed_recorded_paths():
    m = _m()
    _feed(m, SCENE, 3)
    d, why = m.choose(0, m.obs[0])
    assert d is None and '막힘' in why


def test_rescue_finds_a_way_after_three_blocked_scans():
    m = _m()
    out = _feed(m, SCENE, 4)
    # 장애물 확인 3스캔 + 막힘 확인 3스캔(3번째 스캔부터 셈) -> 5번째 스캔쯤. 그 전엔 정지만 (후진 없음)
    assert all(v >= 0.0 for v, _ in out)
    assert m.state != 'EXEC'
    out += _feed(m, SCENE, 3, t0=1.4)
    assert m.state == 'EXEC', m.events
    assert any('다른 길' in e for e in m.events)
    assert m.target in (1, -1) and m.status()['n_rescue'] == 1


def test_rescue_off_keeps_standing():
    m = _m(rescue=False)
    out = _feed(m, SCENE, 40)
    assert m.state == 'CRUISE' and all(v == 0.0 for v, _ in out[3:])


def test_full_wall_stands_still_no_reverse_into_anything():
    """세 차로가 다 막힘 -> 다른 길 없음, 서서 다시 봄 (뒤로 물러나 봐도 길 없으면 후진 안 함)"""
    m = _m(backup_when_blocked=False)
    out = _feed(m, WALL, 40)
    assert m.state != 'EXEC' and all(v == 0.0 for v, _ in out[3:])
    assert any('다른 길도 없음' in e for e in m.events)


def _corners(x, y, h, p):
    c, s = math.cos(h), math.sin(h)
    return [(x + c * a - s * b, y + s * a + c * b)
            for a, b in ((p.robot_front, p.robot_half_width), (p.robot_front, -p.robot_half_width),
                         (-p.robot_rear, p.robot_half_width), (-p.robot_rear, -p.robot_half_width))]


def test_rescue_path_clears_points_and_island_with_margin():
    """찾은 길을 따로 굴려 봄: 모든 자세에서 차체 + 여유 안에 라이다 점 없음, 차 모서리가 섬(+여유)에 안 들어감"""
    m = _m()
    _feed(m, SCENE, 3)
    for allow_back, fwd in ((False, True), (True, False)):
        r = m.rescue_search(allow_back, fwd=fwd)
        assert r is not None
        p = m.p
        mg = min(p.rescue_margin, p.hit_margin)
        for g in r['segs']:
            for i in range(len(g)):
                cx, cy, h = g.x[i], g.y[i], g.h[i]
                c, s = math.cos(h), math.sin(h)
                for lx, ly, _, _ in m.pts:
                    a = c * (lx - cx) + s * (ly - cy)
                    b = -s * (lx - cx) + c * (ly - cy)
                    assert not (-p.robot_rear - mg + 0.01 <= a <= p.robot_front + mg - 0.01
                                and abs(b) <= p.robot_half_width + mg - 0.01), (r['why'], i)
                for qx, qy in _corners(cx, cy, h, p):
                    assert not (qx >= p.section_length - p.rescue_island_margin + 0.01
                                and abs(qy) < p.island_half_width + p.rescue_island_margin - 0.01)
        last = r['segs'][-1]
        assert abs(last.y[-1] - m.center(r['d'])) < 0.01 and abs(last.h[-1]) < 0.01    # 목표 차로 중심, 차로 방향


def test_rescue_near_island_turned_car_never_enters_island():
    """섬 앞 2차선에서 -13° 틀어져 섬 바로 앞 (시뮬레이션에서 섬 침범했던 자세): 찾은 길은 섬 + 여유를 피함"""
    m = _m()
    L = m.p.section_length
    m.offset = (L - 0.35, 0.0, math.radians(-13.0))
    m.on_scan_points(1.0, [])
    x, y, psi = m.pose()
    assert abs(math.degrees(psi) + 13.0) < 0.5
    for allow_back, fwd in ((False, True), (True, False)):
        r = m.rescue_search(allow_back, fwd=fwd)
        if r is None:
            continue
        for g in r['segs']:
            for i in range(len(g)):
                for qx, qy in _corners(g.x[i], g.y[i], g.h[i], m.p):
                    assert not (qx >= L - 0.07 and abs(qy) < m.p.island_half_width + 0.07), r['why']


def test_s_curve_from_turned_start_ends_on_lane_center():
    m = _m()
    for h0 in (-15.0, 0.0, 12.0):
        for d in (25.0, 45.0, 60.0):
            for y1 in (0.52, -0.52):
                got = m._s_curve(1.0, 0.03, math.radians(h0), y1, d, 0.64, 0.15)
                assert got is not None
                P, used = got
                assert abs(P.y[-1] - y1) < 0.01 and abs(P.h[-1]) < 1e-9 and used <= d + 1e-6
                sg = 1.0 if y1 > 0.03 else -1.0
                assert all(sg * (yy - y1) <= 0.005 for yy in P.y)               # 목표 차로 중심 너머로 안 넘침
                assert all(sg * (0.03 - yy) <= 0.08 for yy in P.y)              # 반대로 틀어져 있으면 조금 밀렸다 돌아옴 (8 cm 안)
