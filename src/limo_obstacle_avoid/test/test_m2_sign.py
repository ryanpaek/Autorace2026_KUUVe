"""11/18 시험주행: 갈림길 표지판(섬 앞 꼭짓점 막대) 앞에서 서 버리던 문제."""
import math

from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, default_change
from limo_obstacle_avoid.path_track import Path


def _m2_paths(L=1.6):
    out = {}
    for k in (1, 0, -1):
        for f in ('A', 'B'):
            goal = (1 if f == 'A' else -1) * 0.52 - k * 0.52
            n = int(L / 0.02) + 1
            xs = [i * 0.02 for i in range(n)]
            Ls = 1.2
            ys = [goal * (1 - math.cos(math.pi * min(x, Ls) / Ls)) / 2 for x in xs]
            hs = [math.atan(goal * math.pi / (2 * Ls) * math.sin(math.pi * min(x, Ls) / Ls)) if x < Ls else 0.0
                  for x in xs]
            out[(k, f)] = [Path(xs, ys, hs, 1)]
    return out


def _at(x, lane=0, **kw):
    m = M7Replay(ReplayParams(**kw), default_change(1), default_change(-1))
    m.start()
    m.set_m2(_m2_paths(), {k: 't' for k in _m2_paths()})
    m.offset = (x, m.center(lane), 0.0)
    m.lane = m.target = lane
    m.scan_t = 0.0
    return m


def _scan_world(m, pts):
    """차로 좌표 점 -> 차 기준 점으로 넣기"""
    x, y, psi = m.pose()
    c, s = math.cos(psi), math.sin(psi)
    car = [(c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)) for px, py in pts]
    for _ in range(max(1, m.p.obs_confirm_frames)):      # 서 있는 물체는 스캔마다 보임 (11/19: 연속 3스캔 확인)
        m.on_scan_points(0.0, car)


def _pole(xc, yc=0.0):
    return [(xc + 0.01 * i, yc + 0.01 * j) for i in range(-2, 3) for j in range(-2, 3)]


def test_sign_at_island_tip_is_not_a_lane_obstacle():
    m = _at(3.6)
    tip = m.p.section_length
    _scan_world(m, _pole(tip - 0.05))
    assert m.obs.get(0) is None and m.sign_pts
    # 같은 자리 정차 LIMO(폭 23 cm)는 그대로 장애물
    limo = [(tip - 0.05 + 0.02 * i, -0.115 + 0.023 * j) for i in range(5) for j in range(11)]
    _scan_world(m, limo)
    assert m.obs.get(0) is not None and not m.sign_pts


def test_sign_ignored_off_keeps_old_behaviour():
    m = _at(3.6, sign_ignore=False)
    _scan_world(m, _pole(m.p.section_length - 0.05))
    assert m.obs.get(0) is not None


def test_safety_still_stops_right_before_sign():
    m = _at(3.9)
    x, _, _ = m.pose()
    _scan_world(m, _pole(x + m.p.robot_front + 0.03))        # 범퍼 3 cm 앞 (꼭짓점 구역 안)
    assert m.sign_pts
    m.state = 'EXEC'
    v, _ = m._safety(0.2, 0.0)
    assert v == 0.0 and m.stop_why


def test_center_lane_with_sign_starts_m2_instead_of_stopping():
    m = _at(0.0)
    m.offset = (m.m2_x() - 0.01, 0.0, 0.0)
    _scan_world(m, _pole(m.p.section_length - 0.03))
    m.step(1.0)
    assert m.state == 'EXEC' and m.m2_run == (0, 'A')


def test_late_m2_backs_up_to_start_when_path_hits_now():
    m = _at(0.0)
    x_m2 = m.m2_x()
    m.offset = (x_m2 + 0.35, 0.0, 0.0)                      # 늦게 옴 (late_max 0.40 안)
    # 지금 자리에 붙인 녹화 길에만 닿는 물체: 지금 자리 기준 앞 0.6 m, 왼쪽 0.20 m (녹화 시작 자리 기준이면 이미 더 왼쪽으로 지나감)
    key = (0, 'A')
    hit_now = None
    for dx in [i * 0.05 for i in range(4, 20)]:
        for dy in [j * 0.02 for j in range(0, 20)]:
            obj = [(x_m2 + 0.35 + dx + 0.01 * i, dy + 0.3 * 0 + 0.01 * jj) for i in range(-3, 4) for jj in range(-6, 7)]
            m.pts = [(a, b, 0.0, 0.0) for a, b in obj]
            if m.m2_hit(key, x_m2 + 0.35) is not None and m.m2_hit(key, x_m2) is None:
                hit_now = obj
                break
        if hit_now:
            break
    assert hit_now, '시험 배치를 못 찾음'
    for i in range(m.p.reverse_confirm_frames):          # 11/19: 후진은 연속 3스캔 같은 판단일 때만
        m.pts = [(a, b, 0.0, 0.0) for a, b in hit_now]
        m.sign_pts = []
        m.scan_seq += 1
        m.step(1.0 + 0.01 * i)
        if i < m.p.reverse_confirm_frames - 1:
            assert m.state != 'BACKUP'
    assert m.state == 'BACKUP' and m.backup['to_x'] == x_m2
    # 후진해서 시작 자리에 오면 녹화대로 시작
    m.offset = (x_m2, 0.0, 0.0)
    for t in (1.1, 3.0):
        m.step(t)
    assert m.state in ('CRUISE', 'EXEC')
    m.step(3.1)
    assert m.state == 'EXEC' and m.m2_run == key
