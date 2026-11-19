"""11/19 실차 요청: 앞·옆 막힘이면 앞 장애물까지 라이다 거리를 blocked_align_mm(400)로 앞뒤로 맞춘 뒤 녹화 경로를 무조건 재생.
1차선 -> 오른쪽(right csv), 3차선 -> 왼쪽(left csv) 고정. 2차선은 틈 검사(앞 장애물 빼고 옆 차로 물체만, 차체 그대로)를 통과한 쪽."""
import math
import os

from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, load_change

PATHS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'paths')
LW = 0.52


def _m(lane=0, **kw):
    pl, pr = os.path.join(PATHS, 'm7_left.csv'), os.path.join(PATHS, 'm7_right.csv')
    L, _ = load_change(pl, pr, +1, LW)
    R, _ = load_change(pr, pl, -1, LW)
    p = ReplayParams(start_lane=lane, trigger_mm=540.0, backup_if_mm=460.0, backup_to_mm=500.0, **kw)
    m = M7Replay(p, L, R)
    m.start()
    m.scan_t = 0.0
    return m


def _box(x0, lane, length=0.32, w=0.23):
    """차로 좌표: 앞면 x0, 차로 lane 가운데 정차 LIMO 둘레 점"""
    yc = lane * LW
    pts = [(x0 + length * i / 15, yc + sg * w / 2) for i in range(16) for sg in (-1, 1)]
    pts += [(x0, yc - w / 2 + w * j / 10) for j in range(11)]
    return pts


def _to_car(m, world):
    x, y, psi = m.pose()
    c, s = math.cos(psi), math.sin(psi)
    return [(c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)) for px, py in world]


def _drive(m, world, n=200, dt=0.05, scan_every=2, stop_at=None):
    """명령 속도대로 차를 차로 방향으로 움직이며 (조향은 무시) 스캔·판단. stop_at(m) True면 멈춤."""
    t = 1.0
    log = []
    for i in range(n):
        if i % scan_every == 0:
            m.on_scan_points(t, _to_car(m, world))
        v, k = m.step(t)
        dx, dy, dpsi = m.offset
        m.offset = (dx + v * dt, dy, dpsi)
        log.append((m.state, round(v, 3)))
        t += dt
        if stop_at and stop_at(m):
            break
    return log


def _gap(m):
    x, _, _ = m.pose()
    return m.obs.get(m.lane) - x - m.p.lidar_x


def test_lane3_blocked_aligns_to_400_then_runs_left_csv():
    m = _m(lane=-1)
    # 3차선 앞 55 cm, 2차선 조금 앞(앞면 x 0.2)에 LIMO -> 왼쪽 녹화 경로가 닿음 (3차선은 오른쪽이 없음)
    world = _box(0.10 + 0.55, -1) + _box(0.2, 0)
    _drive(m, world, stop_at=lambda mm: mm.state == 'EXEC')
    assert m.state == 'EXEC', m.events
    assert m.target == 0                                   # 왼쪽(left csv): 3차선 -> 2차선
    assert abs(_gap(m) - 0.40) < 0.03, _gap(m)
    assert any('맞춤 끝' in e and '왼쪽(left csv) 고정' in e for e in m.events), m.events


def test_lane1_blocked_runs_right_csv_only():
    m = _m(lane=1)
    world = _box(0.10 + 0.55, 1) + _box(0.2, 0)
    _drive(m, world, stop_at=lambda mm: mm.state == 'EXEC')
    assert m.state == 'EXEC' and m.target == 0               # 오른쪽(right csv): 1차선 -> 2차선
    assert abs(_gap(m) - 0.40) < 0.03


def test_center_lane_picks_side_that_passes_gap_check():
    """2차선: 양쪽 다 정해진 자리 녹화 경로로는 막힘. 왼쪽 LIMO는 조금 앞(400 mm 자리에선 틈 통과), 오른쪽 LIMO는 더 앞(틈 미달)
    -> 정한 쪽이 B(오른쪽)여도 틈 통과한 왼쪽"""
    m = _m(lane=0, choose_tie_side='B')
    world = _box(0.10 + 0.55, 0) + _box(0.10, 1) + _box(0.25, -1)
    _drive(m, world, stop_at=lambda mm: mm.state == 'EXEC')
    assert m.state == 'EXEC' and m.target == 1, m.events
    assert any('틈 통과한 쪽' in e for e in m.events)


def test_aligns_backward_when_closer_than_target():
    m = _m(lane=-1, blocked_align_mm=530.0)                # 막혀 선 자리(약 495 mm)보다 먼 거리로 맞춤 -> 후진
    world = _box(0.10 + 0.535, -1) + _box(0.2, 0)
    log = _drive(m, world, stop_at=lambda mm: mm.state == 'EXEC')
    assert any(s == 'ADJUST' and v < 0 for s, v in log)       # 뒤로 맞춤
    assert m.state == 'EXEC' and abs(_gap(m) - 0.53) < 0.03, _gap(m)


def test_single_blocked_scan_does_not_move():
    """막힘(옆 차로 물체)이 한 스캔만 -> 맞추기 시작 안 함"""
    m = _m(lane=-1)
    front = _box(0.10 + 0.55, -1)
    t = 1.0
    for pts in (front, front, front + _box(0.2, 0), front, front):
        m.on_scan_points(t, _to_car(m, pts))
        m.step(t)
        t += 0.1
        assert m.state != 'ADJUST'


def test_cap_then_stands_like_before():
    m = _m(lane=-1, blocked_align_max=0, backup_when_blocked=False)
    world = _box(0.10 + 0.55, -1) + _box(0.2, 0)
    log = _drive(m, world, n=80)
    assert all(s == 'CRUISE' for s, _ in log[10:]) and all(v == 0.0 for _, v in log[20:])


def test_off_switch_keeps_old_behaviour():
    m = _m(lane=-1, blocked_align=False, backup_when_blocked=False)
    world = _box(0.10 + 0.55, -1) + _box(0.2, 0)
    log = _drive(m, world, n=80)
    assert 'ADJUST' not in [s for s, _ in log] and m.state == 'CRUISE'


def test_beside_obstacle_is_not_an_empty_lane():
    """bag 보고서: 옆 차로 LIMO가 차 옆(gap -41 mm)인데 '양쪽 비어 있음'으로 그 차로를 고름 -> 막힘"""
    m = _m(lane=0, beside_blocks=True)
    m.pts = []
    x, _, _ = m.pose()
    m.obs = {0: x + 0.10 + 0.54, 1: x + 0.10 - 0.041, -1: None}
    d, why = m.choose(0, m.obs[0])
    assert d == -1
    m.obs = {0: x + 0.10 + 0.54, 1: x + 0.10 - 0.041, -1: x + 0.10 - 0.049}
    d, why = m.choose(0, m.obs[0])
    assert d is None and '옆에 붙어' in why


def test_both_beside_is_a_tie_not_a_coin_flip():
    """기본(beside_blocks 끔): 양쪽 -41/-49 mm(4차 100.9초) -> '더 멀리 빈 쪽' 대신 정한 쪽(또는 출구 쪽)"""
    m = _m(lane=0, choose_tie_side='B')
    m.pts = []
    x, _, _ = m.pose()
    m.obs = {0: x + 0.10 + 0.54, 1: x + 0.10 - 0.041, -1: x + 0.10 - 0.049}
    d, why = m.choose(0, m.obs[0])
    assert d == -1 and '정한 쪽(B)' in why


def test_tie_goes_to_fixed_side_or_exit():
    m = _m(lane=0, choose_tie_side='B')
    m.pts = []
    x, _, _ = m.pose()
    m.obs = {0: x + 0.64, 1: x + 2.0, -1: x + 1.934}          # 66 mm 차이 (4차 84.7초)
    d, why = m.choose(0, m.obs[0])
    assert d == -1 and '정한 쪽(B)' in why
    m.set_exit('A')
    d, why = m.choose(0, m.obs[0])
    assert d == 1 and '출구' in why
    m.set_exit('auto')
    m.obs = {0: x + 0.64, 1: x + 2.5, -1: x + 1.9}            # 60 cm 차이 -> 더 빈 쪽
    d, why = m.choose(0, m.obs[0])
    assert d == 1 and '더 멀리' in why
