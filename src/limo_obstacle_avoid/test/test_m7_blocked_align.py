"""11/19 실차: 앞·옆 막힘이면 앞 장애물까지 녹화 CSV가 실제로 지나가는 거리(470~650 mm, 차체 그대로)를 찾아 앞뒤로 맞추고 재생.
없으면 앞 장애물만 보고 blocked_align_mm(500)에서 무조건 재생. 바깥 차로는 늘 가운데 쪽(+1 -> right csv, -1 -> left csv).
서 있지 않기: 횟수 제한 없음, 후진 중 뒤 막힘 -> 후진 그만, 차로 따라가다 안전 정지 이어짐 -> 물러나 옆 차로, 섬 앞 가운데 막힘 -> B."""
import math
import os

from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, load_change, load_m2, path_ok

PATHS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'paths')
LW = 0.52


def _m(lane=0, **kw):
    pl, pr = os.path.join(PATHS, 'm7_left.csv'), os.path.join(PATHS, 'm7_right.csv')
    L, _ = load_change(pl, pr, +1, LW)
    R, _ = load_change(pr, pl, -1, LW)
    kw.setdefault('trigger_mm', 590.0)
    p = ReplayParams(start_lane=lane, backup_if_mm=460.0, backup_to_mm=500.0, **kw)
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


def _drive(m, world, n=200, dt=0.05, scan_every=2, stop_at=None, t0=1.0):
    """명령 속도대로 차를 차로 방향으로 움직이며 (조향은 무시) 스캔·판단. stop_at(m) True면 멈춤."""
    t = t0
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


def _exec_started(m):
    return m.state == 'EXEC'


def test_outer_lanes_only_go_toward_center():
    """+1(왼쪽 바깥, 노란 실선이 왼쪽) -> right csv(목표 0), -1(오른쪽 바깥) -> left csv(목표 0). 바깥으로는 절대 안 감."""
    for lane in (1, -1):
        m = _m(lane=lane)
        world = _box(0.10 + 0.62, lane) + _box(0.2, 0)          # 지금 차로 앞 + 가운데 차로 조금 앞 LIMO -> 막힘
        _drive(m, world, stop_at=_exec_started)
        assert m.state == 'EXEC' and m.target == 0, (lane, m.events)
        assert any(('right csv' if lane == 1 else 'left csv') in e for e in m.events), m.events
        assert 0.45 <= _gap(m) <= 0.66, _gap(m)                 # 맞춘 거리는 backup_if(460) 밖


def test_center_lane_finds_distance_and_side():
    m = _m(lane=0, choose_tie_side='B')
    world = _box(0.10 + 0.62, 0) + _box(0.10, 1) + _box(0.25, -1)
    _drive(m, world, stop_at=_exec_started)
    assert m.state == 'EXEC', m.events
    assert any('틈 통과 거리' in e for e in m.events), m.events


def test_no_gap_falls_back_to_500_and_runs_anyway():
    """옆 차로가 길게 막혀 어느 거리에서도 틈 없음 -> 앞 장애물만 보고 500 mm에서 무조건 재생 (오른쪽 바깥 -> left csv)"""
    m = _m(lane=-1)
    world = _box(0.10 + 0.62, -1) + _box(0.0, 0) + _box(0.33, 0) + _box(0.66, 0)
    _drive(m, world, stop_at=_exec_started)
    assert m.state == 'EXEC' and m.target == 0
    assert any('틈 통과 거리 없음' in e for e in m.events), m.events
    assert abs(_gap(m) - 0.50) < 0.03, _gap(m)


def test_never_stands_still_keeps_retrying():
    """재생이 끊겨 다시 막혀도 횟수 제한 없이 다시 함 (예전: 3번 뒤 서 있음)"""
    m = _m(lane=-1)
    world = _box(0.10 + 0.62, -1) + _box(0.0, 0) + _box(0.33, 0) + _box(0.66, 0)
    for i in range(5):
        _drive(m, world, stop_at=_exec_started, t0=1.0 + 30 * i)
        assert m.state == 'EXEC'
        m.lane = m.target = -1                                  # 재생이 끊겨 원래 차로로 (2초 포기와 같은 결과)
        m.track = None
        m.state = 'CRUISE'
    assert m.n_align >= 5


def test_trigger_is_590():
    m = _m(lane=0)
    world = _box(0.10 + 0.70, 0)
    _drive(m, world, stop_at=_exec_started)
    assert m.state == 'EXEC'
    assert 0.56 <= _gap(m) <= 0.62, _gap(m)                    # 590 mm (+ 지연 보정) 근처에서 시작


def test_rear_blocked_during_adjust_gives_up_reversing():
    """거리 맞추려 후진하는데 뒤 라이다에 물체 1초 -> 후진 그만, 그 자리에서 CSV"""
    m = _m(lane=-1, blocked_search_tries=0, blocked_align_mm=640.0)   # 막힌 자리보다 멀게 -> 후진
    front = _box(0.10 + 0.62, -1) + _box(0.2, 0)
    _drive(m, front, n=60, stop_at=lambda mm: mm.state == 'ADJUST')
    assert m.state == 'ADJUST'
    x, y, _ = m.pose()
    behind = [(x - m.p.robot_rear - 0.03, y + 0.01 * j) for j in range(-5, 6)]
    _drive(m, front + behind, n=80, stop_at=_exec_started, t0=10.0)
    assert m.state == 'EXEC' and any('후진 그만' in e for e in m.events), m.events


def test_cruise_stall_on_thin_object_backs_off_then_changes_lane():
    """점 2개짜리 가는 물체(장애물로 확정 안 됨)가 차 바로 앞 -> 안전 정지 2초 -> 25 cm 물러난 뒤 옆 차로 CSV"""
    m = _m(lane=1)
    x, y, _ = m.pose()
    pole = [(x + m.p.robot_front + 0.03, y), (x + m.p.robot_front + 0.03, y + 0.01)]
    log = _drive(m, pole, n=200, stop_at=_exec_started)
    assert m.state == 'EXEC' and m.target == 0, m.events
    assert any('안전 정지' in e for e in m.events)
    assert any(s == 'BACKUP' and v < 0 for s, v in log)


def test_island_center_lane_no_way_runs_b_or_exit_side():
    m = _m(lane=0)
    L = m.p.section_length
    m.offset = (L - 0.30, 0.0, m.offset[2])
    walls = _box(L - 0.40, 1) + _box(L - 0.40, -1)            # 양옆 바로 옆에 LIMO
    _drive(m, walls, n=10, stop_at=_exec_started)
    assert m.state == 'EXEC' and m.target == -1 and any('B 쪽' in e for e in m.events), m.events
    m = _m(lane=0)
    m.set_exit('A')
    m.offset = (L - 0.30, 0.0, m.offset[2])
    _drive(m, walls, n=10, stop_at=_exec_started)
    assert m.state == 'EXEC' and m.target == 1


def test_broken_recording_is_not_used():
    """11/17 m2_1-A.csv: odom 튐으로 '뒤로 2.4 m, 옆 7.7 m' 4점 -> 안 쓰고 m2_3-B 거울상(직진)"""
    m2, src = load_m2(PATHS)
    assert '망가짐' in src[(1, 'A')] and '거울상' in src[(1, 'A')]
    for k, segs in m2.items():
        assert path_ok(segs)[0], (k, src[k])
        dy = [y for g in segs for y in g.y]
        lane, fork = k
        want = (1 if fork == 'A' else -1) - lane              # 갈래 A = 왼쪽(+), B = 오른쪽(-)
        assert abs((dy[-1] - dy[0]) - want * LW) < 0.15, (k, src[k], dy[-1] - dy[0])   # 갈래 쪽으로 차로 수만큼


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


def test_both_beside_is_a_tie_not_a_coin_flip():
    m = _m(lane=0, choose_tie_side='B')
    m.pts = []
    x, _, _ = m.pose()
    m.obs = {0: x + 0.10 + 0.54, 1: x + 0.10 - 0.041, -1: x + 0.10 - 0.049}
    d, why = m.choose(0, m.obs[0])
    assert d == -1 and '정한 쪽(B)' in why


def test_off_switch_keeps_old_behaviour():
    m = _m(lane=-1, blocked_align=False, backup_when_blocked=False, stall_s=1e9)
    world = _box(0.10 + 0.62, -1) + _box(0.2, 0)
    log = _drive(m, world, n=80)
    assert 'ADJUST' not in [s for s, _ in log] and m.state == 'CRUISE'
