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
    m = _at(0.0, m2_wait_sign_s=0.0)   # 표지 모름 대기(11/19)는 test_m2_wait_sign에서
    m.offset = (m.m2_x() - 0.01, 0.0, 0.0)
    _scan_world(m, _pole(m.p.section_length - 0.03))
    m.step(1.0)
    assert m.state == 'EXEC' and m.m2_run == (0, 'A')


def test_late_m2_backs_up_to_start_when_path_hits_now():
    m = _at(0.0, m2_wait_sign_s=0.0)   # 표지 모름 대기(11/19)는 test_m2_wait_sign에서
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


def test_m2_replay_reports_branch_as_target_from_start():
    """11/19 bag 4차: 재생 중 target·끝 보고가 '재생 시작 차로'였다 -> 시작하자마자 갈래 차로"""
    m = _at(0.0, lane=1)
    m.set_exit('B')
    m.offset = (m.m2_x() - 0.01, 0.52, 0.0)
    _scan_world(m, [])
    m.step(1.0)
    assert m.state == 'EXEC' and m.m2_run == (1, 'B')
    st = m.status()
    assert st['target'] == -1 and st['end_lane'] == -1 and st['now_lane'] == 1


def test_m2_wait_sign_then_default_or_sign():
    """11/19: M2 시작 자리에서 A/B를 모르면 한 번 m2_wait_sign_s까지 서서 기다림 -> 오면 그 갈래, 안 오면 기본"""
    m = _at(0.0)
    m.offset = (m.m2_x() - 0.01, 0.0, 0.0)
    _scan_world(m, [])
    v, _ = m.step(1.0)
    assert v == 0.0 and m.state == 'CRUISE' and '표지 기다림' in m.why
    m.set_exit('B')
    m.step(1.5)
    assert m.state == 'EXEC' and m.m2_run == (0, 'B')
    m = _at(0.0)
    m.offset = (m.m2_x() - 0.01, 0.0, 0.0)
    _scan_world(m, [])
    m.step(1.0)
    m.step(2.0)
    assert m.state == 'CRUISE'
    m.step(3.05)                                          # 2초 지남 -> 기본 A
    assert m.state == 'EXEC' and m.m2_run == (0, 'A')


def test_sign_slow_when_fork_sign_seen():
    """섬 앞 2.5 m 안에서 표지가 보이는데 아직 A/B 모름 -> 천천히. 정해지면 원래 속도"""
    m = _at(2.0)
    m.p.speed = 0.17
    _scan_world(m, [])
    v, _ = m.step(1.0)
    assert abs(v - 0.17) < 1e-9
    m.on_detections(1.1, [{'cls': 'fork_b', 'conf': 0.8}])
    v, _ = m.step(1.2)
    assert abs(v - m.p.sign_slow_speed) < 1e-9 and m.status()['sign_slow']
    v, _ = m.step(2.5)                                    # 1초 넘게 안 보임 -> 원래 속도
    assert abs(v - 0.17) < 1e-9
    m.on_detections(3.0, [{'cls': 'fork_a', 'conf': 0.3}])  # 낮은 신뢰도는 안 셈
    v, _ = m.step(3.1)
    assert abs(v - 0.17) < 1e-9
    m.on_detections(3.2, [{'cls': 'fork_a', 'conf': 0.9}])
    m.set_exit('A')                                       # 정해짐 -> 원래 속도
    v, _ = m.step(3.3)
    assert abs(v - 0.17) < 1e-9
    m2 = _at(0.5)                                         # 섬에서 멂 -> 보여도 안 늦춤
    m2.p.speed = 0.17
    _scan_world(m2, [])
    m2.on_detections(1.0, [{'cls': 'fork_b', 'conf': 0.9}])
    v, _ = m2.step(1.1)
    assert abs(v - 0.17) < 1e-9


def test_sign_anchor_moves_tip_to_isolated_pole():
    """라이다 표지판 막대가 도면 꼭짓점보다 20 cm 앞에 3스캔 -> 꼭짓점 = 막대 자리"""
    m = _at(3.0)
    tip0 = m.p.section_length
    for _ in range(3):
        _scan_world(m, _pole(tip0 - 0.20))
    assert abs(m.p.section_length - (tip0 - 0.20)) < 0.02, m.events
    assert any('표지판으로 섬 앞 꼭짓점 고침' in e for e in m.events)


def test_sign_anchor_ignores_limo_piece_and_far_or_big_moves():
    m = _at(3.0)
    tip0 = m.p.section_length
    limo = [(tip0 - 0.30 + 0.02 * i, -0.30 + 0.023 * j) for i in range(16) for j in range(8)]   # 막대 옆에 LIMO
    for _ in range(3):
        _scan_world(m, _pole(tip0 - 0.20, 0.0) + [q for q in limo if q[1] < -0.10])
    assert m.p.section_length == tip0                     # 둘레에 다른 물체 -> 안 씀
    m2 = _at(1.0)
    for _ in range(3):
        _scan_world(m2, _pole(tip0 - 0.20))               # 3 m 넘게 멂 -> 안 씀
    assert m2.p.section_length == tip0


def _m2only(pole_gap, lane=0):
    """M7 없이 M2만(m2_test_start): 켤 때 자리 = 녹화 시작 자리라고 가정. 실제 표지판은 라이다 pole_gap 앞."""
    m = M7Replay(ReplayParams(m2_test_start=True, start_lane_no=2), default_change(1), default_change(-1))
    m.set_m2(_m2_paths(), {k: 't' for k in _m2_paths()})
    m.start()
    m.scan_t = 0.0
    m.set_exit('B')
    x, _, _ = m.pose()
    return m, x + m.p.lidar_x + pole_gap


def _run_m2only(m, pole_x, n=30, dt=0.05):
    t = 1.0
    for _ in range(n):
        _scan_world(m, _pole(pole_x))
        v, _ = m.step(t)
        dx, dy, dpsi = m.offset
        m.offset = (dx + v * dt, dy, dpsi)
        t += dt
        if m.state in ('EXEC', 'BACKUP'):
            break
    return m


def test_m2_only_pole_closer_than_recorded_start_backs_up():
    """녹화 시작 = 앞범퍼~꼭짓점 1.0 m (라이다~막대 0.9 m)인데 막대가 0.6 m -> 꼭짓점 고치고 0.3 m 후진"""
    m, pole_x = _m2only(0.60)
    _run_m2only(m, pole_x)
    assert m.sign_anchored is not None and abs(m.p.section_length - pole_x) < 0.03
    assert m.state == 'BACKUP' and m.backup['to_x'] == m.m2_x()


def test_m2_only_pole_farther_drives_up_slowly_then_replays_branch():
    m, pole_x = _m2only(1.40)
    m.on_detections(1.0, [{'cls': 'fork_b', 'conf': 0.9}])
    _run_m2only(m, pole_x, n=200)
    assert abs(m.p.section_length - pole_x) < 0.03
    assert m.state == 'EXEC' and m.m2_run == (0, 'B') and m.status()['end_lane'] == -1
    x, _, _ = m.pose()
    assert abs(x - m.m2_x()) < 0.06                      # 녹화 시작 자리에서 시작


def test_m2_only_waits_for_pole_before_replay():
    m, pole_x = _m2only(0.90)
    _scan_world(m, [])
    v, _ = m.step(1.0)
    assert v == 0.0 and '표지판 찾는 중' in m.why
