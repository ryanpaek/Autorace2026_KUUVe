"""M7 연결 로직 테스트 (ROS 없이). `python3 -m pytest src/autorace_obstacle/test`"""

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'autorace_mission'))

from autorace_obstacle.m7_link import M7Link, M7LinkParams, lane_to_fork  # noqa: E402

ROUTE_YAML = os.path.join(HERE, '..', '..', 'autorace_bringup', 'config', 'track_route.yaml')


def mission(s=None, dist=0.0, state='M7_OBSTACLE', fork=None):
    m = {'state': state, 'dist_mm': dist, 'fork_choice': fork}
    m['route'] = {'s_mm': s} if s is not None else None
    return m


def drive(link, s_from, s_to, step=20.0, t0=0.0, lane=None, fork=None, route=True):
    """경로 위치 s를 step씩 늘리며 update. (s, 동작) 목록을 돌려준다."""
    acts, t, s = [], t0, s_from
    while s <= s_to:
        m = mission(s if route else None, dist=s, fork=fork)
        for a in link.update(t, m, lane(s) if callable(lane) else (lane or {})):
            acts.append((s, a))
        s += step
        t += 0.05
    return acts


def test_route_constants_match_drawing():
    """m7_link의 시작선·섬 위치가 도면 경로(track_route.yaml)와 같아야 한다."""
    from autorace_mission.route import RouteMap
    r = RouteMap.from_yaml(ROUTE_YAML)
    p = M7LinkParams()
    start = next(m['s'] for m in r.landmarks if m['name'] == 'm7_start_line')
    assert p.start_s_mm == pytest.approx(start, abs=2)
    assert p.island_s_mm == pytest.approx(r.points['m7_island_tip'], abs=2)
    assert p.entry_to_start_mm == pytest.approx(start - r.s_of((9580, 1970)), abs=5)


def test_enable_at_start_line_not_at_m7_entry():
    link = M7Link()
    acts = drive(link, 11905.0, 13700.0)                  # 횡단보도 끝(M7 진입) -> 시작선 지나
    en = [(s, a) for s, a in acts if a[0] == 'enable']
    assert len(en) == 1                                    # 재시도(0.5 s) 전
    s, (_, section) = en[0]
    assert 13589 <= s < 13589 + 20                         # 코너 전이 아니라 시작선에서
    assert section == pytest.approx(4.384, abs=0.03)       # 시작선 ~ 섬 끝
    assert link.state()['m7_how'] == 'route'


def test_enable_by_camera_line_with_odometry():
    """선은 앞바퀴에 닿기 전에 화면에서 사라진다 -> 본 거리에서 달린 거리를 빼서 켠다."""
    link = M7Link()
    # route s는 실제보다 120 mm 늦게 간다고 가정 (오도메트리 오차). 카메라는 정확.
    true_start = 13589.0

    def lane(s):
        d = true_start - (s + 120.0)
        return {'stoplines_mm': [d]} if 200.0 <= d <= 600.0 else {}

    acts = []
    for k in range(200):
        s_true = 13000.0 + k * 10.0
        m = mission(s_true - 120.0, dist=s_true)
        for a in link.update(k * 0.05, m, lane(s_true - 120.0)):
            acts.append((s_true, a))
    en = [s for s, a in acts if a[0] == 'enable']
    assert en and en[0] == pytest.approx(true_start, abs=12)   # route만 썼다면 120 mm 늦음
    assert link.state()['m7_how'] == 'line'


def test_line_far_from_start_is_ignored():
    link = M7Link()
    # 시작선보다 1 m 앞에 있는 다른 흰 선 (게이트 400 mm 밖)
    acts = drive(link, 12000.0, 12500.0, lane=lambda s: {'stoplines_mm': [13589.0 - 1000.0 - s]})
    assert not [a for _, a in acts if a[0] == 'enable']
    assert link.line is None


def test_heading_wait_then_forced():
    link = M7Link()
    bent = {'status': 'BOTH', 'heading_rad': math.radians(6.0), 'stoplines_mm': []}
    acts = drive(link, 13500.0, 14100.0, lane=bent)
    en = [(s, a[1]) for s, a in acts if a[0] == 'enable']
    assert en and 13589 + 300 <= en[0][0] < 13589 + 320     # 300 mm 기다렸다가 켬
    assert en[0][1] == pytest.approx(4.384 - 0.3, abs=0.03)  # 켠 위치 기준 남은 거리
    assert link.state()['m7_heading_forced'] is True


def test_heading_ok_enables_immediately():
    link = M7Link()
    ok = {'status': 'BOTH', 'heading_rad': math.radians(1.0)}
    acts = drive(link, 13500.0, 13700.0, lane=ok)
    en = [s for s, a in acts if a[0] == 'enable']
    assert en and en[0] < 13589 + 20
    assert link.state()['m7_heading_forced'] is False


def test_retry_until_avoid_runs():
    link = M7Link(M7LinkParams(retry_sec=0.5))
    acts = drive(link, 13580.0, 14000.0)                   # 0.05 s 간격, 회피 노드 응답 없음
    en = [(s, a[1]) for s, a in acts if a[0] == 'enable']
    assert len(en) >= 2                                    # 다시 보냄
    assert en[1][1] < en[0][1]                             # 그동안 달린 만큼 section을 줄여서
    link.on_avoid_state({'state': 'RUN', 'target': 0})     # dry_run이어도 state로 실행 확인
    assert link.phase == 'running'
    acts = drive(link, 13820.0, 14200.0, t0=10.0)
    assert not [a for _, a in acts if a[0] == 'enable']    # 실행 중엔 다시 안 보냄 (원점이 바뀌면 안 됨)


def test_fork_forwarded_and_fallback():
    link = M7Link(M7LinkParams(fork_fallback='A'))
    acts = drive(link, 13000.0, 13700.0)
    forks = [a for _, a in acts if a[0] == 'fork']
    assert forks == [('fork', 'A')]                        # 표지를 못 읽음 -> 켤 때 대체값
    link2 = M7Link()
    acts = drive(link2, 13000.0, 13700.0, fork='B')
    assert [a for _, a in acts if a[0] == 'fork'] == [('fork', 'B')]   # 한 번만


def test_done_sets_exit_and_state():
    link = M7Link()
    drive(link, 13500.0, 13700.0)
    link.on_avoid_active(True)
    assert link.state()['m7_active'] is True
    link.on_avoid_state({'state': 'RUN', 'target': -1})
    link.on_avoid_done(True)
    st = link.state()
    assert st['m7_done'] is True and st['m7_exit'] == 'B'
    assert st['m7_active'] is False
    # M7을 벗어나도 m7_exit는 남는다 (FSM M2가 씀), disable은 안 보냄
    acts = link.update(99.0, mission(18300.0, state='M2_FORK'), {})
    assert acts == [] and link.state()['m7_exit'] == 'B'


def test_leaving_m7_early_disables():
    link = M7Link()
    drive(link, 13500.0, 13700.0)
    link.on_avoid_active(True)
    acts = link.update(50.0, mission(18500.0, state='M2_FORK'), {})
    assert acts == [('disable',)]
    assert link.phase == 'aborted'


def test_no_route_uses_distance_since_entry():
    link = M7Link()
    acts = drive(link, 0.0, 2000.0, route=False)            # dist_mm만 (M7 진입 = 0)
    en = [s for s, a in acts if a[0] == 'enable']
    assert en and en[0] == pytest.approx(1683.0, abs=20)
    assert link.state()['m7_how'] == 'distance'


def test_lane_to_fork():
    assert lane_to_fork(1) == 'A' and lane_to_fork(-1) == 'B'
    assert lane_to_fork(0) is None and lane_to_fork(None) is None


# --- 회피 노드가 도중에 멈출 때 (m7_hold) -------------------------------------------
def start_running(link, t_end=1.0):
    """시작선에서 켜고 회피 노드가 20 Hz로 active true를 내는 상태까지."""
    drive(link, 13500.0, 13700.0)
    t = 0.0
    while t < t_end:
        link.on_avoid_active(True, t)
        link.update(t, mission(14000.0), {})
        t += 0.05
    return t


def test_hold_when_avoid_node_ctrl_c():
    link = M7Link()
    t = start_running(link)
    assert link.state()['m7_active'] is True and link.state()['m7_hold'] is False
    link.on_avoid_active(False, t)                 # Ctrl+C: active=false 보내고 끝, done 없음
    link.update(t + 0.02, mission(14000.0), {})
    assert link.state()['m7_hold'] is True         # 놓는 즉시 정지 (차선 주행으로 안 넘어감)
    assert link.hold is False                      # '멈춤' 기록은 done을 잠깐 기다린 뒤
    link.update(t + 0.35, mission(14000.0), {})
    assert link.hold is True
    st = link.state()
    assert st['m7_hold'] is True and st['m7_active'] is False and st['m7_done'] is False


def test_hold_when_avoid_node_dies_silently():
    link = M7Link()
    t = start_running(link)                        # 마지막 active true = t - 0.05
    link.update(t + 0.8, mission(14000.0), {})
    assert link.state()['m7_hold'] is False          # 0.8초: 잠깐 늦는 것은 멈춤으로 안 봄 (11/10 0.5 -> 1.0)
    link.update(t + 1.0, mission(14000.0), {})
    assert link.state()['m7_hold'] is True and link.state()['m7_active'] is False


def test_hold_resumes_when_avoid_node_comes_back():
    """CPU가 바빠 active가 1초 넘게 늦게 온 것: 다시 오면 멈춤을 풀고 회피를 계속한다 (계속 서 있지 않음)."""
    link = M7Link()
    t = start_running(link)
    link.update(t + 1.2, mission(14000.0), {})
    assert link.state()['m7_hold'] is True and link.state()['m7_hold_kind'] == 'timeout'
    link.on_avoid_active(True, t + 1.3)
    link.update(t + 1.3, mission(14000.0), {})
    st = link.state()
    assert st['m7_hold'] is False and st['m7_active'] is True and st['m7_hold_resumed'] == 1


def test_ctrl_c_hold_is_not_resumed_by_stray_active():
    link = M7Link()
    t = start_running(link)
    link.on_avoid_active(False, t)
    link.update(t + 0.5, mission(14000.0), {})
    assert link.hold is True and link.hold_kind == 'release'
    link.on_avoid_active(True, t + 0.6)
    assert link.hold is True


def test_no_hold_on_normal_finish():
    """회피 노드는 끝날 때 active false -> done true 순서로 보낸다."""
    link = M7Link()
    t = start_running(link)
    link.on_avoid_state({'state': 'RUN', 'target': 1})
    link.on_avoid_active(False, t)
    link.update(t + 0.02, mission(18300.0), {})
    link.on_avoid_done(True)
    link.update(t + 1.0, mission(18300.0), {})
    st = link.state()
    assert st['m7_done'] is True and st['m7_hold'] is False and st['m7_exit'] == 'A'


def test_no_hold_in_dry_run():
    """dry_run 회피 노드는 active를 안 낸다 -> 제어를 잡은 적이 없으니 정지하지 않는다."""
    link = M7Link()
    drive(link, 13500.0, 13700.0)
    link.on_avoid_state({'state': 'RUN', 'target': 0})
    for k in range(40):
        link.update(10.0 + k * 0.05, mission(14000.0), {})
    assert link.phase == 'running' and link.state()['m7_hold'] is False


def test_hold_released_by_leaving_m7_and_reentering():
    """정지 유지는 M7을 벗어났다가(WAIT_START 등) 다시 들어오면 풀리고, 시작선에서 새로 켠다."""
    link = M7Link()
    t = start_running(link)
    link.on_avoid_active(False, t)
    link.update(t + 0.5, mission(14000.0), {})
    assert link.hold is True
    link.update(t + 1.0, mission(14000.0, state='M7_OBSTACLE'), {})   # M7 그대로면 계속 정지
    assert link.state()['m7_hold'] is True
    link.update(t + 2.0, mission(14000.0, state='WAIT_START'), {})
    acts = drive(link, 13289.0, 13700.0, t0=t + 3.0)                   # set_state M7: s = 시작선 - 300
    assert link.state()['m7_hold'] is False
    assert [a for _, a in acts if a[0] == 'enable']


# --- 3차선 입구에서 인식이 흔들릴 때 (11/08 실차) ----------------------------------
def test_spurious_lines_do_not_keep_pushing_start_back():
    """시작선을 본 뒤에도 입구 사선·점선 끝을 계속 '앞 400~500 mm 흰 선'으로 읽는 경우.

    고치기 전: 볼 때마다 기준을 새로 잡아 '아직 시작선 앞'으로 계속 미뤄져 켜지지 않았다.
    """
    link = M7Link()
    start = 13589.0
    acts = []
    for k in range(120):                       # 시작선 300 mm 앞에서 10 mm씩 1.2 m
        s = start - 300.0 + 10.0 * k
        lines = []
        if start - s > 250.0:
            lines.append(start - s)            # 진짜 시작선 (가까워지면 화면 아래로 사라짐)
        lines.append(420.0 + 80.0 * (k % 2))   # 가짜 선: 늘 400~500 mm 앞에 보임
        lines.sort()
        for a in link.update(k * 0.05, mission(s, dist=s), {'stoplines_mm': lines}):
            acts.append((s, a))
    en = [s for s, a in acts if a[0] == 'enable']
    assert en and abs(en[0] - start) <= 30.0, en[:1]


def test_wild_heading_is_ignored():
    """입구에서 차선 방향을 ±64°로 잘못 읽어도 시작선에서 켠다 (30 cm 미루지 않음)."""
    link = M7Link()

    def lane(s):
        k = int(s) // 20
        return {'status': 'BOTH' if k % 2 else 'R',
                'heading_rad': math.radians(64.0 if k % 2 else -64.0)}

    acts = drive(link, 13500.0, 14000.0, lane=lane)
    en = [s for s, a in acts if a[0] == 'enable']
    assert en and en[0] < 13589 + 20
    assert link.state()['m7_heading_forced'] is False


def test_state_reports_position():
    link = M7Link()
    drive(link, 13289.0, 13400.0)
    st = link.state()
    assert st['m7_pos_mm'] is not None and -300 <= st['m7_pos_mm'] <= 0


# --- 11/09: 라이다 장애물로 켜기 (enable_mode: obstacle) ------------------------------
def obstacle_link(**kw):
    return M7Link(M7LinkParams(enable_mode='obstacle', **kw))


def run_front(link, s_from, s_to, front_of_s, step=20.0):
    """s를 늘리며 라이다 앞 거리(front_of_s(s))를 넣고 update. 켠 위치와 동작을 돌려준다."""
    en = []
    s, t = s_from, 0.0
    while s <= s_to:
        link.on_front(front_of_s(s))
        for a in link.update(t, mission(s, dist=s), {}):
            if a[0] == 'enable':
                en.append((s, a[1]))
        s += step
        t += 0.05
    return en


def test_obstacle_trigger_enables_when_front_within_threshold():
    link = obstacle_link()
    assert link.p.trigger_front_mm == 1800.0                # 11/12 1000 -> 1300 -> 밤 1800
    obs = 13589.0 + 2000.0                                   # 가운데 차로 장애물: 시작선 + 2.0 m (1.8 m 앞이 코너 끝 문턱 뒤가 되게)
    en = run_front(link, 11905.0, 15000.0, lambda s: obs - s if obs - s < 2500 else None)
    assert en and en[0][0] == pytest.approx(obs - link.p.trigger_front_mm, abs=20)   # 기준 거리 앞에서
    assert link.state()['m7_trigger'] == 'obstacle'
    assert en[0][1] == pytest.approx((17973.0 - en[0][0]) / 1000.0, abs=0.03)   # section = 지금 위치 ~ 섬


def test_obstacle_trigger_ignored_in_corner():
    """코너 안(시작선 -250 mm 전)에서 앞에 뭐가 보여도 켜지 않는다."""
    link = obstacle_link()
    en = run_front(link, 11905.0, 13300.0, lambda s: 400.0)   # 계속 400 mm 앞에 뭔가
    assert not en
    en = run_front(link, 13340.0, 13400.0, lambda s: 400.0)
    assert en and en[0][0] >= 13589.0 - 250.0


def test_obstacle_trigger_fallback_without_obstacle():
    link = obstacle_link()
    en = run_front(link, 11905.0, 15000.0, lambda s: None)    # 앞 차로에 끝까지 없음
    assert en and en[0][0] == pytest.approx(13589.0 + 1000.0, abs=20)
    assert link.state()['m7_trigger'] == 'fallback'


def test_obstacle_trigger_side_lane_not_counted_by_node_width():
    """옆 차로 LIMO(중심 ±520)는 obstacle_node가 ±200 폭으로 걸러서 front=None -> 예비 위치까지 안 켬."""
    p = M7LinkParams()
    assert p.trigger_half_width_mm < 520.0 - 110.0           # 옆 차로 LIMO 가장자리(±410)보다 좁다


# --- 11/09: YOLO 갈림길 표지로 끄기 (off_mode: sign) -----------------------------------
def det(seen, h=40):
    return [{'cls': 'fork_b', 'conf': 0.9, 'bbox': [300, 100, 30, h]}] if seen else []


def running_link(**kw):
    kw.setdefault('off_sign_after_mm', 0.0)                  # 아래 예전 시험은 '지나감 = 바로 끔'으로 봄
    link = M7Link(M7LinkParams(off_mode='sign', **kw))
    drive(link, 13500.0, 13700.0)                            # 시작선에서 켬 (line 방식)
    link.on_avoid_active(True, 0.0)
    assert link.phase == 'running'
    return link


def test_sign_passed_turns_off_near_island_in_side_lane():
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    for _ in range(4):
        link.on_detections(det(True))                        # 표지 확인
    for _ in range(5):
        link.on_detections(det(False))                       # 지나가서 안 보임
    acts = link.update(5.0, mission(17973.0 - 600.0), {})    # 섬 앞 0.6 m
    assert ('disable',) in acts
    st = link.state()
    assert st['m7_done'] is True and st['m7_off_by'] == 'sign' and st['m7_exit'] == 'B'


def test_sign_lost_far_from_island_is_ignored():
    """시작선 근처에서 표지가 LIMO에 가려 사라져도 끄지 않는다 (섬 앞 1.5 m 밖)."""
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': 1, 'now_lane': 1})
    for _ in range(4):
        link.on_detections(det(True))
    for _ in range(8):
        link.on_detections(det(False))
    acts = link.update(5.0, mission(14500.0), {})            # 섬 앞 3.5 m
    assert ('disable',) not in acts and link.phase == 'running'


def test_sign_does_not_turn_off_in_middle_lane():
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': 0, 'now_lane': 0})
    for _ in range(4):
        link.on_detections(det(True))
    for _ in range(6):
        link.on_detections(det(False))
    acts = link.update(5.0, mission(17973.0 - 300.0), {})
    assert ('disable',) not in acts and link.phase == 'running'   # 가운데면 섬에 부딪히니 안 끔


def test_sign_near_by_box_height():
    link = running_link(off_sign_near_h_px=120.0)
    link.on_avoid_state({'state': 'RUN', 'target': 1, 'now_lane': 1})
    for _ in range(3):
        link.on_detections(det(True, h=60))
    link.on_detections(det(True, h=130))                     # 가까워짐
    acts = link.update(5.0, mission(17973.0 - 400.0), {})
    assert ('disable',) in acts and link.state()['m7_exit'] == 'A'


def test_sign_off_mode_none_keeps_old_behaviour():
    link = M7Link()
    drive(link, 13500.0, 13700.0)
    link.on_avoid_active(True, 0.0)
    link.on_avoid_state({'state': 'RUN', 'target': 1, 'now_lane': 1})
    for f in [True] * 4 + [False] * 6:
        link.on_detections(det(f))
    acts = link.update(5.0, mission(17973.0 - 300.0), {})
    assert ('disable',) not in acts and link.phase == 'running'


def test_obstacle_trigger_700_if_configured():
    """제안값 0.7 m로 바꾸면 그 거리에서 켠다 (params.yaml m7_trigger_front_mm)."""
    link = obstacle_link(trigger_front_mm=700.0)
    obs = 13589.0 + 1500.0
    en = run_front(link, 11905.0, 15000.0, lambda s: obs - s if obs - s < 2500 else None)
    assert en and en[0][0] == pytest.approx(obs - 700.0, abs=20)


# --- 11/09: M7이 끝날 때 몇 차선인지 (/m7/exit_lane) ------------------------------------
from autorace_obstacle.m7_link import exit_report  # noqa: E402


def test_exit_report_numbers():
    assert exit_report(1, 'x')['lane'] == 1 and exit_report(1, 'x')['fork'] == 'A'
    assert exit_report(0, 'x')['lane'] == 2 and exit_report(0, 'x')['fork'] is None
    assert exit_report(-1, 'x')['lane'] == 3 and exit_report(-1, 'x')['fork'] == 'B'
    assert exit_report(None, 'x') is None and exit_report(5, 'x') is None


def test_exit_report_on_avoid_done_once():
    link = M7Link()
    drive(link, 13500.0, 13700.0)
    link.on_avoid_active(True, 0.0)
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    assert link.pop_report() is None
    link.on_avoid_done(True)
    r = link.pop_report()
    assert r['lane'] == 3 and r['fork'] == 'B' and r['by'] == 'avoid_done'
    assert link.pop_report() is None                         # 한 번만


def test_exit_report_on_sign_and_forced():
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': 1, 'now_lane': 1})
    for f in [True] * 4 + [False] * 5:
        link.on_detections(det(f))
    link.update(5.0, mission(17973.0 - 500.0), {})
    r = link.pop_report()
    assert r['lane'] == 1 and r['by'] == 'sign'
    link2 = M7Link()
    drive(link2, 13500.0, 13700.0)
    link2.on_avoid_active(True, 0.0)
    link2.on_avoid_state({'state': 'RUN', 'target': 0, 'now_lane': -1})
    link2.update(9.0, mission(18500.0, state='M2_FORK'), {})  # FSM 강제 종료
    r2 = link2.pop_report()
    assert r2['lane'] == 3 and r2['by'] == 'forced'          # now_lane 우선


# --- 0단계: 늘 몇 차로에 있는지 (lane_now, /m7/lane) ---------------------------------
def test_lane_known_from_camera_vote_outside_m7():
    """M7 밖(M5 등)에서도 차선 노드 lane_id 7번 중 5번이면 차로를 안다."""
    link = M7Link()
    assert link.lane_now() is None
    for i in range(4):
        link.update(i * 0.05, mission(9000.0, state='M5_SLOW'), {'lane_id': 0})
    assert link.lane_now() is None                            # 4번은 아직 부족
    link.update(0.3, mission(9000.0, state='M5_SLOW'), {'lane_id': 0})
    ln = link.lane_now()
    assert ln['lane'] == 2 and ln['src'] == 'camera' and ln['side'] == '가운데'
    assert link.state()['m7_lane'] == 2 and link.state()['m7_lane_src'] == 'camera'


def test_lane_noise_does_not_flip():
    link = M7Link()
    for lid in (1, 1, 1, 1, 1, -1, None):
        link.update(0.0, mission(9000.0, state='M5_SLOW'), {'lane_id': lid})
    assert link.lane_now()['lane'] == 1
    for lid in (-1, -1):                                       # 섞이면 확정 못 함 -> 마지막 값 유지
        link.update(0.0, mission(9000.0, state='M5_SLOW'), {'lane_id': lid})
    ln = link.lane_now()
    assert ln['lane'] == 1 and ln['src'] == 'last'


def test_lane_from_avoid_node_while_running():
    """회피 중에는 회피 노드 now_lane(odom+IMU+카메라 보정)을 쓴다."""
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': 0})
    assert link.lane_now()['lane'] == 2 and link.lane_now()['src'] == 'avoid'
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    assert link.lane_now()['lane'] == 3


def test_lane_kept_when_camera_lost():
    link = M7Link()
    for _ in range(7):
        link.update(0.0, mission(9000.0, state='M5_SLOW'), {'lane_id': -1})
    assert link.lane_now()['src'] == 'camera'
    for _ in range(7):
        link.update(0.0, mission(9000.0, state='M5_SLOW'), {'status': 'NONE'})
    ln = link.lane_now()
    assert ln['lane'] == 3 and ln['src'] == 'last'


def test_lane_remembered_right_after_avoid_done():
    """회피가 끝나 회피 노드 값을 안 쓰게 돼도, 카메라가 확정하기 전까지 마지막 차로를 유지."""
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    link.phase = 'done'
    ln = link.lane_now()
    assert ln['lane'] == 3 and ln['src'] == 'last'


# --- 11/11: 표지가 화면 옆으로 나가면 지나감, 그 뒤 off_sign_after_mm 더 가서 끔 -------------------
def det_at(x1, w=40, h=40):
    return [{'cls': 'fork_b', 'conf': 0.9, 'bbox': [x1, 100, w, h]}]


def test_sign_exit_at_image_edge_counts_as_passed_quickly():
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    for x1 in (300, 350, 420, 520, 600):                     # 오른쪽으로 흘러가다
        link.on_detections(det_at(x1, w=min(40, 640 - x1)))
    link.on_detections(det_at(630, w=10))                    # 화면 끝에 닿음
    link.on_detections([])
    assert not link.sign_exit                                # 한 번 놓친 것은 아직
    link.on_detections([])
    assert link.sign_exit and link.sign_lost == 2            # 5프레임 기다리지 않음
    acts = link.update(5.0, mission(17973.0 - 600.0), {})
    assert ('disable',) in acts and link.state()['m7_sign'] == 'exit'


def test_sign_lost_mid_image_still_needs_frames():
    """가운데서 사라짐(가림·놓침)은 예전처럼 5프레임."""
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    for _ in range(4):
        link.on_detections(det_at(300))
    for _ in range(3):
        link.on_detections([])
    assert not link.sign_exit
    assert ('disable',) not in link.update(5.0, mission(17973.0 - 600.0), {})


def test_sign_off_waits_after_mm_from_pass_point():
    link = running_link(off_sign_after_mm=400.0)
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1})
    for x1 in (300, 500, 630):
        link.on_detections(det_at(x1, w=min(40, 640 - x1)))
    link.on_detections([])
    link.on_detections([])
    assert ('disable',) not in link.update(5.0, mission(17973.0 - 900.0, dist=5000.0), {})
    assert ('disable',) not in link.update(5.5, mission(17973.0 - 600.0, dist=5300.0), {})   # 300 mm
    assert ('disable',) in link.update(6.0, mission(17973.0 - 500.0, dist=5400.0), {})       # 400 mm


def test_lane_is_2_from_m7_entry_before_camera_knows():
    """M7에 들어오는 순간부터 /m7/lane이 나오게: 카메라 확정 전엔 2차선(start)."""
    link = M7Link()
    assert link.lane_now() is None
    link.update(0.0, mission(12500.0), {})
    ln = link.lane_now()
    assert ln['lane'] == 2 and ln['src'] == 'start'
    for _ in range(5):
        link.update(0.1, mission(12600.0), {'lane_id': 1})
    assert link.lane_now()['lane'] == 1 and link.lane_now()['src'] == 'camera'


# --- 11/11: 코너 지킴 (통주행: M7 진입 -> 우하단 코너 90° -> 시작선) ---------------------------------
def _corner_drive(link, map_err=0.0):
    """앞바퀴 기준 시작선 -1623 mm에서 진입, -1293 ~ -288 코너(90° 우회전). (켠 위치, 켤 때 방향) 반환."""
    s, yaw0 = -1623.0, math.pi / 2
    while s < 1500.0:
        turn = min(max(s + 1293.0, 0.0) / 1005.0, 1.0) * (math.pi / 2)
        link.on_yaw(yaw0 - turn)
        link.on_front(700.0)                          # 코너 내내 앞에 무언가(벽·장애물)가 보임
        acts = link.update(s / 1000.0, mission(13589.0 + s + map_err, dist=s + 1623.0),
                           {'status': 'BOTH', 'heading_rad': 0.0})
        if any(a[0] == 'enable' for a in acts):
            return s, math.degrees(yaw0 - turn)
        s += 20.0
    return None, None


def test_corner_guard_blocks_enable_inside_corner_even_if_map_ahead():
    old = obstacle_link(corner_turn_deg=0.0)
    s_old, _ = _corner_drive(old, map_err=300.0)
    assert s_old is not None and s_old < -288.0 - 200.0            # 예전: 도면이 0.3 m 앞서면 코너 안에서 켬
    new = obstacle_link()
    s_new, head = _corner_drive(new, map_err=300.0)
    assert s_new is not None and s_new >= -288.0 - 1005.0 * (10.0 / 90.0) - 20.0   # 80° 돈 뒤 (남은 10° 안)
    assert new.state()['m7_corner'] == 'done'
    assert abs(new.start_heading_deg - head) < 1.0                  # 직선 대비 남은 각을 회피 노드에 넘김
    assert 0.0 <= new.start_heading_deg <= 11.0


def test_corner_guard_off_for_practice_start_line():
    """연습: set_state M7 = 시작선 -300 에서 진입 -> 코너 지킴 안 함 (예전처럼)."""
    link = obstacle_link()
    link.on_yaw(0.0)
    link.update(0.0, mission(13589.0 - 300.0, dist=0.0), {})
    assert link.corner_need is False and link.start_heading_deg is None


def test_corner_not_detected_falls_back_to_map():
    """IMU가 안 변하면(이상) 도면상 시작선 +350 mm 에서 예전 규칙으로."""
    link = obstacle_link()
    s = -1623.0
    acts_all = []
    while s < 1000.0:
        link.on_yaw(math.pi / 2)
        link.on_front(700.0)
        acts_all += [(s, a) for a in link.update(s / 1000.0, mission(13589.0 + s, dist=s + 1623.0), {})]
        s += 20.0
    en = [x for x, a in acts_all if a[0] == 'enable']
    assert en and en[0] >= 350.0


def test_state_reports_distance_to_island_tip():
    """FSM M2가 도면 위치 없이 끝 거리를 맞추도록 섬 앞 끝까지 남은 거리를 알려 줌."""
    link = M7Link(M7LinkParams())
    assert link.state()['m7_to_tip_mm'] is None
    link.pos = 3384.0
    assert link.state()['m7_to_tip_mm'] == 1000


def test_sign_off_waits_until_settled_in_side_lane():
    """11/12 밤: 표지를 지났어도 차가 비스듬하거나(>8°) 아직 차로 중심 밖이면 끄지 않고, 자리 잡으면 끈다."""
    link = running_link()
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1, 'psi_deg': -20.0, 'y': -0.40})
    for _ in range(4):
        link.on_detections(det(True))
    for _ in range(5):
        link.on_detections(det(False))
    assert ('disable',) not in link.update(5.0, mission(17973.0 - 600.0), {})
    link.on_avoid_state({'state': 'RUN', 'target': -1, 'now_lane': -1, 'psi_deg': -3.0, 'y': -0.50})
    assert ('disable',) in link.update(5.1, mission(17973.0 - 590.0), {})
