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
