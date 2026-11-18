"""11/18 시험: 54~46 cm 사이는 그 자리에서 재생, 46 cm 밑이면 50 cm까지 후진(차선 노드로 가운데 맞추며). 실제 녹화 경로로."""
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


def _limo(gap, yc, x_car=0.0):
    """라이다(차 기준 +0.10 m)에서 gap 앞에 정차 LIMO(폭 0.23, 길이 0.32) 뒷면."""
    x0 = x_car + 0.10 + gap
    return [(x0 + 0.02 * i, yc - 0.115 + 0.023 * j) for i in range(16) for j in range(11)]


def test_paths_need_46cm():
    for tr, ok in ((440, False), (460, True), (540, True)):
        p = ReplayParams(trigger_mm=float(tr))
        pl, pr = os.path.join(PATHS, 'm7_left.csv'), os.path.join(PATHS, 'm7_right.csv')
        L, _ = load_change(pl, pr, +1, 0.52)
        R, _ = load_change(pr, pl, -1, 0.52)
        mm = M7Replay(p, L, R)
        mm.start()
        assert all(mm.check_paths().values()) == ok, tr


def test_between_46_and_54_starts_from_here():
    m = _m()
    m.on_scan_points(0.0, _limo(0.50, 0.0))
    m.step(1.0)
    assert m.state == 'EXEC'
    assert abs(m.track.segs[0].x[0] - 0.0) < 0.02          # 녹화 경로를 지금 위치에 붙임


def test_closer_than_46_backs_up_to_50():
    m = _m()
    m.on_scan_points(0.0, _limo(0.42, 0.0))
    m.step(1.0)
    assert m.state == 'BACKUP'
    assert any('500 mm까지 후진' in e for e in m.events)


def test_blocked_between_46_and_50_backs_up():
    """47 cm인데 옆 차로도 막혀 지금 자세로 갈 곳 없음 -> 50 cm까지 후진"""
    m = _m(lane=1)
    m.on_scan_points(0.0, _limo(0.47, 0.52) + _limo(0.20, 0.0))
    m.step(1.0)
    assert m.state == 'BACKUP'


def test_backup_steers_by_lane_node():
    """후진 중 차선 노드: 차가 차로 가운데보다 왼쪽 10 cm(offset -100 mm) -> 후진 곡률 음수(뒤가 오른쪽으로)"""
    m = _m()
    m.lane_msg, m.lane_t = {'status': 'BOTH', 'offset_mm': -100.0, 'heading_rad': 0.0}, 1.0
    k = m._backup_lane_kappa(1.1)
    assert k is not None and k < 0
    m.lane_msg = {'status': 'L', 'offset_mm': -100.0, 'heading_rad': 0.0}
    assert m._backup_lane_kappa(1.1) is None               # 양쪽 선이 안 보이면 위치 추정으로
    m.p.backup_lane_center = False
    m.lane_msg = {'status': 'BOTH', 'offset_mm': -100.0, 'heading_rad': 0.0}
    assert m._backup_lane_kappa(1.1) is None
