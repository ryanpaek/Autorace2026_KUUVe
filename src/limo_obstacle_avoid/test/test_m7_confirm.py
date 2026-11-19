"""11/19 실차 '앞에 장애물이 없는데 후진': 라이다 점이 한두 스캔 튄 것으로 후진하지 않기.
차로 장애물은 연속 obs_confirm_frames(3) 스캔 같은 자리, 라이다 점으로 정하는 후진은 연속 reverse_confirm_frames(3) 스캔."""
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
    """라이다(차 기준 +0.10 m)에서 gap 앞에 정차 LIMO(폭 0.23, 길이 0.32) 뒷면."""
    x0 = 0.10 + gap
    return [(x0 + 0.02 * i, yc - 0.115 + 0.023 * j) for i in range(16) for j in range(11)]


def _blob(gap, yc=0.0):
    """튄 점 몇 개 (장애물 판정 최소 3점 넘게)"""
    x0 = 0.10 + gap
    return [(x0 + 0.01 * i, yc + 0.01 * j) for i in range(3) for j in range(-1, 2)]


def _run(m, scans, t0=1.0):
    states = []
    for i, pts in enumerate(scans):
        m.on_scan_points(t0 + 0.1 * i, pts)
        v, _ = m.step(t0 + 0.1 * i)
        states.append((m.state, v))
    return states


def test_spike_for_two_scans_does_not_back_up():
    """46 cm보다 가까운 곳에 두 스캔만 점이 튐 -> 후진 안 함, 계속 전진"""
    m = _m()
    st = _run(m, [_blob(0.40), _blob(0.40), [], [], []])
    assert all(s == 'CRUISE' and v > 0 for s, v in st), st
    assert not any('후진' in e for e in m.events)


def test_spike_at_trigger_distance_does_not_start_lane_change():
    """트리거(540 mm) 안에 한 스캔 튐 -> 차로 변경 시작 안 함"""
    m = _m()
    st = _run(m, [_blob(0.50), [], _blob(0.50), []])
    assert all(s == 'CRUISE' for s, _ in st), st


def test_spikes_at_different_places_are_not_confirmed():
    """세 스캔 연속이어도 자리가 매번 다르면(10 cm 넘게) 같은 물체가 아님 -> 안 씀"""
    m = _m()
    st = _run(m, [_blob(0.30), _blob(0.45), _blob(0.60), _blob(0.75)])
    assert all(s == 'CRUISE' for s, _ in st), st
    assert m.obs.get(0) is None


def test_real_obstacle_three_scans_backs_up():
    """진짜 장애물은 매 스캔 같은 자리 -> 3번째 스캔에서 확정, 46 cm 안이면 후진 (예전과 같은 동작)"""
    m = _m()
    st = _run(m, [_limo(0.42, 0.0)] * 3)
    assert [s for s, _ in st] == ['CRUISE', 'CRUISE', 'BACKUP'], st


def test_old_behaviour_with_one_frame():
    m = _m(obs_confirm_frames=1, reverse_confirm_frames=1)
    st = _run(m, [_blob(0.40)])
    assert st[0][0] == 'BACKUP'


def test_far_obstacle_confirmed_early_so_trigger_not_late():
    """멀리서부터 보이는 장애물은 트리거 거리에 왔을 때 이미 확정 -> 그 스캔에 바로 재생"""
    m = _m()
    for i in range(3):
        m.on_scan_points(0.1 * i, _limo(1.2, 0.0))
        m.step(0.1 * i)
    assert m.obs.get(0) is not None and m.state == 'CRUISE'
    m.offset = (0.66, 0.0, m.offset[2])           # 차가 66 cm 앞으로 (장애물까지 라이다 540 mm)
    m.on_scan_points(0.4, _limo(0.54, 0.0))       # 차 기준 거리로 넣음 = 차로 좌표 같은 자리
    m.step(0.4)
    assert m.state == 'EXEC'


def test_side_block_spike_does_not_back_up():
    """47 cm 앞 장애물(확정) + 옆 차로를 막는 점이 한 스캔만 -> 후진 안 하고 서서 봄, 점이 사라지면 그 쪽으로 재생"""
    m = _m(lane=1)
    front = _limo(0.47, 0.0)                       # 1차선(지금 차로) 47 cm 앞
    side = _limo(0.20, -0.52)                      # 2차선(오른쪽) 바로 옆 -> 오른쪽 녹화 경로가 닿음
    st = _run(m, [front, front, front + side, front])
    assert 'BACKUP' not in [s for s, _ in st], st
    assert st[-1][0] == 'EXEC'


def test_side_block_persisting_three_scans_acts():
    """막힘이 연속 3스캔이어야 움직임 (11/19: 후진 대신 400 mm로 거리 맞춤 -> 재생, test_m7_blocked_align)"""
    m = _m(lane=1)
    front = _limo(0.47, 0.0)
    side = _limo(0.20, -0.52)
    st = _run(m, [front, front, front + side, front + side, front + side])
    assert [s for s, _ in st][:4] == ['CRUISE'] * 4 and st[-1][0] == 'ADJUST', st
    m2 = _m(lane=1, blocked_align=False)                  # 끄면 예전처럼 50 cm까지 후진
    st = _run(m2, [front, front, front + side, front + side, front + side])
    assert st[-1][0] == 'BACKUP', st


def test_camera_lane_jump_keeps_confirmation():
    """카메라 차로 번호로 차로가 바뀌어도 장애물 기록은 같은 만큼 밀어 확인을 이어감"""
    m = _m()
    for i in range(3):
        m.on_scan_points(0.1 * i, _limo(1.2, 0.0))
    assert m.obs.get(0) is not None and m.obs.get(-1) is None
    for i in range(5):
        m.on_lane({'status': 'BOTH', 'lane_id': 1, 'offset_mm': 0.0, 'heading_rad': 0.0}, 1.0 + 0.1 * i)
    assert m.lane == 1
    assert m.obs.get(1) is not None                # 같은 물체가 이제 1차로(차 앞)
    m.on_scan_points(2.0, _limo(1.2, 0.0))         # 다음 스캔에서도 바로 확정 유지
    assert m.obs.get(1) is not None
