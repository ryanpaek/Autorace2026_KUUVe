"""11/19 corner_fix: m1_lane_back_frames (4번 강제 원호 -> 차선 복귀 히스테리시스). 1 = 기존 동작."""
import types

from autorace_mission.fsm import MissionFSM


def _lf(back, seq, n_lost=5):
    s = types.SimpleNamespace(p={'lane_first': True, 'lane_first_lost_frames': n_lost})
    out = []
    for good in seq:
        inp = types.SimpleNamespace(lane={'status': 'BOTH' if good else 'NONE', 'low_conf': False})
        out.append(MissionFSM._lane_first(s, inp) if back is None else MissionFSM._lane_first(s, inp, back))
    return out


def test_lane_first_default_unchanged():
    seq = [0] * 6 + [1, 0, 1, 1]
    assert _lf(None, seq) == _lf(1, seq)
    assert _lf(1, seq)[6] is True


def test_lane_first_back_frames():
    r = _lf(3, [0] * 6 + [1, 0, 1, 1, 1])
    assert r[5] is False
    assert r[6] is False and r[8] is False
    assert r[10] is True


# --- 1번 코너 (M6 진입~지그재그 시작선) IMU 코너 미리보기 ---
import math

from autorace_mission.fsm import Inputs

RP = {'c1_corner_start': 7930.0}


def _c1(route_s=7000.0, yaw_m5=0.0, yaw_m6=0.0, why_m5='start', **p):
    fsm = MissionFSM({'start_state': 'WAIT_START', 'route_points': RP, **p})
    fsm._goto('M5_BUMP', Inputs(t=0.0, dist_mm=0.0, yaw=yaw_m5), why_m5)
    fsm._goto('M6_ZIGZAG', Inputs(t=1.0, dist_mm=5000.0, yaw=yaw_m6, route_s=route_s), 'bump passed')
    return fsm


def test_c1_preview_before_corner_then_full_until_turned():
    fsm = _c1()                                           # 코너 930 mm 앞에서 M6
    k = fsm.m7_corner_k(5000.0, 0.0)
    assert k[0] == 0.0 and k[-1] < -1.0                   # 아직 직선: 코너 시작부터만 곡률
    k = fsm.m7_corner_k(6200.0, math.radians(-40.0))      # 돌기 시작 -> 다 돌 때까지 전부 코너 곡률
    assert all(v < -1.0 for v in k)
    assert fsm.m7_corner_k(6900.0, math.radians(-86.0)) is None   # 다 돎 -> 끝


def test_c1_entered_mid_corner_uses_start_straight_axis():
    fsm = _c1(route_s=8300.0, yaw_m6=math.radians(-50.0))  # M5가 길어 코너 도중에 M6
    k = fsm.m7_corner_k(5000.0, math.radians(-50.0))       # 직선 기준 50° 돎 -> 남은 각만큼 계속
    assert k is not None and all(v < -1.0 for v in k)
    assert fsm.m7_corner_k(5400.0, math.radians(-86.0)) is None


def test_c1_off_cases():
    assert _c1(c1_corner_preview=False).m7c is None
    fsm = MissionFSM({'start_state': 'WAIT_START'})        # 도면 위치 없음 -> 안 켬 (거리 추측 안 함)
    fsm._goto('M5_BUMP', Inputs(t=0.0, dist_mm=0.0, yaw=0.0), 'start')
    fsm._goto('M6_ZIGZAG', Inputs(t=1.0, dist_mm=5000.0, yaw=0.0), 'bump passed')
    assert fsm.m7c is None
    # 연습(M5 manual) + 이미 코너 안(300 mm 미만): 직선 방향을 모르니 안 켬
    assert _c1(route_s=7800.0, why_m5='manual').m7c is None


def test_c1_ends_at_zigzag_line():
    fsm = _c1()
    fsm.sub = 'zigzag'
    assert fsm.m7_corner_k(6000.0, math.radians(-60.0)) is None


def test_lead_smaller_is_earlier():
    first = {}
    for lead in (300.0, 250.0):
        k = _c1(m7_corner_lead_mm=lead).m7_corner_k(5000.0, 0.0)
        first[lead] = next(i for i, v in enumerate(k) if v != 0.0)
    assert first[250.0] <= first[300.0]
    assert _c1(m7_corner_lead_mm=250.0).m7_corner_k(5000.0, 0.0) != _c1().m7_corner_k(5000.0, 0.0)


def test_done_deg_m1_separate():
    f = _c1(m7_corner_done_deg=80.0, m1_corner_done_deg=85.0)
    assert f.m7_corner_k(6500.0, math.radians(-81.0)) is None          # 1~3번: 80°에서 끝
    m1 = MissionFSM({'start_state': 'M8_CONES', 'm7_corner_done_deg': 80.0, 'm1_corner_done_deg': 85.0})
    m1._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=1000.0, yaw=0.0), 'cones done')
    assert m1.m7c is not None
    m1.m1fc = None                                                      # 강제 원호 말고 미리보기 끝 각도만 봄
    assert m1.m7_corner_k(1600.0, math.radians(-81.0)) is not None      # 4번: 81°는 아직
    assert m1.m7_corner_k(1700.0, math.radians(-86.0)) is None


def test_m1_inner_line_kept_after_corner():
    def run(after):
        f = MissionFSM({'start_state': 'M8_CONES', 'm1_inner_after_mm': after, 'm1_force_corner': False})
        f._goto('M1_TRAFFIC', Inputs(t=0.0, dist_mm=1000.0, yaw=0.0), 'cones done')
        modes = []
        for d, deg in ((1500.0, -40.0), (2200.0, -86.0), (2500.0, -90.0), (2800.0, -90.0)):
            modes.append(f._m1_lane(Inputs(t=0.0, dist_mm=d, yaw=math.radians(deg)), 0.2).get('mode'))
        return modes
    m = run(500.0)
    assert m[0] == 'right_only'                     # 코너 중 안쪽 선
    assert m[1] == 'right_only' and m[2] == 'right_only'   # 코너 끝(2200) 뒤 500 mm 안: 안쪽 선 유지
    assert m[3] in (None, 'normal')                 # 500 mm 지나면 양쪽 선
    assert run(0.0)[1] in (None, 'normal')          # 0 = 예전 (끝나자마자 양쪽 선)
