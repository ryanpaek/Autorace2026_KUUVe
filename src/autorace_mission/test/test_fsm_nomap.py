"""맵 없이(use_map:=false, route_points 없음) 달릴 때 FSM 예비 판단 테스트 (11/10)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.fsm import Inputs, MissionFSM  # noqa: E402


def test_m5_slow_by_distance_without_map():
    fsm = MissionFSM({'start_state': 'M5_BUMP', 'noroute_m5_slow_mm': 1210.0})
    fsm.step(Inputs(t=0.0, dist_mm=0.0, confirmed_classes={}))
    fsm.step(Inputs(t=1.0, dist_mm=600.0, confirmed_classes={}))
    fsm.step(Inputs(t=2.0, dist_mm=1200.0, confirmed_classes={}))
    assert fsm.sub == ''
    fsm.step(Inputs(t=3.0, dist_mm=1220.0, confirmed_classes={}))
    assert fsm.sub == 'slow'


def test_m6_ends_on_camera_crosswalk_without_map():
    """거리(zigzag_len_mm)까지 기다리면 M3가 정지선 카메라 사각지대에서 시작한다 -> 횡단보도를 보면 바로 M3."""
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'zigzag_start_mm': 100.0, 'zigzag_len_mm': 1800.0})
    fsm.step(Inputs(t=0.0, dist_mm=0.0))
    fsm.step(Inputs(t=1.0, dist_mm=200.0))
    assert fsm.sub == 'zigzag'
    fsm.step(Inputs(t=2.0, dist_mm=700.0, lane={'crosswalk_lines_mm': [1100.0], 'stoplines_mm': [1100.0]}))
    assert fsm.state == 'M3_PEDESTRIAN'


def test_m6_with_map_still_uses_route_end():
    p = {'m6_zigzag_line': 9382.0, 'm6_zigzag_end': 11100.0}
    fsm = MissionFSM({'start_state': 'M6_ZIGZAG', 'route_points': p})
    fsm.step(Inputs(t=0.0, dist_mm=0.0, route_s=9400.0))
    assert fsm.sub == 'zigzag'
    fsm.step(Inputs(t=1.0, dist_mm=700.0, route_s=10200.0,
                    lane={'crosswalk_lines_mm': [1100.0], 'stoplines_mm': [1100.0]}))
    assert fsm.state == 'M6_ZIGZAG'                    # 맵이 있으면 예전처럼 도면 끝 위치까지 지그재그


def test_m4_arms_by_distance_without_map_and_sign():
    fsm = MissionFSM({'start_state': 'M4_PARKING', 'noroute_m4_arm_mm': 1000.0})
    for i, d in enumerate((0.0, 400.0, 800.0)):
        fsm.step(Inputs(t=float(i), dist_mm=d, confirmed_classes={}))
    assert not fsm.parking_armed
    fsm.step(Inputs(t=3.0, dist_mm=1010.0, confirmed_classes={}))
    assert fsm.parking_armed


def test_m7_force_end_ignores_reverse_without_map():
    """회피 노드가 후진하면 누적 거리(dist_mm)는 늘지만 후진을 뺀 거리로는 아직 섬 전 -> 강제 종료 안 함."""
    fsm = MissionFSM({'start_state': 'M7_OBSTACLE', 'm7_external_avoid': True,
                      'm7_len_mm': 6000.0, 'm7_force_after_island_mm': 500.0})
    fsm._goto('M7_OBSTACLE', Inputs(t=0.0, dist_mm=0.0, dist_signed_mm=0.0), 'test')
    act = {'m7_active': True}
    # 3 m 가고 1.5 m 후진 x 3번 = 누적 12 m, 후진 뺀 거리 3 m (odom 점프 감시에 안 걸리게 조금씩)
    dist = signed = 0.0
    for i in range(240):
        back = i % 80 >= 50                    # 50틱 전진, 30틱 후진
        dist += 50.0
        signed += -50.0 if back else 50.0
        fsm.step(Inputs(t=i * 0.1, dist_mm=dist, dist_signed_mm=signed, obstacle=act))
    assert dist == 12000.0 and signed == 3000.0
    assert fsm.state == 'M7_OBSTACLE'          # 예전: 누적 거리로 m7_len(6.5 m)·state_max(10 m)를 넘어 강제 종료
    for i in range(100):
        dist += 50.0
        signed += 50.0
        fsm.step(Inputs(t=30.0 + i * 0.1, dist_mm=dist, dist_signed_mm=signed, obstacle=act))
    assert fsm.state == 'M2_FORK'              # 후진 뺀 거리 6.5 m 넘으면 끝
