"""미션 상태 머신 (ROS 의존성 없음).

트랙이 루프가 아니고 미션 순서가 고정이라, '다음에 올 미션 하나만 기다리는' 순차형으로 만든다.
지금 상태에서 필요한 표지만 보면 되니 광고판 같은 방해 요소의 오검출이 크게 줄어든다.

주행 순서 (규정집 번호)
  WAIT_START -> M5_BUMP -> M6_ZIGZAG -> M3_PEDESTRIAN -> M7_OBSTACLE -> M2_FORK
             -> M8_CONES -> M1_TRAFFIC -> M4_PARKING -> FINISHED
  LANE_ONLY: 차선 주행만 하는 시험용 상태 (1주차 마일스톤용)
  start_state 파라미터로 아무 미션에서나 시작할 수 있다 (미션 하나만 반복 연습할 때)

step()은 매 틱(예: 20Hz) 호출되고 Command를 돌려준다.
  Command.source : 어느 cmd_vel을 /cmd_vel로 넘길지 ('lane' | 'obstacle' | 'parking' | 'stop')
  Command.lane   : /lane/config로 보낼 설정 (바뀔 때만 노드가 발행)

TODO 표시가 붙은 곳은 실차·트랙에서 값을 재거나 로직을 채워야 하는 부분이다.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional

from autorace_mission.corner import CornerWatch

SEQUENCE = ['WAIT_START', 'M5_BUMP', 'M6_ZIGZAG', 'M3_PEDESTRIAN', 'M7_OBSTACLE',
            'M2_FORK', 'M2_EXIT', 'M8_CONES', 'M1_TRAFFIC', 'M4_PARKING', 'FINISHED']
EXTRA_STATES = ['LANE_ONLY']

DEFAULT_PARAMS = {
    'start_state': 'WAIT_START',
    'base_speed': 0.30,          # m/s, 규정 권장 0.3~0.4
    'slow_speed': 0.20,          # 서행 (규정 최대 0.2). 11/09 팀 결정: 0.2
    'approach_speed': 0.18,      # 정지선·장애물 접근 속도
    'm7_speed': 0.20,            # 11/12: M7(횡단보도 끝 코너 ~ 3차선 회피) 차선 주행 속도. 회피 노드 속도도 0.20 (avoid_params.yaml)
    'vote_window': 8,            # 다수결 창 (인식 프레임 수)
    'vote_need': 5,
    'det_conf': 0.5,
    'odom_jump_mm': 600.0,       # 한 틱에 주행거리가 이만큼 넘게 튀면 odom 오류로 보고 그만큼은 무시 (11/09: 체크섬 오류로 7.5km가 찍혀 FINISHED)
    'state_max_mm': 10000.0,     # 한 상태에서 이만큼 달려도 안 끝나면 강제로 다음 상태 (멈춤 방지). M6이 약 9m라 10m
    # M5 방지턱
    'bump_slow_mm': 1800.0,      # 방지턱을 IMU로 못 느꼈을 때 서행을 끝내는 최대 거리 (2순위). TODO 실측
    # 방지턱 통과: IMU 앞뒤/좌우 기울기 속도(rad/s)가 튀면 앞바퀴가 방지턱에 올라간 것 -> bump_after_mm 더 가서 서행 끝
    # (11/09 full_02/03 bag: 평지 주행 99% 0.08~0.16, 정지 때 0.2, 충돌 0.6. 방지턱은 bag에 없어 실측 필요)
    'bump_imu_rate': 0.30,
    'bump_after_mm': 300.0,      # 뒷바퀴까지 넘게 (축거 210 + 여유)
    # M6 지그재그
    'zigzag_start_mm': 7200.0,   # 경로 지도도 카메라 선도 없을 때만: M5 종료 ~ 지그재그 시작선 (3순위)
    'zigzag_len_mm': 1800.0,     # 도면: 지그재그 약 1.7~1.9m. 바로 뒤 횡단보도 정지선까지 약 250mm라 여유를 크게 안 둠
    'zigzag_width_mm': 570.0,
    # 정지 미션 공통 (M3, M1)
    'stop_target_mm': 150.0,     # 11/15 저녁 실차: 100으로도 정지선 직전 -> 150. 실행 중 ros2 param set /mission_node stop_target_mm 으로 바로 조정
    'm3_stop_target_mm': 250.0,  # 11/15 저녁 실차 요청: M7 직전 횡단보도는 여유롭게 25 cm 앞 (신호등은 stop_target_mm). 0이면 stop_target_mm
    'stop_passed_mm': 120.0,
    # 11/14 밤 실차 '횡단보도를 다 지난 다음 섬' (pretrack 뒤에도): 정지선 카메라·도면 위치가 다 틀려도 넘지 않게
    #   지그재그 시작선(카메라로 지난 순간)부터 달린 거리로 정지선을 센다. 도면: 지그재그 시작선 -> 정지선 1983 mm
    #   이 거리 - m3_zz_cap_target_mm - stop_brake_mm 를 달리면 다른 판단과 상관없이 정지 (넘기 전 마지막 안전장치)
    'm3_zz_cap': True,
    'm3_zz_to_stop_mm': 1983.0,  # 도면 (지그재그 시작선 앞 가장자리 -> 정지선 앞 가장자리). 도면 위치가 있으면 그 값으로 다시 계산
    'm3_zz_cap_target_mm': 100.0,  # 이 안전장치로 설 때 정지선 앞 목표 (보통은 m3_stop_target_mm 으로 먼저 섬)
    'm3_pretrack': True,         # 11/15 밤: 지그재그(M6) 중에 횡단보도 정지선을 미리 재 둠 (M3 시작 때는 이미 카메라 사각)     # 11/15 저녁: 정지선을 이만큼 넘게 지났다고 보이면(남은 거리 < -이것) 그 자리에서 섬 (다음 흰 선=끝선까지 가지 않게)
    'stop_brake_mm': 30.0,       # TODO 실측: approach_speed에서 0 명령 후 밀려가는 거리
    'stop_lost_ok_mm': 400.0,    # 정지선이 카메라에서 사라진 뒤 오도메트리로 이어 가는 한계
    # 정지선이 이 거리 안으로 오면 카메라 갱신을 끊고 오도메트리로만 센다.
    # 카메라 사각지대(앞바퀴 약 350mm 안) 직전부터는 뒤따르는 횡단보도 블록을 정지선으로 다시 잡기 쉽다 (11/09)
    'stop_freeze_mm': 450.0,
    'stop_route_gate_mm': 300.0,  # 처음 잡는 선이 도면 예측과 이만큼 넘게 다르면 버림 (끝선 540mm 뒤)
    # 11/19 1119_08·09: 빨간불 확정(PAUSE_UNTIL_GREEN)인데 도면 위치가 3~22 m 틀려 정지선을 도면 기준으로만 계산 -> 안 섬.
    #   카메라는 횡단보도 정지선을 1.3~1.8 m 앞에서 보고 있었는데 stop_route_gate_mm(300)로 버려짐.
    #   M1 정지 준비(빨간불·초록·도면 근처) 뒤에는 도면과 상관없이 카메라 횡단보도 선(이 거리 안, 3프레임 연속)을 정지선으로 씀.
    #   카메라가 선을 보는 동안은 도면 위치로 서지 않음. 카메라에 선이 없을 때만 예전처럼 도면. 0 = 끔
    'm1_red_camera_max_mm': 2200.0,
    'stop_camera_first': False,   # 11/15: True면 도면 예측과 달라도 같은 횡단보도 선이 3프레임 연속이면 카메라 선을 씀
                                  #   (1114 리플레이: 코너 끝 가짜 횡단보도 선을 잡아 0.7 m 일찍 섬 -> 끔. 도면 위치는 코너 맞추기로 정확)
    'stop_pretrack': True,        # 11/15: M6·M1 준비 전부터 횡단보도 정지선을 카메라로 따라감
    'stop_pretrack_mm': 1500.0,   #   도면상 정지선이 이 거리 안일 때부터 (지그재그 시작선도 횡단보도 선으로 잡혀서 너무 멀리선 안 봄)
    'arm_on_crosswalk_mm': 1200.0,  # 표지판을 못 봐도 횡단보도가 이 거리 안이면 정지 준비
    'ped_wait_sec': 3.5,         # 규정: 3초 이상 대기, 15초 안에 출발
    'leave_mm': 600.0,           # 출발 후 다음 미션으로 넘어가기까지 거리 (횡단보도 깊이 약 570mm를 벗어나게)
    # M7 / M8
    'obstacle_source_ready': False,  # 장애물 노드가 완성되면 true. 그 전엔 차선 주행으로 통과
    'm7_len_mm': 6000.0,
    # M7을 limo_obstacle_avoid(avoid_node)로 회피. obstacle_node가 시작선에서 켜고 m7_active/m7_done을 알려 준다.
    #   켜져 있으면: 회피 노드가 실행 중일 때만 obstacle 명령을 쓰고(그 전 코너·시작선까지는 차선),
    #   M7은 m7_done으로 끝낸다. 도면 위치는 안전용 최대값(섬 끝 + m7_force_after_island_mm)으로만 쓴다.
    'm7_external_avoid': False,
    'm7_force_after_island_mm': 500.0,
    'fork_from_m7_exit': True,   # 갈림길 표지를 못 읽었으면 회피가 끝난 차로(A/B)를 그대로 쓴다 (섬을 건너지 않게)
    'fork_follow_m7_exit': True,   # 표지(A/B)와 회피가 끝난 차로가 다를 때 차로를 따를지. 11/11 기본 true = 차로를 따른다:
                                   # M2는 회피가 섬 앞 끝 +0.3 m를 지나 끝난 뒤 시작 -> 이미 그 길에 들어가 있고, M2에는 섬을 건너는
                                   # 조향이 없다. 표지 쪽 한쪽 선 모드(left/right_only)를 주면 반대 길 바깥선은 BEV(±0.8 m) 밖이라
                                   # 섬 테두리를 따르거나 선을 놓친다 -> 결과는 같고 선만 잃는다. false = 예전(표지 쪽 모드)
    # 3차선: 차로 500mm, 바깥 실선은 노랑·안쪽 점선은 흰색(모의 맵 10/30 확인) -> 두 색 다 차선으로
    'three_lane_width_mm': 520.0,       # 도면: 선 사이 500 + 점선 20 = 선 가운데끼리 520 (회피 lane_width 0.52와 같게)
    # 11/17: 3차선(M7) 구간 차선 lookahead. 기본(650)보다 짧게 -> 차로 바꾼 뒤 차선 주행이 옆 차로·장애물 쪽으로 미리 꺾지 않고
    #   지금 차로 가운데에 붙게. 0이면 차선 노드 기본값 그대로
    'm7_lookahead_mm': 450.0,
    'three_lane_color': 'both',         # 도면: M3 종료 ~ 우하단 코너(약1.6m) + 3차선 약4.5m(섬 오른쪽 끝까지)
    'cones_len_mm': 4300.0,      # 도면: M8 진입(갈림길 끝) ~ 좌하단 코너 ~ 라바콘 구간(약2.2m) 끝 + 여유
    # M8 라바콘 (obstacle 노드의 ConeFollower). false면 위 cones_len_mm 동안 차선 주행으로 통과
    'cones_source_ready': False,
    'cones_line_camera_first': True,  # 11/16: 라바콘 시작선 카메라 1순위 (도면과 달라도 같은 선 3프레임이면)
    'cones_enter_before_mm': 0.0,    # 도면 위치 예비: 시작선 이만큼 앞(앞바퀴)에서 라바콘 모드 (1순위는 카메라로 선 통과)
    'cones_enter_cone_mm': 150.0,    # 11/14 밤: 400 -> 150 원복 (d4d4c70 라바콘 변경 되돌림, 실차 '라바콘 인식 못 함')
    'cones_corner_close_mm': 0.0,    # 11/14 밤: 250 -> 0 = 끔 (d4d4c70 원복). >0이면 70° 넘게 돌았고 콘이 이 안이면 콘 모드
    'cones_corner_min_deg': 70.0,
    # 11/19 실차 '라바콘 모드가 늦게(구간 2/5쯤) 켜짐': 시작선은 코너 안쪽 곡선 끝에서 2 cm 뒤라 카메라에 보일 때 아직 30~60° 기울어
    #   있는데 차선 노드는 ±15°까지만 가로선으로 봐서 못 봄 -> 도면 위치(M2 뒤 약 1 m 뒤처짐)·거리 예비로 늦게 켜짐.
    #   아래 셋 중 하나면 바로 라바콘 모드 (기준 방향 = 3차선 직선 방향, 오른쪽으로 돈 각):
    #   1) 코너를 m8_line_min_turn_deg 넘게 돈 뒤 카메라가 시작선을 m8_line_frames번 봄 (그동안 가로선 기울기 m8_line_max_angle_deg까지 찾음)
    #   2) m8_full_turn_deg 넘게 돌고 m8_after_turn_mm 더 감 (선을 놓쳐도)
    #   3) m8_full_turn_deg - 10 넘게 돌았고 차로 안 콘이 cones_enter_cone_mm 안
    #   코너 끝(m8_line_slow_deg 넘게 돎)부터 라바콘 모드까지 m8_line_speed (선이 잘 보이게)
    'm8_line_trigger': True,
    'm8_line_min_turn_deg': 30.0,
    'm8_line_frames': 2,
    'm8_line_max_angle_deg': 60.0,
    'm8_full_turn_deg': 80.0,
    'm8_after_turn_mm': 200.0,
    'm8_line_slow_deg': 45.0,
    'm8_line_speed': 0.12,
    'cones_approach_mm': 1550.0,     # 경로 지도 없을 때: M2 끝 -> 라바콘 모드 (도면 1854 - 300)
    # 11/11 코너 지킴: M2 끝 -> 좌하단 코너(90°) -> 시작선(코너 끝 + 240 mm). 도면 위치가 앞서거나 코너에서 콘이 보이면
    #   코너 안에서 라바콘 모드로 들어가 콘 쪽으로 곧장 감(지름길). 카메라로 시작선을 넘은 것 말고는 IMU로 코너를 다 돈 뒤에만
    # 11/12 밤: M7 직전 코너(횡단보도 끝 -> 직선 390 mm -> R640 오른쪽 90°)를 IMU로 지킨다. 횡단보도에서 넘어와 M7이 되면
    #   코너를 다 돌 때까지(IMU m7_corner_done_deg) 차선 노드에 이 코너 곡률을 '도면 곡률'로 넣는다 -> 차선을 놓쳐도(hold)
    #   직진하지 않고 계속 돈다 (차선이 잘 보이면 차선 우선, 놓치거나 한쪽만이면 섞음). 도면 위치를 잃어도 동작. 0 = 끔
    'm7_corner_r_mm': 640.0,
    'm7_corner_dir': -1.0,           # 오른쪽 코너 = 곡률 음수
    'm7_corner_start_mm': 330.0,     # M7 진입(횡단보도 끝 + 약 6 cm) -> 코너 시작
    'm7_corner_done_deg': 85.0,      # IMU로 이만큼 돌면 끝 (0이면 이 기능 끔)
    'm7_corner_lead_mm': 300.0,
    # 11/14: M2_EXIT (합류 끝 ~ 라바콘 시작선) = 차선 인식 모드만 (양쪽 노란 선, 한쪽 선 모드 안 씀) + 좌하단 코너 도면 곡률 보조.
    #   11/14 실차(1114_01): 한쪽 선 모드(left_only -> right_only)가 화면에 보이는 반대쪽 선을 버려 코너 6초 동안 선 없음(도면 곡률만).
    #   False면 예전처럼 M2 끝에서 바로 M8 (M8 접근에서 한쪽 선 모드)
    'm2_exit': True,
    # 11/15 실차 'A/B 뒤 합류 구간에서 A는 왼쪽, B는 오른쪽으로 차선을 벗어남'(합류 사선은 차선 인식이 잘 안 됨) ->
    #   인식 대신 도면 규격으로 강제 주행: 섬 왼쪽 끝 500 mm 앞(도면 x 3023, 섬이 좁아지기 250 mm 전)부터
    #   A(왼쪽 차로)면 오른쪽으로, B면 왼쪽으로 520 mm를 cos S자로 m2_merge_len_mm 동안 옮긴다 (IMU 방향 + 오도메트리로 옆 위치 계산,
    #   차선 노드는 heading 모드로 그 방향만 따라감). 다 옮기면(보통 차로 폭) 차선 인식으로 돌아감.
    #   시작·길이는 시뮬레이션(test_fsm _merge_sim: 직선 방향 ±3°, 옆 ±30 mm)으로 바깥 선·섬 여유가 같이 가장 크게:
    #   250/1150 -> 바깥 선 -43 mm(밟음), 500/1100 -> 바깥 선 약 +48·섬 약 +75 mm. 끝 = 도면 x 약 1923 (코너 1360 전)
    # 11/15 바닥 갈림길 글자 A/B (차선 노드 floor_letter, 구멍 수로 구분): 최근 floor_letter_window 프레임 중 floor_letter_need 번
    #   같으면 '이 갈림길에 들어왔다'로 확정 -> 표지(YOLO)·회피 노드 차로보다 우선 (차가 실제로 있는 쪽). M7 끝 ~ M2_EXIT
    'floor_letter_use': True,
    'floor_need_exit_agree': True,   # 11/15: 회피가 끝난 차로(A/B)를 알면 바닥 글자가 같을 때만 씀 (글자 잘못 읽어 반대 갈래로 가던 것)
    'floor_letter_window': 6,
    'floor_letter_need': 3,
    # 11/15 실차 '라바콘 끝난 뒤 커브에서 시작 차로 선을 따라가 꼬임': 좌상단 코너(R640, 오른쪽 90°)는 바깥쪽 바로 옆이 출발 차로라
    #   크게 돌면 그 차로 선이 인식 대상이 됨 -> 차선 인식 대신 도면 경로로 강제: M1 시작(라바콘 끝)부터 직선 -> R640 원호 90° ->
    #   직선을 IMU 방향 + 오도메트리로 따라감 (차선 노드 heading 모드). 다 돌고(m1_force_end_deg) 경로에 붙으면 차선 인식으로
    'm1_force_corner': True,
    'm1_force_r_mm': 640.0,
    'm1_force_ld_mm': 350.0,
    'm1_force_end_deg': 85.0,
    'm1_force_tol_mm': 70.0,
    # 11/19 공식 연습 1~4차 '라바콘 뒤 코너에서 출발 차로를 내 차로로 보고 감':
    #   (1) 도면 위치 s가 M7 전후진·방향 판정 오류로 수 m 뒤처져 코너 시작을 3150 mm(09)·11650 mm(08) 앞으로 잡음 -> 원호가 늦게 시작.
    #       도면으로 잡은 코너까지 거리가 m1_corner_start_mm 와 이보다 더 다르면 도면을 버리고 m1_corner_start_mm (라바콘 끝 기준).
    #   (2) lane_first라 선이 보이면 차선 주행 -> 바깥 선만 보며 k -0.38~-0.76으로 덜 꺾어(원호는 -1.56) 출발 차로 쪽으로 나감.
    #       원호 구간에서 차선 조향이 원호 곡률의 m1_lane_min_turn 배보다 덜 꺾으면 강제 원호(IMU heading)로. 0 = 예전 lane_first 그대로
    'm1_corner_start_max_dev_mm': 700.0,
    'm1_lane_min_turn': 0.7,
    'm1_lane_min_hold': 6,           #   한 번 원호로 바꾸면 최소 이만큼 틱(20 Hz) 원호 유지 (왔다갔다 방지)
    'lane_first': True,              # 11/16: 강제 경로(합류 S자·라바콘 뒤 원호)는 선을 놓쳤을 때만, 선이 보이면 차선 주행
    'lane_first_lost_frames': 5,     #   이만큼 연속으로 선을 놓쳐야 강제 경로로
    'm1_lane_back_frames': 1,        # corner_fix (실험): 4번(M1) 강제 원호 중 선이 이만큼 연속 보여야 차선으로 복귀. 1 = 기존 동작
    # 11/18: M2 CSV 재생이 끝나면 그냥 일반 차선 주행. 한쪽 선 모드(left/right_only)·좁은 폭(460)·직선 방향 붙잡기·
    #   합류 강제 S자(m2_merge)를 전부 건너뜀. M2_EXIT의 좌하단 코너 미리보기는 그대로. false면 예전 동작
    'm2_plain_lane': True,
    # 11/19: M2 CSV 재생 뒤 섬 기준 차선 (차선 노드 island_ref): A면 섬 왼쪽 변을 오른쪽 선으로(right_only),
    #   B면 섬 오른쪽 변을 왼쪽 선으로(left_only), 바닥 글자(A/B = 차로 가운데)가 보이면 그 위치로 섬 변을 고름.
    #   1119_09: 일반 차선이 섬 양쪽 변을 한 차로로 묶어 섬 위로 감. false = 예전 일반 차선
    'm2_island_ref': True,
    # 11/19: M2 끝(섬 끝 + m2_exit_after_island_mm)·M2_EXIT 코너 시작을 도면 위치로 잡는데, 도면 위치가 M7 전후진으로
    #   수 m 뒤처지면(1119_09: M2 50초, 합류·코너 미리보기가 엉뚱한 곳) -> M2 시작부터 간 거리로 계산한 값과
    #   이보다 더 다르면 거리 쪽을 씀. M2 시작 = 섬 앞 끝 + (-m7_to_tip_mm). 0 = 예전(도면 우선)
    'm2_route_max_dev_mm': 700.0,
    'm2_island_len_mm': 1745.0,        # 도면: 섬 앞 끝(x 4268) -> 섬 뒤 끝(x 2523)
    'm2_taper_len_mm': 554.0,          # 도면: 섬 뒤 끝 -> 합류 끝(x 1969)
    'm2_island_lane_width_mm': 520.0,   # 섬 변 ~ 차로 가운데 = 이 값의 반 (차로 500 + 선 20)
    'fork_normal': True,             # 11/16: 갈래 안(섬 앞 끝 + fork_normal_after_mm)부터 양쪽 선 보통 차선 모드
    'fork_normal_after_mm': 300.0,
    'fork_normal_after_dist_mm': 700.0,   #   도면 위치가 없으면 갈림길 시작부터 이 거리 뒤
    'm2_merge': True,
    'm2_merge_plain': False,           # 11/20 yubin: m2_plain_lane에서도 합류 강제 S자 (params.yaml에서 켬). sim_1: 직선 그대로 가 합류 사선 바깥선을 넘음
    'm2_merge_before_end_mm': 500.0,   # 섬 왼쪽 끝(m2_island_end)보다 이만큼 앞에서 시작 (자기 차로 안에서 먼저 안쪽으로)
    'm2_merge_from_tip_mm': 1245.0,    # 도면 위치가 없을 때: M2 시작(섬 앞 끝 근처)부터 이만큼 가서 시작 (섬 길이 1745 - 500)
    'm2_merge_len_mm': 1100.0,
    'm2_merge_shift_mm': 520.0,        # A/B 차로 중심 -> 합류 차로 중심 (도면 차로 간격)
    'm2_merge_ld_mm': 350.0,           # 옆 위치 따라가기 전방 주시
    'm2_merge_max_deg': 45.0,          # 직선 방향에서 이보다 틀지 않음
    'm2_merge_tol_mm': 60.0,           # 끝: 다 옮겼고 옆 오차 이 안 + 방향이 m2_merge_end_deg 안 (또는 길이 + 300 mm)
    'm2_merge_end_deg': 6.0,
    'm2_exit_after_island_mm': 200.0,  # M2_EXIT 시작 = 섬 왼쪽 끝(m2_island_end) + 이만큼 (도면 위치). 도면에 없으면 예전처럼 합류 끝
    #   1114_01 리플레이(차선 노드에 실제 영상): 섬 끝부터 normal이면 M2 뒤쪽 선 없음 128 -> 104프레임, 코너 161 -> 0프레임
    'm8_corner_preview': True,       # 11/12 밤: M2 끝 뒤 좌하단 코너도 같은 IMU 코너 미리보기
    # 11/18 실차: A는 합류 뒤 코너에서 크게 꺾어 안쪽 선을 밟고, B는 합류 왼쪽 비스듬한 곳을 못 따라 직진.
    #   원인 1) M2_EXIT는 합류 테이퍼 한가운데(섬 끝 + 200)에서 시작 -> 코너 미리보기 기준 방향을 그때의 '비스듬한' 방향으로 잡아
    #           곧게 펴는 것만으로 '코너를 5° 돌았다'가 되어 코너 앞에서 바로 코너 곡률 전부 (A 일찍 꺾음, B 왼쪽을 오른쪽 코너가 덮음)
    #   원인 2) 미리보기가 켜진 동안 코너 앞(미리보기 곡률 0)에서도 차선과 0을 80% 섞어 차선 조향이 20%로 줄었음
    'm2x_corner_axis': True,         # 기준 방향 = 3차선 직선 방향(없으면 M2 시작 방향), 코너 쪽으로 돈 각만 셈
    'm2x_corner_gate_mm': 200.0,     #   코너 시작(거리) - 이만큼 전에는 각도로 코너 시작이라 보지 않음 (합류 비스듬은 거리로만)
    'm2x_zero_lane': True,           # 코너 앞(미리보기 곡률 0)은 차선 조향 그대로 (차선 노드 route_corner_zero_lane)
    # 11/19 실차 '코너를 아직 크게 돎', 라바콘 첫 짝에 부딪힘: 코너 미리보기 구간(M7 직전·라바콘 직전 코너)만 lookahead를 줄여
    #   선에 바로 반응 (차선 노드 corner_lookahead 700은 꺼짐). 실제 도면 모양(안쪽 R600·바깥 R670) 시뮬레이션:
    #   지금 시작선 옆 평균 49 mm·방향 5.3°(최악 9.9°) -> 500 + 미리보기 x1.1: 38 mm·1.2°(최악 6.0°), 선 닿음 0/12
    'corner_preview_lookahead_mm': 0.0,   # 0 = 그대로. params.yaml에서 정함
    'm2_merge_speed': 0.0,           # 합류 S자 속도 상한 (0 = approach_speed 그대로). params.yaml에서 정함
    'm2x_speed': 0.0,                # M2_EXIT(합류 끝 ~ 라바콘 시작선) 속도 상한 (0 = 그대로)
    'm8_keep_fork_mm': 2500.0,       # 11/12 밤 실차: M2 끝(합류 테이퍼)에서 normal로 바꾸자마자 선을 놓쳐 차선 무시하고 감.
                                     #   M2에서 넘어오면 한쪽 선 모드를 코너를 다 돌 때까지 유지 (이 거리는 안전 상한, 0이면 끔):
                                     #   합류 ~ 코너 앞: 갈림길에서 따라가던 바깥선(A=왼쪽, B=오른쪽)
                                     #   코너(미리보기 앞 600 mm 안에 코너 곡률) ~ 코너 다 돎: 코너 안쪽 선 (오른쪽 코너 = right_only)
                                     #   코너를 다 돌면(IMU 85°, 코너 미리보기 끝) normal
    'm8_both_frames': 0,             # >0이면 양쪽 선이 이만큼 연속 보일 때 코너 전이라도 normal (11/12 밤: 끝까지 한쪽 선 -> 0)
    # 11/12 밤 실차: B차로에서 섬 끝 안쪽 선을 밟고 들어가 합류 사선을 그대로 뚫고 나감 / 합류 뒤 코너에서 선을 밟고 놓침
    'fork_lane_width_mm': 460.0,     # M2·합류에서 한쪽 선 모드의 차로 폭. 바깥선에서 230 mm (기본 530 = 265) -> 섬 쪽 선에서 멀어짐
    'fork_lost_search': True,        # M7 차선 구간·M2·합류에서 선을 놓치면 직선 방향으로 풀고, 직전 조향 반대쪽 30°부터 찾아봄 (차선 노드 lost_search)
    'm8_corner_k_gain': 1.0,         # M2 뒤 코너 미리보기 곡률 배율. 11/13 실차: 1.15(R557)는 차로(R640)보다 작게 돌아 안쪽 노란 선을 밟음 -> 1.0
    # 11/13 실차: 라바콘 뒤 좌상단 코너(R640 오른쪽 90°)에서 미리보기가 없어 0.3 m/s 보통 차선으로 직진하다 늦게 꺾고 안쪽 선을 밟음
    #   -> M8에서 넘어올 때 같은 IMU 코너 미리보기 + 안쪽 선 모드 + 천천히. 기준 방향 = 라바콘 세로 직선(3차선 방향 + 90°)
    'm1_corner_preview': True,
    'c1_corner_preview': True,       # 11/19: 1번 코너(출발 직선 끝) IMU 코너 미리보기 (도면 c1_corner_start 필요). false = 예전
    'm1_corner_start_mm': 200.0,     # 도면 위치를 모를 때: M1 진입 -> 코너 시작 (도면: 끝선 -> 코너 280 mm, handover가 끝선 근처)
    'm1_corner_k_gain': 1.0,
    'm1_corner_done_deg': 85.0,      # 11/19: 4번(M1) 코너 미리보기 끝 각도 (1~3번 m7_corner_done_deg와 따로)
    'm7_corner_k_gain': 1.0,
    'm1_corner_speed': 0.20,         # 코너 다 돌 때까지 속도
    'm1_inner_after_mm': 0.0,        # 11/19: 4번 코너 끝 뒤 이만큼은 안쪽 선만 (위쪽 출발 차로 선 착각 방지). 0 = 예전
    # 코너(M8·M1 미리보기) 동안 차로 중심에서 바깥쪽으로 이만큼 (안쪽 선 밟던 것, 11/13). 차선이 보일 때만 효과 (target_offset_mm)
    'corner_outward_mm': 30.0,
    'm8_hold_speed': 0.15,           # 위 한쪽 선 모드 동안 속도 (M7 직전 코너처럼 코너에서 직진하지 않게 천천히)
    'm8_inner_ahead_mm': 600.0,      # 미리보기에서 이 거리 안에 코너 곡률이 있으면 안쪽 선으로 바꿈
    'm8_corner_start_mm': 650.0,     # M8 진입(합류 끝 m2_merge_end) -> 코너 시작 (도면 20272 -> 20921)      # 차선 노드 route_ff_preview_mm 와 같게
    'm7_corner_give_up_mm': 1500.0,  # 코너 시작 + 호 길이 + 이만큼 가도 IMU가 다 돈 걸 못 보면 그만 (예전 800)
    'm7_corner_plain_lane': True,    # 코너 동안은 3차선 설정 대신 보통 차선 설정 (흰 선을 차선으로 보지 않음)
    'm8_corner_turn_deg': 75.0,      # 0 = 안 씀
    'm8_corner_straight_deg': 4.0,
    'm8_corner_window_mm': 150.0,
    'm8_corner_fail_mm': 300.0,      # 11/14 밤: 100 -> 300 원복 (d4d4c70 되돌림). 예전 메모: 11/15 저녁 300 -> 100. 도면상 시작선 + 이만큼인데 코너를 못 알아채면 예전 규칙 (300이면 콘 사이에 이미 들어감)
    'cones_zone_mm': 2700.0,         # 라바콘 모드 최대 거리 (구간 2100 + 진입 300 + 여유)
    'cones_end_after_mm': 150.0,     # 끝선(앞바퀴)을 이만큼 지나면 obstacle 노드 판단과 상관없이 복귀
    'cones_handover_mm': 400.0,      # 차선을 다시 잡길 기다리는 최대 거리 (끝선 뒤 바로 코너)
    'cones_handover_frames': 4,      # 차선이 연속 이만큼 틱 잡히면 차선 주행으로
    # 라바콘이 더 안 보이면 끝: 앞 cones_gone_mm 안(구간 폭 안) 콘이 cones_gone_sec 동안 0개.
    # 콘을 한 번이라도 봤고 라바콘 모드로 cones_min_mm 이상 간 뒤에만 (시작 직후 아직 못 본 것과 구분)
    'cones_gone_mm': 800.0,
    'cones_gone_sec': 0.5,
    'cones_min_mm': 500.0,
    # M2 갈림길
    'fork_len_mm': 2300.0,       # 도면: 섬 오른쪽 끝부터 왼쪽 합류 테이퍼 끝까지 약 2.4m
    'fork_adj_max_mm': 1500.0,   # 도면 위치가 없을 때 M2 끝 거리를 'M2 시작 ~ 섬 앞 끝'만큼 보정하는 최대값 (/obstacle/state m7_to_tip_mm)
    'fork_seek_timeout_sec': 12.0,  # 규정상 15초 안에 진입해야 함
    'fork_default': 'A',         # 끝까지 못 읽으면 찍을 방향. TODO 팀 정책
    # M1 신호등
    # M4 주차: 주차 표지(parking)를 보면 주차 노드가 앞 선을 받기 시작한다 (서지 않고 서행으로 진입).
    # 표지를 못 봐도 도면상 주차 앞 선 m4_arm_before_mm 앞부터는 받는다 (2순위). 경로 지도가 없으면 항상 받음
    'm4_arm_before_mm': 800.0,
    # 11/12: true면 M4에 들어오자마자 주차 노드에 넘긴다 (표지·도면 위치를 안 기다림). 주차 노드가 1·3번 짝을 찾아 시작.
    # 표지 인식 실패 + 도면 위치 틀어짐이면 주차 노드가 IDLE인 채 지나가던 문제 (params에서 켬)
    'm4_direct': False,
    # 11/12: 주차 노드에 넘긴 뒤 차선 설정에 넣을 왼쪽 오프셋 (mm). 주차 노드가 따로 보내면 이 노드가 1초마다
    # /lane/config를 다시 보내며 덮어써서(차선 노드는 통째로 교체) 왼쪽 붙기가 안 됐음. 0 = 안 넣음
    'm4_target_offset_mm': 0.0,
    'stop_on_green_arrival': True,  # 도착 때 녹색이어도 일단 서고 녹색 확인 후 출발 (안전 기본값)
    'green_go_delay_sec': 0.3,
    # 도면 경로 지도 (track_route.yaml). mission_node가 points를 넣어 준다.
    # route_s가 들어오면 위의 *_len_mm 거리 대신 도면 위치로 구간을 넘긴다.
    'route_points': {},
    'route_slow_before_mm': 300.0,   # 서행 선 이만큼 앞에서 표지를 못 봐도 서행 시작
    'route_island_margin_mm': 300.0, # 섬 끝 이만큼 앞에서 M7 -> M2
}


class Vote:
    """최근 window 프레임 중 need 번 이상 보이면 True (D-Racer의 다수결 방식)."""

    def __init__(self, window: int = 8, need: int = 5):
        self.buf = deque(maxlen=window)
        self.need = need
        self.last_t = None       # 같은 프레임에 두 번 투표하지 않게

    def push(self, seen: bool, t: Optional[float] = None) -> bool:
        if t is not None and t == self.last_t:
            return self.on
        self.last_t = t
        self.buf.append(bool(seen))
        return self.on

    @property
    def on(self) -> bool:
        return sum(self.buf) >= self.need

    def reset(self) -> None:
        self.buf.clear()
        self.last_t = None


@dataclass
class Inputs:
    t: float                                   # 초 (단조 시계)
    dist_mm: float                             # 누적 주행 거리 (오도메트리)
    yaw: Optional[float] = None
    detections: Dict[str, float] = field(default_factory=dict)  # 이번 프레임 클래스별 최고 conf
    new_detection_frame: bool = False          # 인식 노드가 새 프레임을 냈는지 (투표는 이때만)
    lane: dict = field(default_factory=dict)   # /lane/status
    obstacle: dict = field(default_factory=dict)  # /obstacle/state
    parking: dict = field(default_factory=dict)   # /parking/state
    start: bool = False
    confirmed_classes: Optional[dict] = None   # 발행부에서 투표를 통과한 지속 요청. None=기존 원시 검출 경로
    route_s: Optional[float] = None            # 도면 경로상 위치 (route.RouteTracker). 없으면 거리 기준
    tilt_rate: Optional[float] = None          # 지난 틱 이후 IMU 앞뒤·좌우 회전 속도 최대값 (rad/s). 방지턱 감지


@dataclass
class Command:
    source: str = 'stop'
    lane: dict = field(default_factory=lambda: {'mode': 'normal'})
    note: str = ''


class StopApproach:
    """정지선 앞 정밀 정지 (판정은 앞바퀴 기준 ±5cm).

    카메라가 정지선을 보는 동안은 그 거리를 쓰고, BEV 아래로 사라진 뒤에는
    마지막으로 본 거리에서 오도메트리 주행 거리를 빼서 남은 거리를 추정한다.
    """

    def __init__(self, target_mm: float, brake_mm: float, lost_ok_mm: float,
                 key: str = 'crosswalk_lines_mm', freeze_mm: float = 0.0,
                 route_gate_mm: float = 0.0, camera_first: bool = False):
        self.target_mm = target_mm
        self.brake_mm = brake_mm
        self.lost_ok_mm = lost_ok_mm
        self.key = key           # 기본: 바로 뒤에 횡단보도 막대가 있는 가로선만 정지선으로 봄
        self.freeze_mm = freeze_mm          # 0이면 끔 (예전 동작)
        self.route_gate_mm = route_gate_mm  # 0이면 끔
        self.passed_mm = 120.0              # 정지선을 이만큼 지났으면 바로 정지 (stop_passed_mm)
        self.seen_d = None       # 마지막으로 본 정지선 거리 (앞바퀴 기준)
        self.seen_at = None      # 그때의 누적 주행 거리
        self.route_line_s = None  # 도면상 정지선 s. 카메라가 선을 못 볼 때만 쓰는 예비값
        self.how = None          # 'camera' | 'route' (디버그)
        # 11/15: 횡단보도 정지선(막대가 뒤따르는 선)은 카메라가 1순위. 도면 예측과 gate 넘게 달라도
        #   가장 가까운 카메라 선을 쓴다 (도면은 여러 선 중 고를 때만). 1114_03/04: 도면 위치가 1~1.8 m 틀려
        #   맞는 카메라 선을 버리고 도면으로 섬 -> 정지선 0.85 m 앞에서 섬
        self.camera_first = camera_first
        self.persist_frames = 3
        self.allow_route_start = True   # False: 카메라로 못 봤으면 도면 위치로 출발점을 잡지 않음 (준비 전 따라가기용)
        self.camera_force_mm = 0.0      # 11/19: >0이면 도면 무시, 이 거리 안 카메라 선을 3프레임 연속이면 씀 (M1 빨간불)
        self.camera_drop_mm = 550.0     #   그때 가짜 선 거르기: 이보다 멀리 있어야 할 선이
        self.camera_drop_after_mm = 150.0   #   이만큼 달리는 동안 안 보이면 버림

    def _route_pred(self, inp: Inputs) -> Optional[float]:
        if self.route_line_s is None or inp.route_s is None:
            return None
        return self.route_line_s - inp.route_s

    def remaining(self, inp: Inputs) -> Optional[float]:
        # 미션 시작선도 흰 가로선이라, 횡단보도 막대가 뒤따르는 선만 쓴다.
        # (lane 노드가 그 키를 안 주는 옛 버전이면 모든 가로선으로 대신함)
        lines = inp.lane.get(self.key)
        if lines is None:
            lines = inp.lane.get('stoplines_mm') or []
        frozen = (self.freeze_mm > 0.0 and self.seen_d is not None
                  and self._predicted(inp) < self.freeze_mm)
        if self.camera_force_mm > 0.0 and lines and (self.seen_d is None or self.how != 'camera'):
            # 11/19: 아직 카메라로 못 잡았거나(도면만) 도면으로 시작했으면, 가장 가까운 카메라 선이 3프레임 연속일 때 그 선으로
            near = min(lines)
            if 0.0 < near <= self.camera_force_mm:
                c = self._camera_persist(near, inp)
                if c is not None:
                    self.seen_d, self.seen_at, self.how = c, inp.dist_mm, 'camera'
                    self.forced = True
                else:
                    return near                         # 확정 전(1~2프레임)에도 도면 대신 카메라 거리로 판단
        if self.camera_force_mm > 0.0 and self.how == 'camera':
            # 카메라로 잡은 뒤에는 도면 예측과 비교해 갈아타지 않음 (아래 route gate 갈아타기 건너뜀)
            matched = False
            if lines and not frozen:
                ref = self._predicted(inp)
                cand = min(lines, key=lambda d: abs(d - ref))
                if abs(cand - ref) < 150.0:
                    self.seen_d, self.seen_at = cand, inp.dist_mm
                    matched = True
            # 가짜 선 거르기 (도면 없이): 진짜 선은 화면 아래(약 350 mm)까지 보이다 사라진다.
            #   아직 멀리(> camera_drop_mm) 있어야 할 선이 camera_drop_after_mm 넘게 안 보이면 버리고 다시 잡는다
            if (not matched and not frozen and self._predicted(inp) > self.camera_drop_mm
                    and inp.dist_mm - self.seen_at > self.camera_drop_after_mm):
                self.seen_d = self.seen_at = self.how = None
                self._rej = None
                return None
            travelled = inp.dist_mm - self.seen_at
            if travelled > self.seen_d + self.lost_ok_mm:
                return None
            return self.seen_d - travelled
        if lines and not frozen and self.seen_d is not None and self.route_gate_mm > 0.0:
            # 11/15: 처음 잡은 선이 가짜(횡단보도 막대로 잘못 표시된 다른 선)였으면, 도면 예측 gate 안에 있고
            #   지금 따라가는 선과는 gate 넘게 다른 카메라 선이 보일 때 그 선으로 갈아탄다 (1114_04 리플레이: 코너 끝 가짜 선)
            rp, pred = self._route_pred(inp), self._predicted(inp)
            if rp is not None and abs(pred - rp) >= self.route_gate_mm:
                near = [d for d in lines if abs(d - rp) < self.route_gate_mm and abs(d - pred) > 150.0]
                if near:
                    d = min(near, key=lambda x: abs(x - rp))
                    self.seen_d, self.seen_at, self.how = d, inp.dist_mm, 'camera'
        if lines and not frozen:
            if self.seen_d is not None:
                ref, gate = self._predicted(inp), 150.0
            else:
                ref = self._route_pred(inp) if self.route_gate_mm > 0.0 else None
                gate = self.route_gate_mm
            if ref is None:
                cand = lines[0]
            else:
                cand = min(lines, key=lambda d: abs(d - ref))
                if abs(cand - ref) >= gate:
                    cand = self._camera_persist(lines[0], inp) if (self.camera_first and self.seen_d is None) else None
            if cand is not None:
                self.seen_d, self.seen_at, self.how = cand, inp.dist_mm, 'camera'
        if self.seen_d is None and self.freeze_mm > 0.0 and self.route_gate_mm > 0.0 and self.allow_route_start:
            rp = self._route_pred(inp)
            if rp is not None and rp < self.freeze_mm:
                # 이미 카메라 사각지대 근처에서 시작: 도면 위치를 출발점으로 오도메트리로 센다
                self.seen_d, self.seen_at, self.how = rp, inp.dist_mm, 'route'
        if self.seen_d is None:
            if self.route_line_s is not None and inp.route_s is not None:
                return self.route_line_s - inp.route_s
            return None
        travelled = inp.dist_mm - self.seen_at
        if travelled > self.seen_d + self.lost_ok_mm:
            return None
        return self.seen_d - travelled

    def _camera_persist(self, d: float, inp: Inputs) -> Optional[float]:
        """도면 예측과 다른 카메라 선은 같은 선이 persist_frames 연속(오도메트리로 이어서 150 mm 안)일 때만 받는다."""
        prev = getattr(self, '_rej', None)
        if prev is not None and abs((prev[0] - (inp.dist_mm - prev[1])) - d) < 150.0:
            n = prev[2] + 1
        else:
            n = 1
        self._rej = (d, inp.dist_mm, n)
        return d if n >= self.persist_frames else None

    def _predicted(self, inp: Inputs) -> float:
        if self.seen_d is None:
            return 0.0
        return self.seen_d - (inp.dist_mm - self.seen_at)

    def should_stop(self, inp: Inputs) -> bool:
        rem = self.remaining(inp)
        if rem is None and self.seen_d is not None and self.seen_at is not None:
            rem = self.seen_d - (inp.dist_mm - self.seen_at)       # 놓친 뒤에도 '이미 지났나'는 셈
            return rem < -self.passed_mm                        # 정지선을 지났음 -> 끝선까지 가지 말고 섬
        return rem is not None and rem <= self.target_mm + self.brake_mm


class MissionFSM:
    def __init__(self, params: Optional[dict] = None):
        self.p = dict(DEFAULT_PARAMS)
        if params:
            self.p.update(params)
        self.votes: Dict[str, Vote] = {}
        self.fork_choice: Optional[str] = None      # 'A' | 'B'
        self.m7c = None                              # (진입 yaw, 진입 거리): M7 직전 코너 IMU 지킴
        self.fork_adj_mm = 0.0                       # M2 끝 거리 보정 (섬 앞 끝까지 남은 거리)
        self.state = ''
        self.sub = ''
        self.enter_t = self.enter_dist = 0.0
        self.sub_t = self.sub_dist = 0.0
        self.stop: Optional[StopApproach] = None
        self.stop_line_name: Optional[str] = None    # 지금 정지 접근 중인 도면 선 이름 (위치 다시 잡기용)
        # 정지 준비 전에 미리 재 둔 정지선: 지그재그 중 M3 정지선(11/15 밤), 신호등 준비 전 M1 정지선(11/15)
        self.pre_stop: Optional[StopApproach] = None
        self.pre_stop_name: Optional[str] = None
        self.count = 0                               # 서브 상태 안에서 쓰는 연속 카운터
        self.log = []
        self.parking_sign = False                    # 주차 표지를 봤나 (M1~M4 동안 투표, 상태가 바뀌어도 유지)
        self.parking_armed = False                   # 주차 노드에 앞 선을 받으라고 알림 (/mission/state)
        self.parking_result = None                   # 주차 노드 결과 (PARKED / FAIL_... ), 끝난 뒤에만
        self.bump_at = None                          # 방지턱을 느낀 누적 거리
        self.line = None                             # 지그재그·라바콘 시작선 통과 감시 (StopApproach)
        self.cones_seen = False
        self.cones_gone_t = None
        self.m8_corner = CornerWatch(self.p.get('m8_corner_turn_deg', 75.0), self.p.get('m8_corner_straight_deg', 4.0),
                                     self.p.get('m8_corner_window_mm', 150.0))
        self._last_dist = None                       # odom 점프 감시용
        self._dist_adj = 0.0
        # 위치 다시 잡기 (11/10): 카메라로 확인한 미션 선을 앞바퀴가 지나거나 그 앞에 서면
        # (도면 선 이름, 남은 거리 mm)를 남긴다. mission_node가 pop_anchor()로 받아 경로 위치 s를 그 선 기준으로 다시 맞춘다
        self.anchor = None
        self._anchored = set()
        self._goto(self.p['start_state'], Inputs(t=0.0, dist_mm=0.0), 'init')

    # --- 공통 도구 ------------------------------------------------------------
    def _goto(self, state: str, inp: Inputs, why: str = '') -> None:
        if state not in SEQUENCE and state not in EXTRA_STATES:
            raise ValueError(f'unknown state {state}')
        self.log.append(f'{self.state or "-"} -> {state} {why}'.strip())
        prev_state = self.state
        self.state = state
        self.enter_t, self.enter_dist = inp.t, inp.dist_mm
        if state == 'M7_OBSTACLE':
            # 횡단보도에서 넘어올 때만 (연습으로 set_state M7이면 이미 코너 뒤 직선에 놓음)
            use = self.p.get('m7_corner_done_deg', 85.0) > 0 and why not in ('manual', 'init') and inp.yaw is not None
            start = self.p['m7_corner_start_mm']
            xw = self._rp('m3_crosswalk_end')
            if use and inp.route_s is not None and xw is not None:
                # 11/15 저녁: 횡단보도에서 늦게(끝선 근처) 서면 M7 시작이 밀려 코너 보완도 늦었다 -> 도면 위치가 있으면 끝선 기준으로
                start = max(0.0, min(1500.0, xw + self.p['m7_corner_start_mm'] - inp.route_s))
            self.m7c = (inp.yaw, inp.dist_mm, 'M7_OBSTACLE', start) if use else None
            # 3차선·갈림길 직선 방향 = 코너 진입 방향에서 90° 돈 방향 (M2·합류에서 선을 놓쳤을 때 붙잡을 방향)
            self.axis_yaw = (inp.yaw + self.p['m7_corner_dir'] * math.pi / 2.0) if use else None
            # 11/20 yubin: 연습으로 M7부터면 차를 직선에 바로 놓음 -> M7 첫 yaw를 직선 방향으로 (없으면 M2·합류·M8이 재생 끝 방향(최대 20° 틀어짐)을 씀)
            self.m7_placed = why in ('manual', 'init')
            self.m2m = None                          # 합류 강제 S자 (다음 판 위해 지움)
            self.floor_fork, self.floor_buf = None, []
            self.floor_blocked = False
            self._floor_conflict_logged = False
        from_exit = state == 'M8_CONES' and prev_state == 'M2_EXIT' and self.p.get('m2_exit', True)
        if state == 'M2_EXIT':
            # 11/14: 합류 끝 ~ 라바콘 시작선. M8 접근에 있던 좌하단 코너 IMU 미리보기·코너 지킴을 여기로 옮김
            use = (self.p.get('m8_corner_preview', True) and self.p.get('m7_corner_done_deg', 85.0) > 0
                   and why not in ('manual', 'init') and inp.yaw is not None)
            # 섬 끝에서 시작하므로 합류 끝까지 남은 거리만큼 코너 시작·거리 예비값을 늦춘다
            me = self._rp('m2_merge_end')
            dflt = max(0.0, self.p.get('m2_taper_len_mm', 554.0) - self.p.get('m2_exit_after_island_mm', 200.0))
            self.m2x_extra = (max(0.0, me - inp.route_s) if me is not None and inp.route_s is not None
                              else dflt)
            dev = self.p.get('m2_route_max_dev_mm', 0.0)
            if dev > 0 and abs(self.m2x_extra - dflt) > dev and why not in ('manual', 'init'):
                # 11/19: 도면 위치가 틀리면 합류 끝까지 거리를 도면 규격으로 (M2 끝을 거리로 잡았을 때와 같은 기준)
                self.log.append(f'M2_EXIT corner start: route extra {self.m2x_extra:.0f}mm ignored -> {dflt:.0f}mm')
                self.m2x_extra = dflt
            yaw0 = inp.yaw
            if self.p.get('m2x_corner_axis', True):
                ax = getattr(self, 'axis_yaw', None)
                if ax is None:
                    ax = getattr(self, 'm2_yaw', None)
                yaw0 = ax if ax is not None else inp.yaw
            self.m7c = (yaw0, inp.dist_mm, 'M2_EXIT', self.p['m8_corner_start_mm'] + self.m2x_extra) if use else None
            use = self.p.get('m8_corner_turn_deg', 75.0) > 0 and why not in ('manual', 'init')
            self.m8_corner.start(inp.yaw if use else None, inp.dist_mm)
            self.m8_fork_hold = False
            self.m8_inner_on = False
            self.m8_inner_mode = None
            self.m8_from_exit = False
            self._m8_seen, self._m8_full_at, self._m8_line_fresh = None, None, False
        if state == 'M8_CONES' and from_exit:
            # M2_EXIT가 시작선까지 데려옴 -> 접근 단계 없이 바로 콘 따라가기 (코너 미리보기·한쪽 선 모드 없음)
            self.m7c = None
            self.m8_corner.start(None, inp.dist_mm)
            self.m8_fork_hold = False
            self.m8_both_n = 0
            self.m8_inner_on = False
            self.m8_inner_mode = None
            self.m8_from_exit = True
        elif state == 'M8_CONES':
            self.m8_from_exit = False
            self._m8_seen, self._m8_full_at, self._m8_line_fresh = None, None, False
            # 11/12 밤 실차: M2 끝(합류) 뒤 좌하단 코너(R640 오른쪽 90°)에서도 차선을 놓치고(NONE->hold->stop) 멋대로 감
            #   -> M7 직전 코너와 같은 IMU 코너 미리보기 (M2에서 넘어올 때만)
            use = (self.p.get('m8_corner_preview', True) and self.p.get('m7_corner_done_deg', 85.0) > 0
                   and why not in ('manual', 'init') and inp.yaw is not None)
            self.m7c = (inp.yaw, inp.dist_mm, 'M8_CONES', self.p['m8_corner_start_mm']) if use else None
            # 앞 미션(M2)에서 넘어올 때만 코너 지킴. 연습으로 바로 M8(set_state)이면 이미 코너 뒤에 놓음
            use = self.p.get('m8_corner_turn_deg', 75.0) > 0 and why not in ('manual', 'init')
            self.m8_corner.start(inp.yaw if use else None, inp.dist_mm)
            self.m8_fork_hold = (self.p.get('m8_keep_fork_mm', 0.0) > 0 and self.fork_choice in ('A', 'B')
                                 and why not in ('manual', 'init'))
            self.m8_both_n = 0
            self.m8_inner_on = False
            self.m8_inner_mode = None
        if state == 'M2_FORK':
            self.m2m = None
            self._m2_dev_logged = False
        if state == 'M5_BUMP':
            # 출발 직선 방향 (1번 코너 IMU 기준). 연습으로 바로 M5면 없음
            self.c1_axis = inp.yaw if why not in ('manual', 'init', 'skip') else None
        if state == 'M6_ZIGZAG':
            self.pre_stop, self.pre_stop_name = None, None
            self.zz_at = None
            self.zz_capped = False
            self.ped_seen = False                        # M6~M3에서 본 보행자 표지 (상태가 바뀌면 투표는 지워지므로 따로 기억)
            # 11/19 1번 코너(우상단 R640 오른쪽 90°)도 다른 코너처럼 IMU 코너 미리보기 (시작 = 도면 c1_corner_start).
            #   실차: 선 보고 늦게 꺾어 바깥 선 밟음 -> 선 놓침 -> 보호 없어 직진 이탈. 도면 위치 없으면 안 켬 (거리 추측 안 함)
            cs = self._rp('c1_corner_start')
            self.m7c = None
            if (self.p.get('c1_corner_preview', True) and self.p.get('m7_corner_done_deg', 85.0) > 0
                    and why not in ('manual', 'init', 'skip') and inp.yaw is not None
                    and cs is not None and inp.route_s is not None):
                start = max(0.0, cs - inp.route_s)
                ax = getattr(self, 'c1_axis', None)
                # 기준 = 출발 직선 방향. 없으면 아직 직선 위(코너 300 mm 이상 앞)일 때만 지금 방향
                yaw0 = ax if ax is not None else (inp.yaw if start >= 300.0 else None)
                if yaw0 is not None:
                    self.m7c = (yaw0, inp.dist_mm, 'M6_ZIGZAG', start)
                    self.log.append(f'M6 corner1 preview on (start {start:.0f}mm)')
        if state == 'M1_TRAFFIC':
            use = (self.p.get('m1_corner_preview', True) and self.p.get('m7_corner_done_deg', 85.0) > 0
                   and why not in ('manual', 'init', 'skip') and inp.yaw is not None)
            if use:
                ax = getattr(self, 'axis_yaw', None)
                # 기준 = 라바콘 세로 직선 방향. 라바콘 끝에서 이미 조금 돌았어도 그 직선 기준으로 85°까지 돈다
                yaw0 = ax + self.p['m7_corner_dir'] * math.pi / 2.0 if ax is not None else inp.yaw
                cs = self._rp('m1_corner_start')
                start = (max(0.0, cs - inp.route_s) if cs is not None and inp.route_s is not None
                         else self.p['m1_corner_start_mm'])
                dev = self.p.get('m1_corner_start_max_dev_mm', 0.0)
                if dev > 0 and abs(start - self.p['m1_corner_start_mm']) > dev:
                    self.log.append(f'M1 corner start {start:.0f}mm from route ignored (route s off) -> '
                                    f'{self.p["m1_corner_start_mm"]:.0f}mm')
                    start = self.p['m1_corner_start_mm']
                self.m7c = (yaw0, inp.dist_mm, 'M1_TRAFFIC', start)
                self.m1_inner_on = False
                self.m1_done_at = None
                self.m1fc = None
                self._m1_arc_hold, self._m1_under_logged = 0, False
                if self.p.get('m1_force_corner', True):
                    off = (inp.lane or {}).get('offset_mm')
                    y0 = (-float(off) if off is not None and (inp.lane or {}).get('status') not in (None, 'NONE')
                          and abs(float(off)) <= 150.0 else 0.0)
                    self.m1fc = {'axis': yaw0, 'last': inp.dist_mm, 'x': 0.0, 'y': y0, 'xs': start, 's': 0.0,
                                 'done': False}
                    self.log.append(f'M1 forced corner on (corner {start:.0f}mm ahead, side {y0:+.0f}mm)')
                self.log.append(f'M1 corner preview on (start {start:.0f}mm)')
            else:
                self.m7c = None
        self._sub('', inp)
        for v in self.votes.values():
            v.reset()
        self.stop = None
        self.line = None
        self.bump_at = None
        self.cones_seen, self.cones_gone_t = False, None
        self._anchored = set()

    def m7_corner_k(self, dist_mm: float, yaw: Optional[float], step_mm: float = 100.0, n: int = 14):
        """M7 직전 코너 곡률 미리보기 (앞바퀴에서 0, step, 2step ... 앞, 1/m, 왼쪽 +). 코너가 아니면 None.
        아직 안 돌기 시작했으면(5° 미만) 진입 거리로 코너 시작을 잡고, 돌기 시작했으면 IMU로 남은 각만큼."""
        c = getattr(self, 'm7c', None)
        if c is None or yaw is None or self.state != c[2]:
            return None
        if self.state == 'M8_CONES' and self.sub not in ('', 'approach'):
            return None                                  # 라바콘 구간부터는 콘 따라가기
        if self.state == 'M1_TRAFFIC' and self.sub not in ('', 'approach'):
            return None                                  # 정지선에 섰으면 끝
        if self.state == 'M6_ZIGZAG' and self.sub != '':
            return None                                  # 지그재그 시작선을 지났으면 끝 (지그재그에 코너 곡률 안 넣음)
        yaw0, d0, _, start_mm = c
        p = self.p
        r = p['m7_corner_r_mm']
        turned = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
        since = dist_mm - d0
        if self.state == 'M2_EXIT' and p.get('m2x_corner_axis', True):
            # 11/18: 직선 방향 기준, 코너 쪽으로 돈 각만. 코너 시작 전(합류 비스듬)은 거리로만
            turned = max(0.0, p['m7_corner_dir'] * math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
            if since < start_mm - p.get('m2x_corner_gate_mm', 200.0):
                turned = min(turned, math.radians(4.9))
        arc = r * math.pi / 2.0
        # 11/19 실차: 1~3번은 출구에서 더 돌아 안쪽 -> 일찍 끝냄(m7_corner_done_deg), 4번(M1)은 덜 돌아 바깥 -> 따로
        done = p.get('m1_corner_done_deg', p['m7_corner_done_deg']) if self.state == 'M1_TRAFFIC' else p['m7_corner_done_deg']
        if turned >= math.radians(done) or since > start_mm + arc + p['m7_corner_give_up_mm']:
            self.m7c = None                              # 다 돎 (또는 너무 지나 IMU를 못 믿음) -> 끝
            self.log.append(f'{self.state[:2]} corner preview off (turned {math.degrees(turned):.0f} deg, {since:.0f}mm)')
            return None
        k = p['m7_corner_dir'] * 1000.0 / r
        if self.state in ('M8_CONES', 'M2_EXIT'):
            k *= p.get('m8_corner_k_gain', 1.0)
        elif self.state == 'M1_TRAFFIC':
            k *= p.get('m1_corner_k_gain', 1.0)
        else:
            k *= p.get('m7_corner_k_gain', 1.0)      # 11/15: M7 직전 코너도 배율 (실차가 크게 돎)
        if turned < math.radians(5.0):
            # 차선 노드는 미리보기의 route_ff_preview_mm(300) 앞 값을 쓴다 -> 그만큼 밀어 코너 시작에서 꺾게 (일찍 꺾어 안쪽을 자르지 않게)
            start = start_mm - since + p['m7_corner_lead_mm']
            return [round(k, 4) if i * step_mm >= start else 0.0 for i in range(n)]
        # 11/12 밤 실차: 돌기 시작한 뒤에는 IMU로 다 돌 때까지 미리보기 전체를 코너 곡률로.
        #   예전엔 '남은 호 길이'까지만 넣어서, 남은 각이 약 27° 아래로 줄면 차선 노드가 보는 300 mm 앞이 0이 되어
        #   코너 돕기가 끊겼다 -> 60~70°만 돌고 직진 (실차 m7_live: 52~55°에서 멈춤)
        return [round(k, 4)] * n

    def _anchor(self, name: Optional[str], approach: Optional['StopApproach'], rem: Optional[float]) -> None:
        """카메라로 본 선일 때만, 상태마다 선 하나당 한 번 위치 다시 잡기를 요청."""
        if (name and approach is not None and approach.how == 'camera' and rem is not None
                and name not in self._anchored and self._rp(name) is not None):
            self.anchor = (name, float(rem))
            self._anchored.add(name)
            self.log.append(f'{self.state} anchor {name} (camera, rem {rem:.0f}mm)')

    def pop_anchor(self):
        a, self.anchor = self.anchor, None
        return a

    def _next(self, inp: Inputs, why: str = '') -> None:
        if self.state not in SEQUENCE:
            return
        i = SEQUENCE.index(self.state)
        self._goto(SEQUENCE[min(i + 1, len(SEQUENCE) - 1)], inp, why)

    def _sub(self, name: str, inp: Inputs, why: str = '') -> None:
        self.sub = name
        self.sub_t, self.sub_dist = inp.t, inp.dist_mm
        self.count = 0
        if why:
            self.log.append(f'{self.state}/{name} {why}')

    def _vote(self, inp: Inputs, *classes: str) -> bool:
        """classes 중 하나라도 다수결을 넘으면 True. 투표는 새 인식 프레임에서만."""
        if inp.confirmed_classes is not None:
            return any(c in inp.confirmed_classes for c in classes)
        hit = False
        for c in classes:
            v = self.votes.setdefault(c, Vote(self.p['vote_window'], self.p['vote_need']))
            if inp.new_detection_frame:
                v.push(inp.detections.get(c, 0.0) >= self.p['det_conf'], t=inp.t)
            hit = hit or v.on
        return hit

    def _since(self, inp: Inputs, sub: bool = False):
        t0, d0 = (self.sub_t, self.sub_dist) if sub else (self.enter_t, self.enter_dist)
        return inp.t - t0, inp.dist_mm - d0

    @staticmethod
    def _lane(speed: float, mode: str = 'normal', **kw) -> dict:
        cfg = {'mode': mode, 'speed': round(float(speed), 3)}
        cfg.update(kw)
        return cfg

    def _zz_cap(self, inp: Inputs) -> bool:
        """지그재그 시작선부터 달린 거리로 정지선 직전인가 (m3_zz_cap). 기준점이 없으면 False."""
        at = getattr(self, 'zz_at', None)
        if not self.p.get('m3_zz_cap', True) or at is None:
            return False
        a, b = self._rp('m6_zigzag_line'), self._rp('m3_stop_line')
        total = (b - a) if (a is not None and b is not None) else self.p['m3_zz_to_stop_mm']
        return inp.dist_mm - at >= total - self.p['m3_zz_cap_target_mm'] - self.p['stop_brake_mm']

    def _crosswalk_near(self, inp: Inputs) -> bool:
        lines = inp.lane.get('crosswalk_lines_mm') or []
        return bool(lines) and lines[0] <= self.p['arm_on_crosswalk_mm']

    def _new_stop(self, line: Optional[str]) -> 'StopApproach':
        target = self.p['stop_target_mm']
        if line == 'm3_stop_line' and self.p.get('m3_stop_target_mm', 0.0) > 0.0:
            target = self.p['m3_stop_target_mm']        # 11/15: 횡단보도(M3)는 더 넉넉하게
        st = StopApproach(target, self.p['stop_brake_mm'],
                          self.p['stop_lost_ok_mm'],
                          freeze_mm=self.p['stop_freeze_mm'],
                          route_gate_mm=self.p['stop_route_gate_mm'],
                          camera_first=self.p.get('stop_camera_first', False))
        st.route_line_s = self._rp(line) if line else None
        st.passed_mm = self.p.get('stop_passed_mm', 120.0)
        return st

    def _arm_stop(self, inp: Inputs, line: Optional[str] = None) -> None:
        self._sub('approach', inp)
        pre = self.pre_stop if self.pre_stop_name == line else None
        if line == 'm3_stop_line' and pre is not None and (pre.seen_d is not None or pre.route_line_s is not None):
            self.stop = pre                              # 지그재그 중에 미리 잰 정지선을 이어받음
            self.log.append(f'M3 uses pre-tracked stop line ({pre.how})')
        elif line == 'm1_stop_line' and pre is not None and pre.seen_d is not None and pre.remaining(inp) is not None:
            # 11/15: 신호등 준비 전부터 카메라로 따라온 정지선을 그대로 쓴다
            self.stop = pre
            self.log.append(f'{line}: camera line tracked before arming ({pre.remaining(inp):.0f}mm)')
        else:
            self.stop = self._new_stop(line)
        self.pre_stop, self.pre_stop_name = None, None
        self.stop_line_name = line
        cf = self.p.get('m1_red_camera_max_mm', 0.0)
        if line == 'm1_stop_line' and cf > 0 and self.stop is not None:
            self.stop.camera_force_mm = cf               # 11/19: 신호등 정지선은 카메라 우선 (도면 무시)
            self.log.append(f'M1: camera stop line first (within {cf:.0f}mm, route ignored)')

    def _pretrack_stop(self, inp: Inputs) -> None:
        """신호등 정지 준비 전부터 횡단보도 정지선을 카메라로 따라간다 (M1 준비 전 -> M1 정지선)."""
        # M3 정지선은 지그재그(M6) 처리 안에서 미리 잰다 (m3_pretrack). 여기서는 신호등(M1)만.
        name = 'm1_stop_line' if (self.state == 'M1_TRAFFIC' and self.sub == '') else None
        if name is None or not self.p.get('stop_pretrack', True):
            if self.state not in ('M6_ZIGZAG', 'M3_PEDESTRIAN', 'M1_TRAFFIC'):
                self.pre_stop, self.pre_stop_name = None, None
            return
        s0 = self._rp(name)
        if inp.route_s is not None and s0 is not None and s0 - inp.route_s > self.p.get('stop_pretrack_mm', 1500.0):
            return                       # 도면상 아직 멀다 (다른 횡단보도 선과 헷갈리지 않게)
        if self.pre_stop_name != name:
            self.pre_stop, self.pre_stop_name = self._new_stop(name), name
            self.pre_stop.allow_route_start = False  # 준비 전엔 도면 출발점(route 모드)으로 시작하지 않는다
        self.pre_stop.remaining(inp)

    def _line_crossed(self, inp: Inputs, name: str) -> Optional[bool]:
        """흰 가로선을 앞바퀴가 지났나 (1순위 카메라, 2순위 도면 위치). 둘 다 모르면 None.

        정지선 정지와 같은 방식: 카메라가 본 거리에서 그 뒤 달린 거리를 뺀다
        (선은 앞바퀴 약 350mm 앞에서 화면 아래로 사라진다). 도면 위치는 처음 잡는 선을 고르는 데에도 쓴다."""
        if self.line is None:
            # 11/16: 라바콘 시작선은 카메라 1순위. 도면 위치(s)가 M7·M2를 지나며 1 m 넘게 틀어져
            #   (1114 분석) 맞는 카메라 선을 gate로 버리던 것 -> 같은 선이 3프레임 연속이면 도면과 달라도 받음
            cam_first = name == 'm8_start_line' and self.p.get('cones_line_camera_first', True)
            self.line = StopApproach(0.0, 0.0, self.p['stop_lost_ok_mm'], key='stoplines_mm',
                                     freeze_mm=self.p['stop_freeze_mm'],
                                     route_gate_mm=self.p['stop_route_gate_mm'],
                                     camera_first=cam_first)
            self.line.route_line_s = self._rp(name)
        rem = self.line.remaining(inp)
        if rem is not None and self.line.seen_d is not None:
            if rem <= 0.0:
                self._anchor(name, self.line, rem)
            return rem <= 0.0
        past = self._past(inp, name)
        if past is not None:
            return past
        if self.line.seen_at is not None:       # 카메라로 봤는데 lost_ok를 넘게 지남 = 이미 지남
            return True
        return None

    def _rp(self, name: str) -> Optional[float]:
        return (self.p.get('route_points') or {}).get(name)

    def _past(self, inp: Inputs, name: str, offset: float = 0.0) -> Optional[bool]:
        """도면 경로 위치 기준: name 지점 + offset을 지났나. 경로 정보가 없으면 None."""
        s0 = self._rp(name)
        if inp.route_s is None or s0 is None:
            return None
        return inp.route_s >= s0 + offset

    def _stop_near(self, inp: Inputs, line: str) -> bool:
        cam = self._crosswalk_near(inp)
        r = self._past(inp, line, -self.p['arm_on_crosswalk_mm'])
        return cam or bool(r)

    # --- 메인 ---------------------------------------------------------------
    def _guard_odom(self, inp: Inputs) -> None:
        """주행거리가 한 틱에 비정상적으로 튀면 그만큼을 빼서 없던 일로 한다."""
        raw = inp.dist_mm
        if self._last_dist is not None and abs(raw - self._last_dist) > self.p['odom_jump_mm']:
            self._dist_adj += raw - self._last_dist
            self.log.append(f'odom jump ignored {raw - self._last_dist:.0f}mm')
        self._last_dist = raw
        inp.dist_mm = raw - self._dist_adj

    def step(self, inp: Inputs) -> Command:
        self._guard_odom(inp)
        if self.state in ('M1_TRAFFIC', 'M4_PARKING') and not self.parking_sign:
            # 주차 표지는 신호등 구간에서부터 보일 수 있다. 신호등 판단에는 안 끼어들고 기억만 한다
            self.parking_sign = self._vote(inp, 'parking')
        if self.state in SEQUENCE[1:-1]:
            # 표지판·선·도면 위치가 1순위. 거리는 마지막 안전장치(2순위)로만 쓴다.
            _, d = self._since(inp)
            if d > self.p['state_max_mm']:
                self._next(inp, f'FALLBACK: state distance limit {d:.0f}mm (no sign/line seen)')
        self._pretrack_stop(inp)
        cmd = getattr(self, '_s_' + self.state)(inp)
        cmd.note = cmd.note or f'{self.state}/{self.sub}'
        return cmd

    # --- 상태별 -------------------------------------------------------------
    def _s_WAIT_START(self, inp):
        if inp.start:
            self._next(inp, 'start')
        return Command('stop', self._lane(0.0))

    def _s_LANE_ONLY(self, inp):
        return Command('lane', self._lane(self.p['base_speed']))

    def _s_FINISHED(self, inp):
        return Command('stop', self._lane(0.0))

    def _s_M5_BUMP(self, inp):
        """서행 표지(또는 방지턱 표지)를 보면 bump_slow_mm 동안 서행."""
        if self.sub == '':
            # 1순위 표지(명령), 2순위 도면 위치. 명령 경로(use_class_commands)에서도 표지를 놓치면 도면으로 서행
            sign = self._vote(inp, 'slow', 'bump')
            if sign or self._past(inp, 'm5_slow_line', -self.p['route_slow_before_mm']):
                self._sub('slow', inp, 'sign' if sign else 'FALLBACK: slow line by route')
            else:
                return Command('lane', self._lane(self.p['base_speed']))
        _, d = self._since(inp, sub=True)
        if self.bump_at is None and (inp.tilt_rate or 0.0) >= self.p['bump_imu_rate']:
            self.bump_at = inp.dist_mm
            self.log.append(f'M5 bump felt (imu {inp.tilt_rate:.2f} rad/s at {d:.0f}mm)')
        if self.bump_at is not None and inp.dist_mm - self.bump_at >= self.p['bump_after_mm']:
            self._next(inp, f'bump passed (imu) {d:.0f}mm')
            return Command('lane', self._lane(self.p['base_speed']))
        if d >= self.p['bump_slow_mm'] or self._past(inp, 'm6_zigzag_line', -1500.0):
            self._next(inp, f'FALLBACK: bump not felt, slow ended at {d:.0f}mm')
            return Command('lane', self._lane(self.p['base_speed']))
        return Command('lane', self._lane(self.p['slow_speed']))

    def _s_M6_ZIGZAG(self, inp):
        """지그재그: 시작 흰 선을 앞바퀴가 지나면 서행 (1순위 카메라, 2순위 도면, 3순위 거리).
        서행은 바로 뒤 횡단보도 정지 후 재출발할 때까지 유지한다 (M3도 정지 전까지 서행)."""
        if self.sub == '':
            self._remember_ped(inp)
            at = self._line_crossed(inp, 'm6_zigzag_line')
            if at is None:
                _, d = self._since(inp)
                at = d >= self.p['zigzag_start_mm']
            if not at:
                return Command('lane', self._lane(self.p['base_speed']))
            self._sub('zigzag', inp, f'start line ({self.line.how or "route/distance"})')
            # 정지선 안전장치 기준점: 카메라로 선을 지난 순간(가장 정확). 도면 위치로 지난 것도 씀, 거리로만 지난 것은 안 씀
            self.zz_at = inp.dist_mm if self.line.how in ('camera', 'route') else None
            self.zz_how = self.line.how
        self._remember_ped(inp)
        # 11/15 밤 실차 'M7 전 횡단보도를 지나서 섬 -> 다음 코너 탈선': M3는 정지선 266 mm 앞(지그재그 끝)에서 시작하는데
        #   카메라는 앞바퀴 350 mm 앞부터 보인다 -> M3가 시작할 때 정지선은 이미 사각이라 다음 흰 선(막대·끝선)을 잡았다.
        #   지그재그 중에 정지선이 보이면 미리 거리를 재 두고(M3가 이어받음), 지그재그가 끝나기 전에 정지 거리 안이면 바로 M3로
        if self.p.get('m3_pretrack', True):
            if getattr(self, 'pre_stop', None) is None and (
                    self._crosswalk_near(inp) or self._past(inp, 'm3_stop_line', -self.p['arm_on_crosswalk_mm'])):
                self.pre_stop, self.pre_stop_name = self._new_stop('m3_stop_line'), 'm3_stop_line'
                self.log.append('M6: crosswalk stop line pre-tracked for M3')
            if getattr(self, 'pre_stop', None) is not None:
                self.pre_stop.remaining(inp)
                if self.pre_stop.should_stop(inp):
                    self._next(inp, 'stop line close (pre-tracked in zigzag)')
                    return Command('stop', self._lane(0.0))
        if self._zz_cap(inp):
            self.zz_capped = True
            self._next(inp, f'stop line by zigzag-line distance {inp.dist_mm - self.zz_at:.0f}mm ({self.zz_how})')
            return Command('stop', self._lane(0.0))
        _, dz = self._since(inp, sub=True)
        end = self._past(inp, 'm6_zigzag_end')
        if (dz >= self.p['zigzag_len_mm']) if end is None else end:
            self._next(inp, f'zigzag done {dz:.0f}mm')
            return Command('lane', self._lane(self.p['slow_speed']))
        return Command('lane', self._lane(self.p['slow_speed'], 'zigzag',
                                          lane_width_mm=self.p['zigzag_width_mm']))

    def _s_M3_PEDESTRIAN(self, inp):
        """보행자 표지 -> 정지선 ±5cm 정지 -> 3초 이상 대기 -> 15초 안에 출발."""
        p = self.p
        if self.sub == '':
            sign = self._vote(inp, 'pedestrian')
            early = getattr(self, 'ped_seen', False)
            pre = getattr(self, 'pre_stop', None) is not None
            cap = getattr(self, 'zz_capped', False) or self._zz_cap(inp)
            if sign or early or pre or cap or self._stop_near(inp, 'm3_stop_line'):
                self.log.append('M3 stop armed by ' + ('pedestrian sign' if sign else
                                                       'pedestrian sign seen in M6' if early else 'route/crosswalk'))
                self._arm_stop(inp, 'm3_stop_line')
            else:
                return Command('lane', self._lane(p['slow_speed']))    # 지그재그 서행 그대로 (재출발부터 일반 속도)
        if self.sub == 'approach':
            capped = getattr(self, 'zz_capped', False) or self._zz_cap(inp)
            normal = self.stop.should_stop(inp)
            if not normal and not capped:
                return Command('lane', self._lane(p['approach_speed']))
            if normal:
                self._anchor(self.stop_line_name, self.stop, self.stop.remaining(inp))
            else:
                self.log.append(f'M3 stop by zigzag-line distance {inp.dist_mm - self.zz_at:.0f}mm '
                                f'(stop line estimate {self.stop.how}: {self.stop.remaining(inp)})')
            self.zz_capped = False
            self.zz_at = None                       # 한 번 섰으면 끝 (재출발 뒤 다시 서지 않게)
            self._sub('stopped', inp)
        if self.sub == 'stopped':
            t, _ = self._since(inp, sub=True)
            if t < p['ped_wait_sec']:
                return Command('stop', self._lane(0.0))
            self._sub('go', inp)
        _, d = self._since(inp, sub=True)
        if d >= p['leave_mm'] and self._past(inp, 'm3_crosswalk_end') is not False:
            self._next(inp, 'left crosswalk')
        return Command('lane', self._lane(p['base_speed']))

    def _remember_ped(self, inp):
        if not getattr(self, 'ped_seen', False) and self._vote(inp, 'pedestrian'):
            self.ped_seen = True
            self.log.append('pedestrian sign seen (M6, kept for M3)')

    def _remember_fork(self, inp):
        if self.fork_choice is None:
            a = self._vote(inp, 'fork_a')
            b = self._vote(inp, 'fork_b')
            if a != b:
                self.fork_choice = 'A' if a else 'B'

    def _s_M7_OBSTACLE(self, inp):
        """3차선 정적 장애물. 회피 주행은 obstacle 노드가 맡는다.

        갈림길 표지가 이 구간 끝에서 보일 수 있으니 여기서부터 미리 투표해 둔다.
        obstacle 노드는 /mission/state의 fork_choice를 보고 회피 마지막에 그쪽 차로로 들어가야 한다.
        """
        if getattr(self, 'm7_placed', False) and inp.yaw is not None:
            self.axis_yaw, self.m7_placed = inp.yaw, False
        self._remember_fork(inp)
        _, d = self._since(inp)
        done = inp.obstacle.get('m7_done', False)
        if self.p['m7_external_avoid']:
            return self._m7_external(inp, d, done)
        end = self._past(inp, 'm7_island_tip', -self.p['route_island_margin_mm'])
        if done or ((d >= self.p['m7_len_mm']) if end is None else end):
            self._next(inp, 'obstacle zone done' if done else f'm7 end at {d:.0f}mm')
        src = 'obstacle' if self.p['obstacle_source_ready'] else 'lane'
        kw = {'lookahead_mm': float(self.p['m7_lookahead_mm'])} if self.p.get('m7_lookahead_mm', 0.0) > 0.0 else {}
        return Command(src, self._lane(self.p['m7_speed'],
                                       lane_width_mm=self.p['three_lane_width_mm'],
                                       lane_color=self.p['three_lane_color'], **kw))

    def _m7_external(self, inp, d, done):
        """limo_obstacle_avoid 회피. 회피 노드는 섬 끝을 약 30 cm 지나서 끝나므로
        예전처럼 섬 끝 30 cm 앞에서 넘기면 마지막 차로 변경 도중에 차선 주행이 끼어든다 -> m7_done을 기다린다."""
        force = self._past(inp, 'm7_island_tip', self.p['m7_force_after_island_mm'])
        if force is None:
            force = d >= self.p['m7_len_mm'] + self.p['m7_force_after_island_mm']
        if done or force:
            self._next(inp, 'obstacle zone done' if done else f'm7 forced end at {d:.0f}mm')
            return Command('lane', self._lane(self.p['approach_speed']))
        if inp.obstacle.get('m7_hold'):
            # 회피 노드가 도중에 멈춤: 차선 주행으로 넘기면 정차 LIMO 사이를 직진한다 -> 정지
            return Command('stop', self._lane(0.0), note='M7_OBSTACLE/hold (avoid node lost)')
        src = 'obstacle' if inp.obstacle.get('m7_active') else 'lane'
        if src == 'lane' and self.m7c is not None and self.p.get('m7_corner_plain_lane', True):
            # 11/12 밤 실차: 횡단보도 끝 코너는 아직 1차로인데 3차선 설정(흰 선도 차선, 폭 520)으로 바꾸자마자 차선 노드가
            #   직선(흰 선)을 따라 코너를 직진했다(IMU 3°만 돎). 코너를 IMU로 다 돌 때까지는 보통 차선 설정 그대로
            kw = {'lost_search': True} if self.p.get('fork_lost_search', False) else {}
            kw.update(self._corner_look())
            return Command('lane', self._lane(self.p['m7_speed'], **kw), note='M7_OBSTACLE/corner (plain lane)')
        kw = {'lookahead_mm': float(self.p['m7_lookahead_mm'])} if self.p.get('m7_lookahead_mm', 0.0) > 0.0 else {}
        return Command(src, self._lane(self.p['m7_speed'],
                                       lane_width_mm=self.p['three_lane_width_mm'],
                                       lane_color=self.p['three_lane_color'], **kw))

    def _watch_floor_letter(self, inp) -> None:
        """바닥 글자 A/B 투표 -> self.floor_fork (한 번 정하면 그 판 끝까지)."""
        if (not self.p.get('floor_letter_use', True) or getattr(self, 'floor_fork', None) is not None
                or getattr(self, 'floor_blocked', False)):
            return
        buf = getattr(self, 'floor_buf', None)
        if buf is None:
            buf = self.floor_buf = []
        buf.append((inp.lane or {}).get('floor_letter'))
        del buf[:-int(self.p['floor_letter_window'])]
        for ch in ('A', 'B'):
            if buf.count(ch) >= self.p['floor_letter_need']:
                self.floor_fork = ch
                self.log.append(f'floor letter {ch} seen -> in fork {ch}'
                                f'{"" if self.fork_choice in (None, ch) else f" (sign said {self.fork_choice}: follow floor)"}')
                break

    def _s_M2_FORK(self, inp):
        """A(왼쪽) / B(오른쪽): 한쪽 경계선만 따라가는 차선 모드로 분기."""
        self._remember_fork(inp)
        # 11/19 실차 요청: M7을 건너뛰고 M2로 시작하면 obstacle_node가 회피 노드를 'M2만' 모드로 켜 갈림길 녹화를 재생한다
        #   (예전엔 차선 주행으로 직진 -> 표지판에 박음). 그동안은 회피 노드 명령, 노드가 끊기면 정지. 끝나면 m7_exit = 들어간 갈래
        ob = inp.obstacle or {}
        if ob.get('m2_replay_hold'):
            return Command('stop', self._lane(0.0), note='M2_FORK/hold (M2 replay node lost)')
        if ob.get('m2_replay_active'):
            return Command('obstacle', self._lane(self.p['approach_speed']), note='M2_FORK/replay (M7 skipped)')
        self._watch_floor_letter(inp)
        ff = getattr(self, 'floor_fork', None)
        exit_lane = inp.obstacle.get('m7_exit')
        if ff in ('A', 'B') and exit_lane in ('A', 'B') and ff != exit_lane and self.p.get('floor_need_exit_agree', True):
            # 11/15 실차 '1차선(A)으로 들어갔는데 B로 가서 합류에서 탈선': 바닥 글자(구멍 수)가 잡음 하나로 바뀌면 차가 실제로 있는
            #   차로(회피가 끝난 차로)와 반대 갈래를 따라감 -> 둘이 다르면 차로를 믿음
            if not getattr(self, '_floor_conflict_logged', False):
                self._floor_conflict_logged = True
                self.log.append(f'floor letter {ff} ignored: car ended M7 in lane {exit_lane}')
            ff = None
            self.floor_fork, self.floor_blocked = None, True   # 합류 S자도 이 글자를 안 쓰게, 이 판에선 다시 안 봄
        if ff in ('A', 'B') and self.fork_choice != ff:
            self.fork_choice = ff                      # 바닥 글자 = 차가 실제로 들어간 갈림길
        if ff in ('A', 'B'):
            exit_lane = ff                             # 바닥 글자가 회피 노드 차로 추정보다 정확 (회피 차로를 모르거나 같을 때)
        if self.fork_choice is None and self.p['fork_from_m7_exit'] and exit_lane in ('A', 'B'):
            self.fork_choice = exit_lane          # 이미 그 차로에 있다
        if (self.sub == '' and exit_lane in ('A', 'B') and self.fork_choice in ('A', 'B')
                and self.fork_choice != exit_lane):
            # 회피가 섬 전에 표지 쪽 차로로 못 옮김 (거리 부족·장애물)
            if self.p['fork_follow_m7_exit']:
                self.log.append(f'M2 fork conflict: sign {self.fork_choice}, car in {exit_lane} lane -> follow lane {exit_lane} '
                                f'(이미 섬 옆: 표지 쪽으로 건너갈 수 없음)')
                self.fork_choice = exit_lane
            else:
                self.log.append(f'M2 fork conflict: sign {self.fork_choice}, car in {exit_lane} lane -> follow sign '
                                f'{self.fork_choice} (fork_follow_m7_exit false: 표지 쪽 한쪽 선 모드)')
        if self.fork_choice is None:
            t, _ = self._since(inp)
            if t <= self.p['fork_seek_timeout_sec']:
                return Command('lane', self._lane(self.p['slow_speed']))
            self.fork_choice = self.p['fork_default']
        if self.sub == '':
            self._sub('branch', inp)
            self.m2_yaw = inp.yaw                   # 직선 방향을 모를 때(연습 시작) 대신 쓸 방향
            # 도면 위치가 없을 때(길 잃음) 끝 거리 보정: fork_len_mm은 섬 앞 끝부터 잰 길이인데 M2는
            #   표지로 일찍 끄면 섬 앞(최대 1.5 m 전), 회피가 끝나면 섬 앞 끝 +0.3~0.5 m에서 시작한다.
            #   일찍 시작하면 섬 옆에서 normal 모드로 돌아가 버림 -> M7이 마지막으로 잰 섬까지 거리만큼 늘림
            tip = inp.obstacle.get('m7_to_tip_mm')
            self.fork_adj_mm = (max(-self.p['fork_adj_max_mm'], min(self.p['fork_adj_max_mm'], float(tip)))
                                if isinstance(tip, (int, float)) else 0.0)
        _, db = self._since(inp, sub=True)
        if self.p.get('m2_plain_lane', True):
            # 11/18: CSV 재생 뒤 = 일반 차선 (한쪽 선 모드·폭 460·합류 S자 없음). 끝 판정은 그대로
            end = self._past(inp, 'm2_island_end', self.p.get('m2_exit_after_island_mm', 200.0)) \
                if self.p.get('m2_exit', True) else None
            if end is None:
                end = self._past(inp, 'm2_merge_end')
            end = self._m2_end_by_distance(inp, db, end)
            if (db >= self.p['fork_len_mm'] + self.fork_adj_mm) if end is None else end:
                self._next(inp, f'fork {self.fork_choice} done')
                return Command('lane', self._lane(self.p['base_speed']))
            merge_cmd = self._m2_merge(inp, db)      # m2_merge_plain일 때만 (섬 끝 500 mm 앞에서 S자 시작, M2_EXIT까지 이어짐)
            if merge_cmd is not None:
                return merge_cmd
            if self.p.get('m2_island_ref', True) and self.fork_choice in ('A', 'B'):
                mode = 'right_only' if self.fork_choice == 'A' else 'left_only'
                # 11/20 yubin: 선을 놓치면 정지(15초 Time-out -> M2 실패) 대신 갈래 직선 방향 유지 (lost_never_stop)
                #   마지막 차선 방향은 원 맞춤이 틀어지면 수십 도 어긋남 -> 갈래 직선 방향(M7 축, 없으면 갈래 시작 방향)
                kw = {}
                ax = self.axis_yaw if getattr(self, 'axis_yaw', None) is not None else getattr(self, 'm2_yaw', None)
                if ax is not None:
                    kw['hold_axis_deg'] = round(math.degrees(math.atan2(math.sin(ax), math.cos(ax))), 1)
                return Command('lane', self._lane(self.p['approach_speed'], mode, island_ref=True,
                                                  lane_width_mm=self.p.get('m2_island_lane_width_mm', 520.0),
                                                  lost_never_stop=True, **kw))
            return Command('lane', self._lane(self.p['approach_speed'], lost_never_stop=True))
        mode = 'left_only' if self.fork_choice == 'A' else 'right_only'
        if self.p.get('fork_normal', True):
            # 11/16: 섬 앞 끝을 지나 갈래 안에 들어오면 양쪽 선(바깥선 + 섬 선)을 다 보는 보통 차선 모드.
            #   1116_01 리플레이: 한쪽 선 모드는 갈래 48%·합류 굽은 곳 76% 선 없음 -> normal은 13%·14%
            past = self._past(inp, 'm7_island_tip', self.p.get('fork_normal_after_mm', 300.0))
            if past is None:
                past = db >= self.p.get('fork_normal_after_dist_mm', 700.0)
            if past:
                mode = 'normal'
        merge_cmd = self._m2_merge(inp, db)
        end = None
        if self.p.get('m2_exit', True):
            # 11/14: 갈림길 직후(섬 끝)부터 M2_EXIT 차선 인식 모드
            end = self._past(inp, 'm2_island_end', self.p.get('m2_exit_after_island_mm', 200.0))
        if end is None:
            end = self._past(inp, 'm2_merge_end')
        if (db >= self.p['fork_len_mm'] + self.fork_adj_mm) if end is None else end:
            self._next(inp, f'fork {self.fork_choice} done')
            if merge_cmd is not None:
                return merge_cmd
            return Command('lane', self._lane(self.p['base_speed']))
        if merge_cmd is not None:
            return merge_cmd
        return Command('lane', self._lane(self.p['approach_speed'], mode, **self._fork_cfg(inp, mode)))

    def _m2_end_by_distance(self, inp, db: float, route_end: Optional[bool]) -> Optional[bool]:
        """11/19: M2 끝을 도면 위치(route_end)와 M2 시작부터 간 거리로 따로 계산해 크게 다르면 거리 쪽 (m2_route_max_dev_mm)"""
        p = self.p
        dev = p.get('m2_route_max_dev_mm', 0.0)
        if dev <= 0 or not p.get('m2_exit', True):
            return route_end
        # M2 시작 = 섬 앞 끝 + (-fork_adj_mm) (fork_adj_mm = M7이 마지막으로 잰 섬 앞 끝까지 거리, 지나쳤으면 음수)
        need = p.get('m2_island_len_mm', 1745.0) + p.get('m2_exit_after_island_mm', 200.0) + self.fork_adj_mm
        rem_d = need - db
        s0 = self._rp('m2_island_end')
        if route_end is None or s0 is None or inp.route_s is None:
            return route_end
        rem_r = s0 + p.get('m2_exit_after_island_mm', 200.0) - inp.route_s
        if abs(rem_r - rem_d) <= dev:
            return route_end
        if not getattr(self, '_m2_dev_logged', False):
            self._m2_dev_logged = True
            self.log.append(f'M2 end: route says {rem_r:.0f}mm left, distance says {rem_d:.0f}mm -> distance (route s off)')
        return rem_d <= 0.0

    def _lane_first(self, inp, back_frames: int = 1) -> bool:
        """11/16: 선이 보이면 강제 경로(IMU heading) 대신 차선 주행 (lane_first).

        1116_01: 라바콘 뒤 코너·합류 S자에서 차선 노드가 선을 보고 있었는데도(L/R/BOTH) 강제 원호·S자를 따라가다
        크게 돌아 출발선 체커 쪽을 봄. 선을 lane_first_lost_frames 연속으로 놓쳤을 때만 강제 경로를 쓴다."""
        if not self.p.get('lane_first', True):
            return False
        lane = inp.lane or {}
        good = lane.get('status') in ('BOTH', 'L', 'R') and not lane.get('low_conf')
        n_lost = self.p.get('lane_first_lost_frames', 5)
        if good:
            self._lf_good = getattr(self, '_lf_good', 0) + 1
            if getattr(self, '_lf_bad', 0) >= n_lost and self._lf_good < back_frames:
                return False                 # corner_fix: 강제 경로 중엔 선이 back_frames 연속 보여야 복귀 (깜빡임에 모드 전환 X)
            self._lf_bad = 0
        else:
            self._lf_good = 0
            self._lf_bad = getattr(self, '_lf_bad', 0) + 1
        return self._lf_bad < n_lost

    def _m2_merge(self, inp, db: Optional[float] = None) -> Optional[Command]:
        """A/B 뒤 합류: 도면 규격으로 강제 S자 (m2_merge 설명). 하는 중이면 명령, 아니면 None."""
        p = self.p
        if p.get('m2_plain_lane', True) and not p.get('m2_merge_plain', False):
            return None                                  # 11/18: 일반 차선만 (합류 강제 S자 안 씀)
        if not p.get('m2_merge', True) or inp.yaw is None or self.fork_choice not in ('A', 'B'):
            return None
        m = getattr(self, 'm2m', None)
        ff = getattr(self, 'floor_fork', None)
        if m is not None and not m['done'] and ff in ('A', 'B') and (1.0 if ff == 'A' else -1.0) != m['side']:
            # S자 도중에 바닥 글자로 반대쪽 갈림길임을 앎 -> 옆 위치를 그쪽 기준으로 다시 (지금까지 움직인 만큼 유지)
            moved = m['y'] - m['side'] * p['m2_merge_shift_mm']
            m['side'] = -m['side']
            m['y'] = m['side'] * p['m2_merge_shift_mm'] + moved
            self.log.append(f'M2 merge side fixed by floor letter {ff}')
        if m is None:
            start = self._past(inp, 'm2_island_end', -p['m2_merge_before_end_mm'])
            if start is None:
                start = db is not None and db >= p['m2_merge_from_tip_mm'] + getattr(self, 'fork_adj_mm', 0.0)
            if not start:
                return None
            ax = getattr(self, 'axis_yaw', None)
            if ax is None:
                ax = getattr(self, 'm2_yaw', None)
            if ax is None:
                return None
            side = 1.0 if self.fork_choice == 'A' else -1.0       # A = 진행 방향 왼쪽 차로 -> 오른쪽으로 옮김
            y0 = side * p['m2_merge_shift_mm']
            off = (inp.lane or {}).get('offset_mm')
            if off is not None and (inp.lane or {}).get('status') not in (None, 'NONE') and abs(float(off)) <= 150.0:
                y0 -= float(off)                                # 차선 노드: 차로 중심이 차 왼쪽 off mm -> 차는 중심 오른쪽
            m = self.m2m = {'u0': inp.dist_mm, 'last': inp.dist_mm, 'y': y0,
                            'side': side, 'axis': ax, 'done': False}
            self.log.append(f'M2 merge start ({self.fork_choice}: {"오른쪽" if side > 0 else "왼쪽"}으로 '
                            f'{p["m2_merge_shift_mm"]:.0f} mm, {p["m2_merge_len_mm"]:.0f} mm 동안)')
        if m['done']:
            return None
        ds = inp.dist_mm - m['last']
        m['last'] = inp.dist_mm
        err = math.atan2(math.sin(inp.yaw - m['axis']), math.cos(inp.yaw - m['axis']))
        m['y'] += ds * math.sin(err)
        u = inp.dist_mm - m['u0']
        L, A = p['m2_merge_len_mm'], p['m2_merge_shift_mm']
        if (u >= L and abs(m['y']) <= p['m2_merge_tol_mm'] and abs(math.degrees(err)) <= p['m2_merge_end_deg']) \
                or u >= L + 300.0:
            m['done'] = True
            self.log.append(f'M2 merge done (옆 {m["y"]:+.0f} mm, {u:.0f} mm) -> lane')
            return None

        def yref(uu):
            return m['side'] * A * 0.5 * (1.0 + math.cos(math.pi * min(max(uu, 0.0), L) / L))

        ld = p['m2_merge_ld_mm']
        head = math.atan2(yref(u + ld) - m['y'], ld)
        lim = math.radians(p['m2_merge_max_deg'])
        head = max(-lim, min(lim, head))
        yaw_ref = m['axis'] + head
        yaw_ref = math.atan2(math.sin(yaw_ref), math.cos(yaw_ref))
        if self._lane_first(inp):
            # 선이 보이면 S자 위치 계산은 계속(선을 놓치면 이어서 씀)하고 조향은 차선으로
            return Command('lane', self._lane(self._m2x_speed(p['approach_speed']), **self._m2x_kw()),
                           note=f'M2 merge {u:.0f}/{L:.0f}mm by lane (y {m["y"]:+.0f}mm)')
        return Command('lane', self._lane(self._m2x_speed(p['approach_speed']), 'heading', yaw_ref=round(yaw_ref, 4),
                                          **self._m2x_kw()),
                       note=f'M2 merge {u:.0f}/{L:.0f}mm y {m["y"]:+.0f}mm')

    def _corner_look(self) -> dict:
        """11/19: 코너 미리보기 동안 차선 lookahead (corner_preview_lookahead_mm, 0 = 그대로)"""
        la = self.p.get('corner_preview_lookahead_mm', 0.0)
        return {'lookahead_mm': float(la)} if la > 0 else {}

    def _m2x_kw(self) -> dict:
        """M2_EXIT 동안만: 코너 앞(미리보기 0)은 차선 그대로 (m2x_zero_lane), 코너 lookahead (corner_preview_lookahead_mm)"""
        kw = {}
        if self.state == 'M2_EXIT':
            kw['lost_never_stop'] = True                # 11/20 yubin: 합류 끝 ~ 라바콘 시작선에서 선 놓쳐도 정지 안 함
            if self.p.get('m2x_zero_lane', True):
                kw['route_corner_zero_lane'] = True
            kw.update(self._corner_look())          # 11/19: 합류 끝 ~ 라바콘 시작선 (코너 뒤 짧은 직선까지 같은 값)
        return kw

    def _m2x_speed(self, speed: float) -> float:
        """11/18: 합류 S자(M2 끝 ~ M2_EXIT) 속도 상한 m2_merge_speed (0 = 그대로)"""
        cap = self.p.get('m2_merge_speed', 0.0)
        return min(speed, cap) if cap > 0 else speed

    def _cones_entry(self, inp, who: str) -> Optional[str]:
        """라바콘 시작선에 왔나 (들어갈 이유 문자열) / 아직이면 None. M2_EXIT와 (연습) M8 접근이 같이 씀.

        1순위: 시작 흰 선을 앞바퀴가 지나는 순간 (서지 않음). 2순위: 도면 위치. 3순위: 거리.
        좌하단 코너를 IMU로 다 돌기 전에는 카메라로 시작선을 넘은 것만 받는다."""
        p = self.p
        t = self._m8_turn_deg(inp) if p.get('m8_line_trigger', True) else None
        if t is not None and t >= p['m8_line_min_turn_deg'] and not getattr(self, '_m8_line_fresh', False):
            # 11/19: 코너를 돌기 시작한 뒤 본 선만 (그 전에 잡은 다른 흰 선 때문에 진짜 시작선을 '다른 선'으로 버리지 않게)
            self._m8_line_fresh = True
            self.line = None
        near = self._line_crossed(inp, 'm8_start_line')
        early = self._m8_early(inp, who)
        if early is not None:
            return early
        cam_near = bool(near) and self.line is not None and self.line.seen_d is not None
        if near and self.line.seen_d is None:
            near = self._past(inp, 'm8_start_line', -p['cones_enter_before_mm'])
        if near is None:
            _, d = self._since(inp)
            if self.state == 'M2_EXIT':
                d -= getattr(self, 'm2x_extra', 0.0)        # 섬 끝에서 시작 -> 합류 끝 기준으로
            near = d >= p['cones_approach_mm']
            armed = d >= p['cones_approach_mm'] - 500.0
        else:
            armed = bool(self._past(inp, 'm8_start_line', -800.0))
        ahead = inp.obstacle.get('cone_ahead_mm')
        close = ahead is not None and ahead <= p['cones_enter_cone_mm']
        cw = self.m8_corner
        if cw.active and not cw.done:
            cw.update(inp.yaw, inp.dist_mm)
            if cw.done:
                self.log.append(f'{who} corner done (IMU {math.degrees(cw.turned):.0f}deg)')
        # 11/15 저녁 실차 '커브 끝나고 라바콘을 그냥 들이받음': 코너를 IMU로 '다 돎(75° + 다시 직진)'이 늦게 나면
        #   시작선 + 300 mm까지 콘 모드에 안 들어갔다 -> 70° 넘게 돌았고 차로 안 콘이 cones_corner_close_mm 안이면 들어감
        late_close = (cw.active and not cw.done and ahead is not None and p.get('cones_corner_close_mm', 0.0) > 0
                      and ahead <= p.get('cones_corner_close_mm', 0.0)
                      and math.degrees(cw.turned) >= p.get('cones_corner_min_deg', 70.0))
        if late_close:
            self.log.append(f'{who} cone {ahead:.0f}mm close after {math.degrees(cw.turned):.0f}deg turn -> cones')
            return f'cone ahead {ahead:.0f}mm (corner {math.degrees(cw.turned):.0f}deg)'
        if cw.active and not cw.done and not cam_near:
            fail = self._past(inp, 'm8_start_line', p['m8_corner_fail_mm'])
            if fail is None:
                extra = getattr(self, 'm2x_extra', 0.0) if self.state == 'M2_EXIT' else 0.0
                fail = self._since(inp)[1] - extra >= p['cones_approach_mm'] + p['m8_corner_fail_mm']
            if not fail:
                return None
            cw.yaw0 = None                  # IMU로 코너를 못 알아챔 -> 예전 규칙
            self.log.append(f'{who} corner not detected by IMU ({math.degrees(cw.turned):.0f}deg) -> route/distance')
        if near or (armed and close):
            return f'start line ({self.line.how or "route/distance"})' if near else f'cone ahead {ahead:.0f}mm'
        return None

    def _m8_turn_deg(self, inp) -> Optional[float]:
        """11/19: 3차선 직선 방향 기준 코너 쪽(오른쪽)으로 돈 각 (도). 기준을 모르면 None"""
        ax = getattr(self, 'axis_yaw', None)
        if ax is None:
            ax = getattr(self, 'm2_yaw', None)
        if ax is None or inp.yaw is None:
            return None
        d = math.atan2(math.sin(inp.yaw - ax), math.cos(inp.yaw - ax))
        return max(0.0, math.degrees(self.p['m7_corner_dir'] * d))

    def _m8_early(self, inp, who: str) -> Optional[str]:
        """11/19: 라바콘 모드 확실히 켜기 (m8_line_trigger 설명). 아니면 None"""
        p = self.p
        if not p.get('m8_line_trigger', True):
            return None
        t = self._m8_turn_deg(inp)
        if t is None:
            return None
        ln = self.line
        if ln is not None and ln.how == 'camera' and ln.seen_at is not None and t >= p['m8_line_min_turn_deg']:
            last = getattr(self, '_m8_seen', None)
            if ln.seen_at == inp.dist_mm and (last is None or last[1] != ln.seen_at):
                n = (last[0] + 1) if (last is not None and inp.dist_mm - last[1] <= 250.0) else 1
                self._m8_seen = (n, ln.seen_at)
                if n >= p['m8_line_frames']:
                    self.log.append(f'{who} start line seen {ln.seen_d:.0f}mm ahead at {t:.0f}deg -> cones')
                    return f'start line seen by camera ({ln.seen_d:.0f}mm, {t:.0f}deg)'
        if t >= p['m8_full_turn_deg']:
            if getattr(self, '_m8_full_at', None) is None:
                self._m8_full_at = inp.dist_mm
            if inp.dist_mm - self._m8_full_at >= p['m8_after_turn_mm']:
                self.log.append(f'{who} turned {t:.0f}deg + {p["m8_after_turn_mm"]:.0f}mm -> cones (line not seen)')
                return f'corner done + {p["m8_after_turn_mm"]:.0f}mm'
        ahead = inp.obstacle.get('cone_ahead_mm')
        if t >= p['m8_full_turn_deg'] - 10.0 and ahead is not None and ahead <= p['cones_enter_cone_mm']:
            self.log.append(f'{who} cone {ahead:.0f}mm ahead at {t:.0f}deg -> cones')
            return f'cone ahead {ahead:.0f}mm ({t:.0f}deg)'
        return None

    def _m8_line_kw(self, inp) -> dict:
        """11/19: 코너를 돌기 시작하면 가로선을 더 기울어진 것까지 찾음 (시작선이 기울어 보임)"""
        t = self._m8_turn_deg(inp)
        if not self.p.get('m8_line_trigger', True) or t is None or t < 20.0:
            return {}
        return {'stopline_max_angle_deg': float(self.p['m8_line_max_angle_deg'])}

    def _s_M2_EXIT(self, inp):
        """11/14: 갈림길 합류 끝 ~ 라바콘 시작선. 차선 인식 모드만 (양쪽 노란 선, 보통 차로 폭) + 좌하단 코너 도면 곡률 보조.

        코너 곡률은 미션 노드가 /route/preview로 넘기고(m7_corner_k), 차선 노드는 선이 약하거나 없을 때 그 곡률을 섞는다.
        한쪽 선 모드(left_only/right_only)는 쓰지 않는다 (보이는 선이 반대쪽이면 버려 선 없음이 됨, 1114_01).
        시작선에 오면 M8_CONES로 넘기고 M8은 바로 콘 따라가기(cones)로 시작한다."""
        p = self.p
        if not p.get('m2_exit', True):
            self._next(inp, 'm2_exit off')
            self.m8_from_exit = False
            return Command('lane', self._lane(p['slow_speed']))
        self._watch_floor_letter(inp)
        merge_cmd = self._m2_merge(inp)                 # 합류 강제 S자가 아직 안 끝났으면 계속
        if merge_cmd is not None:
            return merge_cmd
        how = self._cones_entry(inp, 'M2_EXIT')
        if how is not None:
            self.m2x_how = how
            self._next(inp, f'cones start: {how}')
            return Command('lane', self._lane(p['slow_speed']))
        c = getattr(self, 'm7c', None)
        corner_on = c is not None and c[2] == 'M2_EXIT'
        speed = min(p['slow_speed'], p.get('m8_hold_speed', p['slow_speed'])) if corner_on else p['slow_speed']
        cap = p.get('m2x_speed', 0.0)
        if cap > 0:
            speed = min(speed, cap)                     # 11/18: 합류 끝 ~ 코너 (선을 쉽게 놓쳐 천천히)
        t = self._m8_turn_deg(inp)
        if p.get('m8_line_trigger', True) and t is not None and t >= p['m8_line_slow_deg']:
            speed = min(speed, p['m8_line_speed'])      # 11/19: 코너 끝 ~ 시작선 천천히 (시작선이 잘 보이게)
        return Command('lane', self._lane(speed, **self._m2x_kw(), **self._m8_line_kw(inp)))

    def _s_M8_CONES(self, inp):
        """라바콘: 접근(차선) -> cones(obstacle 노드가 콘 사이 경로 추종) -> handover -> M1.

        approach : 갈림길 끝 ~ 좌하단 코너. 차선 주행. obstacle 노드는 콘을 미리 추적해 둔다.
        cones    : 시작선 cones_enter_before_mm 앞(도면 위치) 또는 차로 안 콘이 가까우면 시작.
                   차선 노드는 'heading' 모드로 두었다가(출력은 안 씀) handover에서 normal로 바꿔
                   차선 노드 내부 상태를 초기화한다 (콘 때문에 꼬인 차선 추적을 버림).
        handover : obstacle 노드가 구간 방향으로 차로 중심을 계속 따라가는 동안 차선이 다시 잡히면 넘긴다.
        """
        p = self.p
        if not p['cones_source_ready']:
            _, d = self._since(inp)
            done = inp.obstacle.get('cones_done', False)
            end = self._past(inp, 'm8_end_line', 100.0)
            if done or ((d >= p['cones_len_mm']) if end is None else end):
                self._next(inp, 'cones done' if done else f'cones end at {d:.0f}mm')
                return Command('lane', self._lane(p['slow_speed']))
            mode = self._m8_fork_mode(inp)
            return Command('lane', self._lane(self._m8_speed(p['slow_speed']), mode, **self._m8_cfg(inp, mode)))

        if self.sub == '' and getattr(self, 'm8_from_exit', False):
            self.m8_from_exit = False
            self._sub('cones', inp, f'from M2_EXIT ({getattr(self, "m2x_how", "")})')
        if self.sub == '':
            # 연습(set_state M8)이거나 m2_exit 끔: 예전 접근 (M2 직후 갈림길 바깥선 한쪽 모드 유지, m8_keep_fork_mm 설명)
            m8_mode = self._m8_fork_mode(inp)
            how = self._cones_entry(inp, 'M8')
            if how is None:
                return Command('lane', self._lane(self._m8_speed(p['slow_speed']), m8_mode, **self._m8_cfg(inp, m8_mode)))
            self._sub('cones', inp, how)
        if self.sub == 'cones':
            _, d = self._since(inp, sub=True)
            # 1순위: 라바콘이 더 안 보임 (앞 cones_gone_mm 안 구간 폭 안 콘 0개가 cones_gone_sec 동안)
            near_cone = inp.obstacle.get('cone_any_ahead_mm')
            visible = near_cone is not None and near_cone <= p['cones_gone_mm']
            gone = False
            if visible:
                self.cones_seen, self.cones_gone_t = True, None
            elif self.cones_seen and d >= p['cones_min_mm']:
                if self.cones_gone_t is None:
                    self.cones_gone_t = inp.t
                gone = inp.t - self.cones_gone_t >= p['cones_gone_sec']
            done = inp.obstacle.get('cones_done', False)
            end = self._past(inp, 'm8_end_line', p['cones_end_after_mm'])
            if gone or done or end or d >= p['cones_zone_mm']:
                why = ('no cones ahead' if gone else 'obstacle done' if done
                       else 'FALLBACK: end line' if end else f'FALLBACK: distance {d:.0f}mm')
                self._sub('handover', inp, why)
            else:
                return Command('obstacle', self._lane(p['slow_speed'], 'heading'))
        # handover
        st = inp.lane.get('status')
        if st in ('BOTH', 'L', 'R') and not inp.lane.get('low_conf', False):
            self.count += 1
        else:
            self.count = 0
        _, d = self._since(inp, sub=True)
        if self.count >= p['cones_handover_frames'] or d >= p['cones_handover_mm']:
            self._next(inp, f'lane back ({st}, {d:.0f}mm)')
            return Command('lane', self._lane(p['slow_speed']))
        return Command('obstacle', self._lane(p['slow_speed']))

    def _m8_fork_mode(self, inp) -> str:
        """M8 시작 직후 차선 모드: 한쪽 선(left_only/right_only) 또는 normal (m8_keep_fork_mm 설명)"""
        p = self.p
        if not getattr(self, 'm8_fork_hold', False):
            return 'normal'
        _, d = self._since(inp)
        nb = p.get('m8_both_frames', 0)
        both = inp.lane.get('status') == 'BOTH' and not inp.lane.get('low_conf', False)
        self.m8_both_n = self.m8_both_n + 1 if both else 0
        c = getattr(self, 'm7c', None)
        corner_on = c is not None and c[2] == 'M8_CONES'
        if (nb > 0 and self.m8_both_n >= nb) or d >= p.get('m8_keep_fork_mm', 0.0) \
                or (getattr(self, 'm8_inner_on', False) and not corner_on):
            self.m8_fork_hold = False
            why = ('BOTH seen' if nb > 0 and self.m8_both_n >= nb else 'corner done' if self.m8_inner_on else 'distance')
            self.log.append(f'M8 lane normal ({why} {d:.0f}mm)')
            return 'normal'
        if corner_on:
            k = self.m7_corner_k(inp.dist_mm, inp.yaw) or []
            n = int(p.get('m8_inner_ahead_mm', 600.0) / 100.0) + 1
            near = [v for v in k[:n] if abs(v) > 0.1]
            if near and not getattr(self, 'm8_inner_on', False):
                self.m8_inner_on = True
                self.m8_inner_mode = 'right_only' if near[0] < 0 else 'left_only'
                self.log.append(f'M8 corner inner line ({self.m8_inner_mode}, {d:.0f}mm)')
        if getattr(self, 'm8_inner_on', False):
            return self.m8_inner_mode
        return 'left_only' if self.fork_choice == 'A' else 'right_only'

    def _m8_cfg(self, inp, mode: str) -> dict:
        if mode == 'normal':
            return {}
        kw = self._fork_cfg(inp, mode, corner=getattr(self, 'm8_inner_on', False))
        if getattr(self, 'm8_inner_on', False):
            kw.update(self._corner_outward())
        return kw

    def _fork_cfg(self, inp, mode: str, corner: bool = False) -> dict:
        """M2·합류 한쪽 선 모드에 같이 넘길 차선 설정: 좁은 폭(섬 쪽 선에서 멀리), 선 놓치면 찾기, 붙잡을 직선 방향"""
        p = self.p
        kw = {}
        if mode in ('left_only', 'right_only', 'normal') and not corner and p.get('fork_lane_width_mm', 0) > 0:
            kw['lane_width_mm'] = p['fork_lane_width_mm']
        if p.get('fork_lost_search', False):
            kw['lost_search'] = True
            if not corner:
                ax = getattr(self, 'axis_yaw', None)
                if ax is None and getattr(self, 'm2_yaw', None) is not None:
                    ax = self.m2_yaw
                if ax is not None:
                    kw['hold_axis_deg'] = round(math.degrees(math.atan2(math.sin(ax), math.cos(ax))), 1)
        return kw

    def _corner_outward(self) -> dict:
        """코너 동안 바깥쪽으로 비켜 가기 (target_offset_mm, 왼쪽 +). 오른쪽 코너면 왼쪽(+)"""
        o = self.p.get('corner_outward_mm', 0.0)
        return {'target_offset_mm': round(-self.p['m7_corner_dir'] * o, 1)} if o else {}

    def _m1_path(self, s: float, xs: float, R: float, d: float):
        """좌상단 코너 경로 (코너 진입 직선 좌표: x 앞, y 왼쪽 +). s = 경로 길이. 오른쪽 코너는 d = -1."""
        if s <= xs:
            return s, 0.0
        arc = R * math.pi / 2.0
        if s <= xs + arc:
            ph = (s - xs) / R
            return xs + R * math.sin(ph), d * (R - R * math.cos(ph))
        return xs + R, d * (R + (s - xs - arc))

    def _m1_forced(self, inp, speed: float) -> Optional[dict]:
        """M1 코너 강제 원호 (m1_force_corner 설명). 하는 중이면 차선 설정, 아니면 None."""
        m = getattr(self, 'm1fc', None)
        if m is None or m['done'] or inp.yaw is None or self.sub not in ('', 'approach'):
            return None
        p = self.p
        R, d = p['m1_force_r_mm'], p['m7_corner_dir']
        ds = inp.dist_mm - m['last']
        m['last'] = inp.dist_mm
        e = math.atan2(math.sin(inp.yaw - m['axis']), math.cos(inp.yaw - m['axis']))
        m['x'] += ds * math.cos(e)
        m['y'] += ds * math.sin(e)
        # 경로에서 지금 위치 (지난 값 근처에서 가장 가까운 점)
        best = (9e18, m['s'])
        for i in range(-10, 41):
            ss = max(0.0, m['s'] + i * 10.0)
            px, py = self._m1_path(ss, m['xs'], R, d)
            dd = (px - m['x']) ** 2 + (py - m['y']) ** 2
            if dd < best[0]:
                best = (dd, ss)
        m['s'] = best[1]
        err = math.sqrt(best[0])
        turned = abs(e)
        arc_end = m['xs'] + R * math.pi / 2.0
        if (turned >= math.radians(p['m1_force_end_deg']) and m['s'] >= arc_end - 100.0 and err <= p['m1_force_tol_mm']) \
                or m['s'] > arc_end + 400.0:
            m['done'] = True
            self.m7c = None
            self.log.append(f'M1 forced corner done (turned {math.degrees(turned):.0f} deg, off {err:.0f}mm) -> lane')
            self.m1_done_at = inp.dist_mm
            return None
        tx, ty = self._m1_path(m['s'] + p['m1_force_ld_mm'], m['xs'], R, d)
        yaw_ref = m['axis'] + math.atan2(ty - m['y'], tx - m['x'])
        yaw_ref = math.atan2(math.sin(yaw_ref), math.cos(yaw_ref))
        v = min(speed, p.get('m1_corner_speed', speed)) if m['s'] >= m['xs'] - 300.0 else speed
        if (self._lane_first(inp, int(self.p.get('m1_lane_back_frames', 1)))
                and not self._m1_lane_underturn(inp, m, R, d, arc_end)):
            return self._lane(v)                      # 11/16: 선이 보이면 차선 주행 (원호 위치 계산은 계속)
        return self._lane(v, 'heading', yaw_ref=round(yaw_ref, 4))

    def _m1_lane_underturn(self, inp, m: dict, R: float, d: float, arc_end: float) -> bool:
        """11/19: 원호 구간인데 차선 조향이 원호보다 크게 덜 꺾음 (m1_lane_min_turn 설명) -> True면 강제 원호"""
        p = self.p
        r = p.get('m1_lane_min_turn', 0.0)
        hold = getattr(self, '_m1_arc_hold', 0)
        if r <= 0.0:
            return False
        if hold > 0:
            self._m1_arc_hold = hold - 1
            return True
        if not (m['xs'] - 100.0 <= m['s'] <= arc_end):
            return False
        lk = (inp.lane or {}).get('kappa')
        need = r * 1000.0 / R
        if lk is None or float(lk) * d < need:
            if not getattr(self, '_m1_under_logged', False):
                self._m1_under_logged = True
                self.log.append(f'M1 corner: lane turns too little (k {lk}) -> forced arc')
            self._m1_arc_hold = int(p.get('m1_lane_min_hold', 6))
            return True
        return False

    def _m1_lane(self, inp, speed: float) -> dict:
        """M1: 라바콘 뒤 코너를 다 돌 때까지 안쪽 선 + 천천히 + 바깥쪽 비켜 가기 (그 뒤 보통 차선)"""
        fc = self._m1_forced(inp, speed)
        if fc is not None:
            return fc
        c = getattr(self, 'm7c', None)
        if c is None or c[2] != 'M1_TRAFFIC':
            return self._m1_after(inp, speed)
        k = self.m7_corner_k(inp.dist_mm, inp.yaw) or []
        if getattr(self, 'm7c', None) is None:            # 방금 다 돎
            self.log.append('M1 corner done')
            self.m1_done_at = inp.dist_mm
            return self._m1_after(inp, speed)
        n = int(self.p.get('m8_inner_ahead_mm', 600.0) / 100.0) + 1
        near = [v for v in k[:n] if abs(v) > 0.1]
        if near and not getattr(self, 'm1_inner_on', False):
            self.m1_inner_on = True
            self.log.append('M1 corner inner line')
        v = min(speed, self.p.get('m1_corner_speed', speed))
        if getattr(self, 'm1_inner_on', False):
            mode = 'right_only' if self.p['m7_corner_dir'] < 0 else 'left_only'
            return self._lane(v, mode, **self._corner_outward())
        return self._lane(v)

    def _m1_after(self, inp, speed: float) -> dict:
        """11/19 실차: 4번 코너 출구 바로 위(왼쪽)에 출발 차로가 붙어 있어, 코너가 끝나자마자 양쪽 선 모드로 돌아가면
        아직 비스듬히 보이는 그 차로 선을 내 왼쪽 선으로 잡고 올라감 -> 코너 끝 뒤 m1_inner_after_mm 동안은 안쪽(오른쪽) 선만."""
        at = getattr(self, 'm1_done_at', None)
        if at is not None and inp.dist_mm - at < self.p.get('m1_inner_after_mm', 0.0):
            return self._lane(speed, 'right_only' if self.p['m7_corner_dir'] < 0 else 'left_only')
        return self._lane(speed)

    def _m8_speed(self, base: float) -> float:
        """한쪽 선 모드(합류 ~ 코너) 동안은 m8_hold_speed 로 천천히"""
        if getattr(self, 'm8_fork_hold', False):
            return min(base, self.p.get('m8_hold_speed', base))
        return base

    def _s_M1_TRAFFIC(self, inp):
        """신호등: 정지선 ±5cm 정지, 녹색 인지 후 15초 안에 출발."""
        p = self.p
        red = self._vote(inp, 'red', 'yellow')
        green = self._vote(inp, 'green')
        # 11/15: 초록은 정지 지점에 도착한 그 순간에 확정돼 있을 때만 통과. 그 전에 본 초록으로는 출발(go)하지 않는다
        #   (예전: 접근 중 아무 때나 초록이 확정되면 바로 go -> 정지선에서 다시 안 봄). 아니면 일단 서서 초록 기다림
        if inp.confirmed_classes is not None:
            if red and self.sub not in ('approach', 'stopped'):
                self._arm_stop(inp, 'm1_stop_line')
        if self.sub == '':
            if red or green or self._stop_near(inp, 'm1_stop_line'):
                self._arm_stop(inp, 'm1_stop_line')
            else:
                return Command('lane', self._m1_lane(inp, p['base_speed']))
        if self.sub == 'approach':
            # 정지선 위치를 전혀 모르면(카메라도 도면도 없음) 거리를 판단할 수 없으니 초록이면 예전처럼 진행
            unknown = self.stop.remaining(inp) is None
            pass_green = green and not red and (unknown or not p['stop_on_green_arrival'])
            if (self.stop.should_stop(inp) or unknown) and pass_green:
                self._anchor(self.stop_line_name, self.stop, self.stop.remaining(inp))
                self._sub('go', inp)
                self.log.append('M1 green at stop line -> pass')
            elif self.stop.should_stop(inp):
                self._anchor(self.stop_line_name, self.stop, self.stop.remaining(inp))
                self._sub('stopped', inp)
            else:
                return Command('lane', self._m1_lane(inp, p['approach_speed']))
        if self.sub == 'stopped':
            t, _ = self._since(inp, sub=True)
            if not (green and not red and t >= p['green_go_delay_sec']):
                return Command('stop', self._lane(0.0))
            self._sub('go', inp)
        _, d = self._since(inp, sub=True)
        if d >= p['leave_mm']:
            self._next(inp, 'passed traffic light')
        return Command('lane', self._lane(p['base_speed']))

    def _s_M4_PARKING(self, inp):
        """주차는 parking 노드가 맡는다. 끝나면 /parking/state에 done=true."""
        if inp.parking.get('done', False):
            # 11/14: 주차 노드는 성공·실패 모두 done=true. 결과(result)를 보고 성공(PARKED...)일 때만 'parked'
            #   (1114_01: FAIL_NO_CAR인데 'parked' + 띠링 -> 성공한 줄 착각)
            res = str(inp.parking.get('result') or '')
            self.parking_result = res or 'UNKNOWN'
            ok = res.startswith('PARKED')
            self._next(inp, f'parked ({res})' if ok else f'parking failed ({res or "no result"})')
            return Command('stop', self._lane(0.0))
        if not self.parking_armed and self.p.get('m4_direct', False):
            self.parking_armed = True
            self._sub('armed', inp, 'direct (M4 start)')
        if not self.parking_armed:
            near = self._past(inp, 'm4_pre_line', -self.p['m4_arm_before_mm'])
            # 표지(명령) 1순위, 도면상 앞 선 근처 2순위. 경로 지도가 없으면: 원시 검출 경로는 바로, 명령 경로는 표지만
            if self.parking_sign or near or (near is None and inp.confirmed_classes is None):
                self.parking_armed = True
                self._sub('armed', inp, 'parking sign' if self.parking_sign
                          else ('FALLBACK: near pre line' if near else 'no route map'))
        if not self.parking_armed:
            return Command('lane', self._lane(self.p['slow_speed']))   # 아직 주차 노드에 안 넘김. 서행으로 접근
        off = self.p.get('m4_target_offset_mm', 0.0)
        kw = {'target_offset_mm': off} if off else {}
        return Command('parking', self._lane(self.p['slow_speed'], **kw))    # 서지 않고 서행으로 진입
