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
            'M2_FORK', 'M8_CONES', 'M1_TRAFFIC', 'M4_PARKING', 'FINISHED']
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
    'stop_target_mm': 0.0,       # 앞바퀴가 정지선 가까운 가장자리에서 멈출 위치 (+면 선 앞)
    'stop_brake_mm': 30.0,       # TODO 실측: approach_speed에서 0 명령 후 밀려가는 거리
    'stop_lost_ok_mm': 400.0,    # 정지선이 카메라에서 사라진 뒤 오도메트리로 이어 가는 한계
    # 정지선이 이 거리 안으로 오면 카메라 갱신을 끊고 오도메트리로만 센다.
    # 카메라 사각지대(앞바퀴 약 350mm 안) 직전부터는 뒤따르는 횡단보도 블록을 정지선으로 다시 잡기 쉽다 (11/09)
    'stop_freeze_mm': 450.0,
    'stop_route_gate_mm': 300.0,  # 처음 잡는 선이 도면 예측과 이만큼 넘게 다르면 버림 (끝선 540mm 뒤)
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
    'three_lane_color': 'both',         # 도면: M3 종료 ~ 우하단 코너(약1.6m) + 3차선 약4.5m(섬 오른쪽 끝까지)
    'cones_len_mm': 4300.0,      # 도면: M8 진입(갈림길 끝) ~ 좌하단 코너 ~ 라바콘 구간(약2.2m) 끝 + 여유
    # M8 라바콘 (obstacle 노드의 ConeFollower). false면 위 cones_len_mm 동안 차선 주행으로 통과
    'cones_source_ready': False,
    'cones_enter_before_mm': 0.0,    # 도면 위치 예비: 시작선 이만큼 앞(앞바퀴)에서 라바콘 모드 (1순위는 카메라로 선 통과)
    'cones_enter_cone_mm': 500.0,    # 차 앞 차로 안 콘이 앞바퀴에서 이 안이면 조금 일찍 들어간다
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
    'm8_corner_preview': True,       # 11/12 밤: M2 끝 뒤 좌하단 코너도 같은 IMU 코너 미리보기
    'm8_keep_fork_mm': 400.0,        # 11/12 밤 실차: M2 끝(합류 테이퍼)에서 normal로 바꾸자마자 선을 놓쳐(NONE/hold 약 2초) 차선 무시하고 감.
                                     #   M8 시작 후 이 거리까지는 갈림길에서 따라가던 바깥선(A=왼쪽, B=오른쪽) 한쪽 모드를 유지.
                                     #   그 전에 양쪽 선이 m8_both_frames 번 연속 보이면 바로 normal. 0이면 끔
    'm8_both_frames': 3,
    'm8_corner_start_mm': 650.0,     # M8 진입(합류 끝 m2_merge_end) -> 코너 시작 (도면 20272 -> 20921)      # 차선 노드 route_ff_preview_mm 와 같게
    'm7_corner_give_up_mm': 1500.0,  # 코너 시작 + 호 길이 + 이만큼 가도 IMU가 다 돈 걸 못 보면 그만 (예전 800)
    'm7_corner_plain_lane': True,    # 코너 동안은 3차선 설정 대신 보통 차선 설정 (흰 선을 차선으로 보지 않음)
    'm8_corner_turn_deg': 75.0,      # 0 = 안 씀
    'm8_corner_straight_deg': 4.0,
    'm8_corner_window_mm': 150.0,
    'm8_corner_fail_mm': 300.0,      # 도면상 시작선 + 이만큼(지도 없으면 cones_approach + 이만큼)인데 코너를 못 알아채면 예전 규칙
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
                 route_gate_mm: float = 0.0):
        self.target_mm = target_mm
        self.brake_mm = brake_mm
        self.lost_ok_mm = lost_ok_mm
        self.key = key           # 기본: 바로 뒤에 횡단보도 막대가 있는 가로선만 정지선으로 봄
        self.freeze_mm = freeze_mm          # 0이면 끔 (예전 동작)
        self.route_gate_mm = route_gate_mm  # 0이면 끔
        self.seen_d = None       # 마지막으로 본 정지선 거리 (앞바퀴 기준)
        self.seen_at = None      # 그때의 누적 주행 거리
        self.route_line_s = None  # 도면상 정지선 s. 카메라가 선을 못 볼 때만 쓰는 예비값
        self.how = None          # 'camera' | 'route' (디버그)

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
                    cand = None
            if cand is not None:
                self.seen_d, self.seen_at, self.how = cand, inp.dist_mm, 'camera'
        if self.seen_d is None and self.freeze_mm > 0.0 and self.route_gate_mm > 0.0:
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

    def _predicted(self, inp: Inputs) -> float:
        if self.seen_d is None:
            return 0.0
        return self.seen_d - (inp.dist_mm - self.seen_at)

    def should_stop(self, inp: Inputs) -> bool:
        rem = self.remaining(inp)
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
        self.count = 0                               # 서브 상태 안에서 쓰는 연속 카운터
        self.log = []
        self.parking_sign = False                    # 주차 표지를 봤나 (M1~M4 동안 투표, 상태가 바뀌어도 유지)
        self.parking_armed = False                   # 주차 노드에 앞 선을 받으라고 알림 (/mission/state)
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
        self.state = state
        self.enter_t, self.enter_dist = inp.t, inp.dist_mm
        if state == 'M7_OBSTACLE':
            # 횡단보도에서 넘어올 때만 (연습으로 set_state M7이면 이미 코너 뒤 직선에 놓음)
            use = self.p.get('m7_corner_done_deg', 85.0) > 0 and why not in ('manual', 'init') and inp.yaw is not None
            self.m7c = (inp.yaw, inp.dist_mm, 'M7_OBSTACLE', self.p['m7_corner_start_mm']) if use else None
        if state == 'M8_CONES':
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
        yaw0, d0, _, start_mm = c
        p = self.p
        r = p['m7_corner_r_mm']
        turned = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
        since = dist_mm - d0
        arc = r * math.pi / 2.0
        if turned >= math.radians(p['m7_corner_done_deg']) or since > start_mm + arc + p['m7_corner_give_up_mm']:
            self.m7c = None                              # 다 돎 (또는 너무 지나 IMU를 못 믿음) -> 끝
            self.log.append(f'{self.state[:2]} corner preview off (turned {math.degrees(turned):.0f} deg, {since:.0f}mm)')
            return None
        k = p['m7_corner_dir'] * 1000.0 / r
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

    def _crosswalk_near(self, inp: Inputs) -> bool:
        lines = inp.lane.get('crosswalk_lines_mm') or []
        return bool(lines) and lines[0] <= self.p['arm_on_crosswalk_mm']

    def _arm_stop(self, inp: Inputs, line: Optional[str] = None) -> None:
        self._sub('approach', inp)
        self.stop = StopApproach(self.p['stop_target_mm'], self.p['stop_brake_mm'],
                                 self.p['stop_lost_ok_mm'],
                                 freeze_mm=self.p['stop_freeze_mm'],
                                 route_gate_mm=self.p['stop_route_gate_mm'])
        self.stop.route_line_s = self._rp(line) if line else None
        self.stop_line_name = line

    def _line_crossed(self, inp: Inputs, name: str) -> Optional[bool]:
        """흰 가로선을 앞바퀴가 지났나 (1순위 카메라, 2순위 도면 위치). 둘 다 모르면 None.

        정지선 정지와 같은 방식: 카메라가 본 거리에서 그 뒤 달린 거리를 뺀다
        (선은 앞바퀴 약 350mm 앞에서 화면 아래로 사라진다). 도면 위치는 처음 잡는 선을 고르는 데에도 쓴다."""
        if self.line is None:
            self.line = StopApproach(0.0, 0.0, self.p['stop_lost_ok_mm'], key='stoplines_mm',
                                     freeze_mm=self.p['stop_freeze_mm'],
                                     route_gate_mm=self.p['stop_route_gate_mm'])
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
            at = self._line_crossed(inp, 'm6_zigzag_line')
            if at is None:
                _, d = self._since(inp)
                at = d >= self.p['zigzag_start_mm']
            if not at:
                return Command('lane', self._lane(self.p['base_speed']))
            self._sub('zigzag', inp, f'start line ({self.line.how or "route/distance"})')
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
            if self._vote(inp, 'pedestrian') or self._stop_near(inp, 'm3_stop_line'):
                self._arm_stop(inp, 'm3_stop_line')
            else:
                return Command('lane', self._lane(p['slow_speed']))    # 지그재그 서행 그대로 (재출발부터 일반 속도)
        if self.sub == 'approach':
            if not self.stop.should_stop(inp):
                return Command('lane', self._lane(p['approach_speed']))
            self._anchor(self.stop_line_name, self.stop, self.stop.remaining(inp))
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
        self._remember_fork(inp)
        _, d = self._since(inp)
        done = inp.obstacle.get('m7_done', False)
        if self.p['m7_external_avoid']:
            return self._m7_external(inp, d, done)
        end = self._past(inp, 'm7_island_tip', -self.p['route_island_margin_mm'])
        if done or ((d >= self.p['m7_len_mm']) if end is None else end):
            self._next(inp, 'obstacle zone done' if done else f'm7 end at {d:.0f}mm')
        src = 'obstacle' if self.p['obstacle_source_ready'] else 'lane'
        return Command(src, self._lane(self.p['m7_speed'],
                                       lane_width_mm=self.p['three_lane_width_mm'],
                                       lane_color=self.p['three_lane_color']))

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
            return Command('lane', self._lane(self.p['m7_speed']), note='M7_OBSTACLE/corner (plain lane)')
        return Command(src, self._lane(self.p['m7_speed'],
                                       lane_width_mm=self.p['three_lane_width_mm'],
                                       lane_color=self.p['three_lane_color']))

    def _s_M2_FORK(self, inp):
        """A(왼쪽) / B(오른쪽): 한쪽 경계선만 따라가는 차선 모드로 분기."""
        self._remember_fork(inp)
        exit_lane = inp.obstacle.get('m7_exit')
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
            # 도면 위치가 없을 때(길 잃음) 끝 거리 보정: fork_len_mm은 섬 앞 끝부터 잰 길이인데 M2는
            #   표지로 일찍 끄면 섬 앞(최대 1.5 m 전), 회피가 끝나면 섬 앞 끝 +0.3~0.5 m에서 시작한다.
            #   일찍 시작하면 섬 옆에서 normal 모드로 돌아가 버림 -> M7이 마지막으로 잰 섬까지 거리만큼 늘림
            tip = inp.obstacle.get('m7_to_tip_mm')
            self.fork_adj_mm = (max(-self.p['fork_adj_max_mm'], min(self.p['fork_adj_max_mm'], float(tip)))
                                if isinstance(tip, (int, float)) else 0.0)
        mode = 'left_only' if self.fork_choice == 'A' else 'right_only'
        _, db = self._since(inp, sub=True)
        end = self._past(inp, 'm2_merge_end')
        if (db >= self.p['fork_len_mm'] + self.fork_adj_mm) if end is None else end:
            self._next(inp, f'fork {self.fork_choice} done')
            return Command('lane', self._lane(self.p['base_speed']))
        return Command('lane', self._lane(self.p['approach_speed'], mode))

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
            return Command('lane', self._lane(p['slow_speed'], self._m8_fork_mode(inp)))

        if self.sub == '':
            # M2 직후: 합류 테이퍼에서 양쪽 선이 다시 잡힐 때까지 갈림길 바깥선 한쪽 모드 유지 (m8_keep_fork_mm 설명)
            m8_mode = self._m8_fork_mode(inp)
            # 1순위: 시작 흰 선을 앞바퀴가 지나는 순간 (서지 않음). 2순위: 도면 위치. 3순위: 거리
            near = self._line_crossed(inp, 'm8_start_line')
            cam_near = bool(near) and self.line is not None and self.line.seen_d is not None
            if near and self.line.seen_d is None:
                near = self._past(inp, 'm8_start_line', -p['cones_enter_before_mm'])
            if near is None:
                _, d = self._since(inp)
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
                    self.log.append(f'M8 corner done (IMU {math.degrees(cw.turned):.0f}deg)')
            if cw.active and not cw.done and not cam_near:
                # 아직 코너 안: 도면 위치·콘 가까움으로는 안 들어감 (카메라로 시작선을 넘은 것만 받음)
                fail = self._past(inp, 'm8_start_line', p['m8_corner_fail_mm'])
                if fail is None:
                    fail = self._since(inp)[1] >= p['cones_approach_mm'] + p['m8_corner_fail_mm']
                if not fail:
                    return Command('lane', self._lane(p['slow_speed'], m8_mode))
                cw.yaw0 = None                  # IMU로 코너를 못 알아챔 -> 예전 규칙
                self.log.append(f'M8 corner not detected by IMU ({math.degrees(cw.turned):.0f}deg) -> route/distance')
            if near or (armed and close):
                how = f'start line ({self.line.how or "route/distance"})' if near else f'cone ahead {ahead:.0f}mm'
                self._sub('cones', inp, how)
            else:
                return Command('lane', self._lane(p['slow_speed'], m8_mode))
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
        """M8 시작 직후 차선 모드: 갈림길 바깥선 한쪽(left_only/right_only) 또는 normal"""
        p = self.p
        if not getattr(self, 'm8_fork_hold', False):
            return 'normal'
        both = inp.lane.get('status') == 'BOTH' and not inp.lane.get('low_conf', False)
        self.m8_both_n = self.m8_both_n + 1 if both else 0
        _, d = self._since(inp)
        if self.m8_both_n >= p.get('m8_both_frames', 3) or d >= p.get('m8_keep_fork_mm', 0.0):
            self.m8_fork_hold = False
            self.log.append(f'M8 lane normal ({"BOTH seen" if self.m8_both_n else "distance"} {d:.0f}mm)')
            return 'normal'
        return 'left_only' if self.fork_choice == 'A' else 'right_only'

    def _s_M1_TRAFFIC(self, inp):
        """신호등: 정지선 ±5cm 정지, 녹색 인지 후 15초 안에 출발."""
        p = self.p
        red = self._vote(inp, 'red', 'yellow')
        green = self._vote(inp, 'green')
        if inp.confirmed_classes is not None:
            if red and self.sub not in ('approach', 'stopped'):
                self._arm_stop(inp, 'm1_stop_line')
            elif green and not red and self.sub in ('', 'approach'):
                self._sub('go', inp)
        if self.sub == '':
            if red or green or self._stop_near(inp, 'm1_stop_line'):
                self._arm_stop(inp, 'm1_stop_line')
            else:
                return Command('lane', self._lane(p['base_speed']))
        if self.sub == 'approach':
            if green and not red and not p['stop_on_green_arrival']:
                self._sub('go', inp)
            elif self.stop.should_stop(inp):
                self._anchor(self.stop_line_name, self.stop, self.stop.remaining(inp))
                self._sub('stopped', inp)
            else:
                return Command('lane', self._lane(p['approach_speed']))
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
            self._next(inp, 'parked')
            return Command('stop', self._lane(0.0))
        if not self.parking_armed:
            near = self._past(inp, 'm4_pre_line', -self.p['m4_arm_before_mm'])
            # 표지(명령) 1순위, 도면상 앞 선 근처 2순위. 경로 지도가 없으면: 원시 검출 경로는 바로, 명령 경로는 표지만
            if self.parking_sign or near or (near is None and inp.confirmed_classes is None):
                self.parking_armed = True
                self._sub('armed', inp, 'parking sign' if self.parking_sign
                          else ('FALLBACK: near pre line' if near else 'no route map'))
        if not self.parking_armed:
            return Command('lane', self._lane(self.p['slow_speed']))   # 아직 주차 노드에 안 넘김. 서행으로 접근
        return Command('parking', self._lane(self.p['slow_speed']))    # 서지 않고 서행으로 진입
