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

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional

SEQUENCE = ['WAIT_START', 'M5_BUMP', 'M6_ZIGZAG', 'M3_PEDESTRIAN', 'M7_OBSTACLE',
            'M2_FORK', 'M8_CONES', 'M1_TRAFFIC', 'M4_PARKING', 'FINISHED']
EXTRA_STATES = ['LANE_ONLY']

DEFAULT_PARAMS = {
    'start_state': 'WAIT_START',
    'base_speed': 0.30,          # m/s, 규정 권장 0.3~0.4
    'slow_speed': 0.13,          # 서행. 규정 0.2 이하인데 심판 육안이라 확실히 느리게
    'approach_speed': 0.18,      # 정지선·장애물 접근 속도
    'vote_window': 8,            # 다수결 창 (인식 프레임 수)
    'vote_need': 5,
    'det_conf': 0.5,
    'state_max_mm': 10000.0,     # 한 상태에서 이만큼 달려도 안 끝나면 강제로 다음 상태 (멈춤 방지). M6이 약 9m라 10m
    # M5 방지턱
    'bump_slow_mm': 1800.0,      # 도면 추정(방지턱 위치는 도면에 없음): 표지 인식(선 약1m 앞) ~ 마름모 끝+여유. TODO 실측
    # M6 지그재그
    'zigzag_start_mm': 7200.0,   # 도면: M5 종료 지점(상단 차로) ~ 우측 지그재그 시작선. 길어서 오도메트리 오차 큼 -> 흰 선 검출로 바꿀 것
    'zigzag_len_mm': 1800.0,     # 도면: 지그재그 약 1.7~1.9m. 바로 뒤 횡단보도 정지선까지 약 250mm라 여유를 크게 안 둠
    'zigzag_width_mm': 570.0,
    # 정지 미션 공통 (M3, M1)
    'stop_target_mm': 0.0,       # 앞바퀴가 정지선 가까운 가장자리에서 멈출 위치 (+면 선 앞)
    'stop_brake_mm': 30.0,       # TODO 실측: approach_speed에서 0 명령 후 밀려가는 거리
    'stop_lost_ok_mm': 400.0,    # 정지선이 카메라에서 사라진 뒤 오도메트리로 이어 가는 한계
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
    # 3차선: 차로 500mm, 바깥 실선은 노랑·안쪽 점선은 흰색(모의 맵 10/30 확인) -> 두 색 다 차선으로
    'three_lane_width_mm': 500.0,
    'three_lane_color': 'both',         # 도면: M3 종료 ~ 우하단 코너(약1.6m) + 3차선 약4.5m(섬 오른쪽 끝까지)
    'cones_len_mm': 4300.0,      # 도면: M8 진입(갈림길 끝) ~ 좌하단 코너 ~ 라바콘 구간(약2.2m) 끝 + 여유
    # M2 갈림길
    'fork_len_mm': 2300.0,       # 도면: 섬 오른쪽 끝부터 왼쪽 합류 테이퍼 끝까지 약 2.4m
    'fork_seek_timeout_sec': 12.0,  # 규정상 15초 안에 진입해야 함
    'fork_default': 'A',         # 끝까지 못 읽으면 찍을 방향. TODO 팀 정책
    # M1 신호등
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
    route_s: Optional[float] = None            # 도면 경로상 위치 (route.RouteTracker). 없으면 거리 기준


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
                 key: str = 'crosswalk_lines_mm'):
        self.target_mm = target_mm
        self.brake_mm = brake_mm
        self.lost_ok_mm = lost_ok_mm
        self.key = key           # 기본: 바로 뒤에 횡단보도 막대가 있는 가로선만 정지선으로 봄
        self.seen_d = None       # 마지막으로 본 정지선 거리 (앞바퀴 기준)
        self.seen_at = None      # 그때의 누적 주행 거리
        self.route_line_s = None  # 도면상 정지선 s. 카메라가 선을 못 볼 때만 쓰는 예비값

    def remaining(self, inp: Inputs) -> Optional[float]:
        # 미션 시작선도 흰 가로선이라, 횡단보도 막대가 뒤따르는 선만 쓴다.
        # (lane 노드가 그 키를 안 주는 옛 버전이면 모든 가로선으로 대신함)
        lines = inp.lane.get(self.key)
        if lines is None:
            lines = inp.lane.get('stoplines_mm') or []
        if lines:
            d = lines[0]
            if self.seen_d is None or abs(d - self._predicted(inp)) < 150.0:
                self.seen_d, self.seen_at = d, inp.dist_mm
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
        self.state = ''
        self.sub = ''
        self.enter_t = self.enter_dist = 0.0
        self.sub_t = self.sub_dist = 0.0
        self.stop: Optional[StopApproach] = None
        self.log = []
        self._goto(self.p['start_state'], Inputs(t=0.0, dist_mm=0.0), 'init')

    # --- 공통 도구 ------------------------------------------------------------
    def _goto(self, state: str, inp: Inputs, why: str = '') -> None:
        if state not in SEQUENCE and state not in EXTRA_STATES:
            raise ValueError(f'unknown state {state}')
        self.log.append(f'{self.state or "-"} -> {state} {why}'.strip())
        self.state = state
        self.enter_t, self.enter_dist = inp.t, inp.dist_mm
        self._sub('', inp)
        for v in self.votes.values():
            v.reset()
        self.stop = None

    def _next(self, inp: Inputs, why: str = '') -> None:
        if self.state not in SEQUENCE:
            return
        i = SEQUENCE.index(self.state)
        self._goto(SEQUENCE[min(i + 1, len(SEQUENCE) - 1)], inp, why)

    def _sub(self, name: str, inp: Inputs) -> None:
        self.sub = name
        self.sub_t, self.sub_dist = inp.t, inp.dist_mm

    def _vote(self, inp: Inputs, *classes: str) -> bool:
        """classes 중 하나라도 다수결을 넘으면 True. 투표는 새 인식 프레임에서만."""
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
                                 self.p['stop_lost_ok_mm'])
        self.stop.route_line_s = self._rp(line) if line else None

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
    def step(self, inp: Inputs) -> Command:
        if self.state in SEQUENCE[1:-1]:
            _, d = self._since(inp)
            if d > self.p['state_max_mm']:
                self._next(inp, f'state distance limit {d:.0f}mm')
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
            near = self._past(inp, 'm5_slow_line', -self.p['route_slow_before_mm'])
            if self._vote(inp, 'slow', 'bump') or near:
                self._sub('slow', inp)
            else:
                return Command('lane', self._lane(self.p['base_speed']))
        _, d = self._since(inp, sub=True)
        if d >= self.p['bump_slow_mm'] or self._past(inp, 'm6_zigzag_line', -1500.0):
            self._next(inp, f'bump passed {d:.0f}mm')
            return Command('lane', self._lane(self.p['base_speed']))
        return Command('lane', self._lane(self.p['slow_speed']))

    def _s_M6_ZIGZAG(self, inp):
        """지그재그: 도면 경로가 있으면 시작선·끝 위치로, 없으면 거리로 판단."""
        if self.sub == '':
            at = self._past(inp, 'm6_zigzag_line', -50.0)
            if at is None:
                _, d = self._since(inp)
                at = d >= self.p['zigzag_start_mm']
            if not at:
                return Command('lane', self._lane(self.p['base_speed']))
            self._sub('zigzag', inp)
        _, dz = self._since(inp, sub=True)
        end = self._past(inp, 'm6_zigzag_end')
        if (dz >= self.p['zigzag_len_mm']) if end is None else end:
            self._next(inp, f'zigzag done {dz:.0f}mm')
            return Command('lane', self._lane(self.p['base_speed']))
        return Command('lane', self._lane(self.p['slow_speed'], 'zigzag',
                                          lane_width_mm=self.p['zigzag_width_mm']))

    def _s_M3_PEDESTRIAN(self, inp):
        """보행자 표지 -> 정지선 ±5cm 정지 -> 3초 이상 대기 -> 15초 안에 출발."""
        p = self.p
        if self.sub == '':
            if self._vote(inp, 'pedestrian') or self._stop_near(inp, 'm3_stop_line'):
                self._arm_stop(inp, 'm3_stop_line')
            else:
                return Command('lane', self._lane(p['base_speed']))
        if self.sub == 'approach':
            if not self.stop.should_stop(inp):
                return Command('lane', self._lane(p['approach_speed']))
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
        return Command(src, self._lane(self.p['approach_speed'],
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
        return Command(src, self._lane(self.p['approach_speed'],
                                       lane_width_mm=self.p['three_lane_width_mm'],
                                       lane_color=self.p['three_lane_color']))

    def _s_M2_FORK(self, inp):
        """A(왼쪽) / B(오른쪽): 한쪽 경계선만 따라가는 차선 모드로 분기."""
        self._remember_fork(inp)
        exit_lane = inp.obstacle.get('m7_exit')
        if self.fork_choice is None and self.p['fork_from_m7_exit'] and exit_lane in ('A', 'B'):
            self.fork_choice = exit_lane          # 이미 그 차로에 있다
        if self.fork_choice is None:
            t, _ = self._since(inp)
            if t <= self.p['fork_seek_timeout_sec']:
                return Command('lane', self._lane(self.p['slow_speed']))
            self.fork_choice = self.p['fork_default']
        if self.sub == '':
            self._sub('branch', inp)
        mode = 'left_only' if self.fork_choice == 'A' else 'right_only'
        _, db = self._since(inp, sub=True)
        end = self._past(inp, 'm2_merge_end')
        if (db >= self.p['fork_len_mm']) if end is None else end:
            self._next(inp, f'fork {self.fork_choice} done')
            return Command('lane', self._lane(self.p['base_speed']))
        return Command('lane', self._lane(self.p['approach_speed'], mode))

    def _s_M8_CONES(self, inp):
        _, d = self._since(inp)
        done = inp.obstacle.get('cones_done', False)
        end = self._past(inp, 'm8_end_line', 100.0)
        if done or ((d >= self.p['cones_len_mm']) if end is None else end):
            self._next(inp, 'cones done' if done else f'cones end at {d:.0f}mm')
        src = 'obstacle' if self.p['obstacle_source_ready'] else 'lane'
        return Command(src, self._lane(self.p['slow_speed']))

    def _s_M1_TRAFFIC(self, inp):
        """신호등: 정지선 ±5cm 정지, 녹색 인지 후 15초 안에 출발."""
        p = self.p
        red = self._vote(inp, 'red', 'yellow')
        green = self._vote(inp, 'green')
        if self.sub == '':
            if red or green or self._stop_near(inp, 'm1_stop_line'):
                self._arm_stop(inp, 'm1_stop_line')
            else:
                return Command('lane', self._lane(p['base_speed']))
        if self.sub == 'approach':
            if green and not red and not p['stop_on_green_arrival']:
                self._sub('go', inp)
            elif self.stop.should_stop(inp):
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
        return Command('parking', self._lane(self.p['approach_speed']))
