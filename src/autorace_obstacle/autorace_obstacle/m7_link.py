"""M7(3차선 정차 LIMO 회피) 연결 로직 — ROS 없음.

팀 미션 FSM(mission_node)과 회피 패키지(limo_obstacle_avoid의 avoid_node)를 잇는다.
obstacle_node가 매 틱 update()를 부르고, 돌려받은 동작을 토픽으로 보낸다.

  1) 켜기   : FSM이 M7이고 앞바퀴가 3차선 시작선에 닿으면 /avoid/enable true
              (FSM은 횡단보도를 벗어나자마자 M7이 되는데, 그 지점은 시작선 1.68 m 앞 + 90° 코너 전이다.
               회피 노드는 켜는 순간을 원점·진행 방향으로 잡으므로 반드시 시작선에서 켜야 한다)
              켜기 직전에 avoid_node의 section_length(원점 ~ 섬 끝)를 지금 위치로 다시 계산해 넣는다.
  2) 갈림길 : FSM이 투표한 fork_choice(A/B)를 /fork_direction으로 넘긴다
  3) 끝     : /avoid/done → m7_done (FSM이 M2로 넘어감), 마지막 차로 → m7_exit(A/B)
  4) 중단   : 회피가 끝나기 전에 FSM이 M7을 벗어나면(강제 종료·수동 전환) /avoid/enable false
  5) 정지 유지(m7_hold): 회피 노드가 제어를 잡은 뒤 끝나기 전에 놓거나(Ctrl+C) 소식이 끊기면(죽음)
              FSM이 정지한다. 이게 없으면 Ctrl+C 때 차선 주행으로 넘어가 정차 LIMO 사이를 직진한다.
              (단독 시험의 cmd_mux failsafe_hold와 같은 역할. 팀 스택은 cmd_mux를 안 쓰므로 여기서 한다)
  6) 켜기 방식 enable_mode (팀 설정은 params.yaml obstacle_node: m7_enable_mode)
     'line'     : 1) 시작선(카메라 흰 선 + 오도메트리 / 도면 위치)에서 켬 (11/08까지 방식)
     'obstacle' : 횡단보도를 벗어나면(M7 진입) 라이다로 앞 차로를 감시하다가, 시작선 근처(trigger_gate_mm)부터
                  앞 차로 안 장애물이 trigger_front_mm 안에 들어오면 켬. 끝까지 없으면 trigger_fallback_mm에서 켬
  7) 끄기 방식 off_mode
     'none' : 회피 노드가 스스로 끝냄(섬 끝 + done_margin) / FSM 강제(섬 끝 + 50 cm)
     'sign' : 위에 더해, YOLO 갈림길 표지(fork_a/fork_b)를 확실히 본 뒤 지나가면(안 보이게 되면) 또는 가까워지면 끔.
              섬 앞 off_gate_mm 안이고 차가 바깥 차로(A/B)에 있을 때만 (가운데 차로에서 끄면 섬에 부딪힘)

시작선 위치는 세 가지로 판단한다 (위에서부터 우선).
  a. 카메라 흰 가로선: 시작선 근처에서 본 선까지 거리 - 그 뒤 달린 거리 (StopApproach와 같은 방식.
     선은 앞바퀴에 닿기 전에 화면 아래로 사라지므로 오도메트리로 이어 간다)
  b. 도면 경로 위치 route s (mission_node가 흰 선으로 계속 보정)
  c. 둘 다 없으면 M7 진입 후 달린 거리
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

M7_STATE = 'M7_OBSTACLE'


@dataclass
class M7LinkParams:
    # 도면 경로 위치 (track_route.yaml에서 계산. test_m7_link가 도면과 같은지 확인한다)
    start_s_mm: float = 13589.0         # 3차선 시작선
    island_s_mm: float = 17973.0        # 가운데 섬 오른쪽 끝 (= 회피 구간 끝)
    entry_to_start_mm: float = 1683.0   # 경로 정보가 없을 때: M7 진입(횡단보도 끝) ~ 시작선
    # 켜는 시점
    enable_at_mm: float = 0.0           # 앞바퀴가 시작선 + 이만큼에서 켬 (+면 선을 지나서)
    line_gate_mm: float = 400.0         # 카메라 선을 시작선으로 인정하는 도면 위치 오차
    line_min_entry_mm: float = 900.0    # 경로 정보가 없을 때: M7 진입 후 이만큼 달린 뒤 본 선만 인정 (코너 전 오검출 방지)
    max_heading_deg: float = 3.0        # 차선 방향이 이보다 틀어져 있으면 켜기를 잠시 미룸
    heading_valid_deg: float = 20.0     # 이보다 큰 차선 방향은 잘못 읽은 값으로 보고 무시 (3차선 입구 사선 등, 11/08 실차 ±64°)
    line_track_mm: float = 150.0        # 처음 잡은 선 뒤로는, 달린 거리로 예측한 거리와 이만큼 안에서만 이어 받음
    heading_wait_mm: float = 300.0      # 방향이 계속 틀어져 있어도 시작선 + 이만큼에선 켬 (정렬 보정이 ±5°까지 처리)
    # 회피 노드에 넘길 section_length 범위 (m)
    section_min_m: float = 3.5
    section_max_m: float = 4.8
    # 켜기 재시도: 회피 노드가 이 시간 안에 실행 상태를 안 알리면 다시 보냄
    retry_sec: float = 0.5
    # 켤 때까지 갈림길 표지를 못 읽었으면: 'auto'(회피 노드가 고름) | 'A' | 'B'
    fork_fallback: str = 'auto'
    # 정지 유지: 회피 노드는 실행 중 /avoid/active true를 20 Hz로 낸다
    active_timeout_sec: float = 1.0     # 이보다 오래 안 오면 죽은 것으로 보고 정지 (11/10 0.5 -> 1.0: CPU가 바빠 잠깐 늦는 것까지 멈춤으로 봤음)
    hold_resume: bool = True            # 소식 끊김으로 멈췄는데 active true가 다시 오면(노드가 잠깐 느렸던 것) 멈춤을 풀고 계속
    release_grace_sec: float = 0.3      # 정상 종료는 active false 직후 done이 온다 -> 이만큼 지나도 안 오면 '멈춤'으로 기록
    # 켜기 방식 (모듈 설명 6)
    enable_mode: str = 'line'           # 'line' | 'obstacle'
    trigger_front_mm: float = 1000.0    # obstacle: 라이다 -> 앞 차로 장애물이 이 안이면 켬. 시뮬레이션: 0.7 m는 7조건 중 3만 성공(멈춤·이탈), 0.9 m 이상 멈춤 없음
    trigger_half_width_mm: float = 200.0  # 앞 차로로 보는 좌우 폭(차 중심 ±). 옆 차로 LIMO(중심 ±520, 가장자리 ±410)는 안 들어옴
    trigger_gate_mm: float = -250.0     # obstacle: 시작선 기준 이 위치부터 감시 결과로 켬 (코너 끝 = 시작선 -288 mm)
    trigger_gate_early_mm: float = 300.0  # 카메라로 직선이 확인되면 문턱보다 이만큼 일찍부터 (도면 위치 늦음 대비, 0 = 안 씀)
    # 11/11 코너 지킴: 통주행에서는 M7에 들어온 뒤 우하단 코너(90°)를 돈다. 도면 위치가 앞서면 코너 안에서 회피를 켜
    #   회피 노드가 그 방향으로 직진 -> 바깥(노란 실선·벽)으로 나가고 정차 LIMO 위치도 틀어짐.
    #   진입 위치가 corner_need_before_mm 보다 앞이면, IMU로 코너를 다 돈 뒤에만 켠다 (도면 문턱 대신)
    #   코너 끝에서는 차가 아직 덜 펴져 있다(시뮬레이션: 시작선 -0.23 m에서 14°). 다 펴질 때까지 기다리면 늦으므로
    #   80° 돌면 켜고, 대신 '코너 진입 방향 ± 90°'를 직선 방향으로 보고 지금 틀어진 각을 회피 노드에 넘긴다 (start_heading_deg)
    corner_turn_deg: float = 80.0       # 0 = 안 씀
    corner_straight_deg: float = 180.0  # (M7은 다 펴질 때까지 안 기다림)
    corner_window_mm: float = 150.0
    corner_angle_deg: float = 90.0      # 코너 각 (도면 우하단 코너)
    corner_need_before_mm: float = -700.0   # M7 진입 위치(시작선 기준)가 이보다 앞이면 코너 지킴 (연습 set_state = -300 -> 안 씀)
    corner_fail_mm: float = 350.0       # 도면상 시작선 + 이만큼인데도 코너를 못 알아채면(IMU 이상) 예전 규칙으로
    trigger_fallback_mm: float = 1000.0  # obstacle: 앞 차로에 장애물이 끝까지 없으면 시작선 + 이만큼에서 켬
    # 끄기 방식 (모듈 설명 7)
    off_mode: str = 'none'              # 'none' | 'sign'
    off_sign_classes: Tuple[str, ...] = ('fork_a', 'fork_b')
    off_sign_conf: float = 0.5
    off_sign_window: int = 5            # 최근 이만큼 프레임 중
    off_sign_need: int = 3              # 이만큼 보이면 '표지 확인'
    off_sign_lost_frames: int = 5       # 확인한 뒤 이만큼 연속 안 보이면 '지나감'
    off_sign_near_h_px: float = 0.0     # 표지 상자 높이가 이 이상이면 바로 '도착' (0 = 안 씀)
    # 11/11: 프레임 수로 '지나감'을 세면 YOLO 속도·차 속도에 따라 끄는 위치가 30 cm 넘게 흔들림(m7_off_sim.py).
    #   표지는 옆 차로에서 보면 화면 옆으로 나가며 사라진다 -> 마지막 상자가 가장자리였으면 다음 프레임에 바로 '지나감'.
    #   그 지점부터 off_sign_after_mm 만큼 더 가서 끈다 (odom 1 m 안이라 오차 작음. 실차에서 원하는 곳에 맞춤)
    off_sign_edge_px: float = 10.0      # 상자가 화면 좌우 끝 이만큼 안에 닿으면 '가장자리' (검출기가 화면 밖은 잘라냄. 0 = 안 씀)
    off_sign_edge_frames: int = 2       # 가장자리에서 본 뒤 이만큼 연속 안 보이면 '화면 밖으로 나감' (한 번 놓친 것과 구분)
    off_sign_img_w: float = 640.0       # 카메라 가로 화소
    off_sign_after_mm: float = 400.0    # 지나감을 안 뒤 이만큼 더 가서 끔 (화각 70°·표지가 섬 앞 끝이면 섬 앞 끝 약 25 cm 전. 실차로 맞춤)
    off_gate_mm: float = 1500.0         # 섬 앞 끝까지 이 안일 때만 표지로 끔 (멀리서 가려져 사라지는 오판 방지)


LANE_NUMBER = {1: 1, 0: 2, -1: 3}          # 회피 차로(+1 왼쪽) -> 1차선(왼쪽, A) / 2차선 / 3차선(오른쪽, B)


def exit_report(lane, by: str) -> Optional[dict]:
    """M7이 끝날 때 /m7/exit_lane 으로 낼 내용. lane: 회피 노드 차로 번호(+1/0/-1)."""
    try:
        k = int(lane)
    except (TypeError, ValueError):
        return None
    if k not in LANE_NUMBER:
        return None
    n = LANE_NUMBER[k]
    fork = lane_to_fork(k)
    side = {1: '왼쪽', 2: '가운데', 3: '오른쪽'}[n]
    return {'lane': n, 'fork': fork, 'side': side, 'avoid_lane': k, 'by': by,
            'text': f'M7 끝: {n}차선({side}{", 갈림길 " + fork if fork else ""}) — {by}'}


def lane_to_fork(lane) -> Optional[str]:
    """회피 패키지 차로 번호(+1 왼쪽, -1 오른쪽) -> 갈림길 A/B (규정: A 좌측, B 우측)."""
    try:
        k = int(lane)
    except (TypeError, ValueError):
        return None
    return 'A' if k > 0 else 'B' if k < 0 else None


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class CornerWatch:
    """코너를 다 돌았나 (IMU 방향만, 도면 위치 안 씀).

    시작 방향에서 turn_deg 넘게 돌았고, 최근 window_mm 동안 방향 변화가 straight_deg 안이면 '다 돎' (한 번 되면 유지).
    R640 코너는 150 mm 동안 약 13° 돈다 -> 코너 안에서는 '다 돎'이 안 된다."""

    def __init__(self, turn_deg: float = 75.0, straight_deg: float = 4.0, window_mm: float = 150.0):
        self.turn, self.straight, self.window = math.radians(turn_deg), math.radians(straight_deg), window_mm
        self.yaw0 = None
        self.hist = deque()
        self.turned = 0.0
        self.done = False

    def start(self, yaw: Optional[float], dist: float) -> None:
        self.yaw0 = yaw
        self.hist.clear()
        self.turned, self.done = 0.0, False
        if yaw is not None:
            self.hist.append((dist, yaw))

    @property
    def active(self) -> bool:
        return self.yaw0 is not None

    def update(self, yaw: Optional[float], dist: float) -> bool:
        if not self.active or yaw is None:
            return self.done
        self.hist.append((dist, yaw))
        while len(self.hist) > 2 and dist - self.hist[1][0] >= self.window:
            self.hist.popleft()
        self.turned = abs(_wrap(yaw - self.yaw0))
        recent = abs(_wrap(yaw - self.hist[0][1])) if dist - self.hist[0][0] >= self.window else None
        if not self.done and self.turned >= self.turn and recent is not None and recent <= self.straight:
            self.done = True
        return self.done


class M7Link:
    def __init__(self, params: Optional[M7LinkParams] = None):
        self.p = params or M7LinkParams()
        self.phase = 'idle'          # idle -> armed(켜기 보냄) -> running -> done | aborted
        self.in_m7 = False
        self.entry_dist = None       # M7 진입 때 누적 거리 (mission dist_mm)
        self.line = None             # (본 선까지 거리, 그때 누적 거리)
        self.pos = None              # 마지막으로 계산한 앞바퀴 - 시작선 (mm), 진단용
        self.enable_t = None
        self.section_m = None
        self.heading_forced = False
        self.how = None              # 'line' | 'route' | 'distance'
        self.fork_sent = None
        self.exit_fork = None        # 회피가 끝난 차로 (A/B), M7을 벗어나도 유지 (FSM M2가 씀)
        self.avoid_active = False
        self.active_t = None         # 마지막 /avoid/active true 시각
        self.claimed = False         # 회피 노드가 제어를 잡았었나 (dry_run이면 끝까지 False)
        self.release_t = None        # 끝나기 전에 active false를 받은 시각
        self.hold = False
        self.hold_kind = None        # 'timeout' | 'release'
        self.hold_resumed = 0        # 소식이 다시 와서 풀린 횟수
        self.avoid_running = False
        self.avoid_target = None
        self.avoid_lane = None       # 회피 노드가 지금 있다고 보는 차로 (/avoid/state now_lane)
        self.front_mm = None         # 라이다 -> 앞 차로 안 가장 가까운 장애물 (mm), obstacle_node가 넣어 줌
        self.sign_buf = deque(maxlen=max(1, int(self.p.off_sign_window)))
        self.sign_seen = False       # 표지를 확실히 봤나
        self.sign_lost = 0           # 확인한 뒤 연속으로 안 보인 프레임
        self.sign_near = False
        self.sign_edge = False       # 마지막으로 본 표지 상자가 화면 가장자리였나
        self.sign_exit = False       # 가장자리로 나가며 사라짐 (= 지나감)
        self.sign_pass_dist = None   # '지나감'을 안 순간의 달린 거리 (mm)
        self.off_by = None           # 'sign' | 'avoid_done'
        self.report = None           # M7이 끝날 때 한 번 내보낼 차로 (exit_report), obstacle_node가 pop
        # 지금 몇 차선인지 (늘): 회피 중이면 회피 노드 차로(카메라 선 종류로 이미 맞춘 값), 아니면 차선 노드 lane_id 투표
        self.cam_ids = deque(maxlen=7)
        self.cam_lane = None         # 차선 노드 lane_id 를 7번 중 5번 같으면 확정 (+1/0/-1)
        self.last_lane = None        # (차로, 출처) 마지막으로 안 값 — 잠깐 못 봐도 유지
        self.trigger = None          # 켠 이유: 'line' | 'obstacle' | 'fallback'
        self.yaw = None              # 최근 IMU 방향 (obstacle_node가 on_yaw로 넣음)
        self.corner = CornerWatch(self.p.corner_turn_deg, self.p.corner_straight_deg, self.p.corner_window_mm)
        self.corner_need = None      # None: 아직 모름(진입 직후), True: 코너 지킴, False: 안 씀
        self.start_heading_deg = None  # 켤 때 차가 3차선 직선에서 틀어진 각 (obstacle_node가 회피 노드에 넘김)
        self.events: List[str] = []

    def on_yaw(self, yaw: Optional[float]) -> None:
        self.yaw = yaw

    # --- 회피 노드 쪽 입력 ----------------------------------------------------
    def on_avoid_active(self, active: bool, t: Optional[float] = None) -> None:
        self.avoid_active = bool(active)
        if active:
            self.active_t = t
            self.release_t = None
            if self.hold and self.hold_kind == 'timeout' and self.p.hold_resume and self.phase == 'running':
                self.hold = False
                self.hold_kind = None
                self.hold_resumed += 1
                self.events.append('회피 노드 소식 다시 옴 -> 정지 풀고 계속 (잠깐 느렸던 것)')
            if self.phase in ('armed', 'running'):
                self.claimed = True
            if self.phase == 'armed':
                self.phase = 'running'
        elif self.claimed and self.phase == 'running' and self.release_t is None:
            self.release_t = t

    def on_avoid_state(self, st: dict) -> None:
        """/avoid/state (JSON). dry_run이면 active가 안 오므로 state로도 실행 여부를 안다."""
        self.avoid_running = st.get('state') == 'RUN'
        if 'target' in st:
            self.avoid_target = st.get('target')
        if 'now_lane' in st:
            self.avoid_lane = st.get('now_lane')
            if self.phase in ('armed', 'running') and self.avoid_lane in LANE_NUMBER:
                self.last_lane = (self.avoid_lane, 'avoid')   # 회피가 끝난 직후에도 그 차로를 기억
        if self.avoid_running and self.phase == 'armed':
            self.phase = 'running'

    def on_front(self, front_mm: Optional[float]) -> None:
        """라이다: 앞 차로(차 중심 ±trigger_half_width_mm) 안 가장 가까운 장애물까지 거리 (라이다 기준, mm)."""
        self.front_mm = front_mm

    def on_detections(self, dets: Sequence[dict]) -> None:
        """/perception/detections 한 프레임. 갈림길 표지를 확인한 뒤 지나감(안 보임)·가까워짐을 센다."""
        if not self.in_m7:
            return
        hits = [d for d in dets if d.get('cls') in self.p.off_sign_classes
                and float(d.get('conf', 0.0)) >= self.p.off_sign_conf]
        seen = bool(hits)
        self.sign_buf.append(seen)
        if not self.sign_seen and sum(self.sign_buf) >= self.p.off_sign_need:
            self.sign_seen = True
            self.events.append('갈림길 표지 확인 (끄기 준비)')
        if seen and self.p.off_sign_edge_px > 0:
            e, w = self.p.off_sign_edge_px, self.p.off_sign_img_w
            self.sign_edge = any(len(d.get('bbox') or []) == 4
                                 and (d['bbox'][0] <= e or d['bbox'][0] + d['bbox'][2] >= w - e) for d in hits)
        if self.sign_seen:
            self.sign_lost = 0 if seen else self.sign_lost + 1
            if (not seen and self.sign_edge and not self.sign_exit
                    and self.sign_lost >= max(1, int(self.p.off_sign_edge_frames))):
                self.sign_exit = True
                self.events.append('갈림길 표지가 화면 옆으로 나감 (지나감)')
        if self.p.off_sign_near_h_px > 0 and any(
                len(d.get('bbox') or []) == 4 and d['bbox'][3] >= self.p.off_sign_near_h_px for d in hits):
            self.sign_near = True

    def _end_lane(self):
        return self.avoid_lane if self.avoid_lane is not None else self.avoid_target

    def pop_report(self) -> Optional[dict]:
        r, self.report = self.report, None
        return r

    def on_avoid_done(self, done: bool) -> None:
        if done and self.phase in ('armed', 'running'):
            self.off_by = 'avoid_done'
            self.phase = 'done'
            self.report = exit_report(self._end_lane(), 'avoid_done')
            self.exit_fork = lane_to_fork(self.avoid_target)
            self.events.append(f'회피 완료 (마지막 차로 {self.avoid_target} -> {self.exit_fork})')

    # --- 지금 몇 차선 (늘) ------------------------------------------------------
    def watch_lane_id(self, lane: dict) -> None:
        """차선 노드 /lane/status 의 lane_id (선 종류로 본 3차선 번호, +1/0/-1, 양쪽 선이 보일 때만)."""
        lid = (lane or {}).get('lane_id')
        self.cam_ids.append(lid if lid in LANE_NUMBER else None)
        best = None
        for k in LANE_NUMBER:
            if list(self.cam_ids).count(k) >= 5:
                best = k
        self.cam_lane = best

    def lane_now(self) -> Optional[dict]:
        """{'lane': 1|2|3, 'avoid_lane': +1|0|-1, 'side', 'src': 'avoid'|'camera'|'last'|'start'} 또는 None.
        'start': M7에 들어왔는데 아직 아무것도 모름 -> 3차선 구간은 가운데 차로로 들어가므로 2차선."""
        if self.phase in ('armed', 'running') and self.avoid_lane in LANE_NUMBER:
            k, src = self.avoid_lane, 'avoid'          # 회피 노드: odom+IMU + 카메라 선 종류·차선 보정
        elif self.cam_lane is not None:
            k, src = self.cam_lane, 'camera'           # 회피 전·후: 차선 노드 선 종류
        elif self.last_lane is not None:
            k = self.last_lane[0]                      # 잠깐 못 봄: 마지막 값
            src = 'start' if self.last_lane[1] == 'start' else 'last'
        else:
            return None
        if src not in ('last', 'start'):
            self.last_lane = (k, src)
        n = LANE_NUMBER[k]
        return {'lane': n, 'avoid_lane': k, 'side': {1: '왼쪽', 2: '가운데', 3: '오른쪽'}[n], 'src': src}

    # --- 매 틱 -----------------------------------------------------------------
    def update(self, t: float, mission: dict, lane: dict) -> List[Tuple]:
        """mission: /mission/state, lane: /lane/status. 보낼 동작 목록을 돌려준다.

        ('enable', section_m) | ('disable',) | ('fork', 'A'|'B')
        """
        out: List[Tuple] = []
        self.watch_lane_id(lane)
        state = mission.get('state')
        dist = float(mission.get('dist_mm') or 0.0)

        if state != M7_STATE:
            if self.in_m7 and self.phase in ('armed', 'running'):
                out.append(('disable',))
                self.events.append('FSM이 M7을 벗어남 -> 회피 중단')
                self.phase = 'aborted'
                self.report = exit_report(self._end_lane(), 'forced')
            self.in_m7 = False
            return out

        if not self.in_m7:                      # M7에 새로 들어옴
            self.in_m7 = True
            if self.cam_lane is None:
                self.last_lane = (0, 'start')   # 3차선 구간은 가운데로 들어감 -> 카메라가 확정하기 전에도 2차선 (지난 판 값은 버림)
            self.phase = 'idle'
            self.entry_dist = dist
            self.corner.start(self.yaw if self.p.corner_turn_deg > 0 else None, dist)
            self.corner_need = None
            self.line = None
            self.enable_t = self.section_m = self.how = None
            self.heading_forced = False
            self.fork_sent = None
            self.claimed = self.hold = False
            self.hold_kind = None
            self.active_t = self.release_t = None
            self.sign_buf.clear()
            self.sign_seen = self.sign_near = False
            self.sign_edge = self.sign_exit = False
            self.sign_pass_dist = None
            self.sign_lost = 0
            self.off_by = None
            self.trigger = None

        fork = mission.get('fork_choice')
        if fork in ('A', 'B') and fork != self.fork_sent:
            out.append(('fork', fork))
            self.fork_sent = fork

        if self.phase in ('done', 'aborted'):
            return out
        self._check_hold(t)

        route = mission.get('route') or {}
        # 11/10: mission_node가 '길 잃음(lost)'으로 표시하면 도면 위치를 안 쓴다 (카메라 선 / 진입 후 거리로)
        s = None if route.get('lost') else route.get('s_mm')
        self._watch_line(lane, s, dist)
        pos = self._front_from_start(s, dist)   # 앞바퀴 - 시작선 (mm, +면 지남)
        self.pos = pos
        if self.corner_need is None:
            self.corner_need = bool(self.corner.active and (pos is None or pos < self.p.corner_need_before_mm))
            if self.corner_need:
                self.events.append(f'코너 지킴: M7 진입 위치 {pos if pos is None else round(pos)} mm -> IMU로 코너를 다 돈 뒤에 켬')
        if self.corner_need and not self.corner.done and self.corner.update(self.yaw, dist):
            self.events.append(f'코너 다 돎 (IMU {math.degrees(self.corner.turned):.0f}°, 시작선 {pos if pos is None else round(pos)} mm)')

        if self.phase == 'running' and self.p.off_mode == 'sign':
            why = self._sign_off(pos, dist)
            if why:
                self.phase = 'done'
                self.off_by = 'sign'
                lane_now = self._end_lane()
                self.exit_fork = lane_to_fork(lane_now)
                self.report = exit_report(lane_now, 'sign')
                out.append(('disable',))
                self.events.append(f'회피 끔: {why} (시작선 {pos:+.0f} mm, 차로 {lane_now} -> {self.exit_fork})')
                return out

        if self.phase == 'idle' and self.p.enable_mode == 'obstacle':
            if pos is None:
                return out
            if self.corner_need and not self.corner.done:
                if pos < self.p.corner_fail_mm:
                    return out                  # 아직 코너 안 (도면 위치와 상관없이)
                self.corner_need = False        # IMU로 코너를 못 알아챔 -> 예전 규칙
                self.events.append(f'코너를 IMU로 못 알아챔 (돈 각 {math.degrees(self.corner.turned):.0f}°) -> 도면 문턱으로')
            if self.corner_need:
                pass                            # 코너를 다 돎 = 직선: 도면 문턱 대신 이것으로 받음
            elif pos < self.p.trigger_gate_mm:
                # 도면 위치가 늦게 잡혀(11/11 시뮬레이션: -0.3 m면 43 cm 늦게 켬) 이미 직선인데 기다리는 것 방지:
                #   카메라 양쪽 선이 곧게 보이면(코너를 벗어남) 문턱 - trigger_gate_early_mm 부터 받는다
                straight = (lane.get('status') == 'BOTH' and lane.get('heading_rad') is not None
                            and abs(math.degrees(float(lane['heading_rad']))) <= self.p.max_heading_deg)
                if not (straight and pos >= self.p.trigger_gate_mm - self.p.trigger_gate_early_mm):
                    return out
            near = self.front_mm is not None and self.front_mm <= self.p.trigger_front_mm
            if not near and pos < self.p.trigger_fallback_mm:
                return out
            self.trigger = 'obstacle' if near else 'fallback'
            why = f'앞 장애물 {self.front_mm:.0f} mm' if near else '앞 차로에 장애물이 없어 예비 위치'
            return out + self._arm(t, pos, why)

        if self.phase == 'idle':
            if pos is None or pos < self.p.enable_at_mm:
                return out
            if not self._heading_ok(lane):
                if pos < self.p.enable_at_mm + self.p.heading_wait_mm:
                    return out
                self.heading_forced = True
                self.events.append('차선 방향이 계속 틀어져 있어 그대로 켬 (정렬 보정에 맡김)')
            self.trigger = 'line'
            return out + self._arm(t, pos, '시작선')
        elif self.phase == 'armed' and not (self.avoid_active or self.avoid_running):
            if t - self.enable_t >= self.p.retry_sec:
                # 회피 노드가 아직 안 받았다 (센서 준비 전 등). 지금 위치로 다시 계산해 보냄
                if pos is not None:
                    self.section_m = self._section(pos)
                self.enable_t = t
                out.append(('enable', self.section_m))
        return out

    # --- 내부 -------------------------------------------------------------------
    def _straight_heading_deg(self) -> Optional[float]:
        """코너를 IMU로 돈 경우: 지금 차 방향 - 3차선 직선 방향(진입 방향 ± 코너 각). 모르면 None."""
        if not (self.corner_need and self.corner.active and self.yaw is not None):
            return None
        turn = _wrap(self.yaw - self.corner.yaw0)
        axis = self.corner.yaw0 + math.copysign(math.radians(self.p.corner_angle_deg), turn)
        return math.degrees(_wrap(self.yaw - axis))

    def _arm(self, t: float, pos: float, why: str) -> List[Tuple]:
        out: List[Tuple] = []
        self.start_heading_deg = self._straight_heading_deg()
        if self.fork_sent is None and self.p.fork_fallback in ('A', 'B'):
            out.append(('fork', self.p.fork_fallback))
            self.fork_sent = self.p.fork_fallback
        self.section_m = self._section(pos)
        self.phase = 'armed'
        self.enable_t = t
        out.append(('enable', self.section_m))
        hd = '' if self.start_heading_deg is None else f', 직선 대비 {self.start_heading_deg:+.0f}°'
        self.events.append(f'회피 켬 ({why}): {self.trigger}, 위치 기준 {self.how}, 시작선 {pos:+.0f} mm, '
                           f'section {self.section_m:.2f} m{hd}')
        return out

    def _sign_off(self, pos: Optional[float], dist: float = 0.0) -> Optional[str]:
        """표지로 끌 때인지. 안전 조건: 섬 앞 off_gate_mm 안 + 바깥 차로(A/B).
        지나감 = 가까움(상자 높이) | 화면 옆으로 나감 | 확인 뒤 off_sign_lost_frames 연속 안 보임. 그 뒤 off_sign_after_mm 더 감."""
        if not self.sign_seen:
            return None
        if self.sign_near:
            why = '표지 가까움'
        elif self.sign_exit:
            why = '표지 화면 밖으로 나감'
        elif self.sign_lost >= self.p.off_sign_lost_frames:
            why = '표지 지나감'
        else:
            return None
        if self.sign_pass_dist is None:
            self.sign_pass_dist = dist
        if dist - self.sign_pass_dist < self.p.off_sign_after_mm:
            return None
        island = self.p.island_s_mm - self.p.start_s_mm
        if pos is None or pos < island - self.p.off_gate_mm:
            return None
        lane = self.avoid_lane if self.avoid_lane is not None else self.avoid_target
        if lane in (None, 0):
            return None                      # 아직 가운데 차로: 섬에 부딪히므로 끄지 않음
        return why

    def _check_hold(self, t: float) -> None:
        if self.hold or not self.claimed or self.phase != 'running':
            return
        why = kind = None
        if self.release_t is not None and t - self.release_t >= self.p.release_grace_sec:
            why, kind = '회피 노드가 끝나기 전에 제어를 놓음 (Ctrl+C 등)', 'release'
        elif (self.release_t is None and self.active_t is not None
              and t - self.active_t >= self.p.active_timeout_sec):
            why, kind = f'회피 노드 소식이 {self.p.active_timeout_sec:.1f}초 끊김', 'timeout'
        if why:
            self.hold = True
            self.hold_kind = kind
            self.events.append(f'{why} -> 정지 유지 (풀려면 /mission/set_state로 다른 상태)')

    def _watch_line(self, lane: dict, s, dist: float) -> None:
        if self.phase != 'idle':
            return
        lines = lane.get('stoplines_mm') or []
        if not lines:
            return
        if self.line is not None:
            # 이미 잡은 선: 달린 거리로 예측한 위치와 맞는 검출만 이어 받는다
            # (입구 사선·점선 끝을 흰 선으로 잘못 읽어 기준이 계속 앞으로 밀리는 것을 막음)
            d0, at = self.line
            pred = d0 - (dist - at)
            near = [float(x) for x in lines if abs(float(x) - pred) <= self.p.line_track_mm]
            if near:
                self.line = (min(near, key=lambda x: abs(x - pred)), dist)
            return
        d = float(lines[0])
        if s is not None:
            ok = abs(float(s) + d - self.p.start_s_mm) <= self.p.line_gate_mm
        else:
            ok = dist - (self.entry_dist or 0.0) >= self.p.line_min_entry_mm
        if ok:
            self.line = (d, dist)

    def _front_from_start(self, s, dist: float) -> Optional[float]:
        route_pos = float(s) - self.p.start_s_mm if s is not None else None
        if self.line is not None:
            d, at = self.line
            line_pos = (dist - at) - d
            if route_pos is not None and route_pos - line_pos > self.p.line_gate_mm:
                # 카메라 선 기준이 도면 위치보다 크게 늦다 -> 다른 선을 잡은 것. 도면 위치를 따른다
                self.line = None
                self.events.append(f'카메라 선 기준이 도면보다 {route_pos - line_pos:.0f} mm 늦어 버림 -> 도면 위치 사용')
            else:
                self.how = 'line'
                return line_pos
        if s is not None:
            self.how = 'route'
            return float(s) - self.p.start_s_mm
        if self.entry_dist is not None:
            self.how = 'distance'
            return dist - self.entry_dist - self.p.entry_to_start_mm
        return None

    def _heading_ok(self, lane: dict) -> bool:
        h = lane.get('heading_rad')
        if h is None or lane.get('status') not in ('BOTH', 'L', 'R'):
            return True          # 차선 방향을 모르면 막지 않는다
        deg = abs(math.degrees(float(h)))
        if deg > self.p.heading_valid_deg:
            return True          # 말이 안 되게 큰 값은 잘못 읽은 것 (입구 사선) -> 막지 않음
        return deg <= self.p.max_heading_deg

    def _section(self, pos: float) -> float:
        L = (self.p.island_s_mm - self.p.start_s_mm - pos) / 1000.0
        return max(self.p.section_min_m, min(self.p.section_max_m, L))

    def state(self) -> dict:
        """/obstacle/state에 합쳐 FSM에 넘길 값."""
        return {'m7_phase': self.phase,
                'm7_active': self.avoid_active and self.phase == 'running' and not self.hold,
                # 끝나기 전에 놓으면 바로 정지 (done이 곧 오면 FSM이 done을 먼저 보고 넘어간다)
                'm7_hold': self.hold or (self.phase == 'running' and self.release_t is not None),
                'm7_hold_kind': self.hold_kind,
                'm7_hold_resumed': self.hold_resumed,
                'm7_done': self.phase == 'done',
                'm7_exit': self.exit_fork,
                'm7_section_m': self.section_m,
                'm7_how': self.how,
                'm7_pos_mm': None if self.pos is None else round(self.pos),
                # 앞바퀴 -> 섬 앞 끝 (mm, +면 아직 앞). FSM이 도면 위치를 못 쓸 때 M2 끝 거리를 맞추는 데 씀
                'm7_to_tip_mm': (None if self.pos is None
                                 else round(self.p.island_s_mm - self.p.start_s_mm - self.pos)),
                'm7_front_mm': None if self.front_mm is None else round(self.front_mm),
                'm7_sign': ('near' if self.sign_near else 'exit' if self.sign_exit
                            else (f'lost{self.sign_lost}' if self.sign_seen else None)),
                'm7_off_by': self.off_by,
                'm7_trigger': self.trigger,
                'm7_corner': (None if not self.corner_need and not self.corner.done
                              else ('done' if self.corner.done else f'wait{math.degrees(self.corner.turned):.0f}')),
                'm7_lane': (self.lane_now() or {}).get('lane'),
                'm7_lane_src': (self.lane_now() or {}).get('src'),
                'm7_line_d_mm': None if self.line is None else round(self.line[0]),
                'm7_heading_forced': self.heading_forced}
