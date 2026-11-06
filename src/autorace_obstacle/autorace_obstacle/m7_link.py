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

시작선 위치는 세 가지로 판단한다 (위에서부터 우선).
  a. 카메라 흰 가로선: 시작선 근처에서 본 선까지 거리 - 그 뒤 달린 거리 (StopApproach와 같은 방식.
     선은 앞바퀴에 닿기 전에 화면 아래로 사라지므로 오도메트리로 이어 간다)
  b. 도면 경로 위치 route s (mission_node가 흰 선으로 계속 보정)
  c. 둘 다 없으면 M7 진입 후 달린 거리
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

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
    heading_wait_mm: float = 300.0      # 방향이 계속 틀어져 있어도 시작선 + 이만큼에선 켬 (정렬 보정이 ±5°까지 처리)
    # 회피 노드에 넘길 section_length 범위 (m)
    section_min_m: float = 3.5
    section_max_m: float = 4.8
    # 켜기 재시도: 회피 노드가 이 시간 안에 실행 상태를 안 알리면 다시 보냄
    retry_sec: float = 0.5
    # 켤 때까지 갈림길 표지를 못 읽었으면: 'auto'(회피 노드가 고름) | 'A' | 'B'
    fork_fallback: str = 'auto'


def lane_to_fork(lane) -> Optional[str]:
    """회피 패키지 차로 번호(+1 왼쪽, -1 오른쪽) -> 갈림길 A/B (규정: A 좌측, B 우측)."""
    try:
        k = int(lane)
    except (TypeError, ValueError):
        return None
    return 'A' if k > 0 else 'B' if k < 0 else None


class M7Link:
    def __init__(self, params: Optional[M7LinkParams] = None):
        self.p = params or M7LinkParams()
        self.phase = 'idle'          # idle -> armed(켜기 보냄) -> running -> done | aborted
        self.in_m7 = False
        self.entry_dist = None       # M7 진입 때 누적 거리 (mission dist_mm)
        self.line = None             # (본 선까지 거리, 그때 누적 거리)
        self.enable_t = None
        self.section_m = None
        self.heading_forced = False
        self.how = None              # 'line' | 'route' | 'distance'
        self.fork_sent = None
        self.exit_fork = None        # 회피가 끝난 차로 (A/B), M7을 벗어나도 유지 (FSM M2가 씀)
        self.avoid_active = False
        self.avoid_running = False
        self.avoid_target = None
        self.events: List[str] = []

    # --- 회피 노드 쪽 입력 ----------------------------------------------------
    def on_avoid_active(self, active: bool) -> None:
        self.avoid_active = bool(active)
        if active and self.phase == 'armed':
            self.phase = 'running'

    def on_avoid_state(self, st: dict) -> None:
        """/avoid/state (JSON). dry_run이면 active가 안 오므로 state로도 실행 여부를 안다."""
        self.avoid_running = st.get('state') == 'RUN'
        if 'target' in st:
            self.avoid_target = st.get('target')
        if self.avoid_running and self.phase == 'armed':
            self.phase = 'running'

    def on_avoid_done(self, done: bool) -> None:
        if done and self.phase in ('armed', 'running'):
            self.phase = 'done'
            self.exit_fork = lane_to_fork(self.avoid_target)
            self.events.append(f'회피 완료 (마지막 차로 {self.avoid_target} -> {self.exit_fork})')

    # --- 매 틱 -----------------------------------------------------------------
    def update(self, t: float, mission: dict, lane: dict) -> List[Tuple]:
        """mission: /mission/state, lane: /lane/status. 보낼 동작 목록을 돌려준다.

        ('enable', section_m) | ('disable',) | ('fork', 'A'|'B')
        """
        out: List[Tuple] = []
        state = mission.get('state')
        dist = float(mission.get('dist_mm') or 0.0)

        if state != M7_STATE:
            if self.in_m7 and self.phase in ('armed', 'running'):
                out.append(('disable',))
                self.events.append('FSM이 M7을 벗어남 -> 회피 중단')
                self.phase = 'aborted'
            self.in_m7 = False
            return out

        if not self.in_m7:                      # M7에 새로 들어옴
            self.in_m7 = True
            self.phase = 'idle'
            self.entry_dist = dist
            self.line = None
            self.enable_t = self.section_m = self.how = None
            self.heading_forced = False
            self.fork_sent = None

        fork = mission.get('fork_choice')
        if fork in ('A', 'B') and fork != self.fork_sent:
            out.append(('fork', fork))
            self.fork_sent = fork

        if self.phase in ('done', 'aborted'):
            return out

        route = mission.get('route') or {}
        s = route.get('s_mm')
        self._watch_line(lane, s, dist)
        pos = self._front_from_start(s, dist)   # 앞바퀴 - 시작선 (mm, +면 지남)

        if self.phase == 'idle':
            if pos is None or pos < self.p.enable_at_mm:
                return out
            if not self._heading_ok(lane):
                if pos < self.p.enable_at_mm + self.p.heading_wait_mm:
                    return out
                self.heading_forced = True
                self.events.append('차선 방향이 계속 틀어져 있어 그대로 켬 (정렬 보정에 맡김)')
            if self.fork_sent is None and self.p.fork_fallback in ('A', 'B'):
                out.append(('fork', self.p.fork_fallback))
                self.fork_sent = self.p.fork_fallback
            self.section_m = self._section(pos)
            self.phase = 'armed'
            self.enable_t = t
            out.append(('enable', self.section_m))
            self.events.append(f'회피 켬: {self.how}, 시작선 {pos:+.0f} mm, section {self.section_m:.2f} m')
        elif self.phase == 'armed' and not (self.avoid_active or self.avoid_running):
            if t - self.enable_t >= self.p.retry_sec:
                # 회피 노드가 아직 안 받았다 (센서 준비 전 등). 지금 위치로 다시 계산해 보냄
                if pos is not None:
                    self.section_m = self._section(pos)
                self.enable_t = t
                out.append(('enable', self.section_m))
        return out

    # --- 내부 -------------------------------------------------------------------
    def _watch_line(self, lane: dict, s, dist: float) -> None:
        if self.phase != 'idle':
            return
        lines = lane.get('stoplines_mm') or []
        if not lines:
            return
        d = float(lines[0])
        if s is not None:
            ok = abs(float(s) + d - self.p.start_s_mm) <= self.p.line_gate_mm
        else:
            ok = dist - (self.entry_dist or 0.0) >= self.p.line_min_entry_mm
        if ok:
            self.line = (d, dist)

    def _front_from_start(self, s, dist: float) -> Optional[float]:
        if self.line is not None:
            d, at = self.line
            self.how = 'line'
            return (dist - at) - d
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
        return abs(math.degrees(float(h))) <= self.p.max_heading_deg

    def _section(self, pos: float) -> float:
        L = (self.p.island_s_mm - self.p.start_s_mm - pos) / 1000.0
        return max(self.p.section_min_m, min(self.p.section_max_m, L))

    def state(self) -> dict:
        """/obstacle/state에 합쳐 FSM에 넘길 값."""
        return {'m7_phase': self.phase,
                'm7_active': self.avoid_active and self.phase == 'running',
                'm7_done': self.phase == 'done',
                'm7_exit': self.exit_fork,
                'm7_section_m': self.section_m,
                'm7_how': self.how,
                'm7_heading_forced': self.heading_forced}
