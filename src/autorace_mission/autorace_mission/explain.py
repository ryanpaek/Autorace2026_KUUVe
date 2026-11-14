"""디버그 화면용 '왜 지금 이렇게 하나' 설명 (ROS 의존성 없음, 11/11).

mission_node·인식·차선·장애물 노드가 이미 내는 토픽만 보고, 사람이 읽을 문장으로 바꾼다.
제어에는 아무 영향이 없다 (debug_view만 쓴다). FSM 조건이 바뀌면 여기도 같이 고쳐야 화면이 맞다.

explain(...) -> {
  'title':   지금 하는 일 (한 줄),
  'control': 누가 조향하나 · 속도,
  'because': 지금 이 상태가 된 근거 (문장 목록),
  'next':    다음 단계로 넘어가는 조건들 [{'text', 'value', 'progress'(0~1|None), 'met'}],
  'warn':    주의 (FALLBACK·길 잃음·노드 끊김 등),
}
"""

from typing import Dict, List, Optional

STATE_KO = {
    'WAIT_START': '출발 대기', 'LANE_ONLY': '차선 주행만', 'FINISHED': '완료',
    'M5_BUMP': 'M5 방지턱', 'M6_ZIGZAG': 'M6 지그재그', 'M3_PEDESTRIAN': 'M3 보행자',
    'M7_OBSTACLE': 'M7 3차선 장애물', 'M2_FORK': 'M2 갈림길', 'M2_EXIT': 'M2→M8 차선 주행', 'M8_CONES': 'M8 라바콘',
    'M1_TRAFFIC': 'M1 신호등', 'M4_PARKING': 'M4 주차',
}
SOURCE_KO = {'lane': '차선 노드', 'obstacle': '장애물·회피 노드', 'parking': '주차 노드',
             'stop': '정지 (미션 노드)'}
CLASS_KO = {'red': '빨간불', 'yellow': '노란불', 'green': '초록불', 'fork_a': '갈림길 A',
            'fork_b': '갈림길 B', 'pedestrian': '보행자 표지', 'bump': '방지턱 표지',
            'slow': '서행 표지', 'parking': '주차 표지'}

# fsm.DEFAULT_PARAMS 중 설명에 쓰는 값 (실제 값은 debug_view가 params로 덮어쓴다)
DEFAULTS = {
    'route_slow_before_mm': 300.0, 'bump_slow_mm': 1800.0, 'bump_after_mm': 300.0,
    'zigzag_len_mm': 1800.0, 'ped_wait_sec': 3.5, 'leave_mm': 600.0,
    'arm_on_crosswalk_mm': 1200.0, 'route_island_margin_mm': 300.0,
    'm7_force_after_island_mm': 500.0, 'fork_seek_timeout_sec': 12.0, 'fork_default': 'A',
    'cones_enter_before_mm': 0.0, 'cones_enter_cone_mm': 500.0, 'cones_gone_mm': 800.0,
    'cones_end_after_mm': 150.0, 'm4_arm_before_mm': 800.0, 'green_go_delay_sec': 0.3,
    'state_max_mm': 10000.0,
}


def _mm(v: Optional[float]) -> str:
    if v is None:
        return '-'
    return f'{v / 1000:.2f} m' if abs(v) >= 1000 else f'{v:.0f} mm'


class _Ctx:
    def __init__(self, mission, lane, commands, obstacle, parking, points, params, sub_age):
        self.m = mission or {}
        self.lane = lane or {}
        self.cm = commands or {}
        self.ob = obstacle or {}
        self.pk = parking or {}
        self.points = points or {}
        self.p = dict(DEFAULTS, **(params or {}))
        self.sub_age = sub_age
        rt = self.m.get('route') or {}
        self.route = rt
        self.s = rt.get('s_mm') if rt and not rt.get('lost') else None
        self.next: List[dict] = []
        self.because: List[str] = []
        self.warn: List[str] = []

    # --- 인식 ------------------------------------------------------------
    def confirmed(self, *classes) -> bool:
        conf = set(self.cm.get('confirmed_classes') or [])
        return any(c in conf for c in classes)

    def vote(self, cls: str) -> str:
        v = self.cm.get('vote') or {}
        cnt = (v.get('counts') or {}).get(cls)
        if cnt is None:
            return '인식 노드 없음'
        return f"최근 {v.get('frames', '?')}/{v.get('window', '?')}프레임 중 {cnt}번 (필요 {v.get('need', '?')})"

    def vote_progress(self, cls: str) -> Optional[float]:
        v = self.cm.get('vote') or {}
        cnt = (v.get('counts') or {}).get(cls)
        need = v.get('need')
        if cnt is None or not need:
            return None
        return min(1.0, cnt / need)

    # --- 거리 ------------------------------------------------------------
    def to_point(self, name: str, offset: float = 0.0) -> Optional[float]:
        """도면 지점(+offset)까지 앞바퀴 기준 남은 거리. 위치 모르면 None."""
        if self.s is None or name not in self.points:
            return None
        return self.points[name] + offset - self.s

    def cam_line(self) -> Optional[float]:
        lines = [d for d in (self.lane.get('stoplines_mm') or []) if d >= -50.0]
        return min(lines) if lines else None

    # --- 조건 추가 -------------------------------------------------------
    def cond(self, text: str, value: str = '', progress: Optional[float] = None, met: bool = False):
        self.next.append({'text': text, 'value': value,
                          'progress': None if progress is None else round(max(0.0, min(1.0, progress)), 2),
                          'met': bool(met)})

    def sign_cond(self, cls: str, extra: str = ''):
        ok = self.confirmed(cls)
        self.cond(f'{CLASS_KO.get(cls, cls)} 확정{extra}', self.vote(cls), self.vote_progress(cls), ok)

    def route_cond(self, text: str, name: str, offset: float = 0.0, span: float = 3000.0):
        r = self.to_point(name, offset)
        if r is None:
            self.cond(f'{text} (도면)', '도면 위치 모름' if self.route.get('lost') or not self.route else '-')
        else:
            self.cond(f'{text} (도면)', f'{_mm(max(r, 0.0))} 남음', 1.0 - r / span, r <= 0.0)

    def stopline_cond(self, name: str):
        cam = self.cam_line()
        r = self.to_point(name)
        if cam is not None:
            self.cond('정지선까지 (카메라)', _mm(cam), 1.0 - cam / 1500.0, cam <= 30.0)
        if r is not None:
            self.cond('정지선까지 (도면)', _mm(r), 1.0 - r / 1500.0, r <= 30.0)
        if cam is None and r is None:
            self.cond('정지선', '카메라·도면 둘 다 없음 (오도메트리로 이어 감)')


def explain(mission: dict, lane: Optional[dict] = None, commands: Optional[dict] = None,
            obstacle: Optional[dict] = None, parking: Optional[dict] = None,
            points: Optional[Dict[str, float]] = None, params: Optional[dict] = None,
            sub_age: Optional[float] = None, cmd: Optional[tuple] = None,
            stale: Optional[Dict[str, float]] = None) -> dict:
    c = _Ctx(mission, lane, commands, obstacle, parking, points, params, sub_age)
    m = c.m
    state, sub = m.get('state') or '-', m.get('sub') or ''
    src = m.get('source')
    v = cmd[0] if cmd else None
    control = f"조향: {SOURCE_KO.get(src, src or '-')}"
    if v is not None:
        control += f' · 속도 {v:.2f} m/s' + (' (후진)' if v < 0 else '')
    if m.get('manual'):
        control = '수동 조종 (teleop) · 아래는 자율주행이었다면 할 판단'

    note = m.get('note') or ''
    if note:
        c.because.append(f'마지막 전환: {note}')
    if 'FALLBACK' in note:
        c.warn.append('표지·선을 못 보고 도면 위치나 주행 거리로 넘어감 (FALLBACK)')
    if m.get('estop'):
        c.warn.append('비상 정지 켜짐')
    if c.route.get('lost'):
        c.warn.append('길 잃음: 도면 선을 연속으로 못 봄 → 도면 위치를 판단에 안 씀')
    lf = c.route.get('last_fix') or {}
    if lf:
        c.because.append(f"위치 보정: '{lf.get('name')}' 선, 오차 {lf.get('err_mm')} mm")
    for k, age in (stale or {}).items():
        if age is not None and age > 1.5:
            c.warn.append(f'{k} 토픽이 {age:.1f}초째 안 옴')
    st = c.lane.get('status')
    if st and st not in ('BOTH',) and src == 'lane':
        c.warn.append(f'차선 {st} (한쪽 또는 없음)' + (' · 신뢰 낮음' if c.lane.get('low_conf') else ''))

    title = _STATES.get(state, _generic)(c, sub)
    return {'state': STATE_KO.get(state, state) + (f' / {sub}' if sub else ''), 'title': title,
            'control': control, 'because': c.because, 'next': c.next, 'warn': c.warn}


# --- 상태별 -----------------------------------------------------------------

def _generic(c, sub):
    return '-'


def _wait(c, sub):
    c.cond('출발 신호 (/mission/start)')
    return '출발 신호를 기다리는 중'


def _lane_only(c, sub):
    return '차선만 따라 주행 (시험용)'


def _finished(c, sub):
    return '완료 — 정지'


def _m5(c, sub):
    if sub == '':
        c.because.append('방지턱 앞 서행 구간을 찾는 중 (일반 속도)')
        c.sign_cond('slow', ' (또는 방지턱 표지)')
        c.route_cond('서행 선', 'm5_slow_line', -c.p['route_slow_before_mm'])
        return '서행 표지를 찾는 중'
    c.because.append('서행 표지 확정 또는 도면 서행 선 도달 → 서행')
    c.cond('IMU로 방지턱 넘음 감지 후 뒷바퀴까지', f"감지 후 {c.p['bump_after_mm']:.0f} mm")
    c.route_cond('지그재그 1.5 m 앞 (예비)', 'm6_zigzag_line', -1500.0)
    return '서행하며 방지턱 통과 중'


def _m6(c, sub):
    if sub == '':
        cam = c.cam_line()
        c.cond('지그재그 시작선을 앞바퀴가 지남 (카메라)', _mm(cam) if cam is not None else '선 안 보임',
               None if cam is None else 1.0 - cam / 1500.0)
        c.route_cond('지그재그 시작선', 'm6_zigzag_line')
        return '지그재그 시작선을 찾는 중'
    c.because.append('시작선 통과 → 지그재그 모드 (차로 폭 570, 서행)')
    c.route_cond('지그재그 끝', 'm6_zigzag_end', span=c.p['zigzag_len_mm'])
    return '지그재그 구간 서행'


def _m3(c, sub):
    if sub == '':
        c.sign_cond('pedestrian')
        c.route_cond('횡단보도 정지선 근처', 'm3_stop_line', -c.p['arm_on_crosswalk_mm'])
        return '보행자 표지를 찾는 중 (서행)'
    if sub == 'approach':
        c.because.append('보행자 표지 확정 또는 횡단보도 근처 → 정지선에 서기')
        c.stopline_cond('m3_stop_line')
        return '정지선으로 천천히 접근'
    if sub == 'stopped':
        w = c.p['ped_wait_sec']
        a = c.sub_age
        c.cond(f'규정 대기 {w:.1f}초', f'{a:.1f}초' if a is not None else '-',
               None if a is None else a / w, a is not None and a >= w)
        return '정지선에서 대기'
    c.because.append('대기 끝 → 출발')
    c.route_cond('횡단보도 끝 지남', 'm3_crosswalk_end', span=c.p['leave_mm'])
    return '횡단보도 통과'


def _m7(c, sub):
    ob = c.ob
    if ob.get('m7_hold'):
        c.warn.append('회피 노드 명령이 끊겨 정지 (m7_hold)')
    if ob.get('m7_active'):
        c.because.append('회피 노드 실행 중 → 회피 노드가 조향')
    else:
        c.because.append('회피 노드 대기 → 시작선까지는 차선 주행')
    trig = ob.get('m7_trigger')
    if trig:
        c.because.append(f'회피 시작 근거: {trig}')
    c.cond('회피 노드가 끝났다고 알림 (m7_done)', '예' if ob.get('m7_done') else '아직', None, bool(ob.get('m7_done')))
    c.route_cond('섬 끝 지남 (안전용 최대)', 'm7_island_tip', c.p['m7_force_after_island_mm'])
    if c.m.get('fork_choice'):
        c.because.append(f"갈림길 표지 미리 읽음: {c.m['fork_choice']}")
    return '3차선 장애물 회피'


def _m2(c, sub):
    ch = c.m.get('fork_choice')
    ex = (c.ob or {}).get('m7_exit') or c.m.get('m7_exit_lane')
    if ch is None:
        c.sign_cond('fork_a')
        c.sign_cond('fork_b')
        a = c.sub_age
        t = c.p['fork_seek_timeout_sec']
        c.cond(f"{t:.0f}초 안에 못 읽으면 기본값 {c.p['fork_default']}",
               f'{a:.1f}초' if a is not None else '-', None if a is None else a / t)
        return '갈림길 표지를 읽는 중 (서행)'
    c.because.append(f'갈림길 {ch} 선택' + (f' (회피가 끝난 차로: {ex})' if ex else ''))
    c.route_cond('합류 끝', 'm2_merge_end')
    return f"갈림길 {ch}: {'왼쪽' if ch == 'A' else '오른쪽'} 경계선만 따라 주행"


def _m2x(c, sub):
    c.because.append('합류 끝 → 차선 인식 모드 (양쪽 노란 선) + 좌하단 코너 도면 곡률 보조')
    _m8(c, '')
    return '갈림길 뒤 차선 주행 (라바콘 시작선까지)'


def _m8(c, sub):
    ob = c.ob
    if sub == '':
        cam = c.cam_line()
        c.cond('라바콘 시작선 지남 (카메라)', _mm(cam) if cam is not None else '선 안 보임')
        c.route_cond('라바콘 시작선', 'm8_start_line', -c.p['cones_enter_before_mm'])
        ahead = ob.get('cone_ahead_mm')
        c.cond('차로 안 콘이 가까움 (라이다)', _mm(ahead) if ahead is not None else '콘 없음',
               None if ahead is None else 1.0 - (ahead - c.p['cones_enter_cone_mm']) / 1500.0,
               ahead is not None and ahead <= c.p['cones_enter_cone_mm'])
        return '라바콘 구간 접근 (차선 서행)'
    if sub == 'cones':
        c.because.append('라바콘 모드 → 장애물 노드가 콘 사이 경로로 조향')
        near = ob.get('cone_any_ahead_mm')
        c.cond(f"앞 {c.p['cones_gone_mm']:.0f} mm 안 콘이 사라짐", _mm(near) if near is not None else '콘 없음',
               None, near is None or near > c.p['cones_gone_mm'])
        c.route_cond('라바콘 끝선 (예비)', 'm8_end_line', c.p['cones_end_after_mm'])
        return '라바콘 사이 주행'
    c.because.append('라바콘 끝 → 차선을 다시 잡을 때까지 장애물 노드가 구간 방향 유지')
    c.cond('차선 다시 잡힘', c.lane.get('status') or '-', None, c.lane.get('status') in ('BOTH', 'L', 'R'))
    return '라바콘 끝 → 차선 주행으로 넘기는 중'


def _m1(c, sub):
    red = c.confirmed('red', 'yellow')
    green = c.confirmed('green')
    if sub == '':
        c.sign_cond('red')
        c.sign_cond('green')
        c.route_cond('신호등 정지선 근처', 'm1_stop_line', -c.p['arm_on_crosswalk_mm'])
        return '신호등을 찾는 중'
    if sub == 'approach':
        c.because.append('빨간불 확정' if red else ('초록불이어도 일단 정지 (stop_on_green_arrival)' if green
                                                  else '정지선 근처 → 일단 정지'))
        c.stopline_cond('m1_stop_line')
        return '신호등 정지선으로 접근'
    if sub == 'stopped':
        c.sign_cond('green', f" 후 {c.p['green_go_delay_sec']:.1f}초")
        if red:
            c.because.append('빨간불 유지 중')
        return '정지선에서 초록불 대기'
    c.because.append('초록불 확정 → 출발')
    c.cond('정지선에서 벗어남', f"{c.p['leave_mm']:.0f} mm 주행 후")
    return '신호등 통과'


def _m4(c, sub):
    pk = c.pk
    if sub == '':
        c.sign_cond('parking')
        c.route_cond('주차 앞 선 근처', 'm4_pre_line', -c.p['m4_arm_before_mm'])
        return '주차 표지를 찾는 중 (서행)'
    c.because.append('주차 노드에 넘김' + (' (주차 표지)' if c.m.get('parking_sign') else ''))
    phase = pk.get('phase')
    if phase:
        c.because.append(f"주차 단계: {phase}" + (f" · {pk.get('row')}" if pk.get('row') else '')
                         + (f" · 정지 근거 {pk.get('stop_how')}" if pk.get('stop_how') else ''))
    c.cond('주차 완료 (parking done)', '예' if pk.get('done') else '아직', None, bool(pk.get('done')))
    return '주차 진행'


_STATES = {'WAIT_START': _wait, 'LANE_ONLY': _lane_only, 'FINISHED': _finished,
           'M5_BUMP': _m5, 'M6_ZIGZAG': _m6, 'M3_PEDESTRIAN': _m3, 'M7_OBSTACLE': _m7,
           'M2_FORK': _m2, 'M2_EXIT': _m2x, 'M8_CONES': _m8, 'M1_TRAFFIC': _m1, 'M4_PARKING': _m4}
