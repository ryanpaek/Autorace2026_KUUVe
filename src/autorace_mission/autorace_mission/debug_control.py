"""디버그 화면 버튼 -> 미션 노드 명령 (ROS 의존성 없음, 11/11).

debug_view의 POST /control 이 이 함수로 요청을 확인하고, 돌려받은 (토픽, 값)들을 발행한다.

  action        보내는 것                          뜻
  start         /mission/start   (Empty)           출발 (WAIT_START에서)
  set_state     /mission/set_state (String)        그 미션 '시작 위치'에서 시작: 경로 위치 s도 그 미션 시작점으로
                                                   (차를 실행 가이드 3번 표 위치에 놓았을 때)
  skip          /mission/skip    (String)          지금 자리에서 그 미션으로: s는 그대로 (주행 중 건너뛰기)
  next          /mission/skip    (String)          지금 미션의 다음 미션으로 건너뛰기
  estop         /mission/estop   (Bool true)       비상 정지. 버튼 끄기·비밀번호와 상관없이 항상 받는다
  estop_off     /mission/estop   (Bool false)      비상 정지 해제

control_enabled=false면 비상 정지 말고는 다 거절 (대회 때). control_password를 주면 그 값이 맞아야 한다.
"""

from typing import List, Optional, Tuple

from autorace_mission.fsm import EXTRA_STATES, SEQUENCE

# 화면 목록에 보일 미션 (출발 대기·완료 제외)
CHOOSABLE = [s for s in SEQUENCE if s not in ('WAIT_START', 'FINISHED')] + list(EXTRA_STATES)

Publish = Tuple[str, object]     # (토픽, 값) 값: str | bool | None(Empty)


def next_state(cur: Optional[str]) -> Optional[str]:
    if cur not in SEQUENCE:
        return None
    i = SEQUENCE.index(cur)
    return SEQUENCE[i + 1] if i + 1 < len(SEQUENCE) else None


def control_request(body: dict, enabled: bool, password: str,
                    cur_state: Optional[str]) -> Tuple[bool, str, List[Publish]]:
    """(받음?, 화면에 보일 글, 발행할 것들)."""
    if not isinstance(body, dict):
        return False, '잘못된 요청', []
    act = body.get('action')
    if act == 'estop':
        return True, '비상 정지', [('/mission/estop', True)]
    if not enabled:
        return False, '디버그 화면 조작이 꺼져 있음 (debug_control:=false). 비상 정지만 가능', []
    if password and body.get('password') != password:
        return False, '비밀번호가 틀림', []
    if act == 'estop_off':
        return True, '비상 정지 해제', [('/mission/estop', False)]
    if act == 'start':
        return True, '출발', [('/mission/start', None)]
    if act == 'next':
        nxt = next_state(cur_state)
        if nxt is None:
            return False, f'{cur_state or "?"} 다음 미션 없음', []
        return True, f'{cur_state} -> {nxt} 건너뜀 (위치 그대로)', [('/mission/skip', nxt)]
    if act in ('set_state', 'skip'):
        st = body.get('state')
        if st not in CHOOSABLE and st not in SEQUENCE:
            return False, f'모르는 미션: {st}', []
        if act == 'set_state':
            return True, f'{st} 시작 위치에서 시작', [('/mission/set_state', st)]
        return True, f'{st}로 건너뜀 (위치 그대로)', [('/mission/skip', st)]
    return False, f'모르는 동작: {act}', []
