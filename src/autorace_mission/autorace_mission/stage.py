"""/mission/stage: 지금 몇 번 미션인지 (11/15). 확실하지 않으면 null.

확실 = 미션 FSM 상태와 도면 위치(경로 s)가 가리키는 구간이 같을 때.
  도면 위치가 없거나(경로 지도 끔·길 잃음) 둘이 다르면 mission은 null (fsm_mission·map_mission에 각각 남김).
  구간 경계는 FSM이 실제로 넘어가는 곳과 조금 다르므로 경계 앞뒤 boundary_tol_mm 안에서는 이웃 구간도 같다고 본다.
"""
import re
from typing import Dict, Optional

# FSM 순서와 도면에서 그 구간이 시작하는 지점 (track_route.yaml 이름)
ZONES = [
    ('M5_BUMP', None),
    ('M6_ZIGZAG', 'm6_zigzag_line'),
    ('M3_PEDESTRIAN', 'm6_zigzag_end'),
    ('M7_OBSTACLE', 'm3_crosswalk_end'),
    ('M2_FORK', 'm7_island_tip'),
    ('M2_EXIT', 'm2_island_end'),
    ('M8_CONES', 'm8_start_line'),
    ('M1_TRAFFIC', 'm8_end_line'),
    ('M4_PARKING', 'm1_crosswalk_end'),
]
ORDER = [z for z, _ in ZONES]


def mission_no(state: Optional[str]) -> Optional[int]:
    """'M7_OBSTACLE' -> 7. 미션이 아니면(WAIT_START, FINISHED) None"""
    m = re.match(r'^M(\d)_', state or '')
    return int(m.group(1)) if m else None


def map_zone(s: Optional[float], points: Dict[str, float]) -> Optional[str]:
    """경로 s가 속한 구간 이름 (도면 위치가 없으면 None)"""
    if s is None:
        return None
    zone = None
    for name, start in ZONES:
        s0 = 0.0 if start is None else points.get(start)
        if s0 is not None and s >= s0:
            zone = name
    return zone


def _near_boundary(s: float, a: str, b: str, points: Dict[str, float], tol: float) -> bool:
    """구간 a, b가 붙어 있고 s가 그 경계에서 tol 안"""
    ia, ib = ORDER.index(a), ORDER.index(b)
    if abs(ia - ib) != 1:
        return False
    start = ZONES[max(ia, ib)][1]
    s0 = 0.0 if start is None else points.get(start)
    return s0 is not None and abs(s - s0) <= tol


def stage_info(state: Optional[str], s: Optional[float], points: Dict[str, float],
               boundary_tol_mm: float = 800.0) -> dict:
    fsm_no = mission_no(state)
    zone = map_zone(s, points)
    agree = False
    why = ''
    if fsm_no is None:
        why = '미션 중 아님'
    elif zone is None:
        why = '도면 위치 없음 (경로 지도 끔·길 잃음)'
    elif state == zone:
        agree = True
    elif state in ORDER and _near_boundary(s, state, zone, points, boundary_tol_mm):
        agree = True
        why = '구간 경계 근처'
    else:
        why = f'미션은 {state}, 도면 위치는 {zone}'
    return {
        'mission': fsm_no if agree else None,         # 확실할 때만 번호 (M7 -> 7), 아니면 null
        'order': (ORDER.index(state) + 1) if (agree and state in ORDER) else None,   # 몇 번째 미션 (1~9)
        'state': state,
        'fsm_mission': fsm_no,                        # 미션 FSM이 생각하는 번호 (늘 있음)
        'map_mission': mission_no(zone),              # 도면 위치가 가리키는 번호
        's_mm': None if s is None else round(s),
        'sure': agree,
        'why': why,
    }
