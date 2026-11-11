"""fork_core.py — 미션 2 갈림길(A/B) 판단 (ROS 없음, 시험 가능)

규정: A = 왼쪽(좌측 진입), B = 오른쪽(우측 진입). 3차선 구간 끝 가운데 섬에서 두 길로 갈라진다.
차선 번호는 회피 패키지와 같게 (진행 방향 기준)
    +1 = 1차선 = 왼쪽 = A 쪽      0 = 2차선(가운데, 섬이 막음)      -1 = 3차선 = 오른쪽 = B 쪽

하는 일
  1) YOLO 결과(fork_a / fork_b)를 최근 N 프레임 다수결로 A/B 결정 (FSM과 같은 방식: 8프레임 중 5)
  2) 회피 노드 상태(/avoid/state)의 '지금 차선'으로 몇 차선을 옮겨야 하는지 계산
       예) 1차선(+1)에서 A → 0차선 이동(유지), B → 2차선 이동
  3) 회피 노드가 계산한 '섬까지 남은 거리 / 필요한 거리'로 가능 여부를 사람이 읽을 수 있게 정리
실제 차선 이동은 회피 노드(limo_obstacle_avoid)가 한다 (그 구간 바퀴를 회피 노드가 잡고 있으므로).
"""
from __future__ import annotations

import json
from collections import deque
from typing import Iterable, Optional, Set

LANE_OF = {'A': 1, 'B': -1}
LANE_NAME = {1: '1차선(A쪽)', 0: '2차선(가운데)', -1: '3차선(B쪽)'}


def lane_name(lane) -> str:
    return LANE_NAME.get(lane, str(lane))


def parse_detections(data, cls_a='fork_a', cls_b='fork_b', conf=0.5, min_box_h=0.0) -> Set[str]:
    """detector_node 의 /perception/detections JSON → 이번 프레임에 본 방향 집합 {'A','B'}
    형식: {"detections":[{"cls":"fork_a","conf":0.8,"bbox":[x,y,w,h]}, ...]}  (detector_node·yolo_onnx와 같은 [x, y, 폭, 높이] px)
    min_box_h: 표지 상자 높이(픽셀)가 이보다 작으면 무시 (멀리 있는 표지 오검출 방지, 0이면 안 씀)"""
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except json.JSONDecodeError:
            return set()
    seen = set()
    for d in (data or {}).get('detections', []) or []:
        try:
            c, cf = d.get('cls'), float(d.get('conf', 0.0))
        except (AttributeError, TypeError, ValueError):
            continue
        if cf < conf:
            continue
        if min_box_h > 0:
            b = d.get('bbox') or []
            if len(b) == 4 and b[3] < min_box_h:       # 11/11: 예전 [x1,y1,x2,y2]로 보고 b[3]-b[1] -> 높이가 작게 나옴
                continue
        if c == cls_a:
            seen.add('A')
        elif c == cls_b:
            seen.add('B')
    return seen


def parse_sign_text(text: str) -> Set[str]:
    """간단한 문자열 토픽용: 'A' / 'B' / 'fork_a' / 'fork_b' / '좌' / '우' (대소문자 무관)"""
    t = (text or '').strip().lower()
    if t in ('a', 'fork_a', 'left', 'l', '좌', '왼쪽'):
        return {'A'}
    if t in ('b', 'fork_b', 'right', 'r', '우', '오른쪽'):
        return {'B'}
    return set()


class ForkVote:
    """최근 window 프레임 중 한쪽이 need 번 이상 보이고 다른 쪽은 need 미만이면 결정"""

    def __init__(self, window: int = 8, need: int = 5):
        self.a = deque(maxlen=window)
        self.b = deque(maxlen=window)
        self.need = need

    def push(self, seen: Iterable[str]) -> Optional[str]:
        seen = set(seen)
        self.a.append('A' in seen)
        self.b.append('B' in seen)
        return self.result()

    def counts(self):
        return sum(self.a), sum(self.b)

    def result(self) -> Optional[str]:
        na, nb = self.counts()
        if na >= self.need and nb < self.need:
            return 'A'
        if nb >= self.need and na < self.need:
            return 'B'
        return None

    def reset(self):
        self.a.clear()
        self.b.clear()


def plan(decision: Optional[str], avoid: dict) -> dict:
    """결정(A/B)과 회피 노드 상태 → 지금 상황 정리 (/fork/state 로 내보냄)
    avoid: /avoid/state  (now_lane = 지금 위치한 차선, target = 회피 노드가 가려는 차선)"""
    avoid = avoid or {}
    want = LANE_OF.get(decision)
    now, tgt = avoid.get('now_lane'), avoid.get('target')
    out = {'decision': decision, 'want_lane': want, 'lane': now, 'lane_name': lane_name(now) if now is not None else None,
           'target': tgt, 'avoid_state': avoid.get('state'), 'exit_plan': avoid.get('exit_plan'),
           'rem_m': avoid.get('exit_rem_m'), 'need_m': avoid.get('exit_need_m'), 'need_lanes': None}
    if decision is None:
        out['text'] = '표지 판단 전'
        return out
    if now is None:
        out['text'] = f'{decision} → {lane_name(want)} (회피 노드 상태 모름, 방향만 전달)'
        return out
    n = want - int(now)
    out['need_lanes'] = n
    if n == 0:
        out['status'] = '유지'
        out['text'] = f'{decision}: 지금 {lane_name(now)} → 차선 유지'
        return out
    move = f'{abs(n)}차선 {"왼쪽" if n > 0 else "오른쪽"}으로'
    rem, need = out['rem_m'], out['need_m']
    detail = f' (섬까지 {rem:.2f}m / 필요 {need:.2f}m)' if rem is not None and need is not None else ''
    if tgt == want:
        status = '이동 중'
    else:
        status = {'go': '이동 시작', 'wait': '장애물 때문에 대기', 'too_late': '거리 부족 → 지금 쪽 유지',
                  'given_up': '섬 앞까지 옮길 수 없음 → 지금 쪽 유지'}.get(out['exit_plan'], '판단 중')
    out['status'] = status
    out['text'] = f'{decision}: 지금 {lane_name(now)} → {lane_name(want)}, {move}{detail} — {status}'
    return out
