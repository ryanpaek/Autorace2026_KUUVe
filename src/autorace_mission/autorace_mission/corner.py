"""코너를 다 돌았나 (IMU 방향만, 도면 위치 안 씀). FSM의 라바콘 진입에 쓴다.

autorace_obstacle/m7_link.py 의 CornerWatch 와 같은 규칙:
시작 방향에서 turn_deg 넘게 돌았고, 최근 window_mm 동안 방향 변화가 straight_deg 안이면 '다 돎' (한 번 되면 유지).
R640 코너는 150 mm 동안 약 13° 돈다 -> 코너 안에서는 '다 돎'이 안 된다.
"""
import math
from collections import deque
from typing import Optional


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class CornerWatch:
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
