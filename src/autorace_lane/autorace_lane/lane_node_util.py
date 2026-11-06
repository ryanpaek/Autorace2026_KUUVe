"""ROS 없이도 쓰는 작은 도구 (리플레이·평가용)."""

import math


def yaw_of(q) -> float:
    """쿼터니언 -> yaw (rad, 반시계 +)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
