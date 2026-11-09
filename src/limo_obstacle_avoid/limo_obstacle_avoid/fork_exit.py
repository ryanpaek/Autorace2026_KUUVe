"""fork_exit.py — 갈림길(미션 2) 출구 차선 이동 계산 (ROS 없음)

3차선 구간은 가운데 섬 앞 끝(로컬 x = section_length)에서 A/B 두 길로 갈라진다.
  A = 왼쪽 = 1차선 = 차선 번호 +1      B = 오른쪽 = 3차선 = 차선 번호 -1
섬을 지나면 반대 길로 옮길 수 없으므로, 갈림길 방향이 정해지면 섬 앞 끝 전에 출구 차선에 들어가 있어야 한다.
이 구간의 바퀴는 회피 노드가 잡고 있으므로 이동도 회피 계획기(avoid_core)가 한다.

shift_distance(): 차선 n개를 옮기는 데 필요한 앞쪽 거리를 회피 노드와 '같은 제어식'으로 모의해서 구한다.
                  (최소 회전반경·최대 진입각·lookahead 를 바꾸면 자동으로 다시 계산)
"""
import math
from functools import lru_cache

INF = float('inf')


@lru_cache(maxsize=64)
def _simulate(lane_width, n, lookahead, min_r, max_heading_deg, reach_tol, v, clear_y):
    """→ (clear, settle): clear = 차체가 가운데 섬 폭(목표 쪽 경계)을 벗어날 때까지 거리,
                          settle = 목표 차선 중심에 자리 잡을 때까지 거리.
    시작 차선 중심 y=0, 목표 y = n·lane_width. clear_y = 목표 차선 중심에서 이만큼 안쪽까지 오면 '섬 구역 밖'"""
    if n == 0:
        return 0.0, 0.0
    target = n * lane_width
    clear_at = None
    x = y = psi = 0.0
    dt = 0.02
    kmax = 1.0 / min_r
    mh = math.radians(max_heading_deg)
    settle = math.radians(5.0)
    for _ in range(int(20.0 / (v * dt))):              # 최대 20m 까지
        dy = target - y
        xr = lookahead * math.cos(psi) + dy * math.sin(psi)
        yr = -lookahead * math.sin(psi) + dy * math.cos(psi)
        k = 2.0 * yr / (xr * xr + yr * yr)
        k = max(-kmax, min(kmax, k))
        if (psi > mh and k > 0) or (psi < -mh and k < 0):
            k = 0.0
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi += v * k * dt
        if clear_at is None and target - y <= clear_y:
            clear_at = x
        if abs(target - y) < reach_tol and abs(psi) < settle:
            return (clear_at if clear_at is not None else x), x
    return INF, INF


def shift_distance(p, n_lanes):
    """차선 |n_lanes| 개를 옮길 때 → (clear, settle) [m] (여유 미포함).
    clear : 차체(옆 여유 포함)가 가운데 섬 구역(|y| < 0.3·lane_width)을 목표 쪽으로 벗어날 때까지 앞으로 간 거리.
            섬 앞 끝 전에 이것만 끝나면 반대 길로 들어가지 않는다 (나머지 자리 잡기는 출구 길 안에서 해도 됨).
    settle: 목표 차선 중심에 자리 잡을 때까지 거리."""
    island_half = 0.3 * p.lane_width
    # 목표 차선 중심 기준으로, 차체 안쪽 모서리가 섬 경계를 벗어나는 위치까지 남은 옆 거리
    clear_y = p.lane_width - island_half - p.robot_half_width - p.side_margin
    return _simulate(round(p.lane_width, 4), abs(int(n_lanes)), round(p.lookahead, 4),
                     round(p.min_turn_radius, 4), round(p.max_heading_deg, 3),
                     round(p.reach_tol, 4), round(p.v_change, 3), round(max(clear_y, 0.0), 4))


def lane_name(lane):
    """차선 번호 → 사람이 부르는 이름 (+1 = 1차선 = A쪽, 0 = 2차선, -1 = 3차선 = B쪽)"""
    return {1: '1차선(A쪽)', 0: '2차선(가운데)', -1: '3차선(B쪽)'}.get(lane, str(lane))
