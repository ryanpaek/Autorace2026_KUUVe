"""T자 주차 노드 (M4) — 두 리모 중점 + 재정렬. 설계: 윤서(캡틴) 2026-11-06, 설명: docs/CHANGES_20261109_parking.md

하는 일
  1) 흰 선(주차 앞 선)을 볼 때까지 차선 노드 명령(/lane/cmd_vel)을 그대로 넘긴다.
     (mission_node는 신호등을 지나자마자 M4_PARKING이 되므로 fsm.py _s_M1_TRAFFIC)
  2) 흰 선을 보면 차로 중심에서 왼쪽으로 target_offset_mm(왼쪽 차선과 약 8cm) 비켜 차선을 따라간다.
  3) 라이다로 오른쪽 정차 리모 두 대(1·3 또는 4·6)를 같이 찾는다. 두 대 사이가 칸 2개(약 1040mm)이고 옆 거리도
     비슷해야 "짝"으로 인정한다. 짝의 중점 = 목표 칸(2 또는 5) 가운데.
  4) 중점이 앞바퀴보다 turn_lead_mm(320) 앞에 오면 1초 정지 -> 오른쪽 최대 조향 -> 87도 돌면 바퀴를 편다.
     (뒷바퀴축이 칸 가운데에서 회전반경 530mm 앞일 때 꺾으면 90도 돈 뒤 칸 가운데: 530 - 축거 210 = 320)
  5) 꺾은 뒤 양옆 리모의 옆면으로 칸 가운데 선을 다시 구하고, 차체 중심이 칸 가운데에 오면 멈춘다.
     (짝은 꺾는 시점과 방향에만 쓴다. 차선에서 보면 리모의 끝면만 보여서 깊이가 8~17cm 얕게 나온다)
  6) CHECK: 네 모서리 중 하나라도 칸 선(옆·앞뒤)에 corner_margin_mm보다 가까우면 재정렬 —
     바퀴 똑바로 pullout_mm 후진(PULLOUT) -> 칸 가운데 선을 pure pursuit로 따라 다시 들어감(ENTER). 최대 max_realign번.
     꺾은 뒤 앞이 막혀 갇혀도(바퀴 편 뒤 막힘) 짝으로 기억한 선으로 재정렬한다.
  7) 끝나면 /parking/state에 done=true -> fsm.py _s_M4_PARKING이 다음으로 넘어간다.
  * 짝을 못 찾으면 예비로 "첫 리모가 앞바퀴 옆 + extra_mm"에서 꺾는다.

가정
  - 차로 안쪽 폭 480mm(선 중심 간 500 - 선 20), 칸 500 x 600, 흰 선 -> 주차 구역 440mm (2026Track 도면)
  - LIMO 322 x 220mm, 축거 0.21m, 최소 회전반경 약 0.53m (calib/README.md)
  - /lane/status의 stoplines_mm = 앞바퀴 -> 흰 선 가까운 가장자리 (lane_core)
  - 정차 리모는 칸 가운데에 반듯하게 서 있다 (깊이 판단의 기준)
검증 한계
  - ROS·실차 실행 안 함 (미검증). 가상 트랙 시뮬레이션과 단위 시험(test/test_parking_node.py)만 함.
  - 라이다 위치(front_axle_from_lidar_mm, laser_yaw)와 차체 치수는 실측 확인 필요 (tools/parking_scan_check.py).
  - 이 노드도 /lane/config를 보낸다. mission_node도 보내지만 상태가 바뀔 때만 보내서 주차 중에는 안 겹친다.
  - 라이다가 첫 리모(1/4번)를 놓치면 짝이 안 생겨 예비 방식이 약 1m 늦게 꺾는다 (시뮬레이션 0점).

흐름도
① WAIT_LINE: 차선 노드 명령을 넘기며 기다림 (armed = 주차 표지 봤거나 도면상 근처)
│   ├─ skip_wait_line (11/12, 미션 노드 m4_direct와 같이): M4 들어오자마자 → FOLLOW (trigger=direct)
│   └─ armed이고 (흰 선 보임 또는 1·3번 짝 연속 3번) → 주차 모드
│
├─② FOLLOW: 차선 따라가기, 왼쪽 차선과 8cm (target_offset_mm 50)
│   ├─ 짝 찾기 전 앞이 막힘 → 서서 기다림 (wait_blocked_before_pair)
│   ├─③ 라이다: 오른쪽 리모 두 대 짝 찾기 (사이 약 104cm) → 중점 = 목표 칸 가운데
│   │   └─ 중점이 앞바퀴보다 32cm 앞에 오면 → PAUSE
│   └─ (예비, 기본 끔 single_fallback) 짝 못 찾고 리모만 한 물체가 앞바퀴 옆 → EXTRA: 0.2m 더 → PAUSE
│      (11/12: 켜 두면 주차 표지판에 속아 일찍 꺾음 → 짝만 기다리고, 2.5m 가도 없으면 FAIL_NO_CAR)
│
├─③ PAUSE: 1초 정지
│
├─④ TURN: 오른쪽 최대 조향 전진  ←─────────────┐
│   ├─ 앞에 리모 있음 → ④-1 BACK: 5cm 후진 ─────┤
│   │   (바퀴 편 뒤 막힘 + 칸 가운데 선 앎 → 재정렬 PULLOUT으로)
│   ├─ (추가) 87도 돌았으면 → 바퀴 펴고 직진 ───┤
│   └─ ⑤ 짝을 알면: 양옆 옆면 기준 차체 중심이 칸 가운데? / 모르면: 양옆이 앞바퀴 옆? ──┘ 아니면 반복
│
├─(재정렬) CHECK: 양옆 리모로 칸 가운데 선 계산 → 네 모서리 옆 거리·방향 오차
│   ├─ 네 모서리가 칸 안 (옆·깊이 여유 2.5cm) → ⑥
│   └─ 벗어남 → PULLOUT: 바퀴 똑바로 후진 pullout_mm
│              → ENTER: 선 따라 전진, 차체 중심이 칸 가운데 오면 정지 → 다시 CHECK
│
└─⑥ DONE: 정지, 차선 오프셋 0으로 되돌림, /parking/state done=true
"""

import json
import math
import time

KAPPA_FULL_RIGHT = -3.0   # 1/m. 차가 낼 수 있는 한계(약 -1.89)보다 크게 줘서 브리지가 최대 조향(0.40rad)으로 자르게 함


# ---------------------------------------------------------------- ROS 없는 계산 부분
def wrap(a):
    """각도를 -pi ~ pi로 맞춘다."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    """쿼터니언 -> 평면 방향 yaw(rad). 팀 코드와 같은 표준 식."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_clusters(ranges, angle_min, angle_inc, rng_min, rng_max,
                  front_axle_from_lidar_mm=-47.0, laser_yaw=0.0,
                  gap_mm=80.0, min_pts=3, max_size_mm=450.0, max_dist_mm=1500.0, x_mode='mean',
                  min_size_mm=0.0):
    """라이다 스캔 -> 물체 덩어리 중심 목록 [(x, y)] (앞바퀴 기준, 앞 +, 왼쪽 +, mm).

    가까운 점끼리 묶고(gap_mm), LIMO 크기(max_size_mm)보다 큰 덩어리(벽 등)는 버린다.
    min_size_mm: 덩어리의 가로·세로 중 긴 쪽이 이보다 작으면 버린다 (표지판 기둥·사람 다리·라바콘 등.
    리모 차선 쪽 면 폭 220mm. 시야 끝에 반쯤 걸린 리모도 버려져 짝 위치가 쏠리지 않는다)
    """
    pts = []
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and rng_min <= r <= rng_max):
            continue
        a = angle_min + angle_inc * i + laser_yaw
        x = r * 1000.0 * math.cos(a) - front_axle_from_lidar_mm
        y = r * 1000.0 * math.sin(a)
        if math.hypot(x, y) <= max_dist_mm:
            pts.append((x, y))
    clusters, cur = [], []
    for p in pts:
        if cur and math.dist(p, cur[-1]) > gap_mm:
            clusters.append(cur)
            cur = []
        cur.append(p)
    if cur:
        if clusters and math.dist(cur[-1], clusters[0][0]) <= gap_mm:   # 스캔 끝과 처음이 이어지면 합침
            clusters[0] = cur + clusters[0]
        else:
            clusters.append(cur)
    out = []
    for c in clusters:
        if len(c) < min_pts:
            continue
        xs, ys = [p[0] for p in c], [p[1] for p in c]
        size = max(max(xs) - min(xs), max(ys) - min(ys))
        if size > max_size_mm or size < min_size_mm:
            continue
        # x_mode 'extent': 앞뒤(x)는 점들의 양 끝 가운데. 차선에서 리모를 보면 차선 쪽 끝면(폭 전체) + 한쪽 옆면이
        # 보여 평균이 우리 차 쪽으로 쏠린다 -> 짝의 앞뒤 위치는 양 끝 가운데가 정확 (11/10 시뮬레이션 약 6cm)
        x = (min(xs) + max(xs)) / 2.0 if x_mode in ('extent', 'box') else sum(xs) / len(xs)
        y = (min(ys) + max(ys)) / 2.0 if x_mode == 'box' else sum(ys) / len(ys)   # 'box': 5번 앞면 (정면) 가운데
        out.append((x, y))
    return out


def scan_points(ranges, angle_min, angle_inc, rng_min, rng_max, front_axle_from_lidar_mm=-47.0,
                laser_yaw=0.0, max_dist_mm=700.0):
    """라이다 스캔 -> 가까운 점 목록 [(x, y)] (앞바퀴 기준, 앞 +, 왼쪽 +, mm). 옆 감시용."""
    pts = []
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and rng_min <= r <= rng_max) or r * 1000.0 > max_dist_mm:
            continue
        a = angle_min + angle_inc * i + laser_yaw
        pts.append((r * 1000.0 * math.cos(a) - front_axle_from_lidar_mm, r * 1000.0 * math.sin(a)))
    return pts


def front_blocked(ranges, angle_min, angle_inc, rng_min, rng_max, front_axle_from_lidar_mm,
                  laser_yaw, front_bumper_from_axle_mm, front_stop_mm, half_width_mm):
    """앞범퍼 앞 front_stop_mm, 차 폭 안에 점이 있으면 True."""
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and rng_min <= r <= rng_max):
            continue
        a = angle_min + angle_inc * i + laser_yaw
        x = r * 1000.0 * math.cos(a) - front_axle_from_lidar_mm - front_bumper_from_axle_mm
        y = r * 1000.0 * math.sin(a)
        if 0.0 < x <= front_stop_mm and abs(y) <= half_width_mm:
            return True
    return False


def neighbors_abeam(clusters, tol_mm=30.0, side_min_mm=80.0, side_max_mm=700.0, passed_ok=True):
    """양옆 리모가 앞바퀴 옆에 있나 -> (오른쪽, 왼쪽).

    옆 = 덩어리 중심의 앞뒤 위치가 앞바퀴에서 tol_mm 안. passed_ok면 이미 지나친 것(뒤쪽)도 인정.
    """
    r = l = False
    for x, y in clusters:
        if not (side_min_mm <= abs(y) <= side_max_mm):
            continue
        ok = (x <= tol_mm) if passed_ok else (abs(x) <= tol_mm)
        if ok and x > -400.0:          # 너무 뒤(차 뒤쪽 바깥)는 다른 물체로 봄
            if y < 0:
                r = True
            else:
                l = True
    return r, l


def in_view(clusters, front_axle_from_lidar_mm=-47.0, max_angle_deg=180.0):
    """라이다에서 본 각도가 max_angle_deg 안인 덩어리만. 시야 끝에 걸린 리모는 일부만 보여 중심이 앞으로 쏠린다."""
    lx = -front_axle_from_lidar_mm                  # 앞바퀴 기준 라이다 위치 (앞 +)
    lim = math.radians(max_angle_deg)
    return [(x, y) for x, y in clusters if abs(math.atan2(y, x - lx)) <= lim]


def cell_line_from_neighbors(clusters, side_max_mm=700.0, ahead_max_mm=150.0, row_tol_mm=250.0):
    """양옆 리모 중심(앞바퀴 기준 좌표)으로 칸 가운데 선을 구한다.

    반환: (중점 x, 중점 y, 방향 단위벡터 ux, uy) — 차 기준 좌표, 방향은 차 앞쪽과 같은 쪽.
    양옆 중 하나라도 없으면 None. 리모가 둘 다 같은 쪽 얼굴만 보여서 생기는 치우침은 서로 상쇄된다.
    앞바퀴보다 ahead_max_mm 넘게 앞에 있는 리모는 쓰지 않는다: 2번 칸에 들어갈 때 대각선 앞의 4·6번을
    1·3번 대신 고르면 칸 가운데를 뒷줄로 잡아 5번 리모까지 들어간다 (11/10 다섯 대 배치 시뮬레이션).
    좌우 두 대는 같은 줄(앞뒤 차이 row_tol_mm 안)이어야 하고, 그중 옆으로 가장 가까운 짝을 고른다.
    """
    rights = [(x, y) for x, y in clusters if -side_max_mm <= y <= -80.0 and -400.0 < x <= ahead_max_mm]
    lefts = [(x, y) for x, y in clusters if 80.0 <= y <= side_max_mm and -400.0 < x <= ahead_max_mm]
    best = None
    for r in rights:
        for l in lefts:
            if abs(r[0] - l[0]) > row_tol_mm:
                continue
            key = abs(r[1]) + abs(l[1])
            if best is None or key < best[0]:
                best = (key, r, l)
    if best is None:
        return None
    _, r, l = best
    mx, my = (r[0] + l[0]) / 2.0, (r[1] + l[1]) / 2.0
    vx, vy = l[0] - r[0], l[1] - r[1]          # 오른쪽 리모 -> 왼쪽 리모
    n = math.hypot(vx, vy)
    if n < 1e-6:
        return None
    ux, uy = -vy / n, vx / n                    # 그 선에 수직 (차 앞쪽 방향이 되게)
    if ux < 0:
        ux, uy = -ux, -uy
    return mx, my, ux, uy


def corridor_mid(points, x_lo, x_hi, y_min_mm=40.0, gap_min_mm=650.0, gap_max_mm=1000.0):
    """양옆 리모 사이 빈 통로의 가운데 (11/13 캡틴: '양쪽 물체의 정중앙').

    points: 라이다 점 (앞바퀴 기준, 앞 +, 왼쪽 +, mm). 앞 x_lo~x_hi 띠에서 왼쪽 물체의 가장 안쪽 면(y 최소)과
    오른쪽 물체의 가장 안쪽 면(y 최대)을 찾아 그 가운데 (띠 가운데 x, 가운데 y)를 돌려준다.
    리모는 칸 가운데에 서 있으니 두 안쪽 면의 가운데 = 칸 가운데 선. 통로 폭이 gap_min~gap_max(1·3번 사이 약 820)가
    아니면 None (한쪽만 보임, 5번 앞면이 끼어듦 등).
    """
    left = [y for x, y in points if x_lo <= x <= x_hi and y >= y_min_mm]
    right = [y for x, y in points if x_lo <= x <= x_hi and y <= -y_min_mm]
    if len(left) < 2 or len(right) < 2:
        return None
    yl, yr = min(left), max(right)
    if not gap_min_mm <= yl - yr <= gap_max_mm:
        return None
    return (x_lo + x_hi) / 2.0, (yl + yr) / 2.0


def pursuit_kappa_on_line(px, py, yaw, line, look_mm, reverse=False):
    """뒷바퀴축 (px, py, yaw)에서 직선 line=(cx, cy, ux, uy)를 따라가는 pure pursuit 곡률 (1/m, 왼쪽 +).

    reverse=True면 선을 따라 뒤로 간다. 후진은 같은 조향에서 도는 방향이 반대라 부호를 뒤집는다.
    """
    cx, cy, ux, uy = line
    s = (px - cx) * ux + (py - cy) * uy            # 선 위로 내린 발의 위치
    sgn = -1.0 if reverse else 1.0
    tx, ty = cx + (s + sgn * look_mm) * ux, cy + (s + sgn * look_mm) * uy
    dx, dy = tx - px, ty - py
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy   # 차 기준 (앞 +, 왼 +)
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if reverse:
        lx, ly = -lx, -ly                           # 뒤를 앞으로 보는 좌표
    d2 = lx * lx + ly * ly
    if d2 < 1.0:
        return 0.0
    k = 2.0 * ly / d2 * 1000.0                      # 1/mm -> 1/m
    return -k if reverse else k


def find_pair(clusters, side_far_mm, pair_dist_mm, pair_tol_mm, row_tol_mm=200.0):
    """오른쪽 리모 두 대(같은 줄, 칸 2개 간격) 짝 찾기.

    clusters: 앞바퀴 기준 (앞 +, 왼 +). 반환: (중점 x, 중점 y, 앞뒤 방향 단위벡터 vx, vy) 또는 None.
    옆 거리가 가장 가까운 짝(앞줄 우선)을 고른다.
    """
    right = [(x, y) for x, y in clusters if -side_far_mm <= y <= -80.0]
    best = None
    for i in range(len(right)):
        for j in range(i + 1, len(right)):
            a, b = right[i], right[j]
            if abs(a[1] - b[1]) > row_tol_mm:
                continue
            d = math.dist(a, b)
            if abs(d - pair_dist_mm) > pair_tol_mm:
                continue
            near, far = (a, b) if a[0] < b[0] else (b, a)
            key = abs(a[1] + b[1]) / 2.0
            if best is None or key < best[0]:
                best = (key, near, far, d)
    if best is None:
        return None
    _, near, far, d = best
    return ((near[0] + far[0]) / 2.0, (near[1] + far[1]) / 2.0, (far[0] - near[0]) / d, (far[1] - near[1]) / d)


class ParkingV2:
    """버전 2 + 재정렬 상태 기계. step()을 20Hz로 부르면 (속도 m/s, 곡률 1/m 또는 None, 상태 dict)."""

    def __init__(self, p):
        self.p = p
        self.phase = 'WAIT_LINE'
        self.t_phase = self.d_phase = self.yaw_phase = None
        self.yaw_turn0 = None
        self.turn_travel = 0.0
        # 11/15 도는 중 바로잡기: 실제 회전반경을 재서 이대로 가면 칸 가운데에서 얼마나 벗어날지 예측 -> 끊어 넣기
        self.r_ref = None          # (turn_travel, 돈 각도 rad) 재기 시작점
        self.r_meas = None         # 잰 회전반경 (뒷바퀴 축, mm). 한 번 바로잡은 뒤로는 고정
        self.fix_n = 0             # 도는 중 바로잡은 횟수
        self.fix_phi = None        # 마지막으로 바로잡은 각도 (도)
        self.fix_pred = None       # 마지막 예측 옆 오차 (mm, + = 차로 진행 방향으로 지나침 = 칸에서 차 왼쪽)
        self.straight_left = 0.0   # 덜 돌 것 같을 때 바퀴 펴고 더 갈 거리 (mm)
        self.back_k = None         # 이번 BACK 곡률 (None이면 back_kappa)
        self.center_pull_len = None  # 이번 가운데 다시 맞추기 후진 거리
        self.backs = 0
        self.realigns = 0
        self.seen_ahead = False
        self.result = ''
        self.lane_cfg = None
        # 재정렬용 자기 위치 (오도메트리 거리 + IMU 방향, 주차 시작점 기준 mm). 뒷바퀴축 기준
        self.px = self.py = 0.0
        self._last_dist = None
        self._last_v = 0.0
        self.line = None           # 칸 가운데 선 (자기 위치 좌표)
        self.pair_line = None      # 두 리모 짝으로 구한 칸 가운데 선 (자기 위치 좌표)
        self.col5 = None           # 5번 리모 앞면 가운데 (자기 위치 좌표, 평균) -> 2번 칸 세로 가운데선
        self.col5_n = 0
        self.pair_n = 0            # 짝을 본 횟수 (평균 낼 때 씀)
        self.trig_n = 0            # WAIT_LINE에서 짝을 연속으로 본 횟수
        self.corner_s = None       # 1번 상자 '2번 칸 쪽 + 차로 쪽' 모서리: 칸 가운데 선에서 차로 방향으로 몇 mm (보통 약 -410)
        self.corner_n = 0
        self.corner_hist = []
        self.tgt_c5 = []           # 5번 중심 (차로 방향 s, 가까이서 잰 값)
        self.tgt_pm = []           # 1·3번 중점 (가까이서 잰 값)
        self.target_s = None       # 목표 = 1·3번 중심선 ∩ 5번 중심선 (칸 가운데 선 기준 s)
        self.target_src = None
        self.settle_n0 = (0, 0)     # path: 정지 시작 때 잰 개수 (정지 중 새로 잰 값만 쓰려고)
        self.q_s = None            # path: 꺾기 시작점 Q (뒷바퀴 축, 칸 가운데 선 기준 s)
        self.done_logged = False
        self.turn_at = None        # 꺾을 때 (모서리 s, 라이다 s, 잰 횟수) — 실차 확인용
        self.turn_ref_used = None  # 꺾은 기준: 'corner' | 'pair'
        self.trigger = None        # 주차 모드 시작 이유: 'line' | 'pair'
        self.pair_side_mm = None   # 짝 중점까지 옆 거리 (앞줄 약 0.5m, 뒷줄 약 1.1m) -> row
        self.row = None            # 'front' | 'back' (정차 리모가 있는 줄 = 들어갈 줄)
        self.side_seen = False
        self.side_line = None      # 양옆 옆면으로 구한 선 (깊이에만 씀)     # 꺾은 뒤 양옆 리모로 칸 가운데 선을 한 번이라도 구했나
        self.end_d = self.end_at = None   # 칸 끝 선(카메라): 마지막으로 본 거리, 그때 누적 거리
        self.line_seq0 = None             # 11/20 fixed: F_LINE 들어올 때 차선 메시지 번호
        self.stop_how = None       # 'side' | 'memory' | 'camera' | 'cap' (어떻게 멈췄나, 디버그)
        self.err = None            # 마지막 재정렬 판단 값 (옆 mm, 방향 도)
        # 11/13 실차: 양옆 리모 정중앙 맞추기 + 앞뒤 바꿀 때 멈춤 + 옆 감시에 걸리면 끝내지 말고 후진 뒤 다시
        self.switch_to = None      # SWITCH(멈춤) 다음 단계
        self.back_len = None       # 이번 BACK 거리 (앞 막힘·옆 감시 = back_mm, 가운데 다시 맞추기 = center_back_mm)
        self.guard_backs = 0       # 옆 감시로 후진한 횟수
        self.center_tries = 0      # 끝에서 가운데 다시 맞춘 횟수
        self.center_off = None     # 마지막으로 잰 양옆 정중앙에서 차체 중심 옆 거리 (mm, 왼쪽 +)
        self.last_fwd_k = KAPPA_FULL_RIGHT  # 직전 전진 조향 곡률 (기억후진 시 반대 방향 10% 조향용)

    def _enter(self, phase, inp):
        """단계 바꾸기: 시작 시각·거리·방향 기록."""
        self.phase, self.t_phase, self.d_phase, self.yaw_phase = phase, inp['t'], inp['dist_mm'], inp['yaw']

    def _switch(self, nxt, inp, back_len=None):
        """전진 <-> 후진을 바꿀 때: switch_pause_sec 멈춘 뒤 nxt (11/13 실차: 바로 바꾸면 차가 출렁이고 거리가 틀어짐)."""
        if nxt == 'BACK':
            self.back_len = self.p['back_mm'] if back_len is None else back_len
        if self.p.get('switch_pause_sec', 0.0) <= 0.0:
            self._enter(nxt, inp)
        else:
            self.switch_to = nxt
            self._enter('SWITCH', inp)
        return self._out(0.0, 0.0)

    def _centered_line(self):
        """옆 위치·방향 = 양옆 리모 정중앙 선, 깊이 기준점 = 칸 추정선 (정중앙 선 위로 내린 점)."""
        cell = self._cell()
        if self.side_line is None or cell is None:
            return cell
        px, py, ux, uy = self.side_line
        t = (cell[0] - px) * ux + (cell[1] - py) * uy
        return px + t * ux, py + t * uy, ux, uy

    def _corridor_rel(self, inp):
        """앞 두 띠의 통로 가운데로 차 기준 칸 가운데 선 (앞바퀴 기준 mx, my, ux, uy). 띠 하나만 되면 방향은 칸 추정선."""
        p = self.p
        pts = inp.get('points', [])
        f_ = list(p.get('center_bands_mm', [0.0, 170.0, 230.0, 420.0]))
        # ROS 파라미터는 숫자 목록만 받음 -> [시작1, 끝1, 시작2, 끝2] (예전 ((a,b),(c,d)) 모양도 받음)
        if f_ and isinstance(f_[0], (list, tuple)):
            f_ = [x for band in f_ for x in band]
        b = ((float(f_[0]), float(f_[1])), (float(f_[2]), float(f_[3])))
        m1 = corridor_mid(pts, b[0][0], b[0][1], gap_min_mm=p.get('center_gap_min_mm', 650.0),
                          gap_max_mm=p.get('center_gap_max_mm', 1000.0))
        m2 = corridor_mid(pts, b[1][0], b[1][1], gap_min_mm=p.get('center_gap_min_mm', 650.0),
                          gap_max_mm=p.get('center_gap_max_mm', 1000.0))
        if m1 is not None and m2 is not None:
            vx, vy = m2[0] - m1[0], m2[1] - m1[1]
            n = math.hypot(vx, vy)
            if n > 1e-6 and abs(math.degrees(math.atan2(vy, vx))) <= p.get('center_max_dir_deg', 25.0):
                return m1[0], m1[1], vx / n, vy / n
        m = m1 or m2
        cell = self._cell()
        if m is None or cell is None:
            return None
        yaw = inp['yaw'] or 0.0
        ux, uy = cell[2], cell[3]                        # 방향은 칸 추정선 (자기 위치 좌표 -> 차 기준)
        rx, ry = math.cos(yaw) * ux + math.sin(yaw) * uy, -math.sin(yaw) * ux + math.cos(yaw) * uy
        if rx < 0:
            rx, ry = -rx, -ry
        return m[0], m[1], rx, ry

    def _side_mid(self, side, inp):
        """양옆 리모로 구한 칸 가운데 선을 자기 위치 좌표로 바꿔 부드럽게 (튀는 값 줄임)."""
        new = self._line_to_world(side, inp)
        old = self.side_line
        a = self.p.get('center_smooth', 0.5)
        if old is None or not self.p.get('center_on_sides', False):
            return new
        ux, uy = old[2] + a * (new[2] - old[2]), old[3] + a * (new[3] - old[3])
        n = math.hypot(ux, uy) or 1.0
        return (old[0] + a * (new[0] - old[0]), old[1] + a * (new[1] - old[1]), ux / n, uy / n)

    def _turned_deg(self, inp):
        """TURN 시작부터 돈 각도(도)."""
        if inp['yaw'] is None or self.yaw_turn0 is None:
            return 0.0
        return abs(math.degrees(wrap(inp['yaw'] - self.yaw_turn0)))

    def _update_pose(self, inp):
        """오도메트리 거리 + IMU 방향으로 뒷바퀴축 위치를 이어 붙인다 (짧은 거리라 충분)."""
        if self._last_dist is not None and inp['yaw'] is not None:
            ds = inp['dist_mm'] - self._last_dist
            sgn = -1.0 if self._last_v < 0 else 1.0
            self.px += sgn * ds * math.cos(inp['yaw'])
            self.py += sgn * ds * math.sin(inp['yaw'])
        self._last_dist = inp['dist_mm']

    def _out(self, v, k, info=None):
        """명령을 내보내며 마지막 속도 부호와 전진 조향 곡률을 기억 (위치 계산 및 기억후진 반대 조향용)."""
        self._last_v = v
        if v > 0 and k is not None and abs(k) > 1e-6:
            self.last_fwd_k = float(k)
        return v, k, (info or self._info())

    def _line_to_world(self, rel, inp):
        """차 기준 선(앞바퀴 기준)을 자기 위치 좌표로 바꾼다."""
        mx, my, ux, uy = rel
        yaw = inp['yaw'] or 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        fx = self.px + self.p['wheelbase_mm'] * c     # 앞바퀴 축 위치
        fy = self.py + self.p['wheelbase_mm'] * s
        return (fx + mx * c - my * s, fy + mx * s + my * c, ux * c - uy * s, ux * s + uy * c)

    def _misalign(self, line, inp):
        """칸 가운데 선 대비 (모서리 최대 옆 거리 mm, 방향 오차 도, 가장 깊은 모서리 s, 가장 얕은 모서리 s).

        점수는 네 모서리로 정해지니 모서리를 직접 본다. s = 칸 가운데(양옆 리모 중점)에서 선 방향 거리,
        들어가는 쪽이 +. 정차 리모가 칸 가운데에 서 있다고 가정하므로 s=0이 칸 깊이 가운데.
        """
        p = self.p
        cx, cy, ux, uy = line
        yaw = inp['yaw'] or 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        rear = p['body_center_from_rear_mm'] - p['body_length_mm'] / 2.0    # 뒷바퀴축 -> 뒤범퍼 (음수)
        front = p['body_center_from_rear_mm'] + p['body_length_mm'] / 2.0
        worst, ss = 0.0, []
        for dx in (rear, front):
            for dy in (-p['body_width_mm'] / 2.0, p['body_width_mm'] / 2.0):
                qx = self.px + dx * c - dy * s
                qy = self.py + dx * s + dy * c
                worst = max(worst, abs(-(qx - cx) * uy + (qy - cy) * ux))   # 선에서 옆 거리
                ss.append((qx - cx) * ux + (qy - cy) * uy)                  # 선 방향 깊이
        head = math.degrees(wrap(yaw - math.atan2(uy, ux)))
        return worst, head, max(ss), min(ss)

    def _ahead_of_front(self, line, inp):
        """칸 가운데 선(들어가는 방향에 수직인 선)이 앞바퀴 축보다 차 진행 방향으로 얼마나 앞에 있나 (mm)."""
        cx, cy, ux, uy = line
        yaw = inp['yaw'] or 0.0
        fx = self.px + self.p['wheelbase_mm'] * math.cos(yaw)
        fy = self.py + self.p['wheelbase_mm'] * math.sin(yaw)
        # 선 위 점까지 차 앞 방향 거리: 차 앞 방향과 선 방향(ux,uy)은 거의 수직이니 그대로 앞 방향 투영
        return (cx - fx) * math.cos(yaw) + (cy - fy) * math.sin(yaw)

    def _car_rect(self):
        """차 사각형 (뒷바퀴축 기준 앞뒤 [뒤범퍼, 앞범퍼], 옆 반폭)."""
        p = self.p
        return (p['body_center_from_rear_mm'] - p['body_length_mm'] / 2.0,
                p['body_center_from_rear_mm'] + p['body_length_mm'] / 2.0, p['body_width_mm'] / 2.0)

    def _memory_boxes(self):
        """기억한 칸 가운데 선으로 정차 리모 위치를 만든다: 같은 줄 좌우 + (앞줄 목표면) 뒷줄 세 칸.

        반환: [(중심 x, 중심 y, 깊이 방향 ux, uy)] 자기 위치 좌표. 리모는 칸 가운데에 반듯하다고 가정.
        """
        cell = self._cell()
        if cell is None:
            return []
        p = self.p
        cx, cy, ux, uy = cell
        vx, vy = -uy, ux                                  # 차선 방향 (칸 줄 방향)
        w = p['cell_pitch_mm']
        spots = [(0.0, -w), (0.0, w)]                     # (깊이, 옆) 같은 줄 좌우 = 1·3번
        if p['target_row'] != 'back':
            spots += [(p['row_pitch_mm'], -w), (p['row_pitch_mm'], 0.0), (p['row_pitch_mm'], w)]   # 4·5·6번
        return [(cx + d * ux + o * vx, cy + d * uy + o * vy, ux, uy) for d, o in spots]

    def _side_gap(self, inp):
        """차 테두리와 리모 사이 가장 가까운 거리 (mm): 지금 보이는 라이다 점 + 기억한 리모 위치.

        꺾는 도중 1번 리모 모서리는 라이다 시야(약 ±105도) 뒤라 안 보일 수 있어 기억으로 본다.
        """
        p = self.p
        yaw = inp['yaw'] or 0.0
        c, sn = math.cos(yaw), math.sin(yaw)
        r0, r1, hw = self._car_rect()
        gap = 1e9
        # (1) 지금 보이는 점: 앞바퀴 기준 -> 뒷바퀴축 기준으로 옮겨 차 사각형까지 거리
        for x, y in inp.get('points', []):
            xr = x + p['wheelbase_mm']
            dx = max(r0 - xr, 0.0, xr - r1)
            dy = max(abs(y) - hw, 0.0)
            gap = min(gap, math.hypot(dx, dy))
        # (2) 기억한 리모: 차 테두리 위 점들에서 리모 사각형까지 거리
        boxes = self._memory_boxes()
        if boxes:
            hl, hwb = p['parked_half_len_mm'], p['body_width_mm'] / 2.0
            outline = []
            for a in range(9):
                t = a / 8.0
                for (ax, ay), (bx, by) in (((r0, -hw), (r1, -hw)), ((r0, hw), (r1, hw)),
                                           ((r0, -hw), (r0, hw)), ((r1, -hw), (r1, hw))):
                    lx, ly = ax + (bx - ax) * t, ay + (by - ay) * t
                    outline.append((self.px + lx * c - ly * sn, self.py + lx * sn + ly * c))
            for bx, by, ux, uy in boxes:
                for qx, qy in outline:
                    du = (qx - bx) * ux + (qy - by) * uy          # 리모 앞뒤 방향
                    dv = -(qx - bx) * uy + (qy - by) * ux         # 리모 옆 방향
                    gap = min(gap, math.hypot(max(abs(du) - hl, 0.0), max(abs(dv) - hwb, 0.0)))
        return gap

    def _center_offset(self, line, inp):
        """차체 중심이 칸 가운데 선에서 옆으로 벗어난 거리 (mm, 왼쪽 +)."""
        cx, cy, ux, uy = line
        yaw = inp['yaw'] or 0.0
        bx = self.px + self.p['body_center_from_rear_mm'] * math.cos(yaw)
        by = self.py + self.p['body_center_from_rear_mm'] * math.sin(yaw)
        return -(bx - cx) * uy + (by - cy) * ux

    def _lane_dir(self):
        """차로 방향 단위벡터 (칸 가운데 선의 깊이 방향 u를 왼쪽으로 90도: 진행 방향)."""
        _, _, ux, uy = self.pair_line
        return -uy, ux

    def _measure_corner(self, inp):
        """1번 상자의 '2번 칸 쪽 + 차로 쪽' 모서리를 라이다 점으로 잰다 (11/12 캡틴 제안).

        1번 = 칸 가운데 선 - 칸 간격(차로 방향). 그 근처 점 중 차로 쪽 면(가장 얕은 점들)의
        진행 방향 끝점 = 모서리. 차가 가까이 지나가며 보는 점이라 멀리서 본 짝 중점보다 정확하다.
        반환: 칸 가운데 기준 차로 방향 위치 s (mm) 또는 None.
        """
        p = self.p
        self._face_center = None
        if self.pair_line is None or inp['yaw'] is None:
            return None
        cx, cy, ux, uy = self.pair_line
        vx, vy = self._lane_dir()
        yaw = inp['yaw']
        c, sn = math.cos(yaw), math.sin(yaw)
        fx, fy = self.px + p['wheelbase_mm'] * c, self.py + p['wheelbase_mm'] * sn
        b1 = -p['cell_pitch_mm']                     # 1번 중심 (차로 방향, 칸 가운데 기준)
        lx = -p['front_axle_from_lidar_mm']           # 라이다 (앞바퀴 기준 앞 47)
        pts = []
        for x, y in inp.get('points', []):
            wx, wy = fx + x * c - y * sn, fy + x * sn + y * c
            s_ = (wx - cx) * vx + (wy - cy) * vy
            d_ = (wx - cx) * ux + (wy - cy) * uy
            if abs(s_ - b1) <= p['corner_window_mm'] and abs(d_) <= 330.0:
                pts.append((s_, d_, math.hypot(x - lx, y)))
        if len(pts) < p['corner_min_pts']:
            return None
        face = min(d for _, d, _ in pts)              # 차로 쪽 면 = 가장 얕은 깊이
        face_pts = [(s_, r) for s_, d, r in pts if d <= face + 40.0]
        if len(face_pts) < p['corner_min_pts']:
            return None
        sc, rc = max(face_pts)
        smin = min(s_ for s_, _ in face_pts)
        # 앞면 가운데 (상자 크기와 무관). 앞면 양 끝이 다 보여야 함: 위쪽 끝이 창 경계에 잘리지 않고 앞면 폭 15cm 이상
        # 라이다가 앞면 옆을 지나는 중일 때만 (양 끝이 다 보이고 가까움). 멀면 일부만 보여 틀림 (11/13 실차)
        ls = self._lidar_s(inp)
        self._face_center = ((smin + sc) / 2.0
                             if (smin > b1 - p['corner_window_mm'] + 20.0 and sc - smin >= 150.0
                                 and smin - 50.0 <= ls <= sc + 50.0) else None)
        # 11/13 실차: 멀리서는 앞면 일부만 보여(점은 700mm 안만) 모서리를 1번 쪽으로 잘못 잡고, 그 값으로 꺾어
        # 매번 수십 cm씩 다른 곳으로 들어감 -> 모서리가 라이다에서 corner_max_range_mm 안일 때 잰 값만 쓴다
        if rc > p['corner_max_range_mm']:
            return None
        expect = b1 + p['body_width_mm'] / 2.0        # 리모 폭 220 -> 1번 중심 + 110
        if abs(sc - expect) > p['corner_gate_mm']:
            return None
        return sc

    def _measure_target(self, inp):
        """목표(2번 칸 가운데) 차로 방향 위치 = 5번 중심 (없으면 1·3번 중점). 가까이서 잰 값만, 중앙값 (11/13 캡틴).

        상자 크기·간격과 무관: 리모든 넓은 상자든 실제로 잰 중심을 쓴다.
        """
        p = self.p
        cx, cy, ux, uy = self.pair_line
        vx, vy = self._lane_dir()
        yaw = inp['yaw'] or 0.0
        c, sn = math.cos(yaw), math.sin(yaw)
        fx, fy = self.px + p['wheelbase_mm'] * c, self.py + p['wheelbase_mm'] * sn

        def to_sd(x, y):
            wx, wy = fx + x * c - y * sn, fy + x * sn + y * c
            return (wx - cx) * vx + (wy - cy) * vy, (wx - cx) * ux + (wy - cy) * uy
        # 5번: 뒷줄(깊이 + row_pitch), 차로 방향으로 칸 가운데 근처, 가까울 때
        for x, y in inp.get('clusters_box', []):
            if math.hypot(x, y) > p['target_max_range_mm']:
                continue
            s_, d_ = to_sd(x, y)
            if abs(s_) <= p['target_gate_mm'] and abs(d_ - p['row_pitch_mm']) <= 250.0:
                self.tgt_c5.append(s_)
                break
        # 1·3번 중점: 이번 프레임 짝, 1번(가까운 쪽)이 target_max_range_mm 안일 때
        fp = find_pair(self._pair_src(inp), p['side_far_mm'], p['pair_dist_mm'], p['pair_tol_mm'])
        if fp is not None:
            mx, my, pvx, pvy = fp
            near = (mx - pvx * p['pair_dist_mm'] / 2.0, my - pvy * p['pair_dist_mm'] / 2.0)
            far = (mx + pvx * p['pair_dist_mm'] / 2.0, my + pvy * p['pair_dist_mm'] / 2.0)
            # 11/13: 먼 쪽(3번)이 덩어리 거리 1.5m 끝에 걸리면 먼 끝이 잘려 중심이 가까운 쪽으로 쏠림 -> 중점 5cm 앞쪽
            # (꺾기가 4~6cm 일찍, 끝에서 1번 쪽으로 치우침). 3번 전체가 보일 때만, 1번이 시야(±105도) 뒤로 빠지기 전만
            far_ok = math.hypot(*far) <= p['target_far_max_mm']
            near_ok = math.degrees(math.atan2(near[1], near[0] + p['front_axle_from_lidar_mm'])) >= -p['target_near_max_deg']
            if math.hypot(*near) <= p['target_max_range_mm'] and far_ok and near_ok:
                s_, _ = to_sd(mx, my)
                if abs(s_) <= p['target_gate_mm']:
                    self.tgt_pm.append(s_)
        n = p['corner_median_n']
        for hist, src in ((self.tgt_c5, 'col5'), (self.tgt_pm, 'pair')):
            if len(hist) >= p['corner_min_frames']:
                h = sorted(hist[-n:])
                self.target_s, self.target_src = h[len(h) // 2], src
                return

    def _rear_s(self):
        """뒷바퀴 축 가운데: 칸 가운데 선 기준 차로 방향 mm."""
        cx, cy, _, _ = self.pair_line
        vx, vy = self._lane_dir()
        return (self.px - cx) * vx + (self.py - cy) * vy

    def _target_stable(self):
        """path: 목표(5번 중심, 없으면 1·3번 중점)를 가까이서 3번 이상 쟀고 최근 값이 path_stable_mm 안으로 일정한가."""
        p = self.p
        for hist in (self.tgt_c5, self.tgt_pm):
            if len(hist) >= p['corner_min_frames']:
                last = hist[-5:]
                return max(last) - min(last) <= p['path_stable_mm']
        return False

    def _lidar_s(self, inp):
        """라이다 위치: 칸 가운데 선 기준 차로 방향 mm."""
        p = self.p
        cx, cy, _, _ = self.pair_line
        vx, vy = self._lane_dir()
        yaw = inp['yaw'] or 0.0
        ahead = p['wheelbase_mm'] - p['front_axle_from_lidar_mm']   # 뒷바퀴 -> 라이다 (앞바퀴 앞 47)
        lx, ly = self.px + ahead * math.cos(yaw), self.py + ahead * math.sin(yaw)
        return (lx - cx) * vx + (ly - cy) * vy

    def _front_gap(self, inp):
        """앞범퍼 -> 정면(차 폭 안) 가장 가까운 라이다 점 거리 mm. 없으면 None. (5번 앞 정지용)"""
        p = self.p
        hw = p['body_width_mm'] / 2.0
        gaps = [x - p['front_bumper_from_axle_mm'] for x, y in inp.get('points', [])
                if abs(y) <= hw and x > p['front_bumper_from_axle_mm']]
        return min(gaps) if gaps else None

    @staticmethod
    def _pair_src(inp):
        """짝 찾기용 덩어리. 크기로 거른 목록이 비어 있어도 그대로 쓴다.
        (11/12: 'clusters_pair or clusters'였을 때 거른 목록이 비면 거르기 전 목록(표지판 기둥 포함)으로 돌아가
        기둥 + 반쯤 보인 1번이 짝이 됨 -> 1번 앞 빈 곳으로 들어감)"""
        cp = inp.get('clusters_pair')
        return inp['clusters'] if cp is None else cp

    def _guard_hit(self, inp):
        """옆 감시: side_guard_mm보다 가까우면 True (칸 가운데를 알 때만)."""
        p = self.p
        return p['side_guard_mm'] > 0 and self._cell() is not None and self._side_gap(inp) < p['side_guard_mm']

    def _front_s(self, inp):
        """앞범퍼 양 모서리 중 더 깊은 쪽의 깊이 s (칸 가운데 = 0, 들어가는 쪽 +)."""
        p = self.p
        cx, cy, ux, uy = self.line
        yaw = inp['yaw'] or 0.0
        c, sn = math.cos(yaw), math.sin(yaw)
        front = p['body_center_from_rear_mm'] + p['body_length_mm'] / 2.0
        out = []
        for dy in (-p['body_width_mm'] / 2.0, p['body_width_mm'] / 2.0):
            qx = self.px + front * c - dy * sn
            qy = self.py + front * sn + dy * c
            out.append((qx - cx) * ux + (qy - cy) * uy)
        return max(out)

    def _body_s(self, inp, line=None):
        """차체 중심의 깊이 s (칸 가운데 = 0, 들어가는 쪽 +)."""
        cx, cy, ux, uy = line or self.line
        yaw = inp['yaw'] or 0.0
        bx = self.px + self.p['body_center_from_rear_mm'] * math.cos(yaw)
        by = self.py + self.p['body_center_from_rear_mm'] * math.sin(yaw)
        return (bx - cx) * ux + (by - cy) * uy

    def _pair_cell(self):
        """짝으로 구한 칸 가운데 선을 깊이 보정한 것. 차선에서 보면 리모 끝면만 보여 얕게 나온다."""
        if self.pair_line is None:
            return None
        cx, cy, ux, uy = self.pair_line
        b = self.p['pair_depth_bias_mm']
        return (cx + b * ux, cy + b * uy, ux, uy)

    def _cell(self):
        """지금 믿는 칸 가운데 선. 옆 위치·방향은 짝, 깊이는 양옆 옆면(봤으면) > 짝(보정).

        짝은 차선에서 1·3번을 같은 자리·같은 때 봐서 옆 위치·방향이 정확하다 (꺾는 시점이 잘 맞는 이유).
        꺾은 뒤 양옆 옆면은 1번이 시야 뒤로 반쯤 빠져 덩어리 중심이 쏠려 옆으로 약 6cm 틀어진다 (11/10 시뮬레이션).
        대신 깊이는 옆면이 정확하다 (짝은 끝면만 보여 얕게 나옴).
        """
        pc = self._pair_cell()
        if pc is None:
            return self.line if self.side_seen else None
        cx, cy, ux, uy = pc
        if self.side_seen and self.side_line is not None:      # 가로선(깊이) = 1·3번 옆면
            d = (self.side_line[0] - cx) * ux + (self.side_line[1] - cy) * uy
            cx, cy = cx + d * ux, cy + d * uy
        if self.col5 is not None and self.col5_n >= self.p['col5_min_frames']:   # 세로선(옆) = 5번 앞면 가운데
            vx, vy = -uy, ux
            o = (self.col5[0] - cx) * vx + (self.col5[1] - cy) * vy
            cx, cy = cx + o * vx, cy + o * vy
        return (cx, cy, ux, uy)

    def _update_col5(self, inp):
        """정면의 5번 리모 앞면으로 2번 칸 세로 가운데선 갱신 (윤서 아이디어 11/10).

        들어갈 때 5번은 차 정면이라 1번에 안 가리고 시야 밖으로도 안 빠진다 (차선 따라올 땐 1번에 가려짐 -> 안 씀).
        정면에서 보면 앞면 폭 전체가 보이므로 점들의 좌우 양 끝 가운데가 5번 가운데. 짝으로 예상한 위치에서
        col5_gate_mm 안의 덩어리만 받는다.
        """
        p = self.p
        base = self._pair_cell()
        if base is None or self._turned_deg(inp) < p['col5_min_turn_deg']:
            return
        if self.side_seen and self.side_line is not None:
            d = (self.side_line[0] - base[0]) * base[2] + (self.side_line[1] - base[1]) * base[3]
            base = (base[0] + d * base[2], base[1] + d * base[3], base[2], base[3])
        cx, cy, ux, uy = base
        vx, vy = -uy, ux
        face = p['row_pitch_mm'] - p['parked_half_len_mm']          # 칸 가운데 -> 5번 앞면 (깊이)
        ex, ey = cx + face * ux, cy + face * uy                      # 예상 위치
        yaw = inp['yaw'] or 0.0
        c, sn = math.cos(yaw), math.sin(yaw)
        fx, fy = self.px + p['wheelbase_mm'] * c, self.py + p['wheelbase_mm'] * sn
        best = None
        for x, y in inp.get('clusters_box', []):
            wx, wy = fx + x * c - y * sn, fy + x * sn + y * c        # 앞바퀴 기준 -> 자기 위치 좌표
            dl = (wx - ex) * vx + (wy - ey) * vy
            dd = (wx - ex) * ux + (wy - ey) * uy
            if abs(dl) <= p['col5_gate_mm'] and abs(dd) <= 2 * p['col5_gate_mm'] and x > 0:
                if best is None or abs(dl) < abs(best[0]):
                    best = (dl, wx, wy)
        if best is None:
            return
        _, wx, wy = best
        n = self.col5_n
        self.col5 = (wx, wy) if self.col5 is None else ((self.col5[0] * n + wx) / (n + 1),
                                                        (self.col5[1] * n + wy) / (n + 1))
        self.col5_n += 1

    def _end_remaining(self, inp):
        """카메라로 본 칸 끝 선까지 남은 거리 (앞바퀴 기준, mm). 못 봤으면 None.

        칸 끝 선 = 앞줄이면 앞줄·뒷줄 사이 흰 선, 뒷줄이면 주차 구역 바깥 노란 테두리.
        칸 가운데 선으로 예상한 위치와 end_gate_mm 안인 선만 받고, 카메라 사각지대(앞바퀴 약 350mm 안)로
        들어간 뒤에는 오도메트리로 센다 (정지선과 같은 방식).
        """
        p = self.p
        cell = self._cell()
        if cell is not None and inp.get('end_lines_mm'):
            yaw = inp['yaw'] or 0.0
            cx, cy, ux, uy = cell
            fx = self.px + p['wheelbase_mm'] * math.cos(yaw)
            fy = self.py + p['wheelbase_mm'] * math.sin(yaw)
            expect = p['cell_half_depth_mm'] - ((fx - cx) * ux + (fy - cy) * uy)   # 앞바퀴 -> 칸 끝 선
            frozen = (self.end_d is not None
                      and self.end_d - (inp['dist_mm'] - self.end_at) < p['end_freeze_mm'])
            if not frozen:
                cand = min(inp['end_lines_mm'], key=lambda d: abs(d - expect))
                if abs(cand - expect) <= p['end_gate_mm']:
                    self.end_d, self.end_at = cand, inp['dist_mm']
        if self.end_d is None:
            return None
        return self.end_d - (inp['dist_mm'] - self.end_at)

    def step(self, inp):
        """inp: t, dist_mm, yaw, lane_v, lane_w, stoplines_mm, clusters, blocked.
        반환: (v, kappa, info). kappa=None이면 차선 노드 회전(lane_w)을 그대로 씀."""
        p = self.p
        if self.t_phase is None:
            self._enter('WAIT_LINE', inp)
        self._update_pose(inp)
        if p.get('turn_ref') == 'fixed':
            return self._step_fixed(inp)
        moved = inp['dist_mm'] - self.d_phase

        if self.phase == 'WAIT_LINE':       # ① 흰 선 기다림 (차선 노드 명령 넘김)
            lines = [d for d in inp['stoplines_mm'] if d >= p['line_min_mm']]
            # armed: 미션 노드가 주차 표지를 봤거나 도면상 주차 앞 선 근처 (/mission/state parking_armed)
            # 11/10: 표지를 흰 선 지난 뒤에 확정하면 선을 놓친다 -> 오른쪽 두 리모 짝(1·3번)을 연속으로 보면 그것도 시작 신호.
            # 짝은 같은 줄 두 대가 약 104cm 떨어져야 생겨서, 커브 등에 한 대만 서 있는 리모로는 안 켜진다.
            if p['skip_wait_line']:
                # 11/12: M4 들어오면(미션 노드 m4_direct) 흰 선 없이 바로 짝 찾기 (꺾는 시점은 원래도 짝 중점 기준)
                self.trigger = 'direct'
                self.lane_cfg = {'target_offset_mm': p['target_offset_mm']}
                self._enter('FOLLOW', inp)
            armed = inp.get('armed', True)
            seen = self.phase == 'WAIT_LINE' and (p['pair_trigger'] and armed and find_pair(
                self._pair_src(inp), p['side_far_mm'], p['pair_dist_mm'],
                p['pair_tol_mm']) is not None)
            self.trig_n = self.trig_n + 1 if seen else 0
            by_line = moved >= p['ignore_mm'] and lines and armed
            by_pair = self.trig_n >= p['pair_trigger_frames']
            if self.phase != 'WAIT_LINE':
                pass
            elif by_line or by_pair:
                self.trigger = 'line' if by_line else 'pair'
                self.lane_cfg = {'target_offset_mm': p['target_offset_mm']}
                self._enter('FOLLOW', inp)
            else:
                return self._out(inp['lane_v'], None)

        if self.phase == 'FOLLOW':          # ② 왼쪽 붙어 차선 따라가기, ③ 오른쪽 리모 찾기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                if p['wait_blocked_before_pair'] and self.pair_line is None:
                    # 11/12: 짝 찾기 전(주차 칸 전 직선)에 앞이 막히면 끝내지 말고 서서 기다림 (사람 발·물건)
                    return self._out(0.0, 0.0)
                return self._done('BLOCKED', inp)
            follow_max = p['follow_max_direct_mm'] if self.trigger == 'direct' else p['follow_max_mm']
            pair = find_pair(self._pair_src(inp), p['side_far_mm'], p['pair_dist_mm'],
                             p['pair_tol_mm'])
            if pair is not None:                       # ③ 두 리모 짝 -> 칸 가운데 선 (평균)
                mx, my, vx, vy = pair
                n0 = self.pair_n
                self.pair_side_mm = (-my if self.pair_side_mm is None
                                     else (self.pair_side_mm * n0 - my) / (n0 + 1))
                pair_row = 'front' if -my < p['row_split_mm'] else 'back'   # 이번에 본 짝의 줄
                self.row = 'front' if self.pair_side_mm < p['row_split_mm'] else 'back'
                if p['pair_dir_from_heading']:
                    # 들어가는 방향 = 지금 차의 오른쪽 (차선 추종 중이라 차가 차선과 나란함).
                    # 두 리모를 잇는 방향은 1번·3번의 깊이 쏠림이 서로 달라 약 6도 기울었음 (11/10 시뮬레이션)
                    rel = (mx, my, 0.0, -1.0)
                else:
                    rel = (mx, my, vy, -vx)            # 들어가는 방향 = 짝 방향에서 오른쪽 90도
                line = self._line_to_world(rel, inp)
                if p['target_row'] in ('front', 'back'):
                    # 목표 줄 고정 (11/10 대회 배치: 항상 2번 = 앞줄). 다른 줄 짝(1번을 놓쳐 4·6번이 짝)은
                    # 앞뒤 위치·방향은 같으니 깊이만 한 줄 옮겨 쓴다.
                    self.row = p['target_row']
                    if pair_row != p['target_row']:
                        cx, cy, ux, uy = line
                        sh = -p['row_pitch_mm'] if p['target_row'] == 'front' else p['row_pitch_mm']
                        line = (cx + sh * ux, cy + sh * uy, ux, uy)
                if self.pair_line is None:
                    self.pair_line = line
                else:                                  # 여러 번 본 값의 평균 (흔들림 줄이기)
                    n = self.pair_n
                    self.pair_line = tuple((a * n + b) / (n + 1) for a, b in zip(self.pair_line, line))
                self.pair_n += 1
            if self.pair_line is not None and self.pair_n >= p['pair_min_frames']:
                if p['turn_ref'] == 'path':
                    # 11/13 캡틴 경로: ① 1·3·5번 인식 ② 다 되면 정지(서서 다시 잼) ③ 목표 P' = l5 위, 원이 끝나는 깊이
                    # ④ Q(뒷바퀴) = 목표 - 회전 반경 으로 전진/후진 ⑤ 최대 조향 원호 ⑥ 앞범퍼가 5번 앞 30cm(R)면 정지
                    self._measure_target(inp)
                    late = self._rear_s() > -p['arc_radius_mm'] + p['path_late_mm']
                    if self._target_stable() or (late and self.target_s is not None):
                        self.settle_n0 = (len(self.tgt_c5), len(self.tgt_pm))
                        self._enter('SETTLE', inp)
                        return self._out(0.0, 0.0)
                    if late and self.target_s is None and self._rear_s() > -p['arc_radius_mm'] + 2 * p['path_late_mm']:
                        # 5번도 1·3번 가까운 짝도 못 잼: 멀리서 평균 낸 짝 중점으로
                        self.target_s, self.target_src = 0.0, 'pair_avg'
                        self.settle_n0 = (len(self.tgt_c5), len(self.tgt_pm))
                        self._enter('SETTLE', inp)
                        return self._out(0.0, 0.0)
                    return self._out(inp['lane_v'], None)
                if p['turn_ref'] == 'target':
                    # 11/13 캡틴: 목표 = 1·3번 중심선 ∩ 5번 중심선. 1번 꼭짓점은 '언제 꺾을지' 기준점:
                    # 꼭짓점 -> 꺾기 시작점 거리 d = (목표 - 라이다 기준 앞당김) - 꼭짓점 을 매번 계산 (고정값 아님)
                    self._measure_target(inp)
                    sc = self._measure_corner(inp)
                    if sc is not None:
                        self.corner_hist.append(sc)
                        self.corner_n += 1
                        h = sorted(self.corner_hist[-p['corner_median_n']:])
                        self.corner_s = h[len(h) // 2]
                    if self.target_s is not None:
                        # 라이다 기준 꺾기 시작점. 라이다 위치 값(front_axle_from_lidar_mm, 미측정)을 섞지 않는 상수라
                        # 그 값이 틀려도 꺾는 실제 위치는 같다 (목표·라이다 위치가 같은 라이다 점에서 나옴)
                        start = self.target_s - p['turn_lead_lidar_mm']
                        ref = self.corner_s if self.corner_s is not None else 0.0
                        d_from_ref = start - ref
                        if self._lidar_s(inp) - ref >= d_from_ref - p['brake_mm']:
                            cx, cy, ux, uy = self.pair_line
                            vx, vy = self._lane_dir()
                            self.pair_line = (cx + self.target_s * vx, cy + self.target_s * vy, ux, uy)
                            self.line = self.pair_line
                            self.turn_ref_used = 'target_' + self.target_src
                            self.turn_at = (round(self.target_s), None if self.corner_s is None else round(self.corner_s),
                                            round(d_from_ref), round(self._lidar_s(inp)))
                            self._enter('PAUSE', inp)
                            return self._out(0.0, 0.0)
                        return self._out(inp['lane_v'], None)
                    # 목표를 아직 못 잼: 짝 방식 꺾는 점을 15cm 넘게 지나면 짝 방식으로
                    if self._ahead_of_front(self.pair_line, inp) > p['turn_lead_mm'] - 150.0:
                        return self._out(inp['lane_v'], None)
                if p['turn_ref'] == 'center1':
                    # 11/13: 1번 앞면 가운데 기준 (상자 크기와 무관). 2번 가운데 = 1번 가운데 + 칸 간격.
                    # 라이다가 1번 가운데 + turn_from_center1_mm 지나면 꺾기 (247 = 짝 방식 turn_lead 320과 같은 위치)
                    self._measure_corner(inp)
                    fc = self._face_center
                    if fc is not None and abs(fc - (-p['cell_pitch_mm'])) <= p['corner_gate_mm']:
                        self.corner_hist.append(fc)
                        self.corner_n += 1
                        h = sorted(self.corner_hist[-p['corner_median_n']:])
                        self.corner_s = h[len(h) // 2]
                    if self.corner_s is not None and self.corner_n >= p['corner_min_frames']:
                        if self._lidar_s(inp) - self.corner_s >= p['turn_from_center1_mm'] - p['brake_mm']:
                            cx, cy, ux, uy = self.pair_line
                            vx, vy = self._lane_dir()
                            sh = self.corner_s + p['cell_pitch_mm']
                            self.pair_line = (cx + sh * vx, cy + sh * vy, ux, uy)
                            self.line = self.pair_line
                            self.turn_ref_used = 'center1'
                            self.turn_at = (round(self.corner_s), round(self._lidar_s(inp)), self.corner_n)
                            self._enter('PAUSE', inp)
                            return self._out(0.0, 0.0)
                        return self._out(inp['lane_v'], None)
                if p['turn_ref'] == 'corner':
                    # 11/12: 1번 모서리를 가까이서 계속 다시 재고(최신 값), 라이다가 모서리 선 + turn_from_corner_mm를
                    # 지나면 꺾는다. 짝 중점 평균은 멀리서 본 쏠린 값이 섞여 꺾는 위치가 매번 달랐음.
                    sc = self._measure_corner(inp)
                    if sc is not None:
                        # 가까이서 잰 값들의 중앙값 (한 프레임 튀는 값 무시)
                        self.corner_hist.append(sc)
                        self.corner_n += 1
                        h = sorted(self.corner_hist[-p['corner_median_n']:])
                        self.corner_s = h[len(h) // 2]
                    if self.corner_s is not None and self.corner_n >= p['corner_min_frames']:
                        if self._lidar_s(inp) - self.corner_s >= p['turn_from_corner_mm'] - p['brake_mm']:
                            # 칸 가운데도 모서리 기준으로 맞춤: 2번 가운데 = 모서리 + (칸 간격 - 리모 반폭)
                            cx, cy, ux, uy = self.pair_line
                            vx, vy = self._lane_dir()
                            sh = self.corner_s + p['cell_pitch_mm'] - p['body_width_mm'] / 2.0
                            self.pair_line = (cx + sh * vx, cy + sh * vy, ux, uy)
                            self.line = self.pair_line
                            self.turn_ref_used = 'corner'
                            self.turn_at = (round(self.corner_s), round(self._lidar_s(inp)), self.corner_n)
                            self._enter('PAUSE', inp)
                            return self._out(0.0, 0.0)
                        return self._out(inp['lane_v'], None)
                if self._ahead_of_front(self.pair_line, inp) <= p['turn_lead_mm'] + p['brake_mm']:
                    self.line = self.pair_line
                    self.turn_ref_used = 'pair'
                    self._enter('PAUSE', inp)
                    return self._out(0.0, 0.0)
                return self._out(inp['lane_v'], None)
            if not p['single_fallback']:
                # 11/12: 전체 주행에서 주차 표지판(입구 오른쪽)을 리모로 보고 EXTRA -> 일찍 꺾어 1번 칸에 충돌.
                # 대회는 칸 양옆에 리모가 늘 있어(규정: 최소 2대) 짝이 생긴다 -> 한 물체만 보고 꺾지 않는다.
                if moved >= follow_max:
                    return self._done('FAIL_NO_CAR', inp)
                return self._out(inp['lane_v'], None)
            # (켰을 때) 한 물체 예비 규칙: 리모만 한 크기 덩어리만 (표지판 기둥·사람 다리 제외)
            src = inp.get('clusters_single')
            src = inp['clusters'] if src is None else src
            right = [(x, y) for x, y in src if -p['side_far_mm'] <= y <= -80.0]
            if any(x > p['abeam_tol_mm'] for x, _ in right):
                self.seen_ahead = True
            hit = any(abs(x) <= p['abeam_tol_mm'] or (p['passed_ok'] and -400.0 < x < 0.0)
                      for x, _ in right)
            if self.seen_ahead and hit:
                self._enter('EXTRA', inp)
            elif moved >= follow_max:
                return self._done('FAIL_NO_CAR', inp)
            else:
                return self._out(inp['lane_v'], None)

        if self.phase == 'EXTRA':           # (추가) 리모 옆 확인 후 extra_mm 더 가서 꺾기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                return self._done('BLOCKED', inp)
            if moved >= p['extra_mm'] - p['brake_mm']:
                self._enter('PAUSE', inp)
            else:
                return self._out(inp['lane_v'], None)

        if self.phase == 'SETTLE':          # path ②: 서서 다시 재기
            self._measure_target(inp)
            if inp['t'] - self.t_phase < p['settle_sec']:
                return self._out(0.0, 0.0)
            n5, npm = self.settle_n0
            for vals, src in ((self.tgt_c5[n5:], 'col5'), (self.tgt_pm[npm:], 'pair')):
                if len(vals) >= p['corner_min_frames']:
                    v_ = sorted(vals)
                    self.target_s, self.target_src = v_[len(v_) // 2], src + '_still'
                    break
            cx, cy, ux, uy = self.pair_line
            vx, vy = self._lane_dir()
            t0 = self.target_s
            self.pair_line = (cx + t0 * vx, cy + t0 * vy, ux, uy)      # 칸 가운데 선 = l5 (목표가 s = 0)
            self.line = self.pair_line
            self.q_s = -p['arc_radius_mm']                              # Q: 뒷바퀴가 l5에서 회전 반경만큼 앞
            self.turn_ref_used = 'path_' + str(self.target_src)
            self.turn_at = (round(t0), round(self._rear_s() - self.q_s))   # (목표 s, 정지 때 뒷바퀴 - Q)
            self._enter('MOVE_Q', inp)

        if self.phase == 'MOVE_Q':          # path ④: Q까지 똑바로 전진/후진
            err = self._rear_s() - self.q_s                             # + = Q를 지나침 -> 후진
            if abs(err) <= p['q_tol_mm'] or abs(inp['dist_mm'] - self.d_phase) >= p['move_q_max_mm']:
                if abs(err) > p['q_tol_mm']:
                    return self._done('FAIL_Q', inp)
                self._enter('PAUSE', inp)
                return self._out(0.0, 0.0)
            if err < 0:
                if -err <= p['move_q_brake_mm']:
                    self._enter('PAUSE', inp)
                    return self._out(0.0, 0.0)
                return self._out(p['move_q_speed'], 0.0)
            if err <= p['move_q_brake_mm']:
                self._enter('PAUSE', inp)
                return self._out(0.0, 0.0)
            return self._out(-p['move_q_speed'], 0.0)

        if self.phase == 'PAUSE':           # ③ 1초 정지
            if inp['t'] - self.t_phase >= p['pause_sec']:
                self.yaw_turn0 = inp['yaw']
                self._enter('TURN', inp)
            elif self._presteer_due(p['pause_sec'], inp):
                return self._out(p['presteer_speed'], KAPPA_FULL_RIGHT)
            else:
                return self._out(0.0, 0.0)

        if self.phase == 'SWITCH':          # 전진 <-> 후진 사이 멈춤
            if inp['t'] - self.t_phase >= p.get('switch_pause_sec', 0.0):
                self._enter(self.switch_to, inp)
            elif self.switch_to in ('TURN', 'BACK') and self._presteer_due(p.get('switch_pause_sec', 0.0), inp):
                # 다음 동작 조향을 미리 (TURN = 오른쪽 최대, BACK = 이번 후진 곡률)
                if self.switch_to == 'TURN':
                    if inp['blocked']:
                        return self._out(0.0, 0.0)           # 앞이 막혔으면 앞으로는 안 굴림
                    return self._out(p['presteer_speed'], KAPPA_FULL_RIGHT)
                return self._out(-p['presteer_speed'], self._back_kappa())
            else:
                return self._out(0.0, 0.0)

        if self.phase == 'BACK':            # ④-1 앞이 막혀서(또는 옆 감시·가운데 다시 맞추기) 후진
            moved = inp['dist_mm'] - self.d_phase
            if moved >= (self.back_len or p['back_mm']):
                self.back_k = None
                return self._switch('TURN', inp)
            return self._out(-p['back_speed'], self._back_kappa())

        if self.phase == 'TURN':            # ④ 오른쪽 최대 조향 전진 + ⑤ 양옆 확인까지 반복
            moved = inp['dist_mm'] - self.d_phase
            self.turn_travel += max(0.0, moved)
            self.d_phase = inp['dist_mm']
            self._update_col5(inp)
            seen = in_view(inp['clusters'], p['front_axle_from_lidar_mm'], p['side_view_deg'])
            side = (cell_line_from_neighbors(seen, p['side_max_mm'])
                    if self._turned_deg(inp) >= p['stop_min_turn_deg'] else None)
            turned_ok = self._turned_deg(inp) >= p['stop_min_turn_deg']
            if p.get('center_on_sides', False) and self._turned_deg(inp) >= p.get('center_min_turn_deg', 70.0):
                # 양옆 리모 정중앙 선 (조향용): 라이다 점에서 앞 두 띠의 '왼쪽 리모 안쪽 면 ~ 오른쪽 리모 안쪽 면' 가운데.
                # (덩어리 중심은 보이는 면 쪽으로 쏠리고, 1·3번이 옆에 오면 라이다 시야 ±105° 뒤라 안 보임)
                rel = self._corridor_rel(inp)
                if rel is not None:
                    self.side_line = self._side_mid(rel, inp)
                    self.center_off = self._center_offset(self.side_line, inp)
            if self.pair_line is not None and side is not None:
                # 짝을 알고 양옆 리모 옆면이 보이면: 옆면으로 구한 선에서 차체 중심이 칸 가운데 오면 멈춤.
                # (차선에서 본 짝은 리모의 차선 쪽 끝면만 보여 깊이가 8~17cm 얕게 나온다 -> 깊이는 옆면으로)
                if not p.get('center_on_sides', False) or self.side_line is None:
                    # 11/13: center_on_sides인데 정중앙 선을 아직 못 구했으면(통로 폭 조건 등) 옆면 선으로.
                    # 예전엔 None을 그대로 칸 선에 넣어 TURN에서 노드가 죽었음 (시뮬레이션: 1·3 간격 좁은 배치)
                    self.side_line = self._line_to_world(side, inp)
                self.line = self.side_line
                self.side_seen = True
                r = l = self._body_s(inp) >= -p['enter_brake_mm']
                self.stop_how = 'side' if r else None
            elif self.pair_line is not None and self.side_seen:
                # 옆면을 한 번 봤는데 지금 안 보임: 라이다 시야(실측 약 ±105도) 뒤로 리모가 빠진 것.
                # 기억한 선 + 오도메트리로 계속 판단 (11/09 park_02: 5cm 남기고 놓쳐 1.1m 직진 -> 실패)
                r = l = self._body_s(inp) >= -p['enter_brake_mm']
                self.stop_how = 'memory' if r else None
            elif self.pair_line is not None:
                r = l = False
            else:                                     # 모르면: 예전처럼 양옆 리모가 앞바퀴 옆
                r, l = neighbors_abeam(inp['clusters'], p['abeam_tol_mm'], 80.0, p['side_max_mm'],
                                       p['passed_ok'])
            if not (r and l) and self.pair_line is not None and turned_ok:
                # 카메라: 칸 끝 선(앞줄=줄 사이 흰 선, 뒷줄=바깥 노란 테두리)까지 남은 거리
                rem = self._end_remaining(inp)
                if rem is not None and rem <= p['end_stop_mm'] + p['enter_brake_mm']:
                    r = l = True
                    self.stop_how = 'camera'
                # 안전: 칸 가운데를 이만큼 지나면 무조건 멈춤 (칸을 벗어나 한참 가는 것 방지)
                cell = self._cell()
                if not (r and l) and cell is not None and self._body_s(inp, cell) >= p['cell_cap_mm']:
                    r = l = True
                    self.stop_how = 'cap'
            if not (r and l) and p['stop_from_col5_mm'] > 0 and self._turned_deg(inp) >= p['col5_min_turn_deg']:
                # 11/12 캡틴: 5번(정면) 앞 stop_from_col5_mm에서 정지 (11/13 path: R = 앞범퍼 30cm).
                # 11/13: 원이 끝나기 전에 R에 닿으면 덜 돈 채 섬(8~19도) -> stop_turn_deg 넘게 돈 뒤에만,
                # 단 stop_hard_mm보다 가까우면 몇 도든 정지
                gap = self._front_gap(inp)
                if gap is not None and (
                        (gap <= p['stop_from_col5_mm'] + p['enter_brake_mm'] and self._turned_deg(inp) >= p['stop_turn_deg'])
                        or gap <= p['stop_hard_mm'] + p['enter_brake_mm']):
                    r = l = True
                    self.stop_how = 'col5'
            if (not (r and l) and self._turned_deg(inp) >= 10.0 and self.realigns >= p['max_realign']
                    and p['guard_stop_mm'] > 0 and self._cell() is not None
                    and self._side_gap(inp) < p['guard_stop_mm']):
                # 재정렬을 끈 경우 옆 감시: 리모에 guard_stop_mm보다 가까우면 박기 전에 멈춤.
                # 11/13 실차: 그 자리에서 끝내면 칸에 반쯤 걸친 채 끝 -> 멈추고 back_mm 후진 뒤 다시 (guard_back_max번까지)
                gap = round(self._side_gap(inp))
                if p.get('guard_back', False) and self.guard_backs < p.get('guard_back_max', 0):
                    self.guard_backs += 1
                    self.err = ('guard_back', gap, self.guard_backs)
                    return self._switch('BACK', inp)
                self.err = ('guard_stop', gap)
                return self._done('GUARD_STOP', inp)
            if (not (r and l) and self._turned_deg(inp) >= 10.0 and self.realigns < p['max_realign']
                    and self._guard_hit(inp)):
                # 옆 감시: 꺾는 중 리모(보이는 것 + 기억한 것)에 side_guard_mm보다 가까워지면 멈추고 재정렬
                self.line = self._cell()
                self.realigns += 1
                self.err = ('guard', round(self._side_gap(inp)))
                self._enter('PULLOUT', inp)
                return self._out(0.0, 0.0)
            if r and l:
                self._enter('CHECK', inp)
            else:
                straightened = (p['straighten_yaw_deg'] > 0
                                and self._turned_deg(inp) >= p['straighten_yaw_deg'])
                if (inp['blocked'] and straightened and self.pair_line is not None
                        and self.realigns < p['max_realign']):
                    self.line = self.pair_line         # 갇힘: 기억한 칸 가운데 선으로 재정렬
                    self.realigns += 1
                    self.err = ('stuck',)
                    self._enter('PULLOUT', inp)
                    return self._out(-p['realign_speed'], 0.0)
                if inp['blocked']:
                    self.backs += 1
                    if self.backs > p['max_backs']:
                        return self._done('FAIL_TOO_MANY_BACKS', inp)
                    return self._switch('BACK', inp)
                if (p.get('center_on_sides', False) and self.side_line is not None
                        and self._turned_deg(inp) >= p['stop_min_turn_deg']):
                    # 11/13: 양옆 리모가 보이면 그 정중앙 선을 따라 들어감 (예전엔 깊이에만 쓰고 조향은 칸 추정선·5번)
                    k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self.side_line,
                                              p['pursuit_look_mm'])
                    return self._out(p['turn_speed'], max(-2.0, min(2.0, k)))
                if self.turn_travel >= p['turn_max_mm']:
                    return self._done('FAIL_TURN_TOO_LONG', inp)
                straight = (p['straighten_yaw_deg'] > 0
                            and self._turned_deg(inp) >= p['straighten_yaw_deg'])
                if not straight:
                    fix = self._turn_fix(inp, max(0.0, moved))
                    if fix is not None:
                        return fix
                col5_ok = p['follow_col5'] and self.col5 is not None and self.col5_n >= p['col5_min_frames']
                if col5_ok and (straight or p['col5_early']):
                    # 바퀴 편 뒤: 5번 세로선(× 1·3번 가로선)을 따라 들어감
                    k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self._cell(),
                                              p['pursuit_look_mm'])
                    return self._out(p['turn_speed'], max(-2.0, min(2.0, k)))
                if straight:
                    return self._out(p['turn_speed'], self._align_kappa(inp))
                return self._out(p['turn_speed'], KAPPA_FULL_RIGHT)

        if self.phase == 'CHECK':           # (재정렬) 칸 가운데에 반듯하게 들어갔나
            rel = cell_line_from_neighbors(
                in_view(inp['clusters'], p['front_axle_from_lidar_mm'], p['side_view_deg']), p['side_max_mm'])
            if p.get('center_on_sides', False) or p.get('center_check', False):
                # 11/14: center_check만 켜도(center_on_sides 끔 = 원은 접선까지) 끝에서 양옆 정중앙을 잰다.
                #   예전엔 center_on_sides일 때만 재서 center_check가 아무 일도 안 했다
                cr = self._corridor_rel(inp)
                if cr is not None:
                    self.side_line = self._side_mid(cr, inp)
            if p.get('center_check', False) and self.side_line is not None:
                # 11/13: '칸 안이면 끝'만 보던 것 -> 양옆 리모 정중앙에서 final_lat_tol_mm 넘게 벗어났으면 뒤로 빼서 다시
                off = self._center_offset(self.side_line, inp)
                self.center_off = off
                if abs(off) > p.get('center_tol_mm', p['final_lat_tol_mm']) and self.center_tries < p.get('center_retries', 0):
                    # 뒤로 빠졌다가(PULLOUT) 정중앙 선을 따라 다시 들어감(ENTER). 0.5초씩 멈춤
                    self.center_tries += 1
                    self.err = ('center', round(off), self.center_tries)
                    # 11/15: 많이 벗어났을수록 더 빼서 옆으로 옮길 거리를 확보 (1114_03: 450 mm로 2번 해도 64 mm 남음)
                    base = p.get('center_pullout_mm', p['pullout_mm'])
                    self.center_pull_len = min(max(base, p.get('center_pullout_max_mm', base)),
                                               base + p.get('center_pullout_per_mm', 0.0) * abs(off))
                    self.stop_how = None
                    self.line = self._centered_line()
                    return self._switch('PULLOUT', inp)
            if p['max_realign'] <= 0 or (rel is None and self.pair_line is None):
                return self._done('PARKED', inp)
            if rel is not None:
                self.side_line = self._line_to_world(rel, inp)
                self.side_seen = True
            self.line = self._cell()                # 옆 위치·방향 = 짝, 깊이 = 옆면(봤으면)
            lat, head, s_deep, s_shallow = self._misalign(self.line, inp)
            self.err = (round(lat), round(head, 1), round(s_deep), round(s_shallow))
            lim_d = p['cell_half_depth_mm'] - p['corner_margin_mm']
            inside = (lat <= p['cell_half_mm'] - p['corner_margin_mm'] and abs(head) <= p['realign_head_deg']
                      and s_deep <= lim_d and s_shallow >= -lim_d)
            off = self._center_offset(self.line, inp)
            neat = abs(head) <= p['final_head_tol_deg'] and abs(off) <= p['final_lat_tol_mm']
            self.err = self.err + (round(off),)
            if inside and neat:
                return self._done('PARKED', inp)
            if self.realigns >= p['max_realign']:
                # 횟수를 다 썼으면: 네 모서리가 칸 안이면 그대로 끝 (점수는 같음), 아니면 NOT_ALIGNED
                return self._done('PARKED' if inside else 'PARKED_NOT_ALIGNED', inp)
            self.realigns += 1
            self._enter('PULLOUT', inp)

        if self.phase == 'PULLOUT':         # (재정렬) 칸 가운데 선을 따라 뒤로 빠지기
            moved = inp['dist_mm'] - self.d_phase
            if moved >= ((self.center_pull_len or p.get('center_pullout_mm', p['pullout_mm']))
                         if self.center_tries else p['pullout_mm']):
                self.seen_ahead = False
                return self._switch('ENTER', inp)
            else:
                # 사람이 빼듯이: 앞범퍼가 옆 리모 끝을 지나기 전엔 바퀴 똑바로, 지나면 칸 가운데 선 따라 꺾으며 후진.
                # (처음부터 꺾으면 앞이 옆으로 휘둘려 옆 리모에 닿음 — 11/06 시뮬레이션)
                k = 0.0
                if p['pullout_steer'] and self._front_s(inp) <= -(p['parked_half_len_mm'] + p['pullout_clear_mm']):
                    k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self.line,
                                              self._look(), reverse=True)
                return self._out(-p['realign_speed'], max(-2.0, min(2.0, k)))

        if self.phase == 'ENTER':           # (재정렬) 칸 가운데 선을 따라 다시 들어가기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                return self._done('BLOCKED', inp)
            self._update_col5(inp)
            if p.get('center_on_sides', False) and self.center_tries:
                cr = self._corridor_rel(inp)                  # 다시 들어가는 중에도 양옆 정중앙을 계속 잼
                if cr is not None:
                    self.side_line = self._side_mid(cr, inp)
                self.line = self._centered_line()
            elif self.col5 is not None and self.col5_n >= p['col5_min_frames']:
                self.line = self._cell()                      # 5번 세로선이 생기면 그 선으로 따라감
            if self._body_s(inp) >= -p['enter_brake_mm']:      # 차체 중심이 칸 가운데에 옴
                self._enter('CHECK', inp)
                return self._out(0.0, 0.0)
            if self._guard_hit(inp):                          # 옆 감시
                if self.realigns >= p['max_realign']:
                    return self._done('GUARD_STOP', inp)
                self.realigns += 1
                self.err = ('guard', round(self._side_gap(inp)))
                self._enter('PULLOUT', inp)
                return self._out(0.0, 0.0)
            if moved >= max(p['pullout_mm'], self.center_pull_len or 0.0) + p['enter_max_extra_mm']:
                return self._done('FAIL_ENTER_TOO_LONG', inp)
            k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self.line, self._look())
            return self._out(p['realign_speed'], max(-2.0, min(2.0, k)))

        return self._out(0.0, 0.0)          # ⑥ DONE

    # ------------------------------------------------------------------
    # 11/16 캡틴 '고정 거리' 방식 (turn_ref: fixed). 라이다 판단 없이 선 + 오도메트리 + IMU만 씀.
    # ① F_LINE   차선 따라가며 주차선(흰 가로선, 처음엔 25cm보다 먼 것만)을 찾고, 앞바퀴가 선 fixed_line_target_mm 앞에 오면 정지
    #            (선이 카메라 아래로 사라지면 마지막으로 본 거리 - 오도메트리로 셈. fsm.StopApproach와 같은 원리)
    # ② F_PAUSE  fixed_pause_sec 정지 (= 주차 모드로 바뀜). 11/19: 막힌 칸 2개 안 보이면 가짜 선 -> ①로
    # ③ F_MOVE   멈춘 자리부터 오도메트리 fixed_move_mm 직진 (멈출 때 방향 유지)
    # ④ F_TPAUSE fixed_turn_pause_sec 정지 + 끝 presteer_sec 동안 오른쪽 최대 조향 미리 꺾기
    # ⑤ F_TURN   오른쪽 최대 조향 전진. fixed_turn_deg(IMU) 돌거나 fixed_turn_mm(>0이면) 가면 끝
    # ⑥ F_CHECK  앞 50cm 확인 (11/19 1·2행 랜덤): 있으면 앞칸 / 없으면 F_STR 뒷칸으로 fixed_backrow_mm 직진
    # ⑦ F_BPAUSE·F_BACK  멈추고 바퀴 펴서 fixed_back_mm(5cm) 똑바로 후진 (11/18 캡틴, 앞칸만. 뒷칸은 fixed_back_after_backrow)
    # ⑦ DONE     PARKED_FIXED (11/18 공지: 2행 주차 없음). ③·⑤ 중 앞범퍼 front_stop_mm 안에 물체 -> ③은 기다림, ⑤는 그 자리 정지(PARKED_FIXED_BLOCKED)
    # ------------------------------------------------------------------
    def _step_fixed(self, inp):
        p = self.p
        if self.phase == 'WAIT_LINE':
            self.trigger = 'fixed'
            self.turn_ref_used = 'fixed'
            self.lane_cfg = {'target_offset_mm': p['target_offset_mm']}   # (publish_lane_cfg일 때만 보냄. 보통 미션 노드가 50 보냄)
            self.line_seq0 = inp.get('lane_seq')
            self._enter('F_LINE', inp)
        moved = inp['dist_mm'] - self.d_phase

        if self.phase == 'F_LINE':
            lines = [d for d in inp['stoplines_mm'] if d >= p['line_min_mm']]
            if lines:
                if self.end_d is None:
                    # 처음 잡는 선은 fixed_first_min_mm보다 멀리서 보인 것만 (11/16 런 가이드):
                    #   통주행 M4 시작 때 횡단보도 끝 선이 약 12cm 앞이라 버리고, 연습(주차 앞 선 30cm 앞에 놓고
                    #   set_state)은 선이 약 35cm 앞이라 잡는다. (예전 ignore_mm 300은 연습 때 선을 버려 지나쳤음)
                    far = [d for d in lines if d >= p['fixed_first_min_mm']]
                    cand = far[0] if far else None
                    # 11/20 시뮬: 주차 모드가 켜지는 순간 /lane/status가 예전 자리(순간이동 전)의 선 1636 mm를 들고 있어
                    #   그 '없는 선'을 잡고 주차장 한가운데서 섬. -> F_LINE에 들어온 뒤 새로 받은 차선 메시지가
                    #   fixed_line_fresh_n개 넘어야 처음 선을 잡음 (켜기 전 메시지는 안 씀. lane_seq 없으면 그냥 씀)
                    seq, seq0 = inp.get('lane_seq'), getattr(self, 'line_seq0', None)
                    if seq is not None and seq0 is not None and seq - seq0 < p['fixed_line_fresh_n']:
                        cand = None
                else:                                     # 이미 잡은 선과 가까운 것만 (다른 선으로 안 갈아탐)
                    pred = self.end_d - (inp['dist_mm'] - self.end_at)
                    near = [d for d in lines if abs(d - pred) < 150.0]
                    cand = min(near, key=lambda d: abs(d - pred)) if near else None
                if cand is not None:
                    self.end_d, self.end_at = cand, inp['dist_mm']
            if self.end_d is None:
                if moved >= p['fixed_line_max_mm']:
                    return self._done('FAIL_NO_LINE', inp)
                return self._out(inp['lane_v'], None)
            remain = self.end_d - (inp['dist_mm'] - self.end_at)
            if remain <= p['fixed_line_target_mm'] + p['fixed_brake_mm']:
                self.turn_at = {'line_mm': round(remain)}
                self._enter('F_PAUSE', inp)
                return self._out(0.0, 0.0)
            v = inp['lane_v']
            # 11/20 시뮬(simcheck M4): 선을 잡은 뒤(남은 380 mm) 주차 구역에서 차선 노드가 선을 놓쳐 /lane/cmd_vel = 0
            #   -> 그 속도를 그대로 따라 F_LINE에서 영원히 정지(116 s). 선을 잡았으면 차선 속도가 0이어도
            #   fixed_slow_speed로 '선 잡을 때까지 차선이 가던 방향'(IMU)을 지키며 남은 거리만큼 감. 앞에 물체면 기다림.
            if p['fixed_line_creep'] and v < p['fixed_line_creep_min_v']:
                if inp['blocked']:
                    return self._out(0.0, 0.0)
                if not getattr(self, 'creep_on', False):
                    self.creep_on = True
                    self.creep_n = getattr(self, 'creep_n', 0) + 1
                k = 0.0
                if inp['yaw'] is not None and getattr(self, 'creep_yaw', None) is not None:
                    k = -p['fixed_heading_gain'] * wrap(inp['yaw'] - self.creep_yaw)
                    k = max(-1.0, min(1.0, k))
                return self._out(p['fixed_slow_speed'], k)
            self.creep_on = False
            if inp['yaw'] is not None:
                self.creep_yaw = inp['yaw']                   # 차선이 살아 있을 때 방향 (선 놓치면 이 방향 유지)
            if remain < p['fixed_line_target_mm'] + 250.0:
                v = min(v, p['fixed_slow_speed'])          # 선 25cm 앞부터 천천히 (밀림 줄임)
            return self._out(v, None)

        if self.phase == 'F_PAUSE':
            # 11/17 빈 칸 찾기: 서 있는 동안 라이다 점을 앞줄 1·2·3번 칸 상자마다 세어 가장 빈 칸을 고른다
            if p['fixed_detect']:
                if not hasattr(self, 'cell_cnt') or self.cell_cnt is None:
                    self.cell_cnt, self.cell_frames, self.cell_occ = [0, 0, 0], 0, [0, 0, 0]
                    self.detect_extra = False
                pts = inp.get('points_far', [])
                for x, y in pts:                            # B. 칸마다 점 개수
                    c = self._cell_of(x, y)
                    if c is not None:
                        self.cell_cnt[c] += 1
                occ = set()                                 # A. 캡틴 방식: 덩어리 중심이 어느 칸 범위에 있나
                for cx, cy, n in self._blobs(pts):
                    c = self._cell_of(cx, cy)
                    if c is not None and n >= p['fixed_blob_min_pts']:
                        occ.add(c)
                for c in occ:
                    self.cell_occ[c] += 1
                self.cell_frames += 1
            wait = max(p['fixed_pause_sec'], p['fixed_detect_sec'] if p['fixed_detect'] else 0.0)
            if getattr(self, 'detect_extra', False):
                wait += p['fixed_detect_sec']
            if inp['t'] - self.t_phase >= wait:
                # 11/19 캡틴: 진짜 주차선인지 확인. 1·2·3번 중 2칸은 항상 막혀 있으므로 막힌 칸이 fixed_need_occ(2)개 밑이면
                # 가짜 선(횡단보도 끝 선 등. 신호등에서 일찍 출발 판정 -> 주차 모드가 일찍 켜졌을 때) -> 그 선 버리고 다시 찾기
                if self._fake_line():
                    self.rejects = getattr(self, 'rejects', 0) + 1
                    self.reject_log = getattr(self, 'reject_log', []) + [{'occ': self.occ_n, 'avg': self.cell_avg}]
                    self.end_d, self.end_at, self.cell_cnt, self.detect_extra = None, None, None, False
                    self.line_seq0 = inp.get('lane_seq')
                    self._enter('F_LINE', inp)
                    return self._out(inp['lane_v'], None)
                cell = self._pick_cell()
                if cell is None:                            # A·B 불일치 첫 번째 -> 1초 더 모음
                    self.detect_extra = True
                    return self._out(0.0, 0.0)
                self.fixed_cell = cell
                self.yaw_phase0 = inp['yaw']
                self._enter('F_MOVE', inp)
            else:
                return self._out(0.0, 0.0)
            moved = 0.0

        if self.phase == 'F_MOVE':
            if moved >= self._fixed_move() - p['fixed_brake_mm']:
                self.q_s = round(moved)
                self._enter('F_TPAUSE', inp)
                return self._out(0.0, 0.0)
            if inp['blocked']:
                return self._out(0.0, 0.0)                 # 앞에 뭐가 있으면 서서 기다림
            k = 0.0
            if inp['yaw'] is not None and getattr(self, 'yaw_phase0', None) is not None:
                k = -p['fixed_heading_gain'] * wrap(inp['yaw'] - self.yaw_phase0)   # 멈출 때 방향 유지
                k = max(-1.0, min(1.0, k))
            return self._out(p['fixed_move_speed'], k)

        if self.phase == 'F_TPAUSE':
            if inp['t'] - self.t_phase >= p['fixed_turn_pause_sec']:
                self.yaw_turn0 = inp['yaw']
                self._enter('F_TURN', inp)
                moved = 0.0
            elif self._presteer_due(p['fixed_turn_pause_sec'], inp) and not inp['blocked']:
                return self._out(p['presteer_speed'], KAPPA_FULL_RIGHT)
            else:
                return self._out(0.0, 0.0)

        if self.phase == 'F_TURN':
            deg = self._turned_deg(inp)
            by_deg = p['fixed_turn_deg'] > 0 and deg >= p['fixed_turn_deg']
            by_mm = p['fixed_turn_mm'] > 0 and moved >= p['fixed_turn_mm']
            if by_deg or by_mm:
                self.stop_how = 'deg' if by_deg else 'mm'
                self.err = ('turned', round(deg, 1), round(moved))
                self.yaw_str0 = inp['yaw']
                if p['fixed_backrow_mm'] > 0:              # 11/19 공지: 2행도 나옴 -> 앞 50cm 확인 (뒷칸이 비었나)
                    self._enter('F_CHECK', inp)
                    return self._out(0.0, 0.0)
                return self._finish_front(inp)
            if inp['blocked']:
                self.err = ('blocked', round(deg, 1), round(moved))
                return self._done('PARKED_FIXED_BLOCKED', inp)
            return self._out(p['fixed_turn_speed'], KAPPA_FULL_RIGHT)

        if self.phase == 'F_CHECK':
            # 11/19 (11/18 9a79315 되살림): 돌고 나서 앞범퍼 앞 fixed_check_mm(50cm), 차 폭 띠에 물체가 있으면 앞칸 완료.
            # 없으면 뒷칸이 빈 칸(그 앞칸도 비워 둠) -> fixed_backrow_mm(62cm) 더 직진
            if not hasattr(self, 'chk_hits') or self.chk_hits is None:
                self.chk_hits, self.chk_frames = 0, 0
            bump = p['front_bumper_from_axle_mm']
            n = sum(1 for x, y in inp.get('points_far', [])
                    if bump < x <= bump + p['fixed_check_mm'] and abs(y) <= p['fixed_check_half_mm'])
            self.chk_hits += 1 if n >= p['fixed_check_min_pts'] else 0
            self.chk_frames += 1
            if inp['t'] - self.t_phase < p['fixed_check_sec']:
                return self._out(0.0, 0.0)
            ahead = self.chk_hits * 2 >= self.chk_frames      # 절반 넘는 장면에서 보이면 '있음'
            self.row = 'front' if ahead else 'back'
            if ahead:
                return self._finish_front(inp)
            self._enter('F_STR', inp)
            return self._out(0.0, 0.0)

        if self.phase == 'F_STR':                          # 뒷칸으로 더 직진 (돈 끝 방향 유지)
            if moved >= p['fixed_backrow_mm'] - p['fixed_brake_mm']:
                if p['fixed_back_after_backrow'] and p['fixed_back_mm'] > 0:
                    self.d_back0 = inp['dist_mm']
                    self._enter('F_BPAUSE', inp)
                    return self._out(0.0, 0.0)
                return self._done('PARKED_FIXED_BACK', inp)
            if inp['blocked']:
                return self._done('PARKED_FIXED_BLOCKED', inp)
            k = 0.0
            if inp['yaw'] is not None and getattr(self, 'yaw_str0', None) is not None:
                k = max(-1.0, min(1.0, -p['fixed_heading_gain'] * wrap(inp['yaw'] - self.yaw_str0)))
            return self._out(p['fixed_move_speed'], k)

        if self.phase == 'F_BPAUSE':
            # 11/18 캡틴: 돈 자리에서 멈춤 -> 끝 presteer_sec 동안 아주 천천히 뒤로 굴리며 바퀴를 똑바로 폄
            # (브리지는 0.02 m/s 밑이면 조향을 안 보내 서서는 안 펴짐). 이 동안 움직인 것도 후진 거리에 넣음
            if inp['t'] - self.t_phase >= p['fixed_back_pause_sec']:
                self._enter('F_BACK', inp)
            elif self._presteer_due(p['fixed_back_pause_sec'], inp):
                return self._out(-p['presteer_speed'], 0.0)
            else:
                return self._out(0.0, 0.0)

        if self.phase == 'F_BACK':                         # 바퀴 편 채 똑바로 후진 fixed_back_mm
            back = inp['dist_mm'] - getattr(self, 'd_back0', inp['dist_mm'])
            if back >= p['fixed_back_mm'] - p['fixed_back_brake_mm']:
                self.back_done_mm = round(back)
                return self._done('PARKED_FIXED', inp)
            return self._out(-p['fixed_back_speed'], 0.0)

        return self._out(0.0, 0.0)                         # DONE

    def _cell_box(self, c):
        """앞줄 c번째 칸(0=1번)의 라이다 검사 상자 (앞바퀴 기준 mm, 앞 +, 왼쪽 +). 앞범퍼가 선에 닿아 선 자리 기준."""
        p = self.p
        r = list(p['fixed_cell_ranges_mm'])                 # 11/18 캡틴: 라이다 앞 거리 1번 500~1020 / 2번 1030~1530 / 3번 1540~2040
        lx = -p['front_axle_from_lidar_mm']                 # 앞바퀴 -> 라이다 (앞 +). 점은 앞바퀴 기준이라 라이다 거리 + lx
        return (r[2 * c] + lx, r[2 * c + 1] + lx, -p['fixed_cell_y_far_mm'], -p['fixed_cell_y_near_mm'])

    def _cell_of(self, x, y):
        """앞바퀴 기준 점(x, y)이 앞줄 몇 번째 칸 상자(0~2)에 드나. 없으면 None."""
        for c in range(3):
            x0, x1, y0, y1 = self._cell_box(c)
            if x0 <= x <= x1 and y0 <= y <= y1:
                return c
        return None

    def _blobs(self, pts):
        """각도 순서 점들 -> 덩어리 [(중심 x, 중심 y, 점 수)]. 이웃 점이 gap보다 멀면 나눔 (scan_clusters와 같은 원리)."""
        out, cur = [], []
        gap = self.p['fixed_blob_gap_mm']
        for q in pts:
            if cur and math.hypot(q[0] - cur[-1][0], q[1] - cur[-1][1]) > gap:
                out.append(cur)
                cur = []
            cur.append(q)
        if cur:
            out.append(cur)
        return [(sum(a for a, _ in c) / len(c), sum(b for _, b in c) / len(c), len(c)) for c in out]

    def _pick_cell(self):
        """빈 칸 (1~3). A(캡틴: 덩어리 없는 칸)와 B(점이 가장 적은 칸)를 같이 본다.
        같음 -> 그 칸 / A가 0개·2개 이상 -> B / 다름 -> 처음엔 None(1초 더), 두 번째면 B.
        강제(fixed_force_cell>0)면 그 칸, 감지 꺼짐이면 fixed_default_cell."""
        p = self.p
        if int(p['fixed_force_cell']) in (1, 2, 3):
            self.cell_src = 'force'
            return int(p['fixed_force_cell'])
        frames = getattr(self, 'cell_frames', 0)
        if not p['fixed_detect'] or not frames:
            self.cell_src = 'default'
            return int(p['fixed_default_cell'])
        avg = [n / float(frames) for n in self.cell_cnt]
        self.cell_avg = [round(a, 1) for a in avg]
        b = min(range(3), key=lambda c: avg[c]) + 1
        free = [c + 1 for c in range(3) if self.cell_occ[c] * 2 < frames]    # 절반 넘는 장면에서 덩어리가 없던 칸
        a = free[0] if len(free) == 1 else None
        self.cell_a, self.cell_b = a, b
        if a == b:
            self.cell_src = 'agree'
            return b
        if a is None:
            self.cell_src = 'a_fail_b'
            return b
        if not getattr(self, 'detect_extra', False):
            return None
        self.cell_src = 'mismatch_b'
        return b

    def _fake_line(self):
        """막힌 칸(A 덩어리 또는 B 점 fixed_occ_min_pts 이상)이 fixed_need_occ 밑이면 True (= 가짜 선).
        감지 꺼짐·강제 칸·버린 횟수 fixed_max_rejects 넘음이면 False (그대로 주차)."""
        p = self.p
        frames = getattr(self, 'cell_frames', 0)
        if (not p['fixed_detect'] or p['fixed_need_occ'] <= 0 or int(p['fixed_force_cell']) in (1, 2, 3)
                or not frames or getattr(self, 'rejects', 0) >= p['fixed_max_rejects']):
            return False
        avg = [n / float(frames) for n in self.cell_cnt]
        self.cell_avg = [round(a, 1) for a in avg]
        occ = [self.cell_occ[c] * 2 >= frames or avg[c] >= p['fixed_occ_min_pts'] for c in range(3)]
        self.occ_n = sum(occ)
        return self.occ_n < p['fixed_need_occ']

    def _finish_front(self, inp):
        """앞칸 끝: fixed_back_mm > 0이면 바퀴 펴고 후진(F_BPAUSE), 아니면 완료."""
        if self.p['fixed_back_mm'] > 0:                   # 11/18 캡틴: 바퀴 펴고 5cm 후진 뒤 완료
            self.d_back0 = inp['dist_mm']
            self._enter('F_BPAUSE', inp)
            return self._out(0.0, 0.0)
        return self._done('PARKED_FIXED', inp)

    def _fixed_move(self):
        p, c = self.p, getattr(self, 'fixed_cell', None)
        cells = list(p['fixed_cells_move_mm'])
        if c in (1, 2, 3) and len(cells) >= 3:
            return float(cells[c - 1])
        return p['fixed_move_mm']

    def _look(self):
        """선 따라가기 앞보기 거리. 가운데 다시 맞출 때는 짧게(옆으로 더 빨리 붙음)."""
        if self.center_tries and self.p.get('center_look_mm', 0.0) > 0.0:
            return self.p['center_look_mm']
        return self.p['pursuit_look_mm']

    def _align_kappa(self, inp):
        """바퀴 편 뒤 칸 방향(세로)과 나란해지게 좌우로 살짝 (11/16 사용자: 너무 꺾었으면 왼쪽으로도).

        칸 깊이 방향 = 짝 선(pair_line)의 u, 없으면 도는 시작 방향 - 90°. 곡률 = straight_align_gain x 방향 차 (왼쪽 +)."""
        p = self.p
        g = p.get('straight_align_gain', 0.0)
        if g <= 0.0 or inp['yaw'] is None:
            return 0.0
        if self.pair_line is not None:
            _, _, ux, uy = self.pair_line
            target = math.atan2(uy, ux)
        elif self.yaw_turn0 is not None:
            target = self.yaw_turn0 - math.pi / 2.0
        else:
            return 0.0
        err = math.atan2(math.sin(target - inp['yaw']), math.cos(target - inp['yaw']))
        lim = p.get('straight_align_max_kappa', 1.5)
        return max(-lim, min(lim, g * err))

    def _back_kappa(self):
        """BACK 곡률: 도는 중 바로잡기(끊어 넣기)면 정한 곡률, 아니면 back_kappa
        (0이고 back_reverse_scale > 0이면 11/14 기억후진: 앞으로 꺾던 조향의 반대 방향 조금)."""
        p = self.p
        if self.back_k is not None:
            return self.back_k
        k = p['back_kappa']
        if k == 0.0 and p.get('back_reverse_scale', 0.0) > 0.0:
            k = -p['back_reverse_scale'] * self.last_fwd_k
        return k

    def _presteer_due(self, pause_sec, inp):
        """멈춤 끝 presteer_sec 동안 아주 느리게 굴리며 다음 조향을 미리 꺾는다 (11/15).

        브리지는 0.02 m/s 미만이면 조향을 0으로 보내서, 섰다 출발할 때마다 바퀴가 0부터 꺾이며 원이 커졌다
        (1114 bag 주차 TURN 첫 구간 반경 717 mm, 그 뒤 590~620 mm). 이 동안 약 1 cm 움직인다."""
        ps = self.p.get('presteer_sec', 0.0)
        return ps > 0.0 and inp['t'] - self.t_phase >= max(0.0, pause_sec - ps)

    def _turn_fix(self, inp, moved):
        """도는 중 바로잡기 (11/15, 1114 bag): 최대 조향 원이 계획(arc_radius_mm)보다 크면 칸에서 왼쪽으로 치우친다.

        1114_02/03/04 주차 TURN: IMU로 잰 실제 회전반경 590~720 mm (계획 529) -> 02 끝 위치 156 mm 왼쪽.
        - turn_fix_from_deg부터 잰 '돈 거리 / 돈 각도'로 실제 반경 R을 구하고(바퀴가 꺾이는 동안은 빼고),
          이대로 90°까지 돌면 뒷바퀴가 칸 가운데 선에서 얼마나 지나칠지 = rear_s + R(1 - sin φ) 를 예측한다.
        - 지나칠 것(+) 같으면: 멈추고 왼쪽 최대 조향으로 짧게 후진 -> 뒤로 빠지면서 차 머리가 칸 쪽으로 더 돈다
          (한 번 끊어 넣기). 후진 각 Δ = asin(sin φ + e / 2R) - φ, 거리 = R Δ.
        - 덜 갈 것(-) 같으면: 바퀴를 펴고 |e| / cos φ 만큼 더 간 뒤 다시 꺾는다.
        반환: 이번 틱 명령 (바로잡는 중이 아니면 None)
        """
        p = self.p
        if self.straight_left > 0.0:
            self.straight_left -= moved
            if self.straight_left > 0.0:
                return self._out(p['turn_speed'], 0.0)
        if (not p.get('turn_fix', False) or self.pair_line is None or self.q_s is None
                or inp['yaw'] is None):
            return None
        phi_deg = self._turned_deg(inp)
        phi = math.radians(phi_deg)
        if self.r_meas is None:
            if self.r_ref is None:
                if phi_deg >= p['turn_fix_meas_from_deg']:
                    self.r_ref = (self.turn_travel, phi)
                return None
            dphi = phi - self.r_ref[1]
            if dphi >= math.radians(p['turn_fix_meas_span_deg']):
                r_now = (self.turn_travel - self.r_ref[0]) / dphi
                r_lo, r_hi = p['turn_fix_r_range_mm']
                if not (r_lo <= r_now <= r_hi):
                    return None                            # 미끄러짐·후진 섞임 등: 이번엔 안 고침
                if phi_deg < p['turn_fix_from_deg']:
                    return None
                self.r_meas = r_now
            else:
                return None
        if phi_deg > p['turn_fix_to_deg'] or self.fix_n >= p['turn_fix_max']:
            return None
        if self.fix_phi is not None and phi_deg < self.fix_phi + p['turn_fix_gap_deg']:
            return None
        r = self.r_meas
        e = self._rear_s() + r * (1.0 - math.sin(phi))
        self.fix_pred = round(e)
        if abs(e) <= p['turn_fix_tol_mm']:
            return None
        self.fix_n += 1
        self.fix_phi = phi_deg
        e *= p.get('turn_fix_gain', 1.0)              # 11/16: 잰 반경이 흔들려(1116_01 529 vs 실제 585~605) 예측의 일부만 고침
        if e > 0.0:
            target = min(0.99, math.sin(phi) + e / (2.0 * r))
            d = min(p['turn_fix_back_max_mm'], r * max(0.0, math.asin(target) - phi))
            if d < 10.0:
                return None
            self.err = ('turn_fix_back', round(e), round(d), round(r), round(phi_deg))
            self.back_k = -KAPPA_FULL_RIGHT          # 왼쪽 최대 조향으로 후진 = 머리가 칸 쪽으로 더 돎
            return self._switch('BACK', inp, back_len=d)
        d = min(p['turn_fix_straight_max_mm'], -e / max(0.2, math.cos(phi)))
        self.err = ('turn_fix_straight', round(e), round(d), round(r), round(phi_deg))
        self.straight_left = d
        return self._out(p['turn_speed'], 0.0)

    def _done(self, result, inp):
        """끝내기: 정지 + 차선 오프셋 원래대로."""
        self.result = result
        self.lane_cfg = {'target_offset_mm': 0.0}
        self._enter('DONE', inp)
        return self._out(0.0, 0.0)

    def _info(self):
        """/parking/state에 실을 내용."""
        return {'phase': self.phase, 'done': self.phase == 'DONE', 'result': self.result,
                'backs': self.backs, 'realigns': self.realigns, 'last_err': self.err,
                'back_mm': getattr(self, 'back_done_mm', None), 'rejects': getattr(self, 'rejects', 0), 'occ_n': getattr(self, 'occ_n', None), 'cell': getattr(self, 'fixed_cell', None), 'cell_src': getattr(self, 'cell_src', None), 'cell_a': getattr(self, 'cell_a', None), 'cell_b': getattr(self, 'cell_b', None), 'cell_avg': getattr(self, 'cell_avg', None), 'row': self.row, 'stop_how': self.stop_how, 'trigger': self.trigger, 'turn_ref': self.turn_ref_used, 'corner_s': None if self.corner_s is None else round(self.corner_s), 'corner_n': self.corner_n, 'turn_at': self.turn_at, 'target_s': None if self.target_s is None else round(self.target_s), 'target_src': self.target_src, 'n_c5': len(self.tgt_c5), 'n_pm': len(self.tgt_pm), 'q_s': self.q_s, 'r_meas': None if self.r_meas is None else round(self.r_meas), 'fix_n': self.fix_n, 'fix_pred': self.fix_pred,
                'end_mm': None if self.end_d is None else round(self.end_d),
                'creep': getattr(self, 'creep_on', False), 'creep_n': getattr(self, 'creep_n', 0)}


DEFAULTS = {
    'ignore_mm': 300.0,        # M4 시작 직후 이만큼은 흰 선 무시 (신호등 횡단보도 선 오인 방지)
    'line_min_mm': 60.0,
    'target_offset_mm': 50.0,  # ② 캡틴 값 "왼쪽 차선과 8cm" -> 차로 중심에서 왼쪽 50mm
                               #    (차로 안쪽 폭 480 가정: 240 - 80 - 차 반폭 110 = 50. 확인 필요)
    'follow_max_mm': 2500.0,   # 흰 선 뒤 이만큼 가도 리모를 못 찾으면 멈춤 (주차 구역 끝까지 약 2m)
    # 정차 리모로 인정하는 덩어리 크기 (긴 쪽, mm). 짝(1·3/4·6) 찾기에만 씀 (꺾는 시점을 정하는 것).
    # 리모 면은 220mm지만 라이다 시야 끝(약 ±105도)에 반쯤 걸린 리모가 60~130mm로 보이고, 짝은 이걸로 잡힐 때가 많다
    # (11/10 시뮬레이션: 80 이상이면 뒷줄·1번 놓침 상황에서 짝을 못 찾음). 그래서 60: 표지판 기둥·가는 막대(약 30)만 버림.
    # 사람 다리·라바콘(약 60~100)은 못 거름 -> 주차 구역 옆에 사람이 서 있지 않게
    'car_min_size_mm': 60.0,
    'side_far_mm': 1300.0,     # ③ 오른쪽 리모 찾는 거리 (앞줄 약 0.4m, 뒷줄 약 1.0m)
    'skip_wait_line': False,   # 11/12: true면 WAIT_LINE 건너뛰고 바로 FOLLOW(짝 찾기). 미션 노드 m4_direct와 같이 켬 (params)
    'follow_max_direct_mm': 3500.0,  # 바로 시작했을 때 짝 없이 이만큼 가면 멈춤. 도면: M4 시작 약 3.8m -> 꺾는 곳 6.56m, 차선 끝 8.13m
    'wait_blocked_before_pair': True,  # 짝 찾기 전 앞이 막히면 BLOCKED로 끝내지 말고 서서 기다림
    'turn_ref': 'pair',        # 꺾는 기준: 'pair'(짝 중점) | 'corner'(1번 모서리, 리모 폭 가정) | 'center1'(1번 앞면 가운데) | 'target'(꼭짓점 기준 꺾기) | 'path'(캡틴 경로: 정지·Q로 이동·원호, 11/13)
    # 11/16 캡틴 '고정 거리' 방식 (turn_ref: fixed): 주차선 정지 -> 0.3초 -> 104cm 직진 -> 오른쪽 최대 조향 -> (직진) -> 정지
    # 11/17 공지: 빈 칸 위치가 랜덤 (한 칸만 비움) -> 주차선에서 서 있는 동안 라이다로 앞줄 1·2·3번 중 빈 칸을 찾아 직진 거리만 바꿈
    'fixed_detect': True,
    'fixed_detect_sec': 1.0,       # 선에서 서서 라이다를 모으는 시간 (fixed_pause_sec보다 길면 이 시간)
    'fixed_cells_move_mm': [384.0, 870.0, 1378.0],   # 1·2·3번 칸 직진 거리 (11/17 선배 기록 끝 위치 - 59.6cm, 2번 870 실차)
    'fixed_cell_ranges_mm': [500.0, 1020.0, 1030.0, 1530.0, 1540.0, 2040.0],   # 11/18 캡틴: 칸별 라이다 앞 거리 범위 (실차 확인 필요)
    'fixed_cell_y_near_mm': 330.0, # 칸 상자 옆 범위 (오른쪽, 차 중심에서) 가까운 쪽
    'fixed_cell_y_far_mm': 750.0,  #   먼 쪽 (리모 앞면은 약 400~500)
    'fixed_detect_range_mm': 2500.0,
    'fixed_blob_gap_mm': 80.0,     # A(캡틴 방식) 덩어리 나누는 점 간격
    'fixed_blob_min_pts': 2,       # A: 이만큼 점이 있는 덩어리만 물체로 봄
    'fixed_default_cell': 2,       # 감지 꺼짐일 때
    'fixed_force_cell': 0,         # 1~3이면 감지 무시하고 그 칸 (시험용)
    'fixed_need_occ': 2,           # 11/19: 선에서 막힌 칸이 이만큼 안 보이면 가짜 선 -> 다시 찾기 (0 = 확인 안 함)
    'fixed_occ_min_pts': 2.0,      #   B: 장면당 점이 이만큼이면 막힌 칸 (A 덩어리와 둘 중 하나)
    'fixed_max_rejects': 3,        #   가짜 선을 이만큼 버리면 그다음 선은 확인 없이 주차
    'fixed_line_target_mm': 56.0,   # 앞바퀴가 주차선 이만큼 앞에서 정지 (11/16 캡틴: 앞범퍼가 선에 닿게 = front_bumper_from_axle_mm)
    'fixed_first_min_mm': 250.0,   # 처음 잡는 주차선은 앞바퀴에서 이보다 멀리 보인 것만 (횡단보도 끝 선 무시)
    'fixed_brake_mm': 0.0,     #   정지 명령 뒤 밀리는 거리 (미측정 -> 0). 실차에서 지나치면 그만큼 넣기
    'fixed_slow_speed': 0.10,  #   선 25cm 앞부터 이 속도 이하
    'fixed_line_max_mm': 4000.0,   # 주차 모드 시작 뒤 이만큼 가도 선을 못 보면 FAIL_NO_LINE (정지)
    'fixed_line_fresh_n': 2,       # 11/20: F_LINE에 들어온 뒤 새 /lane/status가 이만큼 와야 처음 선을 잡음 (켜기 전 낡은 선 무시). 0 = 바로
    'fixed_line_creep': True,      # 11/20: 선을 잡은 뒤 차선 속도가 0(선 놓침)이어도 fixed_slow_speed로 방향 유지하며 선까지 감
    'fixed_line_creep_min_v': 0.03,   #   차선 속도가 이보다 작으면 '차선이 멈춤'으로 봄 (m/s)
    'fixed_pause_sec': 0.3,    # 선에서 멈추는 시간
    'fixed_move_mm': 870.0,    # 멈춘 자리부터 직진 거리 (11/16 캡틴 실차 확정)
    'fixed_move_speed': 0.10,
    'fixed_heading_gain': 2.0,     # 직진 중 멈출 때 방향 유지 (1/m per rad)
    'fixed_turn_pause_sec': 0.3,   # 104cm 뒤 멈춤 (끝 presteer_sec 동안 조향 미리 꺾기). 0이면 멈추지 않고 바로 꺾음
    'fixed_turn_deg': 87.0,    # 오른쪽 최대 조향을 이만큼(IMU) 돌면 끝. 실차는 펴라 한 뒤 2~4도 더 돎 -> 약 90도
    'fixed_turn_mm': 0.0,      # >0이면 최대 조향으로 이만큼 가면 끝 (각도와 먼저 닿는 쪽). 11/16 캡틴 실차 확정: 0 (87도로 끝)
    'fixed_turn_speed': 0.12,
    'fixed_backrow_mm': 620.0,     # 11/19 공지(1·2행 랜덤): 돌고 앞 50cm가 비면 뒷칸으로 이만큼 더 직진 (칸 깊이 600 + 선 20, 실차 확인 필요). 0 = 안 봄
    'fixed_check_mm': 500.0,       #   돌고 나서 앞범퍼 앞 이 거리 안에 물체 -> 앞칸 완료
    'fixed_check_half_mm': 130.0,  #   검사 띠 반폭 (차 폭 220/2 + 2cm, 양옆 리모는 약 15cm 옆)
    'fixed_check_min_pts': 3,      #   한 장면에 점이 이만큼 있으면 '있음'
    'fixed_check_sec': 0.5,        #   확인 시간 (절반 넘는 장면에서 보이면 있음)
    'fixed_back_after_backrow': False,   # 뒷칸으로 들어간 뒤에도 5cm 후진할지 (11/19 캡틴: 실차 보고 결정, 보류)
    'fixed_back_mm': 50.0,         # 11/18 캡틴: 다 돌고 바퀴 펴고 이만큼 똑바로 후진한 뒤 완료 (0 = 안 함)
    'fixed_back_pause_sec': 0.5,   #   후진 전 멈춤 (끝 presteer_sec 동안 바퀴 펴기)
    'fixed_back_speed': 0.06,
    'fixed_back_brake_mm': 0.0,    #   정지 명령 뒤 밀림 (미측정)
    'turn_from_corner_mm': 140.0,  # corner: 라이다가 1번 모서리 선을 이만큼 지나면 꺾기.
                               #   = turn_lead 320 을 바꾼 값: 2번 가운데 = 모서리 + 410 (칸 52 - 리모 반폭 11), 앞바퀴 = 가운데 - 320,
                               #   라이다 = 앞바퀴 + 47 -> 모서리 + 137. 실차로 맞출 것
    'turn_from_center1_mm': 247.0,  # center1: 라이다가 1번 앞면 가운데를 이만큼 지나면 꺾기.
                               #   = 2번 가운데(1번 + 520) - turn_lead 320 + 라이다 47. 실차로 맞출 것
    'arc_radius_mm': 529.0,    # path: 최대 조향 때 뒷바퀴 축 가운데 회전 반경 (실측 곡률 1.89). Q = l5 - 이 값
    'path_stable_mm': 30.0,    # path: 목표 최근 5번 값이 이 안이면 '인식 다 함'
    'path_late_mm': 150.0,     # path: 인식이 늦어 뒷바퀴가 Q를 이만큼 지나면 그때 정지 (뒤로 돌아옴)
    'settle_sec': 1.0,         # path: 다 인식하면 서서 다시 재는 시간
    'move_q_speed': 0.08,      # path: Q까지 전진/후진 속도 m/s
    'move_q_brake_mm': 5.0,    # path: 0.08m/s 제동 거리 (0.18m/s 23mm 기준 환산)
    'q_tol_mm': 10.0,          # path: Q 도착 허용 오차
    'move_q_max_mm': 800.0,    # path: Q까지 이보다 많이 움직여야 하면 FAIL_Q
    'turn_lead_lidar_mm': 273.0,  # target: 라이다가 목표 이만큼 전에 오면 꺾기 (= turn_lead 320 - 라이다 47, 실차로 맞출 것)
    'target_max_range_mm': 1300.0,  # target: 5번·1번이 라이다에서 이 안일 때 잰 값만 (멀면 일부만 보임)
    'target_gate_mm': 250.0,
    'target_far_max_mm': 1300.0,  # target/path: 먼 쪽(3번) 중심이 이 안일 때만 짝 중점 사용 (덩어리 1.5m 끝에서 잘림)
    'target_near_max_deg': 95.0,  # target/path: 가까운 쪽(1번)이 라이다에서 이 각도 안일 때만 (시야 ±105도 끝에서 잘림)   # target: 짝으로 예상한 칸 가운데에서 차로 방향으로 이 안만
    'corner_min_pts': 4,       # 모서리 잴 때 필요한 차로 쪽 면 점 수
    'corner_gate_mm': 250.0,   # 모서리가 짝으로 예상한 위치(1번 중심 + 110)에서 이보다 멀면 버림 (11/13: 150 -> 250, 칸 폭만 한 상자)
    'corner_window_mm': 300.0, # 1번 중심에서 차로 방향으로 이 안의 점만 1번으로 봄 (11/13: 260 -> 300)
    'corner_max_range_mm': 600.0,  # 11/13: 모서리가 라이다에서 이 안일 때 잰 값만 씀 (멀면 앞면 일부만 보여 틀림)
    'corner_min_frames': 3,    # 가까이서 이만큼 잰 뒤에야 모서리 기준으로 꺾음
    'corner_median_n': 7,      # 최근 이만큼의 중앙값
    'stop_from_col5_mm': 0.0,  # 11/12: 꺾은 뒤(col5_min_turn_deg 이상) 앞범퍼 -> 정면 물체(5번) 이 거리면 정지. 0 = 끔
                               #   220: 리모가 칸 가운데면 5번 앞면 = 칸 가운데 + 459 -> 앞범퍼 239 (칸 끝 선 300 전 약 6cm)
    'stop_turn_deg': 0.0,      # col5 정지는 이만큼 돈 뒤에만 (0 = col5_min_turn_deg부터). path: 85
    'stop_hard_mm': 0.0,       # 이보다 가까우면 몇 도든 정지 (0 = 안 씀). path: 200
    'guard_stop_mm': 0.0,      # 재정렬을 다 썼거나 끈 경우, 꺾는 중 리모에 이보다 가까우면 GUARD_STOP 정지. 0 = 끔
    'publish_lane_cfg': True,  # 주차 노드가 /lane/config를 직접 보냄. 11/12: false (미션 노드가 오프셋까지 보냄, 1초마다 덮어쓰던 문제)
    'single_fallback': False,  # 짝 없이 오른쪽 물체 하나만 보고 꺾기 (V1 방식). 11/12: 표지판에 속아 1번 칸 충돌 -> 기본 끔
    'single_min_size_mm': 150.0,  # 켰을 때 이보다 작은 덩어리는 무시 (리모 옆면 220mm)
    'extra_mm': 200.0,         # (추가) 리모 옆 확인 후 더 갈 거리. 0 = 캡틴 원래 계획 (시뮬레이션: 0이면 0점)
    'brake_mm': 23.0,          # 0.18 m/s 제동 거리 (calib 4번)
    'pause_sec': 1.0,          # ③ 캡틴 값
    'turn_speed': 0.12,
    'straighten_yaw_deg': 87.0,  # (추가) 이만큼 돌면 바퀴 펴고 직진. 0 = 끔 (시뮬레이션: 끄면 5번 칸에서 계속 돎)
    'turn_max_mm': 2000.0,     # 안전: TURN에서 이만큼 움직여도 안 끝나면 멈춤
    'back_mm': 50.0,           # ④-1 캡틴 값 5cm 후진 (params: 11/13 150)
    'switch_pause_sec': 0.0,   # 11/13: 전진 <-> 후진 바꿀 때 멈춤 (params 0.5). 0 = 바로 바꿈 (예전)
    'center_on_sides': False,  # 11/13: 양옆 리모가 보이면(stop_min_turn_deg 이상 돈 뒤) 그 정중앙 선을 따라 조향 (params true)
    'center_smooth': 0.5,      #   정중앙 선 갱신 비율 (1 = 매번 새 값)
    'center_min_turn_deg': 70.0,   #   이만큼 돈 뒤부터 (덜 돌면 통로가 비스듬)
    'center_bands_mm': [-150.0, 170.0, 230.0, 420.0],   # 11/14: 첫 띠 0 -> -150 (1114_02: 칸 안에선 오른쪽 상자가 앞바퀴 뒤에만 보여 정중앙을 못 잼)   # [시작1, 끝1, 시작2, 끝2]. 11/13: ((..),(..))였을 때 ROS가 못 받아 노드가 켜지자마자 죽음.   #   통로 가운데를 재는 앞 두 띠 (앞바퀴 기준)
    'center_gap_min_mm': 650.0,    #   1·3번 사이 통로 폭 (도면: 칸 두 개 1040 - 리모 폭 220 = 820)
    'center_gap_max_mm': 1000.0,
    'center_max_dir_deg': 25.0,    #   두 띠로 구한 방향이 차와 이보다 틀어지면 안 믿음
    'center_check': False,     # 11/13: 끝에서 정중앙에서 final_lat_tol_mm 넘게 벗어났으면 center_back_mm 후진 뒤 다시 (params true)
    'center_retries': 2,
    # 11/15 도는 중 바로잡기 (_turn_fix). 경로(path) 방식에서만
    'presteer_sec': 0.4,              # 11/15: 멈춤 끝 이 시간 동안 다음 조향을 미리 (0 = 끔)
    'presteer_speed': 0.025,          #   그때 속도 (브리지가 조향을 보내는 최소 0.02 m/s 위)
    'straight_align_gain': 3.0,       # 11/16: 바퀴 편 뒤 칸과 나란하게 (방향 0.1 rad 틀리면 곡률 0.3). 0 = 예전(바퀴 0)
    'straight_align_max_kappa': 1.5,
    'turn_fix_gain': 0.7,             # 11/16: 예측 옆 오차의 이만큼만 고침
    'turn_fix': True,
    'turn_fix_meas_from_deg': 12.0,   # 바퀴가 다 꺾인 뒤부터 회전반경을 잼 (출발 직후는 바퀴가 따라오는 중)
    'turn_fix_meas_span_deg': 12.0,   #   이만큼 도는 동안의 거리로 잼
    'turn_fix_from_deg': 30.0,        # 이 각도부터 예측해서 바로잡음
    'turn_fix_to_deg': 65.0,          #   이 각도까지만 (늦으면 옆 리모에 가까움)
    'turn_fix_gap_deg': 12.0,         #   두 번째 바로잡기는 이만큼 더 돈 뒤
    'turn_fix_max': 2,
    'turn_fix_tol_mm': 35.0,          # 예측 옆 오차가 이보다 크면 바로잡음
    'turn_fix_back_max_mm': 200.0,
    'turn_fix_straight_max_mm': 150.0,
    'turn_fix_r_range_mm': [450.0, 900.0],   # 잰 반경이 이 밖이면 믿지 않음
    'center_tol_mm': 15.0,     #   끝에서 차체 중심이 양옆 정중앙에서 이보다 벗어나면 다시 맞춤 (시뮬레이션 18개: 30 -> 최대 30mm, 15 -> 최대 13mm)
    'center_back_mm': 200.0,
    'center_pullout_per_mm': 2.0,     # 11/15: 옆 오차 1 mm당 후진 거리 2 mm 더 (450 + 2 x |off|)
    'center_pullout_max_mm': 700.0,   #   최대
    'center_look_mm': 180.0,          #   다시 들어갈 때 앞보기 (기본 250보다 짧게)
    'center_pullout_mm': 450.0,    #   가운데 다시 맞출 때 뒤로 빠지는 거리 (앞범퍼가 옆 리모 끝을 지나 방향까지 고칠 몫)
    'guard_back': False,       # 11/13: 옆 감시(guard_stop_mm)에 걸리면 끝내지 말고 back_mm 후진 뒤 다시 (params true)
    'guard_back_max': 3,
    'back_speed': 0.10,
    'back_kappa': 0.0,         # 후진할 때 고정 곡률. 0이면 아래 back_reverse_scale로 직전 전진 조향의 반대 10% 조향
                               # (오른쪽 -값으로 후진하면 앞에서 돈 만큼 되돌아가니 쓰지 말 것)
    'back_reverse_scale': 0.10,  # 11/14: 기억후진 시 잘못 들었던 길로 갈 때 꺾었던 조향의 반대 방향 10% 조향
    'max_backs': 15,
    'abeam_tol_mm': 30.0,
    'side_max_mm': 700.0,
    'passed_ok': True,         # 지나친 것도 "옆"으로 인정 (False = 캡틴 원래 뜻 그대로)
    'front_stop_mm': 60.0,     # 앞범퍼 앞 이 거리 안 물체 = "앞에 리모 있음"
    'front_bumper_from_axle_mm': 56.0,   # 확인 필요 (줄자)
    'front_axle_from_lidar_mm': -47.0,   # 확인 필요 (calib 5번 해석)
    'laser_yaw': 0.0,          # 확인 필요 (팀 TODO)
    # ---- 두 리모 중점 (캡틴 아이디어)
    'pair_dist_mm': 1040.0,    # 두 정차 리모 사이 (칸 2개: 500 + 20 + 500 + 20, 도면)
    'pair_tol_mm': 150.0,      # 짝 인정 오차. 칸 1개(520)나 3개 간격과 헷갈리지 않게
    'pair_min_frames': 3,      # 짝을 이만큼 본 뒤에 믿는다 (한 번 튄 값 무시)
    'pair_trigger': True,      # 흰 선을 못 봐도 오른쪽 두 리모 짝을 보면 주차 모드 시작 (armed일 때만)
    'pair_trigger_frames': 3,  # 짝을 연속 이만큼 봐야 시작 (한 번 튄 값 무시)
    'turn_lead_mm': 320.0,     # 칸 가운데가 앞바퀴보다 이만큼 앞일 때 꺾기 시작 (회전반경 530 - 축거 210)
    'stop_min_turn_deg': 60.0, # 이만큼 돈 뒤에만 양옆 옆면으로 '칸 가운데 도착'을 판단 (도는 도중 오판 방지)
    # ---- 줄(앞줄 1~3 / 뒷줄 4~6)과 칸 끝에서 멈추기 (11/09 park_02 반영)
    'row_split_mm': 800.0,     # 짝 중점까지 옆 거리가 이보다 가까우면 앞줄 (앞줄 약 0.5m, 뒷줄 약 1.1m)
    'target_row': 'front',     # 들어갈 줄 고정: 'front'(2번) | 'back'(5번) | 'auto'(짝 줄로 판단). 11/10 대회 배치 = 2번
    'row_pitch_mm': 620.0,     # 앞줄·뒷줄 칸 가운데 사이 (칸 깊이 600 + 선 20, 도면)
    'pair_dir_from_heading': True,  # 칸 방향을 차선 추종 중 차 방향(IMU)으로. False = 두 리모 잇는 방향
    'follow_col5': True,       # 꺾은 뒤 정면 5번 리모로 세로 가운데선을 구해 따라 들어감 (윤서 아이디어 11/10). False = 끔
    'col5_min_turn_deg': 70.0, # 이만큼 돈 뒤부터 5번을 봄 (덜 돌았을 땐 5번 앞면이 비스듬해 가운데가 쏠림)
    'col5_gate_mm': 150.0,     # 짝으로 예상한 5번 위치에서 옆으로 이 안의 덩어리만 5번으로 인정
    'col5_min_frames': 3,      # 5번을 이만큼 본 뒤에 믿음
    'col5_early': True,        # 5번 세로선이 생기면 (87도 전이라도) 바로 그 선을 따라감. False = 87도 뒤부터
    'pair_depth_bias_mm': 125.0,  # 짝으로 본 칸 가운데가 실제보다 얕게 나오는 만큼 (시뮬레이션 80~170)
    'end_stop_mm': 195.0,      # 칸 가운데 정지 = 앞바퀴 -> 칸 끝 선 (깊이 반 300 - (축거 210 - 차체 중심 105))
    'end_gate_mm': 200.0,      # 칸 끝 선 예상 위치와 이만큼 안만 받음 (줄 사이 선과 바깥 테두리는 620mm 차이)
    'end_freeze_mm': 420.0,    # 남은 거리가 이보다 짧으면 카메라 갱신 끊고 오도메트리로만 (BEV 사각지대 약 350)
    'cell_cap_mm': 80.0,       # 차체 중심이 (추정한) 칸 가운데를 이만큼 지나면 무조건 멈춤. 칸 안 여유 약 140 - 추정 오차 여유 60
    'side_view_deg': 95.0,     # 칸 가운데 선은 라이다 각도 이 안의 리모로만 (실측 시야 끝 약 105도. 끝에 걸리면 덩어리 중심이 쏠림)
    # ---- 재정렬 (PULLOUT -> ENTER)
    'max_realign': 2,          # 0 = 재정렬 끔 (= 원래 버전 2)
    'final_head_tol_deg': 4.0, # 깔끔하게: 칸 선과 방향이 이보다 틀어지면 (모서리가 칸 안이어도) 재정렬
    'final_lat_tol_mm': 30.0,  # 깔끔하게: 차체 중심이 칸 가운데에서 이보다 옆으로 벗어나면 재정렬
    'side_guard_mm': 50.0,     # 옆 감시: 꺾는 중·다시 들어가는 중 리모(보이는 것+기억한 것)까지 이보다 가까우면 재정렬. 0 = 끔
    'cell_pitch_mm': 520.0,    # 칸 가운데 사이 옆 간격 (칸 폭 500 + 선 20, 도면)
    'cell_half_mm': 250.0,     # 칸 폭 500의 절반 (도면)
    'corner_margin_mm': 25.0,  # 모서리가 칸 선에서 이만큼 안쪽이어야 통과 (측정 오차 여유)
    'realign_head_deg': 20.0,  # 방향이 이보다 틀어지면 모서리가 괜찮아도 재정렬
    'body_length_mm': 322.0,   # LIMO 길이 (팀 parking_node 주석)
    'body_width_mm': 220.0,
    'pullout_mm': 650.0,       # 뒤로 빠지는 거리. 똑바로 빼서 앞범퍼가 옆 리모 끝을 지난 뒤 꺾으며 후진할 몫까지 (11/10)
    'pursuit_look_mm': 250.0,  # 선 따라가기 앞보기 거리
    'pullout_steer': True,     # 후진 때도 꺾기 (앞범퍼가 옆 리모 끝 + pullout_clear_mm를 지난 뒤부터). False = 늘 똑바로
    'pullout_clear_mm': 150.0, # 앞범퍼가 옆 리모 끝에서 이만큼 더 나온 뒤 꺾기 시작 (5cm면 차 옆면이 1번 리모 모서리에 1mm까지 닿음)
    'parked_half_len_mm': 161.0,   # 정차 리모 길이 322의 절반 (칸 가운데 기준 리모 끝 = ±161)
    'realign_speed': 0.10,
    'enter_brake_mm': 13.0,    # 0.10 m/s 제동 거리 (calib 4번: 약 13 x 속도 cm)
    'cell_half_depth_mm': 300.0,   # 칸 깊이 600의 절반 (도면). 정차 리모가 칸 가운데라고 가정
    'enter_max_extra_mm': 800.0,
    'wheelbase_mm': 210.0,     # 축거 실측 21cm (calib 3번)
    'body_center_from_rear_mm': 105.0,   # 뒷바퀴축 -> 차체 중심 (오버행 앞뒤 같다고 가정, 확인 필요)
}


# ---------------------------------------------------------------- ROS 노드
def main(args=None):
    """parking_node 실행. 팀 parking_node와 같은 이름·토픽 + /lane/config(오프셋)."""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import String

    class ParkingNode(Node):
        """ROS 껍데기: 센서를 모아 ParkingV2.step()에 넣고 /parking/cmd_vel을 낸다."""

        def __init__(self):
            super().__init__('parking_node')
            self.declare_parameter('enabled', False)
            self.declare_parameter('odom_topic', '/odom')
            self.declare_parameter('imu_topic', '/imu')
            self.declare_parameter('scan_topic', '/scan')
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v)
            self.mstate = {}
            self.dist_mm, self._last_xy, self.yaw = 0.0, None, None
            self.lane_v = self.lane_w = 0.0
            self.stoplines = []
            self.end_lines = []
            self.scan = None
            self.logic = None
            self._sent_cfg = None
            self.create_subscription(String, '/mission/state', self._on_state, 10)
            self.create_subscription(String, '/lane/status', self._on_lane_status, 10)
            self.create_subscription(Twist, '/lane/cmd_vel', self._on_lane_cmd, 10)
            self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                     self._on_odom, qos_profile_sensor_data)
            self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                     self._on_imu, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                     self._on_scan, qos_profile_sensor_data)
            self.cmd_pub = self.create_publisher(Twist, '/parking/cmd_vel', 10)
            self.state_pub = self.create_publisher(String, '/parking/state', 10)
            self.lane_cfg_pub = self.create_publisher(String, '/lane/config', 10)
            self.create_timer(0.05, self._tick)

        def _on_state(self, m):
            """/mission/state JSON 저장."""
            try:
                self.mstate = json.loads(m.data)
            except json.JSONDecodeError:
                pass

        def _on_lane_status(self, m):
            """/lane/status에서 흰 선 거리 목록만 꺼낸다."""
            try:
                d = json.loads(m.data)
                self.lane_seq = getattr(self, 'lane_seq', 0) + 1     # 11/20: 메시지 번호 (켜기 전 낡은 선 구분)
                self.stoplines = [float(x) for x in d.get('stoplines_mm', [])]
                # 칸 끝 선: 흰·노란 가로선 (옛 lane 노드면 흰 선만)
                self.end_lines = [float(x) for x in d.get('end_lines_mm', d.get('stoplines_mm', []))]
            except (json.JSONDecodeError, TypeError, ValueError):
                self.stoplines, self.end_lines = [], []

        def _on_lane_cmd(self, m):
            """차선 노드 명령 저장 (차선 따라가는 단계에서 그대로 넘김)."""
            self.lane_v, self.lane_w = m.linear.x, m.angular.z

        def _on_odom(self, m):
            """오도메트리로 누적 이동 거리(mm). 후진도 더한다."""
            pos = m.pose.pose.position
            if self._last_xy is not None:
                self.dist_mm += 1000.0 * math.hypot(pos.x - self._last_xy[0], pos.y - self._last_xy[1])
            self._last_xy = (pos.x, pos.y)

        def _on_imu(self, m):
            """IMU 방향(yaw) 저장."""
            self.yaw = yaw_from_quaternion(m.orientation)

        def _on_scan(self, m):
            """최신 라이다 스캔 저장."""
            self.scan = m

        def _tick(self):
            """20Hz: 주차 계산 -> /parking/cmd_vel, /parking/state, (바뀔 때만) /lane/config."""
            g = lambda k: self.get_parameter(k).value   # noqa: E731
            active = (self.mstate.get('state') == 'M4_PARKING'
                      and self.mstate.get('source') == 'parking')
            tw = Twist()
            info = {'phase': 'IDLE', 'done': False, 'result': ''}
            if active and g('enabled'):
                if self.logic is None:
                    self.logic = ParkingV2({k: g(k) for k in DEFAULTS})
                clusters, blocked, points, clusters_pair, clusters_box, clusters_single = [], False, [], [], [], []
                points_far = []
                if self.scan is not None:
                    s = self.scan
                    clusters = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                             s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                             max_dist_mm=max(1500.0, g('side_far_mm') + 200.0))
                    blocked = front_blocked(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                            s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                            g('front_bumper_from_axle_mm'), g('front_stop_mm'), 120.0)
                    points = scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                                         g('front_axle_from_lidar_mm'), g('laser_yaw'))
                    points_far = scan_points(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                                             g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                             max_dist_mm=g('fixed_detect_range_mm')) if g('turn_ref') == 'fixed' else []
                    clusters_pair = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                                  s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                                  max_dist_mm=max(1500.0, g('side_far_mm') + 200.0), x_mode='extent',
                                                  min_size_mm=g('car_min_size_mm'))
                    clusters_single = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                                    s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                                    max_dist_mm=max(1500.0, g('side_far_mm') + 200.0),
                                                    min_size_mm=g('single_min_size_mm'))
                    clusters_box = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                                 s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                                 max_dist_mm=1500.0, x_mode='box', min_size_mm=g('car_min_size_mm'))
                v, k, info = self.logic.step({
                    't': time.monotonic(), 'dist_mm': self.dist_mm, 'yaw': self.yaw,
                    'lane_v': self.lane_v, 'lane_w': self.lane_w, 'stoplines_mm': self.stoplines,
                    'lane_seq': getattr(self, 'lane_seq', 0),
                    'end_lines_mm': self.end_lines, 'armed': bool(self.mstate.get('parking_armed', True)),
                    'clusters': clusters, 'blocked': blocked, 'points': points, 'points_far': points_far, 'clusters_pair': clusters_pair, 'clusters_single': clusters_single,
                    'clusters_box': clusters_box})
                tw.linear.x = float(v)
                tw.angular.z = float(self.lane_w if k is None else v * k)
                if not g('publish_lane_cfg'):
                    pass
                elif self.logic.lane_cfg is not None and self.logic.lane_cfg != self._sent_cfg:
                    self.lane_cfg_pub.publish(String(data=json.dumps(self.logic.lane_cfg)))
                    self._sent_cfg = dict(self.logic.lane_cfg)
            elif not active:
                if self._sent_cfg and self._sent_cfg.get('target_offset_mm', 0.0) != 0.0:
                    self.lane_cfg_pub.publish(String(data=json.dumps({'target_offset_mm': 0.0})))
                self._sent_cfg = None
                self.logic = None
            self.cmd_pub.publish(tw)
            if self.logic is not None and info.get('done') and not self.logic.done_logged:
                # 11/13 캡틴: 주차 끝나면 터미널(실행 창)에 띄우기
                self.logic.done_logged = True
                self.get_logger().info(f"주차 완료: result={info.get('result')} stop_how={info.get('stop_how')} "
                                       f"turn_ref={info.get('turn_ref')} turn_at={info.get('turn_at')}")
            info.update({'active': active, 'version': '2P', 'stamp': time.monotonic()})
            self.state_pub.publish(String(data=json.dumps(info)))

    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
