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
│   └─ armed이고 (흰 선 보임 또는 1·3번 짝 연속 3번) → 주차 모드
│
├─② FOLLOW: 차선 따라가기, 왼쪽 차선과 8cm (target_offset_mm 50)
│   ├─③ 라이다: 오른쪽 리모 두 대 짝 찾기 (사이 약 104cm) → 중점 = 목표 칸 가운데
│   │   └─ 중점이 앞바퀴보다 32cm 앞에 오면 → PAUSE
│   └─ (예비) 짝 못 찾고 첫 리모가 앞바퀴 옆 → EXTRA: 0.2m 더 → PAUSE
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
                  gap_mm=80.0, min_pts=3, max_size_mm=450.0, max_dist_mm=1500.0, x_mode='mean'):
    """라이다 스캔 -> 물체 덩어리 중심 목록 [(x, y)] (앞바퀴 기준, 앞 +, 왼쪽 +, mm).

    가까운 점끼리 묶고(gap_mm), LIMO 크기(max_size_mm)보다 큰 덩어리(벽 등)는 버린다.
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
        if max(max(xs) - min(xs), max(ys) - min(ys)) > max_size_mm:
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
        self.trigger = None        # 주차 모드 시작 이유: 'line' | 'pair'
        self.pair_side_mm = None   # 짝 중점까지 옆 거리 (앞줄 약 0.5m, 뒷줄 약 1.1m) -> row
        self.row = None            # 'front' | 'back' (정차 리모가 있는 줄 = 들어갈 줄)
        self.side_seen = False
        self.side_line = None      # 양옆 옆면으로 구한 선 (깊이에만 씀)     # 꺾은 뒤 양옆 리모로 칸 가운데 선을 한 번이라도 구했나
        self.end_d = self.end_at = None   # 칸 끝 선(카메라): 마지막으로 본 거리, 그때 누적 거리
        self.stop_how = None       # 'side' | 'memory' | 'camera' | 'cap' (어떻게 멈췄나, 디버그)
        self.err = None            # 마지막 재정렬 판단 값 (옆 mm, 방향 도)

    def _enter(self, phase, inp):
        """단계 바꾸기: 시작 시각·거리·방향 기록."""
        self.phase, self.t_phase, self.d_phase, self.yaw_phase = phase, inp['t'], inp['dist_mm'], inp['yaw']

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
        """명령을 내보내며 마지막 속도 부호를 기억 (위치 계산용)."""
        self._last_v = v
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
        moved = inp['dist_mm'] - self.d_phase

        if self.phase == 'WAIT_LINE':       # ① 흰 선 기다림 (차선 노드 명령 넘김)
            lines = [d for d in inp['stoplines_mm'] if d >= p['line_min_mm']]
            # armed: 미션 노드가 주차 표지를 봤거나 도면상 주차 앞 선 근처 (/mission/state parking_armed)
            # 11/10: 표지를 흰 선 지난 뒤에 확정하면 선을 놓친다 -> 오른쪽 두 리모 짝(1·3번)을 연속으로 보면 그것도 시작 신호.
            # 짝은 같은 줄 두 대가 약 104cm 떨어져야 생겨서, 커브 등에 한 대만 서 있는 리모로는 안 켜진다.
            armed = inp.get('armed', True)
            seen = (p['pair_trigger'] and armed and find_pair(
                inp.get('clusters_pair') or inp['clusters'], p['side_far_mm'], p['pair_dist_mm'],
                p['pair_tol_mm']) is not None)
            self.trig_n = self.trig_n + 1 if seen else 0
            by_line = moved >= p['ignore_mm'] and lines and armed
            by_pair = self.trig_n >= p['pair_trigger_frames']
            if by_line or by_pair:
                self.trigger = 'line' if by_line else 'pair'
                self.lane_cfg = {'target_offset_mm': p['target_offset_mm']}
                self._enter('FOLLOW', inp)
            else:
                return self._out(inp['lane_v'], None)

        if self.phase == 'FOLLOW':          # ② 왼쪽 붙어 차선 따라가기, ③ 오른쪽 리모 찾기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                return self._done('BLOCKED', inp)
            pair = find_pair(inp.get('clusters_pair') or inp['clusters'], p['side_far_mm'], p['pair_dist_mm'],
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
                if self._ahead_of_front(self.pair_line, inp) <= p['turn_lead_mm'] + p['brake_mm']:
                    self.line = self.pair_line
                    self._enter('PAUSE', inp)
                    return self._out(0.0, 0.0)
                return self._out(inp['lane_v'], None)
            right = [(x, y) for x, y in inp['clusters'] if -p['side_far_mm'] <= y <= -80.0]
            if any(x > p['abeam_tol_mm'] for x, _ in right):
                self.seen_ahead = True
            hit = any(abs(x) <= p['abeam_tol_mm'] or (p['passed_ok'] and -400.0 < x < 0.0)
                      for x, _ in right)
            if self.seen_ahead and hit:
                self._enter('EXTRA', inp)
            elif moved >= p['follow_max_mm']:
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

        if self.phase == 'PAUSE':           # ③ 1초 정지
            if inp['t'] - self.t_phase >= p['pause_sec']:
                self.yaw_turn0 = inp['yaw']
                self._enter('TURN', inp)
            else:
                return self._out(0.0, 0.0)

        if self.phase == 'BACK':            # ④-1 앞이 막혀서 back_mm 후진
            moved = inp['dist_mm'] - self.d_phase
            if moved >= p['back_mm']:
                self._enter('TURN', inp)
            else:
                return self._out(-p['back_speed'], p['back_kappa'])

        if self.phase == 'TURN':            # ④ 오른쪽 최대 조향 전진 + ⑤ 양옆 확인까지 반복
            moved = inp['dist_mm'] - self.d_phase
            self.turn_travel += max(0.0, moved)
            self.d_phase = inp['dist_mm']
            self._update_col5(inp)
            seen = in_view(inp['clusters'], p['front_axle_from_lidar_mm'], p['side_view_deg'])
            side = (cell_line_from_neighbors(seen, p['side_max_mm'])
                    if self._turned_deg(inp) >= p['stop_min_turn_deg'] else None)
            turned_ok = self._turned_deg(inp) >= p['stop_min_turn_deg']
            if self.pair_line is not None and side is not None:
                # 짝을 알고 양옆 리모 옆면이 보이면: 옆면으로 구한 선에서 차체 중심이 칸 가운데 오면 멈춤.
                # (차선에서 본 짝은 리모의 차선 쪽 끝면만 보여 깊이가 8~17cm 얕게 나온다 -> 깊이는 옆면으로)
                self.line = self._line_to_world(side, inp)
                self.side_line = self.line
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
                    self._enter('BACK', inp)
                    return self._out(-p['back_speed'], p['back_kappa'])
                if self.turn_travel >= p['turn_max_mm']:
                    return self._done('FAIL_TURN_TOO_LONG', inp)
                straight = (p['straighten_yaw_deg'] > 0
                            and self._turned_deg(inp) >= p['straighten_yaw_deg'])
                col5_ok = p['follow_col5'] and self.col5 is not None and self.col5_n >= p['col5_min_frames']
                if col5_ok and (straight or p['col5_early']):
                    # 바퀴 편 뒤: 5번 세로선(× 1·3번 가로선)을 따라 들어감
                    k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self._cell(),
                                              p['pursuit_look_mm'])
                    return self._out(p['turn_speed'], max(-2.0, min(2.0, k)))
                return self._out(p['turn_speed'], 0.0 if straight else KAPPA_FULL_RIGHT)

        if self.phase == 'CHECK':           # (재정렬) 칸 가운데에 반듯하게 들어갔나
            rel = cell_line_from_neighbors(
                in_view(inp['clusters'], p['front_axle_from_lidar_mm'], p['side_view_deg']), p['side_max_mm'])
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
            if moved >= p['pullout_mm']:
                self._enter('ENTER', inp)
                self.seen_ahead = False
            else:
                # 사람이 빼듯이: 앞범퍼가 옆 리모 끝을 지나기 전엔 바퀴 똑바로, 지나면 칸 가운데 선 따라 꺾으며 후진.
                # (처음부터 꺾으면 앞이 옆으로 휘둘려 옆 리모에 닿음 — 11/06 시뮬레이션)
                k = 0.0
                if p['pullout_steer'] and self._front_s(inp) <= -(p['parked_half_len_mm'] + p['pullout_clear_mm']):
                    k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self.line,
                                              p['pursuit_look_mm'], reverse=True)
                return self._out(-p['realign_speed'], max(-2.0, min(2.0, k)))

        if self.phase == 'ENTER':           # (재정렬) 칸 가운데 선을 따라 다시 들어가기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                return self._done('BLOCKED', inp)
            self._update_col5(inp)
            if self.col5 is not None and self.col5_n >= p['col5_min_frames']:
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
            if moved >= p['pullout_mm'] + p['enter_max_extra_mm']:
                return self._done('FAIL_ENTER_TOO_LONG', inp)
            k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self.line,
                                      p['pursuit_look_mm'])
            return self._out(p['realign_speed'], max(-2.0, min(2.0, k)))

        return self._out(0.0, 0.0)          # ⑥ DONE

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
                'row': self.row, 'stop_how': self.stop_how, 'trigger': self.trigger,
                'end_mm': None if self.end_d is None else round(self.end_d)}


DEFAULTS = {
    'ignore_mm': 300.0,        # M4 시작 직후 이만큼은 흰 선 무시 (신호등 횡단보도 선 오인 방지)
    'line_min_mm': 60.0,
    'target_offset_mm': 50.0,  # ② 캡틴 값 "왼쪽 차선과 8cm" -> 차로 중심에서 왼쪽 50mm
                               #    (차로 안쪽 폭 480 가정: 240 - 80 - 차 반폭 110 = 50. 확인 필요)
    'follow_max_mm': 2500.0,   # 흰 선 뒤 이만큼 가도 리모를 못 찾으면 멈춤 (주차 구역 끝까지 약 2m)
    'side_far_mm': 1300.0,     # ③ 오른쪽 리모 찾는 거리 (앞줄 약 0.4m, 뒷줄 약 1.0m)
    'extra_mm': 200.0,         # (추가) 리모 옆 확인 후 더 갈 거리. 0 = 캡틴 원래 계획 (시뮬레이션: 0이면 0점)
    'brake_mm': 23.0,          # 0.18 m/s 제동 거리 (calib 4번)
    'pause_sec': 1.0,          # ③ 캡틴 값
    'turn_speed': 0.12,
    'straighten_yaw_deg': 87.0,  # (추가) 이만큼 돌면 바퀴 펴고 직진. 0 = 끔 (시뮬레이션: 끄면 5번 칸에서 계속 돎)
    'turn_max_mm': 2000.0,     # 안전: TURN에서 이만큼 움직여도 안 끝나면 멈춤
    'back_mm': 50.0,           # ④-1 캡틴 값 5cm 후진
    'back_speed': 0.10,
    'back_kappa': 0.0,         # 후진할 때 곡률. 0 = 바퀴 똑바로. +값(왼쪽)이면 후진하면서 더 돌아감
                               # (오른쪽 -값으로 후진하면 앞에서 돈 만큼 되돌아가니 쓰지 말 것)
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
                clusters, blocked, points, clusters_pair, clusters_box = [], False, [], [], []
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
                    clusters_pair = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                                  s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                                  max_dist_mm=max(1500.0, g('side_far_mm') + 200.0), x_mode='extent')
                    clusters_box = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                                 s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                                 max_dist_mm=1500.0, x_mode='box')
                v, k, info = self.logic.step({
                    't': time.monotonic(), 'dist_mm': self.dist_mm, 'yaw': self.yaw,
                    'lane_v': self.lane_v, 'lane_w': self.lane_w, 'stoplines_mm': self.stoplines,
                    'end_lines_mm': self.end_lines, 'armed': bool(self.mstate.get('parking_armed', True)),
                    'clusters': clusters, 'blocked': blocked, 'points': points, 'clusters_pair': clusters_pair,
                    'clusters_box': clusters_box})
                tw.linear.x = float(v)
                tw.angular.z = float(self.lane_w if k is None else v * k)
                if self.logic.lane_cfg is not None and self.logic.lane_cfg != self._sent_cfg:
                    self.lane_cfg_pub.publish(String(data=json.dumps(self.logic.lane_cfg)))
                    self._sent_cfg = dict(self.logic.lane_cfg)
            elif not active:
                if self._sent_cfg and self._sent_cfg.get('target_offset_mm', 0.0) != 0.0:
                    self.lane_cfg_pub.publish(String(data=json.dumps({'target_offset_mm': 0.0})))
                self._sent_cfg = None
                self.logic = None
            self.cmd_pub.publish(tw)
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
