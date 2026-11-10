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
① WAIT_LINE: 차선 노드 명령을 넘기며 흰 선 기다림 → 보이면 주차 모드
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
                  gap_mm=80.0, min_pts=3, max_size_mm=450.0, max_dist_mm=1500.0):
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
        out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    return out


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


def cell_line_from_neighbors(clusters, side_max_mm=700.0):
    """양옆 리모 중심(앞바퀴 기준 좌표)으로 칸 가운데 선을 구한다.

    반환: (중점 x, 중점 y, 방향 단위벡터 ux, uy) — 차 기준 좌표, 방향은 차 앞쪽과 같은 쪽.
    양옆 중 하나라도 없으면 None. 리모가 둘 다 같은 쪽 얼굴만 보여서 생기는 치우침은 서로 상쇄된다.
    """
    rights = [(x, y) for x, y in clusters if -side_max_mm <= y <= -80.0 and x > -400.0]
    lefts = [(x, y) for x, y in clusters if 80.0 <= y <= side_max_mm and x > -400.0]
    if not rights or not lefts:
        return None
    r = min(rights, key=lambda c: math.hypot(*c))
    l = min(lefts, key=lambda c: math.hypot(*c))
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
        self.pair_n = 0            # 짝을 본 횟수 (평균 낼 때 씀)
        self.pair_side_mm = None   # 짝 중점까지 옆 거리 (앞줄 약 0.5m, 뒷줄 약 1.1m) -> row
        self.row = None            # 'front' | 'back' (정차 리모가 있는 줄 = 들어갈 줄)
        self.side_seen = False     # 꺾은 뒤 양옆 리모로 칸 가운데 선을 한 번이라도 구했나
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
        """지금 믿는 칸 가운데 선: 양옆 옆면으로 구한 것 > 짝(보정)."""
        return self.line if self.side_seen else self._pair_cell()

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
            if moved >= p['ignore_mm'] and lines and inp.get('armed', True):
                self.lane_cfg = {'target_offset_mm': p['target_offset_mm']}
                self._enter('FOLLOW', inp)
            else:
                return self._out(inp['lane_v'], None)

        if self.phase == 'FOLLOW':          # ② 왼쪽 붙어 차선 따라가기, ③ 오른쪽 리모 찾기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                return self._done('BLOCKED', inp)
            pair = find_pair(inp['clusters'], p['side_far_mm'], p['pair_dist_mm'], p['pair_tol_mm'])
            if pair is not None:                       # ③ 두 리모 짝 -> 칸 가운데 선 (평균)
                mx, my, vx, vy = pair
                n0 = self.pair_n
                self.pair_side_mm = (-my if self.pair_side_mm is None
                                     else (self.pair_side_mm * n0 - my) / (n0 + 1))
                self.row = 'front' if self.pair_side_mm < p['row_split_mm'] else 'back'
                rel = (mx, my, vy, -vx)                # 들어가는 방향 = 짝 방향에서 오른쪽 90도
                line = self._line_to_world(rel, inp)
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
            seen = in_view(inp['clusters'], p['front_axle_from_lidar_mm'], p['side_view_deg'])
            side = (cell_line_from_neighbors(seen, p['side_max_mm'])
                    if self._turned_deg(inp) >= p['stop_min_turn_deg'] else None)
            turned_ok = self._turned_deg(inp) >= p['stop_min_turn_deg']
            if self.pair_line is not None and side is not None:
                # 짝을 알고 양옆 리모 옆면이 보이면: 옆면으로 구한 선에서 차체 중심이 칸 가운데 오면 멈춤.
                # (차선에서 본 짝은 리모의 차선 쪽 끝면만 보여 깊이가 8~17cm 얕게 나온다 -> 깊이는 옆면으로)
                self.line = self._line_to_world(side, inp)
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
                return self._out(p['turn_speed'], 0.0 if straight else KAPPA_FULL_RIGHT)

        if self.phase == 'CHECK':           # (재정렬) 칸 가운데에 반듯하게 들어갔나
            rel = cell_line_from_neighbors(
                in_view(inp['clusters'], p['front_axle_from_lidar_mm'], p['side_view_deg']), p['side_max_mm'])
            if p['max_realign'] <= 0 or (rel is None and self.pair_line is None):
                return self._done('PARKED', inp)
            if rel is not None:
                self.line = self._line_to_world(rel, inp)
            elif not self.side_seen:
                self.line = self._pair_cell()       # 옆면을 못 봤으면 짝(깊이 보정)
            # (옆면을 봤으면 self.line = 기억한 선 그대로)
            lat, head, s_deep, s_shallow = self._misalign(self.line, inp)
            self.err = (round(lat), round(head, 1), round(s_deep), round(s_shallow))
            lim_d = p['cell_half_depth_mm'] - p['corner_margin_mm']
            if (lat <= p['cell_half_mm'] - p['corner_margin_mm'] and abs(head) <= p['realign_head_deg']
                    and s_deep <= lim_d and s_shallow >= -lim_d):
                return self._done('PARKED', inp)
            if self.realigns >= p['max_realign']:
                return self._done('PARKED_NOT_ALIGNED', inp)
            self.realigns += 1
            self._enter('PULLOUT', inp)

        if self.phase == 'PULLOUT':         # (재정렬) 칸 가운데 선을 따라 뒤로 빠지기
            moved = inp['dist_mm'] - self.d_phase
            if moved >= p['pullout_mm']:
                self._enter('ENTER', inp)
                self.seen_ahead = False
            else:
                k = 0.0                              # 기본: 바퀴 똑바로 후진 (꺾고 후진하면 앞이 옆으로 휘둘려 옆 리모에 닿음)
                if p['pullout_steer']:
                    k = pursuit_kappa_on_line(self.px, self.py, inp['yaw'] or 0.0, self.line,
                                              p['pursuit_look_mm'], reverse=True)
                return self._out(-p['realign_speed'], max(-2.0, min(2.0, k)))

        if self.phase == 'ENTER':           # (재정렬) 칸 가운데 선을 따라 다시 들어가기
            moved = inp['dist_mm'] - self.d_phase
            if inp['blocked']:
                return self._done('BLOCKED', inp)
            if self._body_s(inp) >= -p['enter_brake_mm']:      # 차체 중심이 칸 가운데에 옴
                self._enter('CHECK', inp)
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
                'row': self.row, 'stop_how': self.stop_how,
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
    'turn_lead_mm': 320.0,     # 칸 가운데가 앞바퀴보다 이만큼 앞일 때 꺾기 시작 (회전반경 530 - 축거 210)
    'stop_min_turn_deg': 60.0, # 이만큼 돈 뒤에만 양옆 옆면으로 '칸 가운데 도착'을 판단 (도는 도중 오판 방지)
    # ---- 줄(앞줄 1~3 / 뒷줄 4~6)과 칸 끝에서 멈추기 (11/09 park_02 반영)
    'row_split_mm': 800.0,     # 짝 중점까지 옆 거리가 이보다 가까우면 앞줄 (앞줄 약 0.5m, 뒷줄 약 1.1m)
    'pair_depth_bias_mm': 125.0,  # 짝으로 본 칸 가운데가 실제보다 얕게 나오는 만큼 (시뮬레이션 80~170)
    'end_stop_mm': 195.0,      # 칸 가운데 정지 = 앞바퀴 -> 칸 끝 선 (깊이 반 300 - (축거 210 - 차체 중심 105))
    'end_gate_mm': 200.0,      # 칸 끝 선 예상 위치와 이만큼 안만 받음 (줄 사이 선과 바깥 테두리는 620mm 차이)
    'end_freeze_mm': 420.0,    # 남은 거리가 이보다 짧으면 카메라 갱신 끊고 오도메트리로만 (BEV 사각지대 약 350)
    'cell_cap_mm': 80.0,       # 차체 중심이 (추정한) 칸 가운데를 이만큼 지나면 무조건 멈춤. 칸 안 여유 약 140 - 추정 오차 여유 60
    'side_view_deg': 95.0,     # 칸 가운데 선은 라이다 각도 이 안의 리모로만 (실측 시야 끝 약 105도. 끝에 걸리면 덩어리 중심이 쏠림)
    # ---- 재정렬 (PULLOUT -> ENTER)
    'max_realign': 2,          # 0 = 재정렬 끔 (= 원래 버전 2)
    'cell_half_mm': 250.0,     # 칸 폭 500의 절반 (도면)
    'corner_margin_mm': 25.0,  # 모서리가 칸 선에서 이만큼 안쪽이어야 통과 (측정 오차 여유)
    'realign_head_deg': 20.0,  # 방향이 이보다 틀어지면 모서리가 괜찮아도 재정렬
    'body_length_mm': 322.0,   # LIMO 길이 (팀 parking_node 주석)
    'body_width_mm': 220.0,
    'pullout_mm': 450.0,       # 뒤로 빠지는 거리. 옆 10cm 고치는 데 앞으로 약 45cm 필요 (반지름 0.53m S자)
    'pursuit_look_mm': 250.0,  # 선 따라가기 앞보기 거리
    'pullout_steer': False,    # True면 후진하면서도 선을 따라감 (시험에서 옆 리모와 충돌 -> 기본 끔)
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
                clusters, blocked = [], False
                if self.scan is not None:
                    s = self.scan
                    clusters = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                             s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                             max_dist_mm=max(1500.0, g('side_far_mm') + 200.0))
                    blocked = front_blocked(s.ranges, s.angle_min, s.angle_increment, s.range_min,
                                            s.range_max, g('front_axle_from_lidar_mm'), g('laser_yaw'),
                                            g('front_bumper_from_axle_mm'), g('front_stop_mm'), 120.0)
                v, k, info = self.logic.step({
                    't': time.monotonic(), 'dist_mm': self.dist_mm, 'yaw': self.yaw,
                    'lane_v': self.lane_v, 'lane_w': self.lane_w, 'stoplines_mm': self.stoplines,
                    'end_lines_mm': self.end_lines, 'armed': bool(self.mstate.get('parking_armed', True)),
                    'clusters': clusters, 'blocked': blocked})
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
