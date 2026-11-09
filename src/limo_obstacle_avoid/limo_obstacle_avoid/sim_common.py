"""
sim_common.py — 가짜 트랙(정차 LIMO 배치) + 가짜 라이다 + 가짜 차량   (ROS 없음)

s0_fake_world.py(ROS) 와 tools/offline_sim.py(순수 파이썬) 둘 다 이걸 씁니다.
월드 좌표 = 미션 로컬 좌표와 같게 둠: x=0 이 enable 지점, y=0 이 가운데 차선 중심.
"""
import math
import random

LIMO_L = 0.32   # 정차 LIMO 길이
LIMO_W = 0.22   # 정차 LIMO 폭

# (앞면 x 위치[m], 차선)  ← 실제 배치는 대회 당일 모름. 자유롭게 고쳐서 테스트하세요.
LAYOUTS = {
    'single':  [(1.5, 0)],
    # 줄(row) 간격 약 1.4m, 한 줄에 2대씩 → 매번 1칸씩 이동
    'zigzag6': [(1.2, 0), (1.2, -1),
                (2.6, 1), (2.6, -1),
                (3.9, 0), (3.9, 1)],
    # 7대: 0.9줄{0,+1} → 2.3줄{-1,+1} → 3.7줄{0,-1}  (+1 차선에 1대 추가)
    'mixed7':  [(0.9, 0), (0.9, 1), (1.6, 1),
                (2.3, -1), (2.3, 1),
                (3.7, 0), (3.7, -1)],
    # 한 대씩 1.1m 간격 대각선
    'diag3':   [(0.8, 0), (1.9, 1), (3.0, -1)],
}


def layout_rects(name_or_list, lane_width=0.52, jitter=0.0, seed=None):
    """→ [(cx, cy, L, W, yaw)]"""
    items = LAYOUTS[name_or_list] if isinstance(name_or_list, str) else name_or_list
    rnd = random.Random(seed)
    out = []
    for (x, lane) in items:
        jx = rnd.uniform(-jitter, jitter)
        jy = rnd.uniform(-jitter, jitter) * 0.5
        out.append((x + LIMO_L / 2 + jx, lane * lane_width + jy, LIMO_L, LIMO_W, 0.0))
    return out


def rect_corners(cx, cy, L, W, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for dx, dy in ((L / 2, W / 2), (L / 2, -W / 2), (-L / 2, -W / 2), (-L / 2, W / 2)):
        pts.append((cx + dx * c - dy * s, cy + dx * s + dy * c))
    return pts


def rects_to_segments(rects):
    segs = []
    for r in rects:
        cs = rect_corners(*r)
        for i in range(4):
            segs.append((cs[i], cs[(i + 1) % 4]))
    return segs


def raycast(ox, oy, oyaw, angle_min, angle_inc, n, segs, rmax=8.0, noise=0.0, rnd=None):
    """라이다 흉내: 각 빔이 처음 맞는 선분까지 거리. 안 맞으면 inf"""
    rnd = rnd or random
    out = []
    for i in range(n):
        a = oyaw + angle_min + i * angle_inc
        dx, dy = math.cos(a), math.sin(a)
        best = rmax
        for (ax, ay), (bx, by) in segs:
            ex, ey = bx - ax, by - ay
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            wx, wy = ax - ox, ay - oy
            t = (wx * ey - wy * ex) / den
            u = (wx * dy - wy * dx) / den
            if t > 0 and 0 <= u <= 1 and t < best:
                best = t
        if best >= rmax:
            out.append(float('inf'))
        else:
            out.append(best + (rnd.gauss(0, noise) if noise > 0 else 0.0))
    return out


class SimCar:
    """아커만 흉내 (회전반경 제한 있는 자전거 모델, 실측 최소 회전반경 0.53m)"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, min_r=0.53, L=0.32, W=0.22, front=0.20):
        self.x, self.y, self.yaw = x, y, yaw
        self.min_r = min_r
        self.L, self.W, self.front = L, W, front
        self.odom_x = self.odom_y = 0.0     # 오도메트리 (오차 포함)
        self.odom_yaw = yaw
        self.imu_yaw = yaw

    def step(self, v, w, dt, odom_scale=1.0, imu_drift=0.0, steer_noise=0.0, rnd=None):
        rnd = rnd or random
        if abs(v) > 1e-6:
            wmax = abs(v) / self.min_r
            w = max(-wmax, min(wmax, w))
        else:
            w = 0.0
        w_real = w + (rnd.gauss(0, steer_noise) if steer_noise > 0 else 0.0) * abs(v)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w_real * dt
        ds = v * dt * odom_scale
        self.odom_x += ds * math.cos(self.odom_yaw)
        self.odom_y += ds * math.sin(self.odom_yaw)
        self.odom_yaw += w * dt                 # odom 방향은 '명령' 기준 → 실제와 어긋남
        self.imu_yaw = self.yaw + imu_drift     # IMU 는 실제 방향 + 드리프트
        return w

    def footprint(self):
        """차 사각형 (base_link 가 차 중심보다 뒤에 있다고 가정)"""
        cx = self.x + (self.front - self.L / 2) * math.cos(self.yaw)
        cy = self.y + (self.front - self.L / 2) * math.sin(self.yaw)
        return (cx, cy, self.L, self.W, self.yaw)


def _sat_overlap(a, b):
    pa, pb = rect_corners(*a), rect_corners(*b)
    for poly in (pa, pb):
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            nx, ny = y2 - y1, x1 - x2
            sa = [nx * x + ny * y for x, y in pa]
            sb = [nx * x + ny * y for x, y in pb]
            if max(sa) < min(sb) or max(sb) < min(sa):
                return False
    return True


def min_gap(car_rect, rects):
    """차와 장애물 사이 최소 간격(대략) — 겹치면 음수(-1)"""
    best = float('inf')
    cc = rect_corners(*car_rect)
    for r in rects:
        if _sat_overlap(car_rect, r):
            return -1.0
        rc = rect_corners(*r)
        for poly_a, poly_b in ((cc, rc), (rc, cc)):
            for (px, py) in poly_a:
                for i in range(4):
                    (ax, ay), (bx, by) = poly_b[i], poly_b[(i + 1) % 4]
                    ex, ey = bx - ax, by - ay
                    t = max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / (ex * ex + ey * ey)))
                    best = min(best, math.hypot(px - ax - t * ex, py - ay - t * ey))
    return best
