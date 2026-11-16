"""경로 기록/재생 공통 (path_recorder.py, path_follower.py, path_check.py가 같이 씀).

위치 추정 = odom이 잰 '움직인 거리' + IMU 자이로 '방향'
  - 거리: /odom 위치(pose) 변화량의 길이. 1 m당 0.6 cm 오차(calib 2번)라 믿을 만함.
    앞/뒤는 위치 변화 방향과 odom 차 방향을 비교해서 정함 (twist 속도값 부호에 의존하지 않음)
  - 방향: /imu angular_velocity.z 적분. 멈춰 있을 때마다 자이로 편향을 다시 잼.
    odom 방향은 커브에서 3~12% 틀려서(calib 3번) 쓰지 않음
  - 기록과 재생이 같은 계산을 써야 같은 길이 나온다 → 이 파일 하나로 공유

부호 점검: 돌 때 odom 방향 변화와 IMU 방향 변화의 부호가 다르면, 또는 앞으로 가는데
odom 속도값이 음수면 경고한다 (어느 한쪽이 뒤집혀 있으면 재생이 한쪽으로 끝까지 꺾임).
"""
import csv
import math

SPACING = 0.02          # 경로 다시 뽑는 간격 m
MAX_CURV = 1 / 0.53     # 최소 회전반경 약 53 cm (calib 3번)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PoseFusion:
    CAL = 1.0           # 시작 후 이 시간은 정지 상태로 보고 자이로 편향 측정
    STILL_SEC = 0.5     # odom 위치가 이만큼 안 변하면 '멈춤' → 편향 다시 잼

    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.s = 0.0                    # 부호 있는 누적 거리 (뒤로 가면 줄어듦)
        self.dist = 0.0                 # 부호 없는 누적 거리
        self.bias, self.n = 0.0, 0
        self.t_cal0 = self.t_imu = None
        self.t_odom = None
        self.last_odom = None           # (x, y, yaw)
        self.still_since = None
        # 부호 점검
        self.turn_odom = 0.0
        self.turn_imu = 0.0
        self.sign_bad_turn = 0
        self.sign_ok_turn = 0
        self.v_bad = 0
        self.v_ok = 0
        self.warn = []

    @property
    def ready(self):
        return self.t_cal0 is not None and self.t_imu is not None and self.t_imu - self.t_cal0 >= self.CAL

    def is_still(self, t):
        return self.still_since is not None and t - self.still_since >= self.STILL_SEC

    def on_imu(self, t, gz):
        if self.t_cal0 is None:
            self.t_cal0 = t
        calibrating = t - self.t_cal0 < self.CAL
        if calibrating or (self.t_odom is not None and self.is_still(t)):
            self.n = min(self.n + 1, 400)
            self.bias += (gz - self.bias) / self.n
        elif self.t_imu is not None and 0.0 < t - self.t_imu < 0.5:
            d = (gz - self.bias) * (t - self.t_imu)
            self.yaw += d
            self.turn_imu += d
        self.t_imu = t

    def on_odom(self, t, x, y, yaw_odom, v_twist):
        """odom 한 번 들어올 때. 반환: 이번에 움직인 부호 있는 거리"""
        self.t_odom = t
        if self.last_odom is None:
            self.last_odom = (x, y, yaw_odom)
            self.still_since = t
            return 0.0
        x0, y0, yaw0 = self.last_odom
        dx, dy = x - x0, y - y0
        ds = math.hypot(dx, dy)
        if ds > 0.0005:
            self.still_since = t
        elif self.still_since is None:
            self.still_since = t
        sgn = 0.0
        if ds > 1e-5:
            sgn = 1.0 if math.cos(math.atan2(dy, dx) - yaw0) >= 0 else -1.0
            if abs(v_twist) > 0.05 and ds > 0.001:
                if (v_twist > 0) == (sgn > 0):
                    self.v_ok += 1
                else:
                    self.v_bad += 1
        step = sgn * ds
        self.turn_odom += wrap(yaw_odom - yaw0)
        self.last_odom = (x, y, yaw_odom)
        if not self.ready:
            return 0.0
        self.x += step * math.cos(self.yaw)
        self.y += step * math.sin(self.yaw)
        self.s += step
        self.dist += ds
        self._check_turn()
        return step

    def _check_turn(self):
        # 0.35 rad(20°) 넘게 돌 때마다 두 방향 변화의 부호 비교
        if abs(self.turn_imu) > 0.35 or abs(self.turn_odom) > 0.35:
            if abs(self.turn_imu) > 0.1 and abs(self.turn_odom) > 0.1:
                if (self.turn_imu > 0) == (self.turn_odom > 0):
                    self.sign_ok_turn += 1
                else:
                    self.sign_bad_turn += 1
            self.turn_imu = self.turn_odom = 0.0

    def sign_problem(self):
        """문제 있으면 설명 문자열, 없으면 None"""
        if self.sign_bad_turn >= 1 and self.sign_bad_turn >= self.sign_ok_turn:
            return ('odom 방향과 IMU 방향이 반대로 돎 (/imu angular_velocity.z 또는 /odom 방향 부호 확인). '
                    '왼쪽으로 돌 때 둘 다 양수여야 함')
        if self.v_bad >= 10 and self.v_bad > self.v_ok:
            return '앞으로 가는데 /odom twist.linear.x가 음수 (속도 부호 뒤집힘)'
        return None


# ---------------- 경로 ----------------
class Path:
    """기록 CSV → 일정 간격 점 (x, y, 방향, 곡률, 누적거리 s)"""

    def __init__(self, xs, ys, yaws):
        self.x, self.y, self.h, self.k, self.s = xs, ys, yaws, [], []
        acc = 0.0
        for i in range(len(xs)):
            if i:
                acc += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
            self.s.append(acc)
        n = len(xs)
        w = 5   # 곡률은 ±5점(±10 cm)으로 매끄럽게
        for i in range(n):
            a, b = max(0, i - w), min(n - 1, i + w)
            ds = self.s[b] - self.s[a]
            self.k.append(wrap(self.h[b] - self.h[a]) / ds if ds > 1e-6 else 0.0)

    @property
    def length(self):
        return self.s[-1] if self.s else 0.0

    def __len__(self):
        return len(self.x)

    def k_at(self, s):
        if not self.s:
            return 0.0
        if s <= 0:
            return self.k[0]
        if s >= self.s[-1]:
            return self.k[-1]
        i = min(int(s / SPACING), len(self.s) - 2)
        while i > 0 and self.s[i] > s:
            i -= 1
        while i < len(self.s) - 2 and self.s[i + 1] < s:
            i += 1
        return self.k[i]

    def project(self, x, y, i0, window=60):
        """가까운 선분에 투영. 반환 (i, s, 옆 오차 e(왼쪽 +), 경로 방향)"""
        best = None
        hi = min(len(self.x) - 1, i0 + window)
        for i in range(max(0, i0 - 5), hi):
            ax, ay, bx, by = self.x[i], self.y[i], self.x[i + 1], self.y[i + 1]
            vx, vy = bx - ax, by - ay
            L2 = vx * vx + vy * vy
            u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
            px, py = ax + u * vx, ay + u * vy
            d = math.hypot(x - px, y - py)
            if best is None or d < best[0]:
                L = math.sqrt(L2)
                e = ((x - ax) * vy - (y - ay) * vx) / L if L > 1e-9 else 0.0
                best = (d, i, self.s[i] + u * L, -e, math.atan2(vy, vx) if L > 1e-9 else self.h[i])
        _, i, s, e, h = best
        # 끝점 넘어간 경우(u=1에서 멈춤) s는 앞으로 늘려 줌
        if i == len(self.x) - 2:
            hx, hy = math.cos(h), math.sin(h)
            over = (x - self.x[-1]) * hx + (y - self.y[-1]) * hy
            if over > 0:
                s = self.s[-1] + over
        return i, s, e, h


def read_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def _f(r, k):
    try:
        v = r.get(k, '')
        return float(v) if v not in ('', None) else None
    except ValueError:
        return None


def load_path(path, pose='fused'):
    """pose: fused(odom 거리 + IMU 방향, 새 기록) / imu(옛 xf,yf) / odom(odom 위치·방향)
    반환 (Path, 정보 dict). 뒤로 간 구간이 있으면 정보에 표시."""
    rows = read_rows(path)
    cols = {'fused': ('xs', 'ys', 'yaws'), 'imu': ('xf', 'yf', 'yawf'), 'odom': ('x', 'y', 'yaw')}[pose]
    if pose == 'fused' and rows and 'xs' not in rows[0]:
        raise ValueError('이 CSV에는 xs, ys 열이 없음 (예전 기록기로 만든 파일) → 새 path_recorder.py로 다시 기록하거나 --pose odom')
    raw = []
    for r in rows:
        x, y, h = _f(r, cols[0]), _f(r, cols[1]), _f(r, cols[2])
        if x is None or y is None or h is None:
            continue
        raw.append((x, y, h, _f(r, 's')))
    info = {'rows': len(rows), 'used': len(raw), 'reverse_m': 0.0}
    if len(raw) < 2:
        return Path([], [], []), info
    # 뒤로 간 거리 (s 열이 있으면 그걸로, 없으면 방향 기준 투영)
    for a, b in zip(raw, raw[1:]):
        if a[3] is not None and b[3] is not None:
            d = b[3] - a[3]
        else:
            d = (b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2])
        if d < 0:
            info['reverse_m'] -= d
    # 일정 간격으로 다시 뽑기 (방향은 기록된 IMU 방향을 그대로 씀)
    xs, ys, hs = [raw[0][0]], [raw[0][1]], [raw[0][2]]
    for x, y, h, _ in raw[1:]:
        d = math.hypot(x - xs[-1], y - ys[-1])
        while d >= SPACING:
            u = SPACING / d
            nx, ny = xs[-1] + u * (x - xs[-1]), ys[-1] + u * (y - ys[-1])
            xs.append(nx)
            ys.append(ny)
            hs.append(hs[-1] + u * wrap(h - hs[-1]))
            d = math.hypot(x - nx, y - ny)
    if math.hypot(raw[-1][0] - xs[-1], raw[-1][1] - ys[-1]) > 0.002:
        xs.append(raw[-1][0])
        ys.append(raw[-1][1])
        hs.append(raw[-1][2])
    # 방향을 이어지게 (unwrap)
    for i in range(1, len(hs)):
        hs[i] = hs[i - 1] + wrap(hs[i] - hs[i - 1])
    return Path(xs, ys, hs), info
