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

    def __init__(self, use_odom_yaw=False):
        self.use_odom_yaw = use_odom_yaw   # True면 방향도 odom (IMU 없을 때)
        self.yaw0_odom = None
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
        self.rev_by = {'pos': 0, 'twist': 0, 'cmd': 0}   # 후진을 무엇으로 알았나 (걸음 수)
        self.rev_steps = 0
        self.warn = []

    @property
    def ready(self):
        if self.use_odom_yaw:
            return self.last_odom is not None
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
            if not self.use_odom_yaw:
                self.yaw += d
            self.turn_imu += d
        self.t_imu = t

    def on_odom(self, t, x, y, yaw_odom, v_twist, cmd_v=0.0):
        """odom 한 번 들어올 때. 반환: 이번에 움직인 부호 있는 거리.
        후진 판단: odom 위치 변화 방향 / odom 속도값 부호 / 명령(cmd_v) 부호 중 하나라도 후진이면 후진
        (LIMO 드라이버가 후진에도 위치를 앞으로 쌓거나 속도를 양수로 내는 경우 대비. mission_node와 같은 규칙)"""
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
            pos_sgn = 1.0 if math.cos(math.atan2(dy, dx) - yaw0) >= 0 else -1.0
            if abs(v_twist) > 0.05 and ds > 0.001:
                if (v_twist > 0) == (pos_sgn > 0):
                    self.v_ok += 1
                else:
                    self.v_bad += 1
            by = [k for k, c in (('pos', pos_sgn < 0), ('twist', v_twist < -0.02), ('cmd', cmd_v < -0.02)) if c]
            sgn = -1.0 if by else 1.0
            if by:
                self.rev_steps += 1
                for k in by:
                    self.rev_by[k] += 1
        step = sgn * ds
        self.turn_odom += wrap(yaw_odom - yaw0)
        self.last_odom = (x, y, yaw_odom)
        if self.use_odom_yaw:
            if self.yaw0_odom is None:
                self.yaw0_odom = yaw_odom
            self.yaw = self.yaw + wrap(yaw_odom - yaw0)
        if not self.ready:
            return 0.0
        self.x += step * math.cos(self.yaw)
        self.y += step * math.sin(self.yaw)
        self.s += step
        self.dist += ds
        if step < 0:
            # 후진 중엔 비교 안 함 (드라이버가 후진을 전진처럼 계산하면 odom 방향도 반대로 돎)
            self.turn_imu = self.turn_odom = 0.0
        else:
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
        if self.v_bad >= 10 and self.v_bad > self.v_ok and self.rev_by['cmd'] == 0:
            return '앞으로 가는데 /odom twist.linear.x가 음수 (속도 부호 뒤집힘)'
        return None

    def reverse_note(self):
        """후진을 명령 부호로만 알았으면 알림 (드라이버가 후진을 안 알려 줌)"""
        if self.rev_steps >= 5 and self.rev_by['pos'] + self.rev_by['twist'] == 0:
            return 'LIMO odom이 후진을 전진처럼 냄 → 명령 부호로 후진 판단 중 (정상 동작)'
        return None


# ---------------- 경로 ----------------
class Path:
    """한 방향(전진 또는 후진) 구간. 점 (x, y, 차 방향 h, 곡률 k, 누적거리 s).
    dir = +1 전진 / -1 후진. h는 '차가 향한 방향'(후진이면 진행 방향의 반대).
    k = 차 방향 변화 / 진행 거리 (진행 거리는 부호 없음). 명령 곡률(부호 있는 거리 기준)은 dir * k."""

    def __init__(self, xs, ys, yaws, direction=1):
        self.x, self.y, self.h, self.k, self.s = xs, ys, yaws, [], []
        self.dir = 1 if direction >= 0 else -1
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
        """가까운 선분에 투영. 반환 (i, s, e, 경로 차 방향)
        e = 차 방향 기준 왼쪽 + 옆 오차 (후진 구간도 차 기준)"""
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
                cross = ((x - ax) * vy - (y - ay) * vx) / L if L > 1e-9 else 0.0
                h_travel = math.atan2(vy, vx) if L > 1e-9 else None
                best = (d, i, self.s[i] + u * L, -cross, h_travel)
        _, i, s, e_travel, h_travel = best
        if h_travel is None:
            h_travel = self.h[i] if self.dir > 0 else self.h[i] + math.pi
        if i == len(self.x) - 2:        # 끝점 넘어가면 s를 앞으로 늘림
            over = (x - self.x[-1]) * math.cos(h_travel) + (y - self.y[-1]) * math.sin(h_travel)
            if over > 0:
                s = self.s[-1] + over
        h_car = h_travel if self.dir > 0 else wrap(h_travel + math.pi)
        return i, s, self.dir * e_travel, h_car


def read_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def _f(r, k):
    try:
        v = r.get(k, '')
        return float(v) if v not in ('', None) else None
    except ValueError:
        return None


def _resample(pts, direction):
    xs, ys, hs = [pts[0][0]], [pts[0][1]], [pts[0][2]]
    for x, y, h in pts[1:]:
        d = math.hypot(x - xs[-1], y - ys[-1])
        while d >= SPACING:
            u = SPACING / d
            nx, ny = xs[-1] + u * (x - xs[-1]), ys[-1] + u * (y - ys[-1])
            xs.append(nx)
            ys.append(ny)
            hs.append(hs[-1] + u * wrap(h - hs[-1]))
            d = math.hypot(x - nx, y - ny)
    if math.hypot(pts[-1][0] - xs[-1], pts[-1][1] - ys[-1]) > 0.002:
        xs.append(pts[-1][0])
        ys.append(pts[-1][1])
        hs.append(pts[-1][2])
    for i in range(1, len(hs)):
        hs[i] = hs[i - 1] + wrap(hs[i] - hs[i - 1])
    return Path(xs, ys, hs, direction)


CUSP_MIN = 0.03     # 이보다 짧게 반대로 간 건 흔들림으로 보고 무시 (m)


def load_segments(path, pose='fused'):
    """pose: fused(odom 거리 + IMU 방향, 새 기록) / imu(옛 xf,yf) / odom(odom 위치·방향)
    반환 ([Path, ...], 정보). 전진·후진이 바뀌는 곳마다 구간을 나눈다."""
    rows = read_rows(path)
    cols = {'fused': ('xs', 'ys', 'yaws'), 'imu': ('xf', 'yf', 'yawf'), 'odom': ('x', 'y', 'yaw')}[pose]
    if pose == 'fused' and rows and 'xs' not in rows[0] and 'gz' not in rows[0]:
        raise ValueError('이 CSV에는 xs, ys 열이 없음 (예전 기록기로 만든 파일) → 새 path_recorder.py로 다시 기록하거나 --pose odom')
    info = {'rows': len(rows), 'fwd_m': 0.0, 'rev_m': 0.0, 'n_seg': 0, 'rebuilt': False, 'note': None}
    raw, sg = [], []
    have_raw = rows and all(k in rows[0] for k in ('t', 'x', 'y', 'yaw', 'vx', 'cmd_vx'))
    if pose in ('fused', 'odom') and have_raw and (pose == 'odom' or 'gz' in rows[0]):
        # 원본 열(odom 위치·속도, 명령, 자이로)로 다시 계산: 기록 때와 같은 PoseFusion, 후진 판단 포함
        fu = PoseFusion(use_odom_yaw=(pose == 'odom'))
        for r in rows:
            t = _f(r, 't')
            x, y, yaw = _f(r, 'x'), _f(r, 'y'), _f(r, 'yaw')
            if None in (t, x, y, yaw):
                continue
            if pose == 'fused':
                gz = _f(r, 'gz')
                if gz is not None:
                    fu.on_imu(t, gz)
            fu.on_odom(t, x, y, yaw, _f(r, 'vx') or 0.0, _f(r, 'cmd_vx') or 0.0)
            if fu.ready:
                raw.append((fu.x, fu.y, fu.yaw))
                sg.append(fu.s)
        info['rebuilt'] = True
        info['note'] = fu.reverse_note()
    else:
        for r in rows:
            x, y, h = _f(r, cols[0]), _f(r, cols[1]), _f(r, cols[2])
            if x is None or y is None or h is None:
                continue
            raw.append((x, y, h))
        # 부호 있는 누적 거리: 위치 변화를 차 방향에 투영
        sg = [0.0]
        for a, b in zip(raw, raw[1:]):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            if d < 1e-6:
                sg.append(sg[-1])
                continue
            fwd = (b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2])
            sg.append(sg[-1] + (d if fwd >= 0 else -d))
    info['used'] = len(raw)
    if len(raw) < 2:
        return [], info
    # 방향 바뀌는 점(cusp) 찾기: 현재 방향의 끝값에서 반대로 CUSP_MIN 넘게 가면 그 끝값이 cusp
    cuts = [0]
    cur = 0
    ext_i = 0
    for i in range(1, len(sg)):
        if cur == 0:
            if abs(sg[i] - sg[0 if len(cuts) == 1 else cuts[-1]]) > CUSP_MIN:
                cur = 1 if sg[i] > sg[cuts[-1]] else -1
                ext_i = i
            continue
        if (sg[i] - sg[ext_i]) * cur > 0:
            ext_i = i
        elif (sg[ext_i] - sg[i]) * cur > CUSP_MIN:
            cuts.append(ext_i)
            cur = -cur
            ext_i = i
    end_i = ext_i if cur != 0 else len(sg) - 1
    cuts.append(end_i)
    segs = []
    for a, b in zip(cuts, cuts[1:]):
        if b - a < 1:
            continue
        d = 1 if sg[b] >= sg[a] else -1
        pts = raw[a:b + 1]
        seg = _resample(pts, d)
        if seg.length < 0.01:
            continue
        if segs and segs[-1].dir == d:          # 같은 방향이 이어지면 합침
            prev = segs[-1]
            seg = _resample(list(zip(prev.x, prev.y, prev.h)) + pts[1:], d)
            segs[-1] = seg
        else:
            segs.append(seg)
    for g in segs:
        info['fwd_m' if g.dir > 0 else 'rev_m'] += g.length
    info['n_seg'] = len(segs)
    return segs, info


def load_path(path, pose='fused'):
    """예전 호환: 구간을 하나로 (방향 구분 없이). 새 코드는 load_segments를 쓸 것"""
    segs, info = load_segments(path, pose)
    if not segs:
        return Path([], [], []), info
    if len(segs) == 1:
        return segs[0], info
    xs, ys, hs = [], [], []
    for g in segs:
        xs += g.x
        ys += g.y
        hs += g.h
    return Path(xs, ys, hs), info
