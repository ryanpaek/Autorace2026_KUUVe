"""가속도계로 전진/후진 판단 (11/17). autorace_mission/motion_dir.py와 같은 내용 (고치면 둘 다).

왜: LIMO 드라이버는 조종기(RC)로 후진해도 /odom 위치를 앞으로 쌓고 속도값도 양수로 낸다 (11/17 실차).
    우리 /cmd_vel 부호로는 우리가 낸 후진만 알 수 있어서, 조종기로 물렸다가 다시 시험하면 도면 위치(route s)가
    계속 늘어 M7이 '섬 지남'으로 끝나 버렸다.
어떻게: 멈췄다가 움직이기 시작하는 순간(start_win초) 앞뒤 가속도(/imu linear_acceleration.x - 멈춰 있을 때 평균)를
    적분한 속도 변화의 부호 = 그 다음 멈출 때까지의 방향.
    가속도계 축 방향(+가 앞인지)은 우리가 /cmd_vel 전진·후진을 내며 출발할 때마다 배운다 (learn_need번 같고 반대보다
    learn_margin번 많아야 켜짐). 다 배우기 전엔 판단 안 함(0) -> 예전과 같음. accel_sign 파라미터로 고정 가능.
"""
from typing import Optional, Tuple


class AccelDirection:
    def __init__(self, still_v: float = 0.012, move_v: float = 0.03, still_sec: float = 0.08,
                 start_win: float = 0.35, dv_min: float = 0.04, learn_need: int = 2, learn_margin: int = 2,
                 fixed_sign: int = 0):
        self.still_v, self.move_v, self.still_sec = still_v, move_v, still_sec
        self.start_win, self.dv_min = start_win, dv_min
        self.learn_need, self.learn_margin = learn_need, learn_margin
        self.fixed_sign = fixed_sign
        self.bias = 0.0
        self.nb = 0
        self.t_imu = None
        self.last = None            # (t, x, y)
        self.speed = 0.0
        self.still_since = None
        self.moving = False
        self.win_t0 = None          # 출발 창 시작
        self.dv = 0.0               # 창 안 적분
        self.dir = 0                # +1 전진 / -1 후진 / 0 모름
        self.raw = 0                # 축 보정 전 마지막 판단
        self.n_pos = self.n_neg = 0  # 배운 횟수: 가속도 +가 앞 / 뒤
        self.cmd_at_start = 0.0
        self.events = []

    @property
    def sign(self) -> int:
        if self.fixed_sign in (1, -1):
            return self.fixed_sign
        if self.n_pos >= self.learn_need and self.n_pos - self.n_neg >= self.learn_margin:
            return 1
        if self.n_neg >= self.learn_need and self.n_neg - self.n_pos >= self.learn_margin:
            return -1
        return 0

    def on_imu(self, t: float, ax: float) -> None:
        dt = (t - self.t_imu) if self.t_imu is not None else 0.0
        self.t_imu = t
        if not (0.0 < dt < 0.2):
            return
        still = not self.moving and self.still_since is not None and t - self.still_since >= self.still_sec
        if still:
            # 멈춰 있음: 편향(기울기·센서 영점) 평균, 적분 비움
            self.nb = min(self.nb + 1, 200)
            self.bias += (ax - self.bias) / self.nb
            self.dv = 0.0
        else:
            # 멈춤이 풀린 순간부터 쌓음 (출발 창 끝에서 판단, 그 뒤엔 다음 멈춤까지 안 씀)
            self.dv += (ax - self.bias) * dt

    def on_odom(self, t: float, x: float, y: float, cmd_v: Optional[float] = None) -> int:
        """반환: 지금 방향 (+1/-1/0 모름)"""
        if self.last is not None:
            t0, x0, y0 = self.last
            dt = t - t0
            if dt > 1e-3:
                self.speed = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5 / dt
        self.last = (t, x, y)
        if self.speed < self.still_v:
            # 멈춤 또는 전진<->후진이 바뀌며 속도가 0을 지나는 순간: 다음 출발 창을 새로 (적분도 여기서부터)
            if self.still_since is None:
                self.still_since = t
                self.dv = 0.0
            self.moving = False
            self.win_t0 = None
            if t - self.still_since >= self.still_sec:
                self.dir = 0
            return self.dir
        # 움직이는 중
        if not self.moving and self.win_t0 is None:
            # 멈춰 있다가 막 출발: 창 시작 (적분은 멈춤이 끝난 뒤부터 쌓이고 있음)
            self.win_t0 = t
            self.cmd_at_start = cmd_v or 0.0
        self.still_since = None
        self.moving = True
        if self.win_t0 is not None and t - self.win_t0 >= self.start_win:
            self.win_t0 = None
            if abs(self.dv) >= self.dv_min:
                self.raw = 1 if self.dv > 0 else -1
                c = self.cmd_at_start
                if abs(c) > 0.05:
                    if self.raw * (1 if c > 0 else -1) > 0:
                        self.n_pos += 1
                    else:
                        self.n_neg += 1
                s = self.sign
                self.dir = self.raw * s if s else 0
                if self.dir < 0:
                    self.events.append(f'가속도로 후진 판단 (속도 변화 {self.dv:+.3f} m/s)')
            else:
                self.dir = 0
            self.dv = 0.0
        return self.dir

    def state(self) -> Tuple[int, int, int, int]:
        return self.dir, self.sign, self.n_pos, self.n_neg
