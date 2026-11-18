"""속도 S자(시그모이드 모양) 가감속 (ROS 없음, 11/19).

목표 속도로 바로 뛰지 않고 저크(가속도 변화율) 제한 + 가속도 제한으로 따라간다. 속도 그래프가 S자(로지스틱 비슷)가 된다.
  a_des = 부호(오차) x min(sqrt(2 x 저크 x |오차|), |오차| / tau)  (가속 a_acc / 감속 a_dec 상한)  -> 가속도는 저크 한도로만 바뀜
  tau: 작은 오차(|오차| < band, 차선 노드 커브 감속이 프레임마다 흔들리는 것)는 이 시간 상수로 천천히 따라감.
       큰 변화(속도 바꾸기)는 S자 그대로 빨리
조향은 곡률이 그대로 되게 angular.z도 같은 비율로 줄이고 늘린다 (w = v x kappa).

바로 그대로 넘기는 경우 (S자 안 씀):
  - 목표 0 (정지 명령, 비상 정지, 회피 노드 안전 정지 등 모든 정지)
  - 전진 <-> 후진이 바뀔 때
  - bypass (미션 노드: M7 구간 전체)
"""
import math


class SpeedSmoother:
    def __init__(self, a_acc: float = 0.5, a_dec: float = 0.8, jerk: float = 3.0, tau: float = 0.0, band: float = 0.06):
        self.a_acc, self.a_dec, self.jerk, self.tau, self.band = a_acc, a_dec, jerk, tau, band
        self.v = 0.0
        self.a = 0.0

    def reset(self, v: float = 0.0) -> None:
        self.v, self.a = v, 0.0

    def step(self, target: float, dt: float, bypass: bool = False) -> float:
        if (bypass or abs(target) < 1e-3 or target * self.v < 0.0 or dt <= 0.0
                or self.jerk <= 0.0 or self.a_acc <= 0.0 or self.a_dec <= 0.0):
            self.reset(target)
            return target
        err = target - self.v
        speeding_up = abs(target) > abs(self.v)
        amax = self.a_acc if speeding_up else self.a_dec
        mag = min(amax, math.sqrt(2.0 * self.jerk * abs(err)))
        if self.tau > 0.0 and abs(err) < self.band:
            mag = min(mag, abs(err) / self.tau)
        a_des = math.copysign(mag, err)
        da = max(-self.jerk * dt, min(self.jerk * dt, a_des - self.a))
        self.a = max(-max(self.a_acc, self.a_dec), min(max(self.a_acc, self.a_dec), self.a + da))
        v_new = self.v + self.a * dt
        if (target - v_new) * err <= 0.0:          # 넘어섰으면 목표에서 멈춤
            v_new, self.a = target, 0.0
        self.v = v_new
        return v_new
