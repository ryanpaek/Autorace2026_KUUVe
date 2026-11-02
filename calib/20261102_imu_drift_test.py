#!/usr/bin/env python3
# 20261102_imu_drift_test.py — IMU 정지 드리프트 자동 시험 (읽기 전용, 로봇 안 움직임)
#
# 흐름도
# ① 시작 → /imu 구독, 첫 데이터 기다림
# │
# ├─② 회차 시작: 지금 imu yaw 저장
# │   └─ 60초 동안 자이로 z(회전 속도)도 모아서 평균 계산
# │
# ├─③ 60초 끝: 끝 yaw 저장 → 드리프트(±180° 경계 보정) 계산
# │
# ├─④ 5회 될 때까지 ②③ 반복
# │
# └─⑤ 결과표 출력 + ~/calib_test/imu_drift_결과.csv 저장 → 자동 종료
#
# 원리: 로봇이 정지해 있으면 yaw는 안 변해야 함. 변한 만큼이 센서 오차(드리프트).
#       자이로 z 평균 = 정지 바이어스. 나중에 측정값에서 이 값을 빼면 보정됨.
#
# 실행 (Global 환경 터미널, 드라이버 켜진 상태):
#   python3 ~/calib_test/20261102_imu_drift_test.py
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

TRIALS = 5         # 회차 수
DURATION = 60.0    # 한 회차 시간 (초)
LIMIT = 1.0        # 판정 기준 (deg/분) — 근거 없는 임시값, 팀이 정할 것


def quat_to_yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def yaw_diff(a, b):
    # b - a 를 -180 ~ +180 범위로 (179.8 → -179.9 는 +0.3)
    return (b - a + 180.0) % 360.0 - 180.0


class ImuDriftTest(Node):
    def __init__(self):
        super().__init__('imu_drift_test')
        self.yaw = None
        self.gz_sum = 0.0
        self.gz_n = 0
        self.trial = 0
        self.start_time = None
        self.start_yaw = None
        self.last_print = 0
        self.results = []
        self.done = False
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.1, self.step)
        print(f'IMU 드리프트 시험: {TRIALS}회 × {DURATION:.0f}초. 로봇을 건드리지 마세요.')

    def on_imu(self, msg):
        self.yaw = quat_to_yaw_deg(msg.orientation)
        if self.start_time is not None:
            self.gz_sum += msg.angular_velocity.z
            self.gz_n += 1

    def step(self):
        if self.done:
            return
        if self.yaw is None:
            print('대기 중: /imu 데이터 없음 (T1 드라이버 켜져 있나?)')
            return
        now = time.monotonic()

        # ② 회차 시작
        if self.start_time is None:
            self.trial += 1
            self.start_time = now
            self.start_yaw = self.yaw
            self.gz_sum, self.gz_n = 0.0, 0
            self.last_print = 0
            print(f'\n[{self.trial}/{TRIALS}회] 시작 yaw = {self.start_yaw:+.2f}°')
            return

        # 10초마다 진행 상황
        elapsed = now - self.start_time
        sec = int(elapsed // 10) * 10
        if sec > self.last_print and sec < DURATION:
            self.last_print = sec
            print(f'  {sec:2d}초 … yaw = {self.yaw:+.2f}°')

        # ③ 회차 끝
        if elapsed >= DURATION:
            end_yaw = self.yaw
            drift = yaw_diff(self.start_yaw, end_yaw)
            per_min = abs(drift) / elapsed * 60.0
            gz_bias = math.degrees(self.gz_sum / self.gz_n) if self.gz_n else float('nan')
            ok = '일치' if per_min <= LIMIT else '불일치'
            self.results.append((self.trial, round(elapsed, 1), self.start_yaw, end_yaw,
                                 drift, per_min, gz_bias, ok))
            print(f'  끝 yaw = {end_yaw:+.2f}° | 드리프트 {drift:+.2f}° | '
                  f'{per_min:.2f}°/분 | 자이로 바이어스 {gz_bias:+.4f}°/s | {ok}')
            self.start_time = None
            if self.trial >= TRIALS:
                self.finish()

    def finish(self):
        self.done = True
        print('\n========== 결과 (기록표 3_IMU드리프트 시트 B·C·D열에 옮기기) ==========')
        print('회차\t시간(초)\t시작yaw\t끝yaw\t드리프트\tdeg/분\t자이로바이어스(deg/s)\t판정')
        for r in self.results:
            print(f'{r[0]}\t{r[1]}\t{r[2]:.2f}\t{r[3]:.2f}\t{r[4]:+.2f}\t{r[5]:.2f}\t{r[6]:+.4f}\t{r[7]}')
        n_ok = sum(1 for r in self.results if r[7] == '일치')
        avg = sum(r[5] for r in self.results) / len(self.results)
        bias = sum(r[6] for r in self.results) / len(self.results)
        print(f'\n통과 {n_ok}/{TRIALS} | 평균 {avg:.2f}°/분 | 평균 자이로 바이어스 {bias:+.4f}°/s')
        print('결론:', '통과 (보정 불필요)' if n_ok == TRIALS else '보정 필요 → 자이로 바이어스 빼기 검토')

        path = os.path.expanduser('~/calib_test/imu_drift_결과.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['회차', '시간(초)', '시작yaw', '끝yaw', '드리프트', 'deg/분',
                        '자이로바이어스(deg/s)', '판정'])
            w.writerows(self.results)
        print(f'저장: {path}')


def main():
    rclpy.init()
    node = ImuDriftTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n중단됨')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
