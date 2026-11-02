#!/usr/bin/env python3
# 20261102_drive_curve_test.py — 커브 회전 시험 (★ 로봇이 실제로 움직임 ★)
#
# 흐름도
# ① 시작 → /odom, /imu 구독, /limo/ack_cmd 발행 준비
# │
# ├─② 회차 준비: "옆면을 타일선에 맞추고 엔터" 기다림
# │
# ├─③ 출발: 0.2 m/s + 최대 조향(0.40 rad)으로 커브
# │   ├─ IMU yaw가 90° 바뀌면 → 정지 명령
# │   └─ 안전: 제한 시간 넘으면 무조건 정지
# │
# ├─④ 정지 후 1.5초 대기 → 결과 계산
# │   ├─ odom 회전각 vs IMU 회전각 → 비율
# │   ├─ 회전 반지름 R = 이동 거리 ÷ 회전각(rad)
# │   └─ 축거 추정 L = R × tan(조향각)   ← 팀 코드 축거 0.2 m 확인용
# │
# ├─⑤ (선택) 타일선으로 실제 각도 확인 입력
# │
# ├─⑥ 5회 될 때까지 ②~⑤ 반복
# │
# └─⑦ 결과표 출력 + ~/calib_test/curve_결과.csv 저장 → 종료
#
# 원리:
#  - 애커만은 제자리 회전이 안 되므로 커브로 90° 돈다.
#  - IMU는 드리프트 시험에서 믿을 만하다고 확인함 → IMU 각도를 기준으로 삼는다.
#  - odom 회전각은 드라이버가 (조향각, 축거)로 계산 → IMU와 다르면 드라이버 축거/조향 값이 틀린 것.
#  - 자전거 모델: R = L / tan(δ)  →  L = R · tan(δ)
#
# 실행 (Global 환경 터미널, T1 드라이버 켜짐, 조종기 끔, T3 비상정지 준비):
#   python3 ~/calib_test/20261102_drive_curve_test.py
# Ctrl+C → 즉시 정지 명령 후 종료.
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ackermann_msgs.msg import AckermannDrive

TRIALS = 5
SPEED = 0.2          # m/s
STEER = 0.40         # rad, 팀 코드 상한과 같음 (위키 최대 0.42)
DIRECTION = -1       # -1 = 오른쪽(시계방향, 대회 트랙 방향), +1 = 왼쪽
TARGET_DEG = 90.0    # IMU 기준 목표 회전각
TIMEOUT = 15.0       # 안전 제한 시간 (초)


def quat_to_yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def yaw_diff(a, b):
    return (b - a + 180.0) % 360.0 - 180.0


class CurveTest(Node):
    def __init__(self):
        super().__init__('curve_test')
        self.pose = None       # (x, y, yaw_deg)
        self.imu_yaw = None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDrive, '/limo/ack_cmd', 10)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, quat_to_yaw_deg(p.orientation))

    def on_imu(self, msg):
        self.imu_yaw = quat_to_yaw_deg(msg.orientation)

    def send(self, speed, steer):
        m = AckermannDrive()
        m.speed = float(speed)
        m.steering_angle = float(steer)
        self.pub.publish(m)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(10):
            self.send(0.0, 0.0)
            self.spin_for(0.05)


def run_trial(node):
    node.spin_for(0.5)
    o0, i0 = node.pose, node.imu_yaw
    steer = STEER * DIRECTION
    path = 0.0
    last = o0
    t0 = time.monotonic()
    reason = 'IMU 90° 도달'
    print(f'  출발! 시작 odom yaw={o0[2]:+.2f}° | imu yaw={i0:+.2f}°')
    while True:
        node.send(SPEED, steer)
        node.spin_for(0.05)
        cur = node.pose
        path += math.hypot(cur[0] - last[0], cur[1] - last[1])
        last = cur
        if abs(yaw_diff(i0, node.imu_yaw)) >= TARGET_DEG:
            break
        if time.monotonic() - t0 > TIMEOUT:
            reason = '⚠ 제한 시간 초과 (조종기 모드/막힘 확인)'
            break
    node.stop()
    node.spin_for(1.5)
    cur = node.pose
    path += math.hypot(cur[0] - last[0], cur[1] - last[1])
    d_odom = abs(yaw_diff(o0[2], cur[2]))
    d_imu = abs(yaw_diff(i0, node.imu_yaw))
    ratio = d_odom / d_imu if d_imu else float('nan')
    R = path / math.radians(d_imu) if d_imu else float('nan')
    L = R * math.tan(STEER)
    print(f'  정지 ({reason})')
    print(f'  odom 회전 {d_odom:.1f}° | IMU 회전 {d_imu:.1f}° | 비율(odom÷IMU) {ratio:.3f}')
    print(f'  이동 거리 {path * 100:.1f} cm | 회전 반지름 R {R * 100:.1f} cm | 축거 추정 L {L * 100:.1f} cm')
    return d_odom, d_imu, ratio, path * 100, R * 100, L * 100


def ask_tile(d_imu):
    s = input('  (선택) 타일 확인: 앞틈 뒤틈 두점간격(cm) 띄어쓰기로 입력, 건너뛰려면 엔터: ').strip()
    if not s:
        return None
    try:
        f, r, gap = (float(v) for v in s.split())
        err = math.degrees(math.atan2(abs(f - r), gap))
    except ValueError:
        print('  형식 오류 → 건너뜀 (예: 2.0 0.5 20)')
        return None
    if err < 0.05:
        return 90.0
    k = input('  덜 돌았으면 1, 더 돌았으면 2: ').strip()
    real = 90.0 - err if k == '1' else 90.0 + err
    print(f'  실제 회전 ≈ {real:.1f}° | IMU 오차 {d_imu - real:+.1f}°')
    return real


def main():
    rclpy.init()
    node = CurveTest()
    results = []
    try:
        side = '오른쪽' if DIRECTION < 0 else '왼쪽'
        print(f'커브 시험: {TRIALS}회, {SPEED} m/s, 조향 {STEER} rad {side}, IMU {TARGET_DEG:.0f}°에서 정지')
        print('⚠ 1.5 m × 1.5 m 이상 비우고, 조종기 끄고, T3 비상정지를 준비하세요.')
        node.spin_for(1.0)
        if node.pose is None or node.imu_yaw is None:
            print('/odom 또는 /imu 데이터 없음 → T1 드라이버 확인 후 다시 실행')
            return
        for n in range(1, TRIALS + 1):
            input(f'\n[{n}/{TRIALS}회] 로봇 옆면을 타일선에 맞추고 엔터 → 바로 출발합니다')
            d_odom, d_imu, ratio, path_cm, R_cm, L_cm = run_trial(node)
            real = ask_tile(d_imu)
            results.append((n, round(d_odom, 2), round(d_imu, 2), round(ratio, 4),
                            round(path_cm, 1), round(R_cm, 1), round(L_cm, 1),
                            '' if real is None else round(real, 1)))
    except KeyboardInterrupt:
        print('\n중단됨 → 정지 명령 보냄')
    finally:
        node.stop()

    if results:
        print('\n========== 결과 (기록표 2_회전 시트) ==========')
        print('회차\todom°\tIMU°\todom÷IMU\t거리cm\tR cm\tL추정cm\t실제°')
        for r in results:
            print('\t'.join(str(v) for v in r))
        k = len(results)
        avg_ratio = sum(r[3] for r in results) / k
        avg_R = sum(r[5] for r in results) / k
        avg_L = sum(r[6] for r in results) / k
        print(f'\n평균 odom÷IMU {avg_ratio:.3f} | 평균 R {avg_R:.1f} cm | 평균 축거 추정 {avg_L:.1f} cm')
        print('해석: odom÷IMU가 1에서 멀면 드라이버의 축거/조향 설정이 실제와 다름.')
        print('      축거 추정을 팀 코드 값(20 cm)과 줄자 실측 축거와 비교할 것.')
        path = os.path.expanduser('~/calib_test/curve_결과.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['회차', 'odom회전deg', 'IMU회전deg', 'odom/IMU', '거리cm',
                        'R cm', '축거추정cm', '실제deg'])
            w.writerows(results)
        print(f'저장: {path}')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
