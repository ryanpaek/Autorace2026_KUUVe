#!/usr/bin/env python3
# 20261102_drive_straight_test.py — 직진 거리 시험 (★ 로봇이 실제로 움직임 ★)
#
# 흐름도
# ① 시작 → /odom 구독, /limo/ack_cmd 발행 준비
# │
# ├─② 회차 준비: "출발선에 놓고 엔터" 기다림
# │
# ├─③ 출발: 0.2 m/s 직진 (조향 0)
# │   ├─ odom 거리 ≥ 1.0 m → 정지 명령
# │   └─ 안전: 제한 시간 넘으면 무조건 정지
# │
# ├─④ 정지 후 1.5초 대기 → 끝 위치 저장
# │   └─ 정지 명령 후 더 밀린 거리(제동 거리)도 계산
# │
# ├─⑤ 줄자 실측(cm) 입력 → 오차·비율·판정 출력
# │
# ├─⑥ 5회 될 때까지 ②~⑤ 반복
# │
# └─⑦ 결과표 출력 + ~/calib_test/straight_결과.csv 저장 → 종료
#
# 원리: odom 거리 = 바퀴 엔코더로 계산한 거리. 줄자(실제)와 비교해서
#       비율(실제÷odom)을 구하면 그게 거리 보정 계수.
# 판정: |오차| ≤ 5 cm → 일치 (규정집 정지선 ±5 cm 기준)
#
# 실행 (Global 환경 터미널, T1 드라이버 켜진 상태, T3 비상정지 준비):
#   python3 ~/calib_test/20261102_drive_straight_test.py
# Ctrl+C 누르면 즉시 정지 명령 보내고 종료.
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive

TRIALS = 5
SPEED = 0.2          # m/s
TARGET = 1.0         # m (odom 기준 목표 거리)
TOL_CM = 5.0         # 판정 기준 (cm)
TIMEOUT = TARGET / SPEED * 2.0 + 2.0   # 안전 제한 시간 (초) = 12초


class StraightTest(Node):
    def __init__(self):
        super().__init__('straight_test')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDrive, '/limo/ack_cmd', 10)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def send(self, speed):
        m = AckermannDrive()
        m.speed = float(speed)
        m.steering_angle = 0.0
        self.pub.publish(m)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def stop(self):
        for _ in range(10):
            self.send(0.0)
            self.spin_for(0.05)


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def run_trial(node, n):
    node.spin_for(0.5)                      # 최신 odom 받기
    start = node.pose
    print(f'  출발! 시작 odom x={start[0]:+.3f} y={start[1]:+.3f}')
    t0 = time.monotonic()
    reason = '목표 거리 도달'
    while True:
        node.send(SPEED)
        node.spin_for(0.05)
        if dist(start, node.pose) >= TARGET:
            break
        if time.monotonic() - t0 > TIMEOUT:
            reason = '⚠ 제한 시간 초과 (바퀴 헛돎/막힘 확인)'
            break
    at_stop = dist(start, node.pose)
    node.stop()
    node.spin_for(1.5)                      # 완전히 멈출 때까지
    end = node.pose
    odom_cm = dist(start, end) * 100.0
    brake_cm = odom_cm - at_stop * 100.0
    print(f'  정지 ({reason}) | odom 거리 {odom_cm:.1f} cm | 정지 명령 후 더 밀림 {brake_cm:.1f} cm')
    return start, end, odom_cm, brake_cm


def ask_real_cm():
    while True:
        s = input('  줄자로 잰 실제 이동 거리(cm)를 입력하고 엔터 (건너뛰기 s): ').strip()
        if s.lower() == 's':
            return None
        try:
            return float(s)
        except ValueError:
            print('  숫자로 입력해 주세요 (예: 98.5)')


def main():
    rclpy.init()
    node = StraightTest()
    results = []
    try:
        print(f'직진 시험: {TRIALS}회, {SPEED} m/s, odom 목표 {TARGET} m')
        print('⚠ 앞에 2 m 이상 비워 두고, T3 비상정지를 준비하세요.')
        node.spin_for(1.0)
        if node.pose is None:
            print('/odom 데이터 없음 → T1 드라이버 확인 후 다시 실행')
            return
        for n in range(1, TRIALS + 1):
            input(f'\n[{n}/{TRIALS}회] 로봇 앞바퀴를 출발선에 맞추고 엔터 → 바로 출발합니다')
            start, end, odom_cm, brake_cm = run_trial(node, n)
            real = ask_real_cm()
            if real is None:
                print('  이 회차는 건너뜀')
                continue
            err = odom_cm - real
            ratio = real / odom_cm if odom_cm else float('nan')
            ok = '일치' if abs(err) <= TOL_CM else '불일치'
            print(f'  → 오차 {err:+.1f} cm ({err / real * 100:+.1f}%) | 비율 {ratio:.3f} | {ok}')
            results.append((n, SPEED, real, start[0], start[1], end[0], end[1],
                            round(odom_cm, 1), round(brake_cm, 1), round(err, 1),
                            round(ratio, 4), ok))
    except KeyboardInterrupt:
        print('\n중단됨 → 정지 명령 보냄')
    finally:
        node.stop()

    if results:
        print('\n========== 결과 (기록표 1_직진거리 시트 B~G열에 옮기기) ==========')
        print('회차\t속도\t실제cm\t시작x\t시작y\t끝x\t끝y\todom cm\t제동cm\t오차cm\t비율\t판정')
        for r in results:
            print(f'{r[0]}\t{r[1]}\t{r[2]:.1f}\t{r[3]:.3f}\t{r[4]:.3f}\t{r[5]:.3f}\t{r[6]:.3f}'
                  f'\t{r[7]:.1f}\t{r[8]:.1f}\t{r[9]:+.1f}\t{r[10]:.3f}\t{r[11]}')
        n_ok = sum(1 for r in results if r[11] == '일치')
        avg_ratio = sum(r[10] for r in results) / len(results)
        avg_brake = sum(r[8] for r in results) / len(results)
        print(f'\n통과 {n_ok}/{len(results)} | 평균 비율 {avg_ratio:.3f} | 평균 제동 밀림 {avg_brake:.1f} cm')
        if len(results) < TRIALS:
            print(f'결론: 입력 중 ({len(results)}/{TRIALS})')
        elif n_ok == TRIALS:
            print('결론: 통과 (보정 불필요)')
        else:
            print(f'결론: 보정 필요 → odom 거리 × {avg_ratio:.3f}')
        path = os.path.expanduser('~/calib_test/straight_결과.csv')
        with open(path, 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(['회차', '속도', '실제cm', '시작x', '시작y', '끝x', '끝y',
                        'odom cm', '제동cm', '오차cm', '비율', '판정'])
            w.writerows(results)
        print(f'저장: {path}')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
