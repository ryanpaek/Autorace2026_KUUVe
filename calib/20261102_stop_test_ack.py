#!/usr/bin/env python3
# 20261102_stop_test_ack.py — 속도별 정지 시간·거리 정밀 측정 (★ 로봇이 실제로 움직임 ★)
#
# 흐름도
# ① 시작 → /odom 구독, /limo/ack_cmd 직접 발행 (브리지 안 거침 → 변환 지연 없음)
# │
# ├─② 회차 준비: "출발선에 맞추고 엔터" (속도 0.2 → 0.3 → 0.4 순, 각 5회)
# │
# ├─③ 가속: 목표 속도로 직진 (50 Hz로 명령)
# │   └─ 속도가 목표의 ±5% 안에서 0.3초 유지(=정속) + 0.5 m 지난 순간 → 정지 명령
# │       (0.8 m까지 정속이 안 되면 그냥 정지하고 '정속 미달' 표시)
# │
# ├─④ 정지 후 분석 (odom 50 Hz 기록 사용)
# │   ├─ 지연  = 정지 명령 → 속도가 90% 아래로 떨어지기 시작할 때까지
# │   ├─ 정지 시간 = 정지 명령 → 속도 < 0.01 m/s (0.3초 유지) 처음 된 때
# │   └─ 제동 거리 = 정지 명령 순간 위치 → 최종 정지 위치
# │
# ├─⑤ 줄자: 출발선 → 앞바퀴 최종 위치(cm) 입력 → odom 총거리와 비교(미끄러짐 확인)
# │
# ├─⑥ 15회 반복 → 속도별 평균 ± 표준편차
# │
# └─⑦ 저장: ~/calib_test/stop_결과.csv (요약), ~/calib_test/stop_raw/*.csv (회차별 원자료)
#
# 원리: 정지선 ±5 cm(규정집 미션 1·3)를 맞추려면 "정지선 몇 cm 앞에서 정지 명령을 줄지"를
#       속도별 제동 거리로 정한다. 흩어짐(표준편차)이 작아야 매번 같은 자리에 선다.
# 정밀도: odom 50 Hz → 0.4 m/s에서 샘플 간격 약 8 mm, 시간 분해능 20 ms.
#
# 실행 (Global 환경, T1 애커만 드라이버만 켜짐, 브리지·teleop·다른 제어 노드는 끔, 조종기 끔):
#   python3 ~/calib_test/20261102_stop_test_ack.py
#   python3 ~/calib_test/20261102_stop_test_ack.py 0.3 5    ← 한 속도만 5회
import csv
import math
import os
import statistics as st
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive

TRIGGER = 0.5        # m, 이 거리 이후 정속이면 정지 명령
FORCE = 0.8          # m, 정속이 안 돼도 여기서는 무조건 정지
STEADY_TOL = 0.05    # 정속 판정 ±5%
STEADY_HOLD = 0.3    # s
STOP_V = 0.01        # m/s, 이 아래면 멈춘 것
STOP_HOLD = 0.3      # s
TIMEOUT = 15.0       # s, 안전 제한
RATE = 0.02          # s, 명령 주기 50 Hz
OUT = os.path.expanduser('~/calib_test')


class StopTest(Node):
    def __init__(self):
        super().__init__('stop_test_ack')
        self.pos = None
        self.last_odom = 0.0
        self.twist_v = 0.0
        self.trace = []             # (t, x, y, v_twist)
        self.win = deque(maxlen=6)  # 속도 계산용 (0.1초 창)
        self.v = 0.0
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDrive, '/limo/ack_cmd', 10)

    def on_odom(self, m):
        t = time.monotonic()
        self.last_odom = t
        p = m.pose.pose.position
        self.pos = (p.x, p.y)
        self.twist_v = m.twist.twist.linear.x
        self.win.append((t, p.x, p.y))
        if len(self.win) >= 2:
            t0, x0, y0 = self.win[0]
            dt = t - t0
            self.v = math.hypot(p.x - x0, p.y - y0) / dt if dt > 0 else 0.0
        # 드라이버가 twist를 채우면 그 값을 우선 사용 (더 즉각적)
        if abs(self.twist_v) > 1e-4:
            self.v = abs(self.twist_v)
        self.trace.append((t, p.x, p.y, self.v))

    def send(self, speed):
        m = AckermannDrive()
        m.speed = float(speed)
        m.steering_angle = 0.0
        self.pub.publish(m)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)


def dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def run_trial(node, speed):
    node.spin_for(0.3)
    node.trace = []
    start = node.pos
    t0 = time.monotonic()
    steady_since = None
    flag = ''
    # ③ 가속 + 정속 주행
    while True:
        node.send(speed)
        node.spin_for(RATE)
        d = dist(start, node.pos)
        now = time.monotonic()
        if abs(node.v - speed) <= STEADY_TOL * speed:
            steady_since = steady_since or now
        else:
            steady_since = None
        steady = steady_since is not None and now - steady_since >= STEADY_HOLD
        if d >= TRIGGER and steady:
            break
        if d >= FORCE:
            flag = '정속 미달'
            break
        if now - node.last_odom > 0.2:
            flag = '⚠ /odom 끊김 → 비상 정지'
            break
        if now - t0 > FORCE / speed * 2.0 + 2.0:
            flag = '시간 초과'
            break
    # 정지 명령 순간
    t_cmd = time.monotonic()
    p_cmd = node.pos
    v_cmd = node.v
    d_cmd = dist(start, p_cmd)
    # ④ 정지 명령을 50 Hz로 계속 보내며 완전히 설 때까지 관찰
    below_since = None
    t_stop = None
    while time.monotonic() - t_cmd < 5.0:
        node.send(0.0)
        node.spin_for(RATE)
        now = time.monotonic()
        if node.v < STOP_V:
            below_since = below_since or now
            if now - below_since >= STOP_HOLD:
                t_stop = below_since
                break
        else:
            below_since = None
    node.spin_for(0.5)
    p_end = node.pos
    brake_cm = dist(p_cmd, p_end) * 100
    total_cm = dist(start, p_end) * 100
    stop_s = (t_stop - t_cmd) if t_stop else float('nan')
    # 지연: 명령 후 속도가 90% 아래로 처음 떨어진 시각
    delay_s = float('nan')
    for t, x, y, v in node.trace:
        if t > t_cmd and v < 0.9 * v_cmd:
            delay_s = t - t_cmd
            break
    if not t_stop:
        flag = (flag + ' / ' if flag else '') + '정지 판정 실패'
    return dict(v_cmd=v_cmd, d_cmd=d_cmd, brake_cm=brake_cm, total_cm=total_cm,
                stop_s=stop_s, delay_s=delay_s, flag=flag, trace=list(node.trace),
                t_cmd=t_cmd)


def ask_tape():
    s = input('  줄자: 출발선 → 앞바퀴 최종 위치(cm), 건너뛰려면 엔터: ').strip()
    try:
        return float(s) if s else None
    except ValueError:
        print('  숫자가 아니라 건너뜀')
        return None


def save_raw(res, speed, n):
    os.makedirs(os.path.join(OUT, 'stop_raw'), exist_ok=True)
    path = os.path.join(OUT, 'stop_raw', f'stop_{speed:.1f}_{n}.csv')
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_from_cmd_s', 'x', 'y', 'v'])
        for t, x, y, v in res['trace']:
            w.writerow([f'{t - res["t_cmd"]:.3f}', f'{x:.4f}', f'{y:.4f}', f'{v:.4f}'])


def main():
    if len(sys.argv) >= 3:
        plan = [(float(sys.argv[1]), int(sys.argv[2]))]
    else:
        plan = [(0.2, 5), (0.3, 5), (0.4, 5)]
    rclpy.init()
    node = StopTest()
    rows = []
    try:
        print('정지 시험:', ', '.join(f'{s} m/s × {k}회' for s, k in plan))
        print(f'⚠ 앞에 {FORCE + 1.0:.1f} m 이상 비우기, 조종기·브리지·teleop 끄기, T3 비상정지 준비')
        node.spin_for(1.0)
        if node.pos is None:
            print('/odom 데이터 없음 → T1 드라이버 확인')
            return
        for speed, k in plan:
            for n in range(1, k + 1):
                input(f'\n[{speed} m/s  {n}/{k}회] 앞바퀴를 출발선에 맞추고 엔터 → 바로 출발')
                r = run_trial(node, speed)
                print(f'  정지 명령: {r["d_cmd"] * 100:.1f} cm 지점, 속도 {r["v_cmd"]:.3f} m/s')
                print(f'  → 지연 {r["delay_s"]:.2f} s | 정지 시간 {r["stop_s"]:.2f} s | '
                      f'제동 거리 {r["brake_cm"]:.1f} cm  {r["flag"]}')
                tape = ask_tape()
                ratio = tape / r['total_cm'] if tape and r['total_cm'] else float('nan')
                if tape:
                    print(f'  odom 총거리 {r["total_cm"]:.1f} cm vs 줄자 {tape:.1f} cm '
                          f'(비율 {ratio:.3f}) → 보정 제동 거리 ≈ {r["brake_cm"] * ratio:.1f} cm')
                save_raw(r, speed, n)
                rows.append([speed, n, round(r['v_cmd'], 3), round(r['d_cmd'] * 100, 1),
                             round(r['delay_s'], 3), round(r['stop_s'], 3),
                             round(r['brake_cm'], 1), round(r['total_cm'], 1),
                             tape if tape else '', '' if math.isnan(ratio) else round(ratio, 4),
                             r['flag']])
    except KeyboardInterrupt:
        print('\n중단됨 → 정지 명령 보냄')
    finally:
        for _ in range(20):
            node.send(0.0)
            time.sleep(0.02)

    if rows:
        os.makedirs(OUT, exist_ok=True)
        path = os.path.join(OUT, 'stop_결과.csv')
        new = not os.path.exists(path)
        with open(path, 'a', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['명령속도', '회차', '명령순간속도', '명령위치cm', '지연s', '정지시간s',
                            '제동거리cm', 'odom총거리cm', '줄자cm', '줄자/odom', '비고'])
            w.writerows(rows)
        print('\n========== 속도별 요약 (평균 ± 표준편차, 비고 없는 회차만) ==========')
        for speed in sorted({r[0] for r in rows}):
            ok = [r for r in rows if r[0] == speed and not r[10]]
            if not ok:
                print(f'{speed} m/s: 정상 회차 없음')
                continue
            def ms(i):
                vals = [r[i] for r in ok if not math.isnan(r[i])]
                if not vals:
                    return 'nan'
                sd = st.stdev(vals) if len(vals) > 1 else 0.0
                return f'{st.mean(vals):.2f}±{sd:.2f}'
            print(f'{speed} m/s (n={len(ok)}): 지연 {ms(4)} s | 정지시간 {ms(5)} s | 제동거리 {ms(6)} cm')
        print(f'\n저장: {path} (이어 쌓임)\n원자료: {OUT}/stop_raw/')
        print('정지선 미션: "정지선 (평균 제동거리) cm 앞에서 정지 명령". 표준편차가 2 cm 넘으면 ±5 cm 판정이 불안정.')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
