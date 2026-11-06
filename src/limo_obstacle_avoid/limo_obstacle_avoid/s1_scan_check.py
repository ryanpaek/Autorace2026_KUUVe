#!/usr/bin/env python3
"""
[1단계] 라이다 방향 확인  — 로봇은 가만히, 상자를 앞/왼쪽/오른쪽에 놓아보며 확인

화면 출력 예)
  가장 가까운 점: 0.42m  각도 +2°  → 앞   (x=+0.52, y=+0.01)
  앞 0.42 | 왼 1.30 | 오 inf | 뒤 inf

  상자를 '앞'에 놨는데 '뒤'로 나오면  → scan_yaw_offset_deg: 180.0
  상자를 '왼쪽'에 놨는데 '오'로 나오면 → scan_flip: true
  (값은 config/avoid_params.yaml 에 적고 다시 실행)
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from limo_obstacle_avoid.avoid_core import AvoidParams, scan_to_base
from limo_obstacle_avoid.ros_common import load_params, init_rclpy, shutdown_rclpy, declare


def side_name(deg):
    if -30 <= deg <= 30:
        return '앞'
    if 60 <= deg <= 120:
        return '왼쪽'
    if -120 <= deg <= -60:
        return '오른쪽'
    if abs(deg) >= 150:
        return '뒤'
    return '대각선'


class ScanCheck(Node):
    def __init__(self):
        super().__init__('s1_scan_check')
        self.p = load_params(self, AvoidParams())
        declare(self, 'scan_topic', '/scan')
        topic = self.get_parameter('scan_topic').value
        self.create_subscription(LaserScan, topic, self.cb, qos_profile_sensor_data)
        self.last = None
        self.printed_info = False
        self.create_timer(0.5, self.show)
        self.get_logger().info(f'{topic} 기다리는 중... (안 나오면 ros2 topic list 로 이름 확인)')

    def cb(self, msg):
        self.last = msg

    def show(self):
        m = self.last
        if m is None:
            return
        if not self.printed_info:
            self.printed_info = True
            self.get_logger().info(
                f'frame={m.header.frame_id}  angle_min={math.degrees(m.angle_min):.1f}°  '
                f'angle_max={math.degrees(m.angle_max):.1f}°  step={math.degrees(m.angle_increment):.2f}°  '
                f'beams={len(m.ranges)}  range=[{m.range_min:.2f},{m.range_max:.2f}]')
        pts = scan_to_base(m.ranges, m.angle_min, m.angle_increment, self.p)
        sectors = {'앞': math.inf, '왼쪽': math.inf, '오른쪽': math.inf, '뒤': math.inf}
        best = None
        for q in pts:
            if q is None:
                continue
            x, y = q
            lx = x - self.p.lidar_x
            d = math.hypot(lx, y)
            deg = math.degrees(math.atan2(y, lx))
            name = side_name(deg)
            if name in sectors:
                sectors[name] = min(sectors[name], d)
            if best is None or d < best[0]:
                best = (d, deg, x, y)
        if best:
            d, deg, x, y = best
            self.get_logger().info(
                f'가장 가까운 점: {d:.2f}m  각도 {deg:+.0f}°  → {side_name(deg)}  (x={x:+.2f}, y={y:+.2f})')
        self.get_logger().info(' | '.join(f'{k} {v:.2f}' for k, v in sectors.items()))


def main():
    init_rclpy()
    n = ScanCheck()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_rclpy(n)


if __name__ == '__main__':
    main()
