"""주차 라이다 확인 도구 — 차는 안 움직인다. /scan만 읽어서 1초마다 화면에 출력.

무엇을 보나
  - 라이다가 정차 리모를 "덩어리"로 잡는지 (위치)
  - 두 리모를 "짝"으로 인정하는지 (사이 거리가 1040mm ± 150 안인지)
  - 차 정면 물체가 0도 근처에 찍히는지 (laser_yaw 확인)

주차 노드(autorace_mission.parking_node)의 scan_clusters, find_pair를 그대로 써서 노드가 보는 것과 똑같이 본다.
실행 (빌드·source 후): python3 tools/parking_scan_check.py  [라이다->앞바퀴 축 mm, 앞 +, 기본 -47]
순서: docs/CHANGES_20261109_parking.md "실차 시험" 2번
검증 한계: ROS에서 안 돌려 봄 (미검증). 문법 검사만 함.

흐름도
① /scan 받기 (1초에 한 번만 출력)
│
├─② 덩어리 찾기 (주차 노드와 같은 함수)
│   └─ 각 덩어리: 앞바퀴 기준 앞(+)/왼(+) 위치 mm
│
├─③ 짝 찾기 (오른쪽 두 덩어리, 같은 줄, 사이 1040±150mm)
│   ├─ 있음 → 중점 위치 출력
│   └─ 없음 → "짝 없음"
│
└─④ 정면(±5도) 가장 가까운 점의 각도·거리 출력 (laser_yaw 확인용)
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from autorace_mission.parking_node import find_pair, scan_clusters

FRONT_AXLE_FROM_LIDAR_MM = float(sys.argv[1]) if len(sys.argv) > 1 else -47.0


class ScanCheck(Node):
    """/scan을 받아 덩어리·짝·정면 각도를 1초마다 출력."""

    def __init__(self):
        super().__init__('parking_scan_check')
        self.last = 0.0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, s):
        """스캔 한 장 처리."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last < 1.0:
            return
        self.last = now
        cl = scan_clusters(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max,
                           FRONT_AXLE_FROM_LIDAR_MM, 0.0, max_dist_mm=1500.0)
        print('--------------------------------------------------')
        print(f'덩어리 {len(cl)}개 (앞바퀴 기준 mm, 앞 +, 왼 +):')
        for x, y in sorted(cl, key=lambda c: c[0]):
            side = '오른쪽' if y < 0 else '왼쪽'
            print(f'   앞 {x:7.0f}  옆 {y:7.0f}  ({side} {abs(y):.0f}mm)')
        pair = find_pair(cl, 1300.0, 1040.0, 150.0)
        if pair is None:
            print('짝: 없음')
        else:
            mx, my, vx, vy = pair
            print(f'짝: 있음 -> 중점 앞 {mx:.0f} / 옆 {my:.0f} mm (칸 가운데로 쓰는 점)')
        best = None
        for i, r in enumerate(s.ranges):
            a = s.angle_min + s.angle_increment * i
            if abs(a) <= math.radians(5.0) and math.isfinite(r) and s.range_min <= r <= s.range_max:
                if best is None or r < best[1]:
                    best = (a, r)
        if best:
            print(f'정면 ±5도 가장 가까운 점: 각도 {math.degrees(best[0]):+.1f}도, 거리 {best[1]:.3f} m')


def main():
    """실행."""
    rclpy.init()
    node = ScanCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
