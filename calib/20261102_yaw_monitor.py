#!/usr/bin/env python3
"""
20261102_yaw_monitor.py — 센서검증 기록표 입력용 모니터 (읽기 전용, 로봇을 움직이지 않음)

흐름도
① 시작
│
├─② /odom 구독 ─→ 위치 x, y [m] + 쿼터니언 → yaw [deg] 저장
│
├─③ /imu 구독 ──→ 쿼터니언 → yaw [deg] 저장
│
└─④ 0.5초마다 화면에 한 줄 출력
    └─ odom x | odom y | odom yaw | imu yaw
       → 시험 시작/끝에 이 값을 기록표 노란 칸에 적는다

원리: ROS 자세(쿼터니언 x,y,z,w)를 평면 회전각 yaw로 바꾸는 표준 식
      yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
QoS: 센서 토픽이라 sensor_data(Best Effort) 구독. Reliable 발행자와도 연결됨.

실행 (Global 환경 터미널, 드라이버가 켜진 상태):
  python3 20261102_yaw_monitor.py
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_yaw_deg(q):
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return math.degrees(yaw)


class YawMonitor(Node):
    def __init__(self):
        super().__init__('yaw_monitor')
        self.odom = None  # (x, y, yaw_deg)
        self.imu_yaw = None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.5, self.show)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, quat_to_yaw_deg(p.orientation))

    def on_imu(self, msg):
        self.imu_yaw = quat_to_yaw_deg(msg.orientation)

    def show(self):
        if self.odom is None or self.imu_yaw is None:
            print('대기 중: /odom 또는 /imu 데이터 없음')
            return
        x, y, oyaw = self.odom
        print(f'odom x={x:+.3f} m | y={y:+.3f} m | yaw={oyaw:+7.2f}° || imu yaw={self.imu_yaw:+7.2f}°')


def main():
    rclpy.init()
    node = YawMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
