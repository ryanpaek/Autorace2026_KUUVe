#!/usr/bin/env python3
"""
[3단계] GPS 없는 '로컬 위치' 확인  — 줄자 + 키보드 조종(teleop)

시작한 곳이 (0,0), 그때 바라보던 방향이 +x.
  ① 바닥에 테이프로 시작점 표시 → 노드 실행
  ② teleop 으로 '똑바로' 2m 전진 → 출력 x≈2.00, y≈0.00 인지, 줄자로 잰 값과 비교
  ③ 차선 하나(0.52m) 옆으로 옮겨 가며 전진 → y≈0.52 인지
  ④ 원점 다시 잡기:  ros2 topic pub --once /avoid/reset std_msgs/msg/Empty
출력에 IMU yaw 와 odom yaw 를 둘 다 보여줌 → 차이가 크면 yaw_source 결정에 참고
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from limo_obstacle_avoid.avoid_core import LocalPose, wrap
from limo_obstacle_avoid.ros_common import PoseInputs, init_rclpy, shutdown_rclpy


class PoseTest(Node):
    def __init__(self):
        super().__init__('s3_local_pose')
        self.inp = PoseInputs(self)
        self.inp.on_odom = self.update
        self.pose = LocalPose()
        self.odom_yaw0 = 0.0
        self.create_subscription(Empty, '/avoid/reset', lambda _m: self.reset(), 1)
        self.create_timer(0.5, self.show)

    def reset(self):
        if not self.inp.ready():
            self.get_logger().warn('odom/imu 아직 안 들어옴 (토픽 이름 확인)')
            return
        ox, oy, yaw, _ = self.inp.get()
        self.pose.reset(ox, oy, yaw)
        self.odom_yaw0 = self.inp.odom[2]
        self.get_logger().info('원점 다시 잡음 (여기가 x=0, y=0)')

    def update(self):
        if not self.pose.ready:
            self.reset()
            return
        ox, oy, yaw, fwd = self.inp.get()
        self.pose.update(ox, oy, yaw, fwd)

    def show(self):
        if not self.pose.ready:
            return
        s = self.pose
        odom_psi = math.degrees(wrap(self.inp.odom[2] - self.odom_yaw0))
        self.get_logger().info(
            f'x={s.x:+.3f}  y={s.y:+.3f}  방향(사용중)={math.degrees(s.psi):+6.1f}°  '
            f'odom방향={odom_psi:+6.1f}°  주행거리={s.dist:.3f}m  '
            f'IMU={self.inp.mode_now()} 자이로편향={self.inp.bias:+.4f}rad/s')


def main():
    init_rclpy()
    n = PoseTest()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_rclpy(n)


if __name__ == '__main__':
    main()
