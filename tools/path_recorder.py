"""키보드로 몬 길을 CSV로 기록 (재생: path_follower.py, 점검: path_check.py).

  python3 path_recorder.py m7_left.csv
  처음 1초는 차를 움직이지 말 것 (자이로 편향 측정). 끝나면 k로 세우고 Ctrl+C.

열:
  t                 시간 (s)
  x, y, yaw         odom 위치·방향 (출발점 기준)
  vx, wz            odom 속도값
  cmd_vx, cmd_wz    /cmd_vel 명령
  xs, ys, yaws, s   ★ 재생에 쓰는 위치: odom이 잰 거리 + IMU 방향 (path_common.PoseFusion)
  gz                IMU 자이로 z
"""
import csv
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from path_common import PoseFusion, wrap, yaw_of


class Recorder(Node):
    def __init__(self, path):
        super().__init__('path_recorder')
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'yaw', 'vx', 'wz', 'cmd_vx', 'cmd_wz', 'xs', 'ys', 'yaws', 's', 'gz'])
        self.cmd = (0.0, 0.0)
        self.origin = None
        self.t0 = None
        self.gz = 0.0
        self.fu = PoseFusion()
        self.rows = 0
        self.last = None
        self.failed = False
        self.t_begin = self.now()
        self.tick = 0
        self.warned = None
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_timer(1.0, self.status)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return t if t > 0 else self.now()

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_imu(self, m):
        self.gz = m.angular_velocity.z
        self.fu.on_imu(self.stamp(m), self.gz)

    def on_odom(self, m):
        t = self.stamp(m)
        p = m.pose.pose.position
        yaw = yaw_of(m.pose.pose.orientation)
        self.fu.on_odom(t, p.x, p.y, yaw, m.twist.twist.linear.x)
        if self.origin is None:
            self.origin = (p.x, p.y, yaw)
            self.t0 = t
        x0, y0, yaw0 = self.origin
        dx, dy = p.x - x0, p.y - y0
        xr = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        yr = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        yawr = wrap(yaw - yaw0)
        fu = [self.fu.x, self.fu.y, self.fu.yaw, self.fu.s] if self.fu.ready else ['', '', '', '']
        self.w.writerow([round(t - self.t0, 3), xr, yr, yawr,
                         m.twist.twist.linear.x, m.twist.twist.angular.z, *self.cmd, *fu, self.gz])
        self.rows += 1
        self.last = (xr, yr, yawr, fu)
        if self.rows % 50 == 0:
            self.f.flush()

    def status(self):
        t = self.now()
        if self.origin is None:
            if t - self.t_begin > 5.0:
                print('[오류] 5초 동안 /odom 안 옴: 드라이버, 조종기 끄기, 브리지 확인')
                self.failed = True
            return
        if self.fu.t_imu is None and t - self.t_begin > 3.0 and not self.warned:
            self.warned = 'imu'
            print('[오류] /imu 안 옴 → xs, ys 열이 비어서 재생은 --pose odom만 가능')
        bad = self.fu.sign_problem()
        if bad and self.warned != bad:
            self.warned = bad
            print(f'[경고] {bad} → 이대로면 재생이 안 맞음')
        self.tick += 1
        if self.tick % 2 == 0 and self.last:
            xr, yr, yawr, fu = self.last
            if fu[0] != '':
                print(f'  기록 {self.rows}줄  거리 {self.fu.dist:.2f}m  x={fu[0]:.2f} y={fu[1]:+.2f}  '
                      f'방향 IMU {math.degrees(fu[2]):+.0f}° / odom {math.degrees(yawr):+.0f}°')
            else:
                print(f'  기록 {self.rows}줄  (자이로 보정 중)  odom x={xr:.2f} y={yr:+.2f}')


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'lap_log.csv'
    rclpy.init()
    node = Recorder(path)
    print('기록 시작: 처음 1초는 차를 움직이지 마세요(자이로 보정). 끝나면 k로 세운 뒤 Ctrl+C')
    try:
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        if node.last:
            xr, yr, yawr, fu = node.last
            print(f'저장 완료: {node.rows}줄 → {path}')
            if fu[0] != '':
                print(f'끝 위치 (재생 기준) x={fu[0]:.3f} y={fu[1]:+.3f} 방향 {math.degrees(fu[2]):+.1f}°  '
                      f'/ odom x={xr:.3f} y={yr:+.3f} 방향 {math.degrees(yawr):+.1f}°')
            bad = node.fu.sign_problem()
            if bad:
                print(f'[경고] {bad}')
            if node.fu.v_bad == 0 and node.fu.sign_bad_turn == 0:
                print(f'부호 점검 통과 (회전 비교 {node.fu.sign_ok_turn}번)')
            print(f'점검: python3 path_check.py {path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
