import csv, math, sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class Fusion:
    """odom 속도 + IMU 자이로로 만든 위치 (방향은 IMU 기준). 시작 1초는 정지 상태로 보고 자이로 편향을 측정."""
    CAL = 1.0
    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.bias, self.n = 0.0, 0
        self.t_cal0 = self.t_imu = self.t_odom = None
    @property
    def ready(self):
        return self.t_cal0 is not None and self.t_imu is not None and self.t_imu - self.t_cal0 >= self.CAL
    def on_imu(self, t, gz):
        if self.t_cal0 is None:
            self.t_cal0 = t
        if t - self.t_cal0 < self.CAL:
            self.n += 1
            self.bias += (gz - self.bias) / self.n
        elif self.t_imu is not None:
            self.yaw += (gz - self.bias) * (t - self.t_imu)
        self.t_imu = t
    def on_odom(self, t, v):
        if self.ready and self.t_odom is not None:
            dt = t - self.t_odom
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
        self.t_odom = t

class Recorder(Node):
    def __init__(self, path):
        super().__init__('path_recorder')
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'x', 'y', 'yaw', 'vx', 'wz', 'cmd_vx', 'cmd_wz', 'xf', 'yf', 'yawf'])
        self.cmd = (0.0, 0.0)
        self.origin = None
        self.t0 = None
        self.fu = Fusion()
        self.rows = 0
        self.last = None
        self.failed = False
        self.t_begin = self.now()
        self.tick = 0
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
        self.fu.on_imu(self.stamp(m), m.angular_velocity.z)

    def on_odom(self, m):
        t = self.stamp(m)
        p = m.pose.pose.position
        yaw = yaw_of(m.pose.pose.orientation)
        self.fu.on_odom(t, m.twist.twist.linear.x)
        if self.origin is None:
            self.origin = (p.x, p.y, yaw)
            self.t0 = t
        x0, y0, yaw0 = self.origin
        dx, dy = p.x - x0, p.y - y0
        xr = math.cos(yaw0) * dx + math.sin(yaw0) * dy
        yr = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
        yawr = wrap(yaw - yaw0)
        fu = [self.fu.x, self.fu.y, wrap(self.fu.yaw)] if self.fu.ready else ['', '', '']
        self.w.writerow([round(t - self.t0, 3), xr, yr, yawr,
                         m.twist.twist.linear.x, m.twist.twist.angular.z, *self.cmd, *fu])
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
        self.tick += 1
        if self.tick % 2 == 0 and self.last:
            xr, yr, yawr, fu = self.last
            imu = f'IMU yaw {math.degrees(fu[2]):.0f}°' if fu[0] != '' else 'IMU 보정 중/없음'
            print(f'  기록 {self.rows}줄  x={xr:.2f} y={yr:.2f}  odom yaw {math.degrees(yawr):.0f}°  {imu}')

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
            print(f'저장 완료: {node.rows}줄, 끝 위치 x={xr:.3f} y={yr:.3f}')
            if fu[0] != '':
                print(f'끝 방향: odom {math.degrees(yawr):.1f}° / IMU {math.degrees(fu[2]):.1f}° (크게 다르면 odom 방향이 틀어진 것)')
            else:
                print('IMU 값이 없어 xf, yf 열은 비어 있음 (재생은 --pose odom만 가능)')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
