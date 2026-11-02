import argparse, csv, math, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

WHEELBASE = 0.21         # m, 축거 (ackermann 타입일 때 조향각 계산용)
MAX_CURV = 1 / 0.53      # 최소 회전반경 약 53cm
SPACING = 0.05           # 경로 점 간격 m

def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def load_path(path, pose):
    cx, cy = ('x', 'y') if pose == 'odom' else ('xf', 'yf')
    pts, last = [], None
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                x, y = float(r[cx]), float(r[cy])
            except (KeyError, TypeError, ValueError):
                continue
            if not pts or math.hypot(x - pts[-1][0], y - pts[-1][1]) >= SPACING:
                pts.append((x, y))
            last = (x, y)
    if pts and math.hypot(last[0] - pts[-1][0], last[1] - pts[-1][1]) > 0.005:
        pts.append(last)
    return pts

class Fusion:
    """odom 속도 + IMU 자이로로 만든 위치 (path_recorder.py와 동일). 시작 1초는 차를 움직이지 말 것."""
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

class Follower(Node):
    def __init__(self, a):
        super().__init__('path_follower')
        self.a = a
        self.pts = load_path(a.csv, a.pose)
        self.idx = 0
        self.finished = False
        self.origin = None
        self.fu = Fusion()
        self.last_odom = None
        self.last_print = 0.0
        self.checked = False
        self.ok_to_move = False
        self.t_begin = self.now()
        if a.type == 'twist':
            self.msg_cls, self.type_name = Twist, 'geometry_msgs/msg/Twist'
        else:
            try:
                from ackermann_msgs.msg import AckermannDrive
            except ImportError:
                print('[오류] ackermann_msgs를 불러올 수 없음: --type twist로 쓰거나 패키지 확인')
                self.pts = []
                self.finished = True
                return
            self.msg_cls, self.type_name = AckermannDrive, 'ackermann_msgs/msg/AckermannDrive'
        self.pub = self.create_publisher(self.msg_cls, a.topic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.1, self.watchdog)
        if len(self.pts) < 10:
            print(f'[오류] 경로 점이 {len(self.pts)}개뿐: CSV가 짧거나 비었음'
                  + (' (xf, yf 열이 없으면 --pose odom)' if a.pose == 'imu' else ''))
            self.finished = True

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return t if t > 0 else self.now()

    def send(self, v, curv):
        m = self.msg_cls()
        if self.a.type == 'twist':
            m.linear.x = v
            m.angular.z = v * curv
        else:
            m.speed = v
            m.steering_angle = math.atan(WHEELBASE * curv)
        self.pub.publish(m)

    def finish(self):
        try:
            self.send(0.0, 0.0)
        except Exception:
            pass
        self.finished = True

    def watchdog(self):
        if self.finished:
            return
        t = self.now()
        if not self.checked and t - self.t_begin > 2.0:
            self.checked = True
            if self.count_subscribers(self.a.topic) == 0:
                print(f'[오류] {self.a.topic}을 받는 노드가 없음: 드라이버/브리지 확인')
                self.finish()
                return
            try:
                types = {i.topic_type for i in self.get_subscriptions_info_by_topic(self.a.topic)}
                if self.type_name not in types:
                    print(f'[오류] {self.a.topic} 타입 불일치: 받는 쪽은 {sorted(types)}, 보내는 쪽은 {self.type_name}')
                    self.finish()
                    return
            except Exception:
                pass
            self.ok_to_move = True
        if self.last_odom is None and t - self.t_begin > 5.0:
            print('[오류] 5초 동안 /odom 안 옴: 드라이버 확인')
            self.finish()
        elif self.last_odom is not None and t - self.last_odom > 0.5:
            print('[오류] /odom 끊김: 정지')
            self.finish()
        elif self.a.pose == 'imu' and self.fu.t_imu is None and t - self.t_begin > 3.0:
            print('[오류] /imu 안 옴: --pose odom으로 하거나 IMU 확인')
            self.finish()

    def on_imu(self, m):
        self.fu.on_imu(self.stamp(m), m.angular_velocity.z)

    def on_odom(self, m):
        if self.finished:
            return
        t = self.now()
        self.last_odom = t
        p = m.pose.pose.position
        yaw = yaw_of(m.pose.pose.orientation)
        self.fu.on_odom(self.stamp(m), m.twist.twist.linear.x)
        if not self.ok_to_move:
            return          # 출력 토픽 연결 확인(약 2초) 전에는 움직이지 않음
        if self.a.pose == 'odom':
            if self.origin is None:
                self.origin = (p.x, p.y, yaw)
                print(f'경로 점 {len(self.pts)}개, 출발 (odom 기준)')
            x0, y0, yaw0 = self.origin
            dx, dy = p.x - x0, p.y - y0
            x = math.cos(yaw0) * dx + math.sin(yaw0) * dy
            y = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
            yw = wrap(yaw - yaw0)
        else:
            if not self.fu.ready:
                self.send(0.0, 0.0)
                return
            if self.origin is None:
                self.origin = True
                print(f'경로 점 {len(self.pts)}개, 출발 (IMU 방향 기준)')
            x, y, yw = self.fu.x, self.fu.y, wrap(self.fu.yaw)
        n = len(self.pts)
        hi = min(self.idx + 40, n)
        self.idx = min(range(self.idx, hi), key=lambda i: math.hypot(self.pts[i][0] - x, self.pts[i][1] - y))
        ex, ey = self.pts[-1]
        d_end = math.hypot(ex - x, ey - y)
        if self.idx >= n - 10:
            sx, sy = ex - self.pts[-2][0], ey - self.pts[-2][1]
            along = ((ex - x) * sx + (ey - y) * sy) / (math.hypot(sx, sy) or 1.0)
            if along <= self.a.stop_early or d_end < 0.02:
                print(f'종료: 끝점까지 {d_end*100:.1f}cm (추정 위치 기준, 줄자로도 재세요)')
                self.finish()
                return
        L = self.a.lookahead
        tgt = next((i for i in range(self.idx, n) if math.hypot(self.pts[i][0] - x, self.pts[i][1] - y) >= L), n - 1)
        dx, dy = self.pts[tgt][0] - x, self.pts[tgt][1] - y
        l2 = max(dx*dx + dy*dy, 1e-4)
        ly = -math.sin(yw) * dx + math.cos(yw) * dy
        curv = max(-MAX_CURV, min(MAX_CURV, 2 * ly / l2))
        self.send(self.a.speed, curv)
        if t - self.last_print >= 1.0:
            self.last_print = t
            print(f'  진행 {self.idx}/{n}  곡률 {curv:+.2f}  끝점까지 {d_end*100:.0f}cm')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('speed', type=float, nargs='?', default=0.2)
    ap.add_argument('stop_early', type=float, nargs='?', default=0.0)
    ap.add_argument('--pose', choices=['odom', 'imu'], default='odom')
    ap.add_argument('--topic', default='/cmd_vel')
    ap.add_argument('--type', choices=['twist', 'ackermann'], default='twist')
    ap.add_argument('--lookahead', type=float, default=0.3)
    a = ap.parse_args()
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    node = Follower(a)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('중단(Ctrl+C): 정지 명령 전송')
    finally:
        for _ in range(10):
            try:
                node.send(0.0, 0.0)
            except Exception:
                break
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
