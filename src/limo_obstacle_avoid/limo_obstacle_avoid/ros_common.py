"""ros_common.py — ROS 2 노드들이 같이 쓰는 도우미

- init_rclpy()   : Ctrl+C 를 눌러도 정지 명령을 보낼 수 있게 초기화
- load_params()  : AvoidParams 전 항목을 ROS 파라미터로 (실행 중 ros2 param set 가능)
- PoseInputs     : /odom + /imu → (odom x, odom y, 사용할 yaw). IMU 방향 자동 판별, 자이로 편향 보정
- CmdOut         : 바퀴 명령 내보내기 (twist: /cmd_vel → 팀 브리지 / ack: /limo/ack_cmd 직접)
"""
import math
import time
from dataclasses import fields

from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def init_rclpy():
    """ROS 기본 SIGINT 처리를 끄고 시작 → Ctrl+C 때 KeyboardInterrupt 가 나도 통신이 살아 있어 정지 명령을 보낼 수 있음"""
    import rclpy
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()


def shutdown_rclpy(node):
    import rclpy
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


def declare(node, name, default):
    """이미 선언된 파라미터면 다시 선언하지 않고 값만 돌려줌"""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return node.get_parameter(name).value


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_params(node, p):
    """AvoidParams 의 모든 항목을 ROS 파라미터로 선언하고 값 읽기.
    실행 중에도 `ros2 param set /노드 trigger_dist 1.2` 로 바로 바뀜."""
    names = set()
    for f in fields(p):
        default = getattr(p, f.name)
        val = declare(node, f.name, default)
        if isinstance(default, float) and isinstance(val, int):
            val = float(val)
        setattr(p, f.name, val)
        names.add(f.name)

    def on_set(params):
        for prm in params:
            if prm.name in names:
                old = getattr(p, prm.name)
                val = prm.value
                if isinstance(old, float) and isinstance(val, int):
                    val = float(val)
                setattr(p, prm.name, val)
                node.get_logger().info(f'[param] {prm.name}: {old} → {val}')
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(on_set)
    return p


class PoseInputs:
    """/odom + /imu 받아서 (odom x, odom y, 사용할 yaw) 제공.
    yaw_source: 'imu'(기본) | 'odom'
    imu_mode  : 'auto'(기본) | 'orientation' | 'gyro'
      auto = 회전해서 IMU 방향값이 실제로 변한다고 확인되면 방향값, 비었거나 안 변하면 자이로 적분.
             아직 확인 전이면 정지 중 편향을 잰 자이로를 우선 (확인은 차가 17° 넘게 돌 때 자동)
             → 3단계 s3 에서 미리 확인하고 yaml 에 고정하는 것을 권장
             미션 시작(lock) 때 정한 방식은 미션이 끝날 때까지 바꾸지 않는다 (도중에 바뀌면 방향이 튐).
    자이로 적분은 차가 멈춰 있을 때 편향을 계속 다시 잰다."""

    def __init__(self, node):
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        self.node = node
        self.odom_topic = declare(node, 'odom_topic', '/odom')
        self.imu_topic = declare(node, 'imu_topic', '/imu')
        declare(node, 'yaw_source', 'imu')
        declare(node, 'imu_mode', 'auto')
        self.odom = None            # (x, y, yaw, vx)
        self.imu_yaw = None
        self.gyro_yaw = 0.0
        self.bias = 0.0
        self._bias_n = 0
        self._gyro_t = None
        self._still_since = None
        self._last_xy = None
        self.t_odom = None          # 마지막 수신 (time.monotonic)
        self.t_imu = None
        self.orient_ok = None       # None=판별 전, True/False
        self._ref = None            # (gyro_yaw, imu_yaw) 판별 기준
        self.on_odom = None         # 콜백 훅
        self.locked = None          # 미션 중 고정된 방식
        node.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        node.create_subscription(Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data)

    # ---------- 콜백 ----------
    def _odom_cb(self, m):
        pp = m.pose.pose
        x, y = pp.position.x, pp.position.y
        now = time.monotonic()
        # 정지 판정: 0.5초 동안 1mm 이상 안 움직임
        if self._last_xy is None or math.hypot(x - self._last_xy[0], y - self._last_xy[1]) > 0.001:
            self._still_since = now
            self._last_xy = (x, y)
        self.odom = (x, y, yaw_from_quat(pp.orientation), m.twist.twist.linear.x)
        self.t_odom = now
        if self.on_odom:
            self.on_odom()

    def still(self):
        return self._still_since is not None and time.monotonic() - self._still_since > 0.5

    def _imu_cb(self, m):
        q = m.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        self.imu_yaw = yaw_from_quat(q) if norm > 0.5 else None
        gz = m.angular_velocity.z
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t <= 0:
            t = time.monotonic()
        if self.still():                        # 멈춰 있으면 편향 다시 재기 (지수평균)
            self._bias_n = min(self._bias_n + 1, 200)
            self.bias += (gz - self.bias) / self._bias_n
        elif self._gyro_t is not None and 0.0 < t - self._gyro_t < 0.5:
            self.gyro_yaw += (gz - self.bias) * (t - self._gyro_t)
        self._gyro_t = t
        self.t_imu = time.monotonic()
        self._check_orientation()

    def _check_orientation(self):
        """auto 모드: 자이로로 0.3rad 넘게 돌았는데 IMU 방향 변화가 그 1/5도 안 되면 '방향 값 없음'으로 판정"""
        if self.orient_ok is not None:
            return
        if self.imu_yaw is None:
            if self.odom is not None:
                self.orient_ok = False
                self.node.get_logger().warn('IMU 방향(쿼터니언)이 비어 있음 → 자이로 적분으로 방향 계산')
            return
        if self._ref is None:
            self._ref = (self.gyro_yaw, self.imu_yaw)
            return
        dg = abs(self.gyro_yaw - self._ref[0])
        di = abs(math.atan2(math.sin(self.imu_yaw - self._ref[1]), math.cos(self.imu_yaw - self._ref[1])))
        if dg > 0.3:
            self.orient_ok = di > 0.2 * dg
            self.node.get_logger().info(
                f'IMU 방향 판별: 자이로 {math.degrees(dg):.0f}° / 방향값 {math.degrees(di):.0f}° → '
                + ('방향값 사용' if self.orient_ok else '방향값이 안 변함 → 자이로 적분 사용'))

    # ---------- 조회 ----------
    def mode_now(self):
        m = self.node.get_parameter('imu_mode').value
        if m == 'auto':
            if self.orient_ok is False or self.imu_yaw is None:
                return 'gyro'
            if self.orient_ok is True:
                return 'orientation'
            # 아직 판별 전: 정지 상태에서 편향을 잰 자이로가 있으면 그쪽이 안전
            return 'gyro' if self._bias_n >= 20 else 'orientation'
        return m

    def mode(self):
        return self.locked or self.mode_now()

    def lock(self):
        """미션 시작 때 호출: 지금 방식으로 고정"""
        self.locked = self.mode_now()
        if self.locked == 'gyro' and self._bias_n < 20:
            self.node.get_logger().warn('자이로 편향을 아직 못 잼 → 방향이 서서히 틀어질 수 있음 '
                                        '(시작 전 1초 이상 정지해 두면 자동으로 잼)')
        return self.locked

    def unlock(self):
        self.locked = None

    def ready(self):
        if self.odom is None:
            return False
        if self.node.get_parameter('yaw_source').value == 'imu':
            if self.mode() == 'gyro':
                return self._gyro_t is not None
            return self.imu_yaw is not None
        return True

    def fresh(self, odom_timeout=0.3, imu_timeout=0.3):
        now = time.monotonic()
        if self.t_odom is None or now - self.t_odom > odom_timeout:
            return False
        if self.node.get_parameter('yaw_source').value == 'imu':
            if self.t_imu is None or now - self.t_imu > imu_timeout:
                return False
        return True

    def yaw(self):
        if self.node.get_parameter('yaw_source').value != 'imu':
            return self.odom[2]
        if self.mode() == 'gyro':
            return self.gyro_yaw
        return self.imu_yaw if self.imu_yaw is not None else self.gyro_yaw

    def get(self):
        """→ (ox, oy, yaw, forward)"""
        ox, oy, _, vx = self.odom
        return ox, oy, self.yaw(), vx >= -0.01


class CmdOut:
    """바퀴 명령 내보내기.
    out_mode: 'twist' → Twist 를 cmd_topic(/cmd_vel)으로 (팀 ackermann_bridge 가 /limo/ack_cmd 로 변환)
              'ack'   → AckermannDrive 를 ack_topic(/limo/ack_cmd)으로 직접 (브리지 끄기)
    angular_mode (twist 일 때만): 'yaw_rate'(angular.z=회전속도) | 'steer_angle'(angular.z=조향각)"""

    def __init__(self, node, cmd_topic_default='/cmd_vel', prefix=''):
        self.node = node
        self.mode = declare(node, prefix + 'out_mode', 'twist')
        self.L = float(declare(node, 'wheelbase', 0.222))
        self.min_r = float(declare(node, 'min_turn_radius', 0.53))
        declare(node, 'angular_mode', 'yaw_rate')
        if self.mode == 'ack':
            from ackermann_msgs.msg import AckermannDrive
            self.cls, self.type_name = AckermannDrive, 'ackermann_msgs/msg/AckermannDrive'
            self.topic = declare(node, 'ack_topic', '/limo/ack_cmd')
        elif self.mode == 'twist':
            from geometry_msgs.msg import Twist
            self.cls, self.type_name = Twist, 'geometry_msgs/msg/Twist'
            self.topic = declare(node, prefix + 'cmd_topic', cmd_topic_default)
        else:
            raise ValueError(f'out_mode 는 twist 또는 ack: {self.mode}')
        self.pub = node.create_publisher(self.cls, self.topic, 10)

    def send(self, v, w):
        """v [m/s], w [rad/s] (회전속도)"""
        m = self.cls()
        if self.mode == 'ack':
            m.speed = float(v)
            if abs(v) > 1e-3:
                kmax = 1.0 / self.min_r
                k = max(-kmax, min(kmax, w / v))
                m.steering_angle = float(math.atan(self.L * k))
            else:
                m.steering_angle = 0.0
        else:
            m.linear.x = float(v)
            if self.node.get_parameter('angular_mode').value == 'steer_angle':
                m.angular.z = math.atan(self.L * w / v) if abs(v) > 1e-3 else 0.0
            else:
                m.angular.z = float(w)
        self.pub.publish(m)

    def send_msg(self, twist):
        """Twist(회전속도 기준) 를 받아 설정된 방식으로 그대로 내보냄 (cmd_mux 용)"""
        if self.mode == 'twist':
            self.pub.publish(twist)
        else:
            self.send(twist.linear.x, twist.angular.z)

    def stop(self):
        self.send(0.0, 0.0)

    def check(self):
        """연결 상태 점검 → (ok, 설명)"""
        n = self.node.count_subscribers(self.topic)
        if n == 0:
            who = '아커만 드라이버' if self.mode == 'ack' else '팀 브리지(ackermann_bridge) 또는 드라이버'
            return False, f'{self.topic} 을 받는 노드가 없음 → {who} 확인'
        others = self.node.count_publishers(self.topic) - 1
        if others > 0:
            hint = '브리지' if self.mode == 'ack' else 'teleop/차선추종(→/lane_follow/cmd_vel 로 remap)'
            return False, f'{self.topic} 에 다른 발행자 {others}개 → {hint} 확인 (명령 충돌)'
        return True, f'{self.topic} 연결 OK (방식 {self.mode})'


def make_twist(node, v, w):
    """회피 노드 → cmd_mux 로 보내는 Twist (angular.z = 회전속도)"""
    from geometry_msgs.msg import Twist
    t = Twist()
    t.linear.x = float(v)
    t.angular.z = float(w)
    return t


def make_marker(frame, ns, mid, mtype, stamp, color=(1.0, 1.0, 1.0, 1.0), scale=(0.02, 0.02, 0.02)):
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def pt(x, y, z=0.0):
    q = Point()
    q.x, q.y, q.z = float(x), float(y), float(z)
    return q
