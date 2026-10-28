"""미션 노드: 상태 머신(fsm.py) + 최종 /cmd_vel 선택기(cmd_mux).

로봇에 /cmd_vel을 내는 노드는 이것 하나뿐이다. 각 행동 노드는 자기 토픽에만 낸다.
  /lane/cmd_vel, /obstacle/cmd_vel, /parking/cmd_vel  ->  [mux]  ->  /cmd_vel

구독
  /perception/detections (String JSON)  {"detections":[{"cls":"red","conf":0.8,"bbox":[...]}]}
  /lane/status, /obstacle/state, /parking/state (String JSON)
  odom_topic (nav_msgs/Odometry) -> 누적 주행 거리, imu_topic (sensor_msgs/Imu) -> yaw
  /mission/start (std_msgs/Empty)       출발
  /mission/estop (std_msgs/Bool)        true면 즉시 정지 (false로 해제)
  /mission/set_state (std_msgs/String)  연습용: 원하는 상태로 점프
발행
  /cmd_vel (geometry_msgs/Twist), /lane/config (String JSON), /mission/state (String JSON)

출발:  ros2 topic pub --once /mission/start std_msgs/msg/Empty
정지:  ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}"
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, String

from autorace_mission.fsm import DEFAULT_PARAMS, Inputs, MissionFSM

SOURCES = ('lane', 'obstacle', 'parking')


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        dyn = ParameterDescriptor(dynamic_typing=True)
        for k, v in DEFAULT_PARAMS.items():
            self.declare_parameter(k, v, dyn)
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('max_speed', 0.45)          # m/s 상한 (안전)
        self.declare_parameter('max_yaw_rate', 2.0)        # rad/s 상한
        self.declare_parameter('source_timeout_sec', 0.3)  # 행동 노드 명령이 이보다 오래되면 정지

        params = {k: self.get_parameter(k).value for k in DEFAULT_PARAMS}
        self.fsm = MissionFSM(params)
        self.get_logger().info(f'mission start state: {self.fsm.state}')

        self.dist_mm = 0.0
        self._last_xy = None
        self.yaw = None
        self.det = {}
        self.new_det = False
        self.lane = {}
        self.obstacle = {}
        self.parking = {}
        self.start_flag = False
        self.estop = False
        self.twists = {s: (None, 0.0) for s in SOURCES}   # (Twist, 받은 시각)
        self._last_lane_cfg = None
        self._last_cfg_pub = 0.0
        self._last_state = None

        self.create_subscription(String, '/perception/detections', self._on_det, 10)
        self.create_subscription(String, '/lane/status', self._json_cb('lane'), 10)
        self.create_subscription(String, '/obstacle/state', self._json_cb('obstacle'), 10)
        self.create_subscription(String, '/parking/state', self._json_cb('parking'), 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._on_imu, qos_profile_sensor_data)
        for s in SOURCES:
            self.create_subscription(Twist, f'/{s}/cmd_vel', self._twist_cb(s), 10)
        self.create_subscription(Empty, '/mission/start', self._on_start, 10)
        self.create_subscription(Bool, '/mission/estop', self._on_estop, 10)
        self.create_subscription(String, '/mission/set_state', self._on_set_state, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cfg_pub = self.create_publisher(String, '/lane/config', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.create_timer(1.0 / float(self.get_parameter('tick_hz').value), self._tick)

    # --- 입력 ---------------------------------------------------------------
    def _json_cb(self, name):
        def cb(msg):
            try:
                setattr(self, name, json.loads(msg.data))
            except json.JSONDecodeError:
                pass
        return cb

    def _twist_cb(self, source):
        def cb(msg):
            self.twists[source] = (msg, time.monotonic())
        return cb

    def _on_det(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        best = {}
        for d in data.get('detections', []):
            c = d.get('cls')
            best[c] = max(best.get(c, 0.0), float(d.get('conf', 0.0)))
        self.det = best
        self.new_det = True

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self._last_xy is not None:
            self.dist_mm += 1000.0 * math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
        self._last_xy = (p.x, p.y)

    def _on_imu(self, msg):
        self.yaw = yaw_from_quaternion(msg.orientation)

    def _on_start(self, _msg):
        self.start_flag = True

    def _on_estop(self, msg):
        self.estop = bool(msg.data)
        self.get_logger().warning(f'E-STOP {"ON" if self.estop else "off"}')

    def _on_set_state(self, msg):
        try:
            self.fsm._goto(msg.data.strip(), self._inputs(), 'manual')
            self.get_logger().info(f'state set to {self.fsm.state}')
        except ValueError as e:
            self.get_logger().error(str(e))

    def _inputs(self) -> Inputs:
        return Inputs(t=time.monotonic(), dist_mm=self.dist_mm, yaw=self.yaw,
                      detections=dict(self.det), new_detection_frame=self.new_det,
                      lane=self.lane, obstacle=self.obstacle, parking=self.parking,
                      start=self.start_flag)

    # --- 틱 -----------------------------------------------------------------
    def _tick(self):
        inp = self._inputs()
        self.new_det = False
        self.start_flag = False
        cmd = self.fsm.step(inp)

        now = time.monotonic()
        if cmd.lane != self._last_lane_cfg or now - self._last_cfg_pub > 1.0:
            self.cfg_pub.publish(String(data=json.dumps(cmd.lane)))
            self._last_lane_cfg, self._last_cfg_pub = dict(cmd.lane), now

        out = Twist()
        if not self.estop and cmd.source in SOURCES:
            tw, stamp = self.twists[cmd.source]
            if tw is not None and now - stamp <= self.get_parameter('source_timeout_sec').value:
                vmax = self.get_parameter('max_speed').value
                wmax = self.get_parameter('max_yaw_rate').value
                out.linear.x = max(-vmax, min(vmax, tw.linear.x))
                out.angular.z = max(-wmax, min(wmax, tw.angular.z))
        self.cmd_pub.publish(out)

        state = {'state': self.fsm.state, 'sub': self.fsm.sub, 'source': cmd.source,
                 'fork_choice': self.fsm.fork_choice, 'dist_mm': round(self.dist_mm),
                 'estop': self.estop, 'note': cmd.note}
        self.state_pub.publish(String(data=json.dumps(state)))
        if (self.fsm.state, self.fsm.sub) != self._last_state:
            self.get_logger().info(f'[{self.fsm.state}/{self.fsm.sub}] {self.fsm.log[-1]}')
            self._last_state = (self.fsm.state, self.fsm.sub)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
