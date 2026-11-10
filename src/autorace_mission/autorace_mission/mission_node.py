"""미션 노드: 상태 머신(fsm.py) + 최종 /cmd_vel 선택기(cmd_mux).

로봇에 /cmd_vel을 내는 노드는 이것 하나뿐이다. 각 행동 노드는 자기 토픽에만 낸다.
  /lane/cmd_vel, /obstacle/cmd_vel, /parking/cmd_vel  ->  [mux]  ->  /cmd_vel

구독
  /perception/commands (기본, 확인된 지속 요청). use_class_commands=false면 /perception/detections (String JSON)  {"detections":[{"cls":"red","conf":0.8,"bbox":[...]}]}
  /lane/status, /obstacle/state, /parking/state (String JSON)
  odom_topic (nav_msgs/Odometry) -> 누적 주행 거리, imu_topic (sensor_msgs/Imu) -> yaw
  /mission/start (std_msgs/Empty)       출발
  /mission/estop (std_msgs/Bool)        true면 즉시 정지 (false로 해제)
  /mission/set_state (std_msgs/String)  연습용: 원하는 상태로 점프 (경로 위치 s도 그 미션 시작점으로)
  /route/set_s (std_msgs/Float32)       연습용: 경로 위치 s(mm)를 직접 지정

도면 경로 지도: route_file 파라미터에 track_route.yaml 경로를 주면 켜진다.
  오도메트리 + 흰 가로선(/lane/status stoplines_mm)으로 경로상 위치 s를 추정해
  미션 구간 전환을 도면 위치로 한다. /mission/state의 "route"에 s와 다음 선이 나온다.
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
from std_msgs.msg import Bool, Empty, Float32, String

from autorace_mission.command_inbox import CommandInbox
from autorace_mission.fsm import DEFAULT_PARAMS, Inputs, MissionFSM
from autorace_mission.route import MapPose, RouteMap, RouteTracker

# /mission/set_state로 점프할 때 차를 놓는 곳 (도면 경로 지점 + 오프셋 mm)
STATE_START_POINT = {
    'M5_BUMP': (None, 0.0),
    'M6_ZIGZAG': ('m6_zigzag_line', -1500.0),
    'M3_PEDESTRIAN': ('m6_zigzag_end', 0.0),
    'M7_OBSTACLE': ('m7_start_line', -300.0),
    'M2_FORK': ('m7_island_tip', -300.0),
    'M8_CONES': ('m8_start_line', -500.0),
    'M1_TRAFFIC': ('m1_pre_line', -500.0),
    'M4_PARKING': ('m4_pre_line', -300.0),
}

SOURCES = ('lane', 'obstacle', 'parking')


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        dyn = ParameterDescriptor(dynamic_typing=True)
        for k, v in DEFAULT_PARAMS.items():
            if not isinstance(v, dict):              # route_points는 yaml 파일에서 읽는다
                self.declare_parameter(k, v, dyn)
        self.declare_parameter('tick_hz', 20.0)
        self.declare_parameter('use_class_commands', True)
        self.use_class_commands = self.get_parameter('use_class_commands').value
        self.command_inbox = CommandInbox()
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('max_speed', 0.45)          # m/s 상한 (안전)
        self.declare_parameter('max_yaw_rate', 2.0)        # rad/s 상한
        self.declare_parameter('source_timeout_sec', 0.3)  # 행동 노드 명령이 이보다 오래되면 정지
        self.declare_parameter('route_file', '')            # track_route.yaml. 비우면 거리 기준
        self.declare_parameter('route_gate_mm', 250.0)
        self.declare_parameter('route_gain', 0.6)
        self.declare_parameter('odom_scale', 1.0)           # 실측: 실제 거리 / 오도메트리 거리
        # 흰 선을 도면 순서대로 짝짓는 허용 범위 (지그재그 흔들림으로 odom이 최대 ~80cm 길어짐). 0이면 예전 방식
        self.declare_parameter('route_ordered_gate_mm', 1200.0)
        # 시작하자마자 경로 위치를 이 값(mm)으로 맞춘다 (lane_test에서 START 체커 출발 = 0.0). 음수면 안 함
        self.declare_parameter('route_start_s', -1.0)

        params = {k: self.get_parameter(k).value for k in DEFAULT_PARAMS
                  if not isinstance(DEFAULT_PARAMS[k], dict)}
        self.route = None
        self.tracker = None
        self.mappose = None
        self.route_active = False
        path = self.get_parameter('route_file').value
        if path:
            try:
                self.route = RouteMap.from_yaml(path)
                self.tracker = RouteTracker(self.route,
                                            gate_mm=self.get_parameter('route_gate_mm').value,
                                            gain=self.get_parameter('route_gain').value,
                                            odom_scale=self.get_parameter('odom_scale').value,
                                            ordered_gate_mm=float(self.get_parameter(
                                                'route_ordered_gate_mm').value))
                params['route_points'] = self.route.points
                self.mappose = MapPose(self.route)
                self.get_logger().info(f'route map: {path} ({self.route.length / 1000:.1f} m, '
                                       f'{len(self.route.landmarks)} lines)')
            except Exception as e:      # 지도 파일이 깨져도 거리 기준으로 달린다
                self.get_logger().error(f'route map load failed ({path}): {e} -> 거리 기준')
        self.fsm = MissionFSM(params)
        self.get_logger().info(f'mission start state: {self.fsm.state}')

        self.dist_mm = 0.0
        self._last_xy = None
        self.yaw = None
        self.tilt_peak = 0.0
        self.odom_step_max_mm = 200.0
        self.odom_jumps = 0
        self.det = {}
        self.new_det = False
        self.lane = {}
        self.new_lane = False
        self.obstacle = {}
        self.parking = {}
        self.start_flag = False
        self.estop = False
        self.twists = {s: (None, 0.0) for s in SOURCES}   # (Twist, 받은 시각)
        self._last_lane_cfg = None
        self._last_cfg_pub = 0.0
        self._last_state = None
        self._last_fallback_warn = 0.0
        self._auto_s_done = False

        if self.use_class_commands:
            self.create_subscription(String, '/perception/commands', self._on_commands, 10)
        else:
            self.create_subscription(String, '/perception/detections', self._on_det, 10)
        self.create_subscription(String, '/lane/status', self._on_lane, 10)
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
        self.create_subscription(Float32, '/route/set_s', self._on_set_s, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cfg_pub = self.create_publisher(String, '/lane/config', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        # 차선 노드의 도면 곡률 미리 넣기용: 앞바퀴에서 0~1300mm 앞 곡률 (왼쪽 +, 1/m)
        self.preview_pub = self.create_publisher(String, '/route/preview', 10)
        self.create_timer(1.0 / float(self.get_parameter('tick_hz').value), self._tick)

    # --- 입력 ---------------------------------------------------------------
    def _json_cb(self, name):
        def cb(msg):
            try:
                setattr(self, name, json.loads(msg.data))
            except json.JSONDecodeError:
                pass
        return cb

    def _on_lane(self, msg):
        try:
            self.lane = json.loads(msg.data)
            self.new_lane = True
        except json.JSONDecodeError:
            pass

    def _route_reset(self, s: float, why: str):
        if self.tracker is None:
            return
        self.tracker.reset(s, self.dist_mm)
        self.route_active = True
        self.get_logger().info(f'route s = {s:.0f} mm ({why})')

    def _on_set_s(self, msg):
        self._route_reset(float(msg.data), 'manual')

    def _twist_cb(self, source):
        def cb(msg):
            self.twists[source] = (msg, time.monotonic())
        return cb

    def _on_commands(self, msg):
        try:
            self.command_inbox.receive(json.loads(msg.data), self.fsm.state)
        except (ValueError, TypeError):
            self.get_logger().warning('invalid class commands', throttle_duration_sec=5.0)

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
            dx, dy = p.x - self._last_xy[0], p.y - self._last_xy[1]
            step = 1000.0 * math.hypot(dx, dy)
            # 한 메시지에 이만큼 넘게 튀면 odom 오류(11/09 체크섬 오류 때 7.5km)로 보고 버린다.
            # FSM뿐 아니라 도면 위치(RouteTracker)도 이 거리를 쓰므로 여기서 거른다 (11/10 통주행 시뮬레이션에서 발견)
            if step <= self.odom_step_max_mm:
                # 후진은 빼기 (11/10 실차: M7 롤백 1.5m x3 후진이 앞으로 간 거리로 쌓여 도면 위치가
                # 최대 9m 앞서고 갈림길·라바콘·신호등 모드가 다 어긋남). 차 방향으로 투영해 부호를 정한다
                yaw = yaw_from_quaternion(msg.pose.pose.orientation)
                if 1000.0 * (dx * math.cos(yaw) + dy * math.sin(yaw)) < -0.5 * step:
                    step = -step
                self.dist_mm += step
            else:
                self.odom_jumps += 1
                self.get_logger().warning(f'odom jump ignored: {step:.0f}mm in one message '
                                          f'({self.odom_jumps} so far)', throttle_duration_sec=2.0)
        self._last_xy = (p.x, p.y)

    def _on_imu(self, msg):
        self.yaw = yaw_from_quaternion(msg.orientation)
        av = msg.angular_velocity          # 앞뒤·좌우 기울기 속도 최대값 (방지턱 감지, 틱마다 0으로)
        self.tilt_peak = max(self.tilt_peak, math.hypot(av.x, av.y))

    def _on_start(self, _msg):
        self.start_flag = True
        if self.fsm.state == 'WAIT_START':
            self._route_reset(0.0, 'start line')

    def _on_estop(self, msg):
        self.estop = bool(msg.data)
        self.get_logger().warning(f'E-STOP {"ON" if self.estop else "off"}')

    def _on_set_state(self, msg):
        try:
            self.fsm._goto(msg.data.strip(), self._inputs(), 'manual')
            self.get_logger().info(f'state set to {self.fsm.state}')
            if self.route is not None and self.fsm.state in STATE_START_POINT:
                name, off = STATE_START_POINT[self.fsm.state]
                base = self.route.points.get(name, 0.0) if name else 0.0
                self._route_reset(base + off, f'{self.fsm.state} start point')
        except ValueError as e:
            self.get_logger().error(str(e))

    def _inputs(self) -> Inputs:
        route_s = None
        if self.route_active:
            lines = self.lane.get('stoplines_mm') if self.new_lane else None
            xw = self.lane.get('crosswalk_lines_mm') if self.new_lane else None
            route_s = self.tracker.update(self.dist_mm, lines, xw)
            self.new_lane = False
        self.command_inbox.set_mode(self.fsm.state)
        return Inputs(t=time.monotonic(), dist_mm=self.dist_mm, yaw=self.yaw,
                      confirmed_classes=(dict(self.command_inbox.classes)
                                         if self.use_class_commands else None),
                      detections=dict(self.det), new_detection_frame=self.new_det,
                      lane=self.lane, obstacle=self.obstacle, parking=self.parking,
                      start=self.start_flag, route_s=route_s, tilt_rate=getattr(self, 'tilt_peak', None))

    # --- 틱 -----------------------------------------------------------------
    def _tick(self):
        s0 = float(self.get_parameter('route_start_s').value)
        if not self._auto_s_done and s0 >= 0.0 and self.tracker is not None and self._last_xy:
            self._route_reset(s0, 'route_start_s')
            self._auto_s_done = True
        inp = self._inputs()
        self.new_det = False
        self.start_flag = False
        self.tilt_peak = 0.0
        cmd = self.fsm.step(inp)

        now = time.monotonic()
        if cmd.lane != self._last_lane_cfg or now - self._last_cfg_pub > 1.0:
            self.cfg_pub.publish(String(data=json.dumps(cmd.lane)))
            self._last_lane_cfg, self._last_cfg_pub = dict(cmd.lane), now

        out = Twist()
        source = cmd.source
        timeout = self.get_parameter('source_timeout_sec').value
        if source == 'obstacle' and self.fsm.state == 'M8_CONES':
            # M8: obstacle 노드가 죽거나 늦으면 서지 말고 차선 주행으로 (부분점수는 거리 비례라 멈추면 손해)
            # M7은 여기서 바꾸지 않는다: 회피 노드가 멈추면 m7_link가 m7_hold로 정지시킨다
            # (정차 LIMO 사이를 차선 주행으로 직진하지 않게)
            tw, stamp = self.twists['obstacle']
            if tw is None or now - stamp > timeout:
                source = 'lane'
                if now - self._last_fallback_warn > 2.0:
                    self.get_logger().warning('/obstacle/cmd_vel 끊김 -> 차선 주행으로 대신')
                    self._last_fallback_warn = now
        if not self.estop and source in SOURCES:
            tw, stamp = self.twists[source]
            if tw is not None and now - stamp <= timeout:
                vmax = self.get_parameter('max_speed').value
                wmax = self.get_parameter('max_yaw_rate').value
                out.linear.x = max(-vmax, min(vmax, tw.linear.x))
                out.angular.z = max(-wmax, min(wmax, tw.angular.z))
        self.cmd_pub.publish(out)
        if self.route_active and inp.route_s is not None:
            step = 100.0
            ks = [round(self.route.curvature(inp.route_s + i * step), 4) for i in range(14)]
            msg = {'s': round(inp.route_s), 'step_mm': step, 'k': ks}
            mp = self.mappose
            if mp is not None:
                mp.predict(self.dist_mm, inp.route_s, self.yaw)
                mp.update_lane(inp.route_s, self.yaw, self.lane, self.dist_mm)
                msg['pose_ok'] = mp.valid(self.dist_mm) and self.yaw is not None
                if msg['pose_ok']:
                    msg['d_mm'] = round(mp.d)
                    msg['psi'] = round(self.yaw + mp.yaw_off, 4)      # 도면 좌표계 차 방향 (디버그 화면용)
                    msg['pts'] = mp.points_ahead(inp.route_s, self.yaw)
            self.preview_pub.publish(String(data=json.dumps(msg)))

        state = {'state': self.fsm.state, 'sub': self.fsm.sub, 'source': source,
                 'fork_choice': self.fsm.fork_choice, 'dist_mm': round(self.dist_mm),
                 'estop': self.estop, 'note': cmd.note,
                 'parking_sign': self.fsm.parking_sign, 'parking_armed': self.fsm.parking_armed,
                 'route': self.tracker.state() if self.route_active else None}
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
