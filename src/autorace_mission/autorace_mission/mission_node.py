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
  /mission/skip (std_msgs/String)       주행 중 건너뛰기: 상태만 바꾸고 경로 위치 s는 그대로 (디버그 화면 버튼, 11/11)
  /route/set_s (std_msgs/Float32)       연습용: 경로 위치 s(mm)를 직접 지정
  /cmd_vel (Twist)                      후진 판단 보조용 (자기가 낸 명령 / 수동 조종 명령)

위치 확인 수동 주행 (manual_drive:=true, 11/11)
  /cmd_vel을 내지 않는다. 사람이 teleop으로 몰고 디버그 화면에서 도면 위 위치가 맞는지만 본다.
  FSM·경로 추정은 그대로 돌아서 미션 전환이 어디서 일어나는지도 같이 보인다.

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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, String

from autorace_mission.command_inbox import CommandInbox
from autorace_mission.fsm import DEFAULT_PARAMS, SEQUENCE, Inputs, MissionFSM
from autorace_mission.route import MapPose, RouteMap, RouteTracker, motion_sign

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
        # 11/10 위치 회복
        # 후진(odom twist.linear.x < 0) 거리는 경로 위치 s에서 뺀다. FSM 누적 거리(dist_mm)는 예전처럼 그대로 둔다.
        # TODO 실측: LIMO 후진 때 /odom twist.linear.x가 음수로 나오는지 (안 나오면 예전과 똑같이 동작)
        self.declare_parameter('route_signed_dist', True)
        # 11/11 위치 추정 강화
        # 후진 판단: odom 속도 부호 + odom 위치 변화 방향 + 최근 /cmd_vel 부호 중 하나라도 후진이면 후진
        # 경로 방향 투영: 차가 경로와 비스듬히 달린 거리는 cos(방향 차)만큼만 s에 더하고 나머지는 좌우 d로
        self.declare_parameter('route_project_heading', True)
        # 출발선·연습 시작점·/route/set_s에서 차가 경로 방향으로 반듯이 놓였다고 보고 방향을 맞춘다
        self.declare_parameter('route_align_on_reset', True)
        # 위치 확인용 수동 주행: /cmd_vel을 내지 않는다 (teleop이 낸다)
        self.declare_parameter('manual_drive', False)
        # 마지막으로 맞춘 선 뒤로 도면 선을 이만큼 연속 못 보면 '길 잃음' -> FSM에 경로 위치를 안 줌. 0이면 끔
        self.declare_parameter('route_lost_misses', 2)
        # 카메라로 확인한 미션 선으로 s를 다시 맞출 때, 지금 s와 이보다 크게 다르면 오검출로 보고 안 맞춤
        self.declare_parameter('route_anchor_max_mm', 3000.0)

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
                                                'route_ordered_gate_mm').value),
                                            lost_misses=int(self.get_parameter('route_lost_misses').value))
                params['route_points'] = self.route.points
                self.mappose = MapPose(self.route)
                self.get_logger().info(f'route map: {path} ({self.route.length / 1000:.1f} m, '
                                       f'{len(self.route.landmarks)} lines)')
            except Exception as e:      # 지도 파일이 깨져도 거리 기준으로 달린다
                self.get_logger().error(f'route map load failed ({path}): {e} -> 거리 기준')
        self.fsm = MissionFSM(params)
        self.get_logger().info(f'mission start state: {self.fsm.state}')

        self.dist_mm = 0.0
        self.route_dist_mm = 0.0      # 경로 위치용 거리: 후진이면 뺌 (route_signed_dist)
        self.route_s_map = None       # 길을 잃어도 화면·곡률 미리보기용으로는 계속 내는 s
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
        self.m7_exit_lane = None     # /m7/exit_lane: M7이 끝난 차로 {'lane': 1|2|3, 'fork': 'A'|'B'|None, ...}
        self.parking = {}
        self.start_flag = False
        self.estop = False
        self.twists = {s: (None, 0.0) for s in SOURCES}   # (Twist, 받은 시각)
        self._last_lane_cfg = None
        self._last_cfg_pub = 0.0
        self._last_state = None
        self._last_fallback_warn = 0.0
        self._auto_s_done = False
        self.manual = bool(self.get_parameter('manual_drive').value)
        self.move_dir = 1             # 마지막 오도메트리 걸음 방향 (+1 전진, -1 후진)
        self.rev_src = ''             # 후진 판단 근거
        self.reverse_mm = 0.0         # 지금까지 후진한 거리 (디버그)
        self._cmd_v = (None, 0.0)     # 최근 /cmd_vel 속도, 받은 시각
        if self.manual:
            self.get_logger().warning('manual_drive: /cmd_vel을 내지 않습니다. teleop으로 조종하세요 (위치 확인용)')

        if self.use_class_commands:
            self.create_subscription(String, '/perception/commands', self._on_commands, 10)
        else:
            self.create_subscription(String, '/perception/detections', self._on_det, 10)
        self.create_subscription(String, '/lane/status', self._on_lane, 10)
        self.create_subscription(String, '/obstacle/state', self._json_cb('obstacle'), 10)
        # M7이 끝난 차로 (latched): M2 시작 때 이 차로에서 이어 주행. /obstacle/state의 m7_exit가 없을 때 대신 씀
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/m7/exit_lane', self._on_m7_exit, latched)
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
        self.create_subscription(String, '/mission/skip', self._on_skip, 10)
        self.create_subscription(Float32, '/route/set_s', self._on_set_s, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

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

    def _on_m7_exit(self, msg):
        try:
            self.m7_exit_lane = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.get_logger().info(f'/m7/exit_lane 받음: {self.m7_exit_lane.get("text") or self.m7_exit_lane}')

    def _obstacle_input(self) -> dict:
        """M2에서 /obstacle/state에 m7_exit가 없으면(obstacle_node 재시작 등) /m7/exit_lane 값으로 채운다."""
        ob = self.obstacle
        if getattr(self, 'practice_m2', False) and self.fsm.state == 'M2_FORK':
            # 연습으로 M2에 바로 들어옴: 지난 주행의 M7 종료 차로(obstacle_node가 계속 내는 m7_exit)는 쓰지 않는다
            return {k: v for k, v in (ob or {}).items() if k != 'm7_exit'}
        ex = getattr(self, 'm7_exit_lane', None) or {}
        if self.fsm.state == 'M2_FORK' and ob.get('m7_exit') not in ('A', 'B') and ex.get('fork') in ('A', 'B'):
            ob = dict(ob, m7_exit=ex['fork'])
        return ob

    def _on_lane(self, msg):
        try:
            self.lane = json.loads(msg.data)
            self.new_lane = True
        except json.JSONDecodeError:
            pass

    def _route_reset(self, s: float, why: str, align: bool = False):
        if self.tracker is None:
            return
        self.tracker.reset(s, self.route_dist_mm)
        self.route_active = True
        aligned = ''
        if (align and self.mappose is not None
                and self.get_parameter('route_align_on_reset').value):
            aligned = ', heading aligned' if self.mappose.align(s, self.yaw, self.dist_mm) else ', no IMU yet'
        self.get_logger().info(f'route s = {s:.0f} mm ({why}{aligned})')

    def _on_cmd_vel(self, msg):
        self._cmd_v = (msg.linear.x, time.monotonic())

    def _apply_anchor(self):
        """FSM이 카메라로 확인한 미션 선(정지선·시작선)을 지나거나 그 앞에 서면 s를 그 선 기준으로 다시 맞춘다."""
        a = self.fsm.pop_anchor()
        if a is None or self.tracker is None or not self.route_active:
            return
        name, rem = a
        s0 = self.route.points.get(name)
        if s0 is None:
            return
        s_new = s0 - rem                  # 앞바퀴 위치 = 선 - 남은 거리 (지났으면 rem < 0)
        jump = s_new - self.tracker.s
        if abs(jump) > float(self.get_parameter('route_anchor_max_mm').value):
            self.get_logger().warning(f'anchor {name} ignored: {jump:+.0f}mm off (route_anchor_max_mm)')
            return
        self._route_reset(s_new, f'anchor {name} (camera, {jump:+.0f}mm)')

    def _on_set_s(self, msg):
        self._route_reset(float(msg.data), 'manual', align=True)

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
            step = 1000.0 * math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
            # 한 메시지에 이만큼 넘게 튀면 odom 오류(11/09 체크섬 오류 때 7.5km)로 보고 버린다.
            # FSM뿐 아니라 도면 위치(RouteTracker)도 이 거리를 쓰므로 여기서 거른다 (11/10 통주행 시뮬레이션에서 발견)
            if step <= self.odom_step_max_mm:
                self.dist_mm += step
                sign, src = 1, ''
                if self.get_parameter('route_signed_dist').value:
                    v, t = self._cmd_v
                    cmd_v = v if v is not None and time.monotonic() - t < 0.5 else None
                    sign, src = motion_sign(p.x - self._last_xy[0], p.y - self._last_xy[1],
                                            yaw_from_quaternion(msg.pose.pose.orientation),
                                            msg.twist.twist.linear.x, cmd_v)
                if step > 1.0:
                    self.move_dir, self.rev_src = sign, src
                if sign < 0:
                    self.reverse_mm += step
                signed = sign * step
                along = signed
                if self.route_active and self.mappose is not None:
                    scale = float(self.get_parameter('odom_scale').value) or 1.0
                    proj = self.mappose.advance(signed * scale, self.tracker.s, self.yaw) / scale
                    if self.get_parameter('route_project_heading').value:
                        along = proj
                self.route_dist_mm += along
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
            self._route_reset(0.0, 'start line', align=True)

    def _on_estop(self, msg):
        self.estop = bool(msg.data)
        self.get_logger().warning(f'E-STOP {"ON" if self.estop else "off"}')

    def _on_set_state(self, msg):
        target = msg.data.strip()
        if target in SEQUENCE and SEQUENCE.index(target) <= SEQUENCE.index('M2_FORK'):
            # 11/11: 갈림길 이전 미션부터 다시 하면 지난 주행의 갈림길 선택·M7 종료 차로를 버린다
            # (예전엔 M2를 연습해도 지난번 A/B를 바로 써서 표지 인식을 시험할 수 없었음)
            self.fsm.fork_choice = None
            self.m7_exit_lane = None
        self.practice_m2 = target == 'M2_FORK'
        try:
            self.fsm._goto(target, self._inputs(), 'manual')
            self.get_logger().info(f'state set to {self.fsm.state}')
            if self.route is not None and self.fsm.state in STATE_START_POINT:
                name, off = STATE_START_POINT[self.fsm.state]
                base = self.route.points.get(name, 0.0) if name else 0.0
                self._route_reset(base + off, f'{self.fsm.state} start point', align=True)
        except ValueError as e:
            self.get_logger().error(str(e))

    def _on_skip(self, msg):
        """지금 자리에서 다른 미션으로 (경로 위치·갈림길 선택은 그대로)."""
        target = msg.data.strip()
        try:
            self.fsm._goto(target, self._inputs(), 'skip')
            self.practice_m2 = False
            self.get_logger().info(f'state skipped to {self.fsm.state} (route s kept)')
        except ValueError as e:
            self.get_logger().error(str(e))

    def _inputs(self) -> Inputs:
        route_s = None
        if self.route_active:
            lines = self.lane.get('stoplines_mm') if self.new_lane else None
            xw = self.lane.get('crosswalk_lines_mm') if self.new_lane else None
            route_s = self.tracker.update(self.route_dist_mm, lines, xw)
            self.new_lane = False
            self.route_s_map = route_s
            if self.tracker.lost:
                # 길 잃음: 틀린 위치로 미션·모드를 바꾸지 않게 FSM에는 '경로 정보 없음'으로 준다
                # (FSM은 표지·카메라 선·구간 거리로 판단. 카메라 미션 선을 보면 pop_anchor로 다시 맞춤)
                route_s = None
        self.command_inbox.set_mode(self.fsm.state)
        return Inputs(t=time.monotonic(), dist_mm=self.dist_mm, yaw=self.yaw,
                      confirmed_classes=(dict(self.command_inbox.classes)
                                         if self.use_class_commands else None),
                      detections=dict(self.det), new_detection_frame=self.new_det,
                      lane=self.lane, obstacle=self._obstacle_input(), parking=self.parking,
                      start=self.start_flag, route_s=route_s, tilt_rate=getattr(self, 'tilt_peak', None))

    # --- 틱 -----------------------------------------------------------------
    def _tick(self):
        s0 = float(self.get_parameter('route_start_s').value)
        if not self._auto_s_done and s0 >= 0.0 and self.tracker is not None and self._last_xy:
            self._route_reset(s0, 'route_start_s', align=True)
            self._auto_s_done = True
        inp = self._inputs()
        self.new_det = False
        self.start_flag = False
        self.tilt_peak = 0.0
        cmd = self.fsm.step(inp)
        self._apply_anchor()

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
        if not self.manual:
            self.cmd_pub.publish(out)
        rs = self.route_s_map if self.route_active else None
        # 11/12 밤: M7 직전 코너는 IMU로 지킨 코너 곡률을 도면 곡률 대신 넣는다 (도면 위치를 잃어도)
        ck = self.fsm.m7_corner_k(self.dist_mm, self.yaw)
        if rs is None and ck is not None:
            self.preview_pub.publish(String(data=json.dumps(
                {'step_mm': 100.0, 'k': ck, 'lost': False, 'm7_corner': True})))
        if rs is not None:
            step = 100.0
            ks = [round(self.route.curvature(rs + i * step), 4) for i in range(14)]
            msg = {'s': round(rs), 'step_mm': step, 'k': ks,
                   'lost': bool(self.tracker.lost), 'dir': self.move_dir, 'rev_src': self.rev_src}
            if ck is not None:
                msg['k'], msg['lost'], msg['m7_corner'] = ck, False, True
            mp = self.mappose
            if mp is not None:
                # 좌우 d는 _on_odom에서 걸음마다 mp.advance()로 적분한다 (후진·방향 반영)
                mp.update_lane(rs, self.yaw, self.lane, self.dist_mm)
                e = mp.heading_error(rs, self.yaw)
                if e is not None:
                    msg['dpsi'] = round(e, 4)
                msg['pose_ok'] = mp.valid(self.dist_mm) and self.yaw is not None
                if mp.yaw_off is not None and self.yaw is not None:
                    # 11/11: 차선으로 최근에 못 맞췄어도(pose_ok=false) 오도메트리로 적분한 d·방향은 계속 낸다.
                    # 예전엔 pose_ok일 때만 내서 차로를 벗어나면 디버그 화면이 차를 가운데에 그렸다
                    msg['d_mm'] = round(mp.d)
                    msg['psi'] = round(self.yaw + mp.yaw_off, 4)      # 도면 좌표계 차 방향 (디버그 화면용)
                    msg['lane_fix'] = mp.lane_fix
                    msg['lane_c'] = round(mp.lane_center)
                if msg['pose_ok']:
                    msg['pts'] = mp.points_ahead(rs, self.yaw)
            self.preview_pub.publish(String(data=json.dumps(msg)))

        state = {'state': self.fsm.state, 'sub': self.fsm.sub, 'source': source,
                 # 디버그 화면 '판단 설명'용 (11/11): 자율주행이 낼 명령(수동 모드에서도 계산), 차선 설정, 정지선 남은 거리
                 'auto_cmd': [round(out.linear.x, 3), round(out.angular.z, 3)],
                 'lane_cfg': cmd.lane, 'stop': self._stop_info(),
                 'manual': self.manual, 'dir': self.move_dir, 'reverse_mm': round(self.reverse_mm),
                 'fork_choice': self.fsm.fork_choice, 'dist_mm': round(self.dist_mm),
                 'estop': self.estop, 'note': cmd.note,
                 'parking_sign': self.fsm.parking_sign, 'parking_armed': self.fsm.parking_armed,
                 'm7_exit_lane': (self.m7_exit_lane or {}).get('lane'),
                 'route': self.tracker.state() if self.route_active else None}
        self.state_pub.publish(String(data=json.dumps(state)))
        if (self.fsm.state, self.fsm.sub) != self._last_state:
            self.get_logger().info(f'[{self.fsm.state}/{self.fsm.sub}] {self.fsm.log[-1]}')
            if self.fsm.state != 'M2_FORK':
                self.practice_m2 = False
            if self.fsm.state == 'M7_OBSTACLE' and (self._last_state or (None,))[0] != 'M7_OBSTACLE':
                self.m7_exit_lane = None             # 지난 주행의 latched 값은 버림
            if self.fsm.state == 'M2_FORK' and (self._last_state or (None,))[0] != 'M2_FORK':
                ex = self.m7_exit_lane or {}
                self.get_logger().info(f'[M2] 시작 차로: {ex.get("lane", "?")}차선 (fork {ex.get("fork")}, '
                                       f'/m7/exit_lane) / 표지 {self.fsm.fork_choice}')
            self._last_state = (self.fsm.state, self.fsm.sub)


    def _stop_info(self):
        """정지선 접근 중이면 남은 거리(앞바퀴 기준)와 근거. FSM 상태를 바꾸지 않게 저장된 값으로만 계산한다."""
        st = getattr(self.fsm, 'stop', None)
        if st is None or self.fsm.sub not in ('approach', 'stopped'):
            return None
        rem, how = None, None
        if st.seen_d is not None and st.seen_at is not None:
            rem, how = st.seen_d - (self.dist_mm - st.seen_at), st.how
        elif st.route_line_s is not None and self.route_active and not self.tracker.lost:
            rem, how = st.route_line_s - self.tracker.s, 'route'
        return {'line': self.fsm.stop_line_name, 'rem_mm': None if rem is None else round(rem),
                'how': how, 'stop_at_mm': round(st.target_mm + st.brake_mm)}


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.manual:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
