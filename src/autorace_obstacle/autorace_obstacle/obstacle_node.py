"""라이다 장애물 노드.

/scan -> 전방 덩어리(클러스터) -> /obstacle/state (String JSON)
미션 상태(/mission/state)에 따라 /obstacle/cmd_vel을 낸다 (mission_node가 골라서 /cmd_vel로).

  M8_CONES   : 라바콘 회피 (cone_core.ConeFollower). 순서
                 sub ''       (접근, 차선 주행 중) 콘을 미리 추적만 한다. 차 앞 차로 안 콘 거리를 cone_ahead_mm로 보고
                 sub 'cones'  처음 보는 순간 구간 좌표계를 고정하고 콘 사이 경로를 따라간다
                 sub 'handover' 경로 끝(차로 중심, 구간 방향 직진)을 계속 따라가며 차선 노드가 차선을 다시 잡길 기다림
               마지막 콘을 지나면 /obstacle/state의 cones_done = true
               카메라: 콘 바닥 위치를 BEV 호모그래피로 이미지에 찍어 주황색인지 본다 (아니면 그 콘을 뺀다)
  M7_OBSTACLE: 회피 주행은 limo_obstacle_avoid 패키지의 avoid_node가 한다 (그 출력은 launch에서
               /obstacle/cmd_vel로 remap). 이 노드는 연결만 맡는다 (m7_link.py):
                 시작선에서 /avoid/enable, 갈림길 방향 -> /fork_direction,
                 /avoid/done -> /obstacle/state의 m7_done, 회피 중이면 m7_active.
  그래서 /obstacle/cmd_vel은 M8_CONES일 때만 낸다 (다른 상태에서 0을 내면 avoid_node 명령과 섞인다).

구독: /scan, /mission/state, /lane/status, odom_topic, imu_topic, image_topic(카메라 확인용),
      /avoid/active, /avoid/done, /avoid/state
발행: /obstacle/cmd_vel, /obstacle/state (20 Hz, 라이다가 없어도 m7_done이 FSM에 가게),
      /obstacle/debug/compressed, /avoid/enable, /fork_direction

녹화 다시 보기: ros2 bag play <bag> 와 이 노드만 켜면 디버그 그림을 볼 수 있다 (rqt_image_view).
  bag에 /mission/state가 없으면: ros2 topic pub -r 5 /mission/state std_msgs/msg/String \
      "{data: '{\"state\": \"M8_CONES\", \"sub\": \"cones\"}'}"
"""

import json
import math
import time
from dataclasses import fields

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, String

from autorace_obstacle import cone_debug
from autorace_obstacle.cone_core import (ConeFollower, ConeParams, GroundProjector, Pose2D,
                                         PoseIntegrator, YawOffsetEstimator, orange_ratio, wrap)
from autorace_obstacle.m7_link import M7Link, M7LinkParams
from autorace_obstacle.scan_utils import cluster_points, lane_occupancy, roi, scan_to_points

# 이 상태들에서는 차선 방향이 도면 방향과 다를 수 있어 yaw 차이를 배우지 않는다
NO_LEARN_STATES = ('M6_ZIGZAG', 'M2_FORK', 'M8_CONES', 'M4_PARKING', 'FINISHED')


def g_name(name: str) -> str:
    return name if name.startswith('/') else '/' + name


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def image_to_bgr(msg: Image):
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    h, w, step = msg.height, msg.width, msg.step
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img
    if enc in ('yuv422', 'yuyv', 'yuv422_yuy2'):
        img = buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    return None


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('obstacle_node')
        p = self.declare_parameter
        dyn = ParameterDescriptor(dynamic_typing=True)
        p('scan_topic', '/scan')
        p('odom_topic', '/odom')
        p('imu_topic', '/imu')
        p('image_topic', '/camera/color/image_raw')
        p('bev_config', '')               # 차선 노드와 같은 bev_limo.yaml (카메라 확인용)
        p('laser_x_mm', 100.0)            # TODO 실측: 뒤차축 -> 라이다 중심 전방 거리
        p('laser_y_mm', 0.0)
        p('laser_yaw', 0.0)               # TODO 확인: 라이다 0도가 차 정면인지 (2°만 틀려도 여유가 크게 줆)
        p('roi_x_max', 2000.0)
        p('roi_y_half', 900.0)
        p('cluster_gap_mm', 60.0)
        p('cluster_min_pts', 3)
        p('lane_centers_y', [500.0, 0.0, -500.0])   # 3차선 (왼쪽 +). TODO 실측
        p('lane_half_width', 230.0)
        p('control_hz', 20.0)
        p('odom_scale', 1.0)
        # 라바콘 구간 방향: 도면 좌표 기준 (좌측 직선을 북쪽으로 = 90도)
        p('m8_axis_track_deg', 90.0)
        p('lane_row_mm', 550.0)           # 차선 노드 offset_mm을 재는 행 (BEV 맨 아래 = 뒤차축 앞 550)
        p('camera_check', True)
        p('cam_every', 3)                 # 카메라 N프레임마다 한 번 확인
        p('cam_orange_min', 0.12)         # 상자 안 주황 화소 비율이 이 이상이면 '주황으로 보임'
        p('publish_debug', True)
        defaults = ConeParams()
        for f in fields(ConeParams):
            v = getattr(defaults, f.name)
            p('cones.' + f.name, list(v) if isinstance(v, tuple) else v, dyn)
        # M7 연결 (m7_link.M7LinkParams 항목을 m7_ 접두어로 선언. 예: m7_start_s_mm)
        p('m7_avoid_node', '/avoid_node')   # section_length를 넣을 회피 노드 이름
        lp = M7LinkParams()
        for f in fields(lp):
            v = getattr(lp, f.name)
            v = list(v) if isinstance(v, tuple) else v        # ROS 파라미터는 tuple을 못 받음
            setattr(lp, f.name, p('m7_' + f.name, v).value)
        self.link = M7Link(lp)

        g = self.get_parameter
        cp = {f.name: g('cones.' + f.name).value for f in fields(ConeParams)}
        cp['lookahead_scales'] = tuple(cp['lookahead_scales'])
        self.cone_p = ConeParams.from_dict(cp)
        self.fol = ConeFollower(self.cone_p)
        self.integ = PoseIntegrator(g('odom_scale').value)
        self.yaw_est = YawOffsetEstimator()
        self.proj = None
        path = g('bev_config').value
        if path and g('camera_check').value:
            try:
                self.proj = GroundProjector.from_yaml(path)
                if not self.proj.calibrated:
                    self.get_logger().warning('BEV 미캘리브레이션: 카메라 콘 확인 끔')
                    self.proj = None
            except Exception as e:
                self.get_logger().error(f'BEV 로드 실패 ({path}): {e} -> 카메라 콘 확인 끔')

        self.mission = {}
        self.mstate = None
        self.msub = None
        self.lane = {}
        self.lane_t = 0.0
        self.last = {}
        self.cands = []
        self.img_count = 0
        self.dbg_t = 0.0
        self.lidar_xy = (g('laser_x_mm').value, g('laser_y_mm').value)

        self.create_subscription(LaserScan, g('scan_topic').value, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, g('odom_topic').value, self._on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, g('imu_topic').value, self._on_imu, qos_profile_sensor_data)
        img_topic = g('image_topic').value
        if self.proj is not None:
            if img_topic.endswith('/compressed'):
                self.create_subscription(CompressedImage, img_topic, self._on_compressed,
                                         qos_profile_sensor_data)
            else:
                self.create_subscription(Image, img_topic, self._on_image,
                                         qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self._on_mission, 10)
        self.create_subscription(String, '/lane/status', self._on_lane, 10)
        self.state_pub = self.create_publisher(String, '/obstacle/state', 10)
        self.cmd_pub = self.create_publisher(Twist, '/obstacle/cmd_vel', 10)
        self.dbg_pub = self.create_publisher(CompressedImage, '/obstacle/debug/compressed', 2)
        # M7 연결
        self.create_subscription(Bool, '/avoid/active',
                                 lambda m: self.link.on_avoid_active(m.data, time.monotonic()), 10)
        self.create_subscription(Bool, '/avoid/done', lambda m: self.link.on_avoid_done(m.data), 10)
        self.create_subscription(String, '/avoid/state', self._on_avoid_state, 10)
        self.create_subscription(String, '/perception/detections', self._on_detections, 10)  # M7 끄기 (off_mode: sign)
        self.enable_pub = self.create_publisher(Bool, '/avoid/enable', 10)
        self.fork_pub = self.create_publisher(String, '/fork_direction', 10)
        # M7이 끝나는 순간 몇 차선인지 (M2가 받아서 이어 주행). 늦게 켠 노드도 받도록 마지막 값 보관(latched)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.exit_pub = self.create_publisher(String, '/m7/exit_lane', latched)
        # 지금 몇 차선인지 늘 (회피 중: 회피 노드 / 그 밖: 차선 노드 선 종류). 알 때만, 바뀌거나 1초마다
        self.lane_pub = self.create_publisher(String, '/m7/lane', 10)
        self.lane_pub_last = (None, 0.0)
        self.param_cli = self.create_client(
            SetParameters, g_name(g('m7_avoid_node').value) + '/set_parameters')
        self.create_timer(1.0 / float(g('control_hz').value), self._tick)
        self.get_logger().info(f'obstacle_node up: camera check {"on" if self.proj else "off"}, '
                               f'lidar at {self.lidar_xy} mm')

    # --- 입력 ---------------------------------------------------------------
    def _on_imu(self, msg):
        self.integ.on_imu(yaw_from_quaternion(msg.orientation))

    def _on_odom(self, msg):
        q = msg.pose.pose.position
        self.integ.on_odom(q.x, q.y, msg.twist.twist.linear.x)

    def _on_mission(self, msg):
        try:
            m = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        st, sub = m.get('state'), m.get('sub')
        if st != self.mstate:
            if st == 'M8_CONES' or self.mstate == 'M8_CONES':
                self.fol.reset()                       # 들어갈 때·나올 때 깨끗이
                self.get_logger().info(f'cones: reset ({self.mstate} -> {st})')
        self.mission, self.mstate, self.msub = m, st, sub
        if (st == 'M8_CONES' and sub in ('cones', 'handover') and self.fol.corridor is None
                and self.integ.ready):
            self._start_corridor()

    def _on_lane(self, msg):
        try:
            self.lane = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.lane_t = time.monotonic()
        self._learn_yaw()

    def _on_detections(self, msg):
        try:
            dets = json.loads(msg.data).get('detections', [])
        except (json.JSONDecodeError, AttributeError):
            return
        self.link.on_detections(dets)

    def _on_avoid_state(self, msg):
        try:
            self.link.on_avoid_state(json.loads(msg.data))
        except (json.JSONDecodeError, AttributeError):
            pass

    def _enable(self, section_m):
        """M7: 회피 노드 section_length(원점 ~ 섬 끝)를 지금 위치로 맞춘 뒤 켠다."""
        if not self.param_cli.service_is_ready():
            self.get_logger().warning('[M7] 회피 노드 파라미터 서비스 없음 -> section_length 기본값으로 켬')
            self.enable_pub.publish(Bool(data=True))
            return
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name='section_length', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(section_m)))]
        self.param_cli.call_async(req).add_done_callback(
            lambda _f: self.enable_pub.publish(Bool(data=True)))

    def _learn_yaw(self):
        """직선 구간에서 'IMU yaw + 차선 방향 - 도면 방향'을 모은다."""
        r = self.mission.get('route')
        ln, yaw = self.lane, self.integ.yaw_imu
        if not r or yaw is None or self.mstate in NO_LEARN_STATES:
            return
        if ln.get('status') != 'BOTH' or ln.get('heading_rad') is None:
            return
        if abs(r.get('kappa') or 0.0) > 1e-6 or abs(r.get('kappa_ahead') or 0.0) > 1e-6:
            return
        if abs(ln['heading_rad']) > 0.15 or abs(ln.get('path_kappa') or 0.0) > 0.3:
            return
        self.yaw_est.add(yaw, ln['heading_rad'], r['heading'])

    def _lane_recent(self, max_age=0.5):
        return self.lane if time.monotonic() - self.lane_t <= max_age else {}

    def _start_corridor(self):
        pose = Pose2D(self.integ.pose.x, self.integ.pose.y, self.integ.pose.yaw)
        ln = self._lane_recent()
        off = self.yaw_est.offset()
        if off is not None:
            axis, src = wrap(math.radians(self.get_parameter('m8_axis_track_deg').value) + off), \
                'route'
        elif ln.get('heading_rad') is not None and ln.get('status') != 'NONE' \
                and abs(ln['heading_rad']) < 0.35:
            axis, src = wrap(pose.yaw + ln['heading_rad']), 'lane'
        else:
            axis, src = pose.yaw, 'yaw'
        lane_pt = None
        if ln.get('offset_mm') is not None and ln.get('status') != 'NONE':
            lane_pt = (self.get_parameter('lane_row_mm').value, float(ln['offset_mm']))
        k0 = float(ln.get('kappa') or 0.0)
        v0 = float(ln.get('speed_cmd') or 0.0)
        cor = self.fol.start(pose, axis, lane_pt, src, kappa0=k0, v0=v0)
        spread = self.yaw_est.spread()
        self.get_logger().info(
            f'cones: corridor start axis={math.degrees(axis):.1f}deg ({src}'
            f'{"" if spread is None else f", spread {math.degrees(spread):.1f}deg"}, '
            f'samples {len(self.yaw_est.buf)}) y_lane={cor.y_lane:.0f}mm '
            f'heading_err={math.degrees(wrap(pose.yaw - axis)):.1f}deg')

    def _on_scan(self, msg: LaserScan):
        g = self.get_parameter
        pts = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                             msg.range_min, msg.range_max, g('laser_x_mm').value,
                             g('laser_y_mm').value, g('laser_yaw').value)
        front = roi(pts, 0.0, g('roi_x_max').value, -g('roi_y_half').value,
                    g('roi_y_half').value)
        clusters = cluster_points(front, g('cluster_gap_mm').value, g('cluster_min_pts').value)
        occ = lane_occupancy(clusters, g('lane_centers_y').value,
                             g('lane_half_width').value, g('roi_x_max').value)
        near = [c['near_x'] for c in clusters if abs(c['y']) < 150.0]
        # M7 켜기(enable_mode: obstacle): 라이다 -> 앞 차로(차 중심 ±trigger_half_width_mm) 안 가장 가까운 장애물
        lx, hw = g('laser_x_mm').value, self.link.p.trigger_half_width_mm
        ahead = [c['near_x'] - lx for c in clusters if abs(c['y']) <= hw and c['near_x'] > lx]
        self.link.on_front(min(ahead) if ahead else None)
        state = {'clusters': clusters[:12], 'lane_occupancy': occ,
                 'front_min_mm': min(near) if near else None,
                 'cones_done': False, 'cone_ahead_mm': None, 'cone_any_ahead_mm': None}
        if self.mstate == 'M8_CONES' and self.integ.ready:
            pose = self.integ.pose
            self.cands = self.fol.on_scan(pts, Pose2D(pose.x, pose.y, pose.yaw), self.lidar_xy)
            state['cones_done'] = self.fol.done
            state['cone_ahead_mm'] = self.fol.cone_ahead_mm(pose)
            state['cone_any_ahead_mm'] = self.fol.nearest_ahead_mm(pose)
            state['cones'] = self.fol.state_dict()
        self.last = state       # /obstacle/state는 _tick에서 m7 값과 합쳐 발행

    # --- 카메라 확인 ----------------------------------------------------------
    def _on_compressed(self, msg):
        if self._want_image():
            bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if bgr is not None:
                self._check_camera(bgr)

    def _on_image(self, msg):
        if self._want_image():
            bgr = image_to_bgr(msg)
            if bgr is not None:
                self._check_camera(bgr)

    def _want_image(self) -> bool:
        if self.mstate != 'M8_CONES' or not self.fol.tracker.tracks:
            return False
        self.img_count += 1
        return self.img_count % max(1, int(self.get_parameter('cam_every').value)) == 0

    def _check_camera(self, bgr):
        proj = self.proj
        if bgr.shape[1] != proj.w or bgr.shape[0] != proj.h:
            bgr = cv2.resize(bgr, (proj.w, proj.h), interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        thr = self.get_parameter('cam_orange_min').value
        pose = self.integ.pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        for t in self.fol.tracker.tracks:
            dx, dy = t.x - pose.x, t.y - pose.y
            xv, yv = c * dx + s * dy, -s * dx + c * dy
            if xv < 300.0 or xv > 2000.0:
                continue
            box = proj.cone_box(xv, yv, self.cone_p.cone_base_radius_mm)
            if box is None:
                continue
            t.add_camera(orange_ratio(hsv, box) >= thr, self.cone_p)

    # --- 제어 ---------------------------------------------------------------
    def _tick(self):
        now = time.monotonic()
        # --- M7 연결 (켜기/끄기/갈림길 방향). 회피 주행 명령은 avoid_node가 직접 낸다
        for act in self.link.update(now, self.mission, self.lane):
            if act[0] == 'enable':
                self._enable(act[1])
            elif act[0] == 'disable':
                self.enable_pub.publish(Bool(data=False))
            elif act[0] == 'fork':
                self.fork_pub.publish(String(data=act[1]))
        ln = self.link.lane_now()
        if ln is not None:
            key = (ln['lane'], ln['src'])
            if key != self.lane_pub_last[0] or now - self.lane_pub_last[1] >= 1.0:
                ln['state'] = self.mstate
                self.lane_pub.publish(String(data=json.dumps(ln, ensure_ascii=False)))
                self.lane_pub_last = (key, now)
        rep = self.link.pop_report()
        if rep is not None:
            self.exit_pub.publish(String(data=json.dumps(rep, ensure_ascii=False)))
            self.get_logger().info(f'[M7] {rep["text"]} -> /m7/exit_lane')
        for e in self.link.events:
            self.get_logger().info(f'[M7] {e}')
        self.link.events.clear()

        # --- M8 라바콘: 이 상태에서만 /obstacle/cmd_vel을 낸다
        if self.mstate == 'M8_CONES' and self.fol.corridor is not None and self.integ.ready:
            # 접근 중(corridor 없음)이거나 odom/IMU가 없으면 아예 안 낸다
            # -> mission_node가 끊김으로 보고 차선 주행으로 대신 (0 명령으로 차를 세우지 않기 위해)
            pose = self.integ.pose
            v, k = self.fol.control(Pose2D(pose.x, pose.y, pose.yaw))
            tw = Twist()
            tw.linear.x, tw.angular.z = float(v), float(v * k)
            self.cmd_pub.publish(tw)

        state = dict(self.last)
        state.update(self.link.state())
        self.state_pub.publish(String(data=json.dumps(state)))

        if (self.mstate == 'M8_CONES' and self.get_parameter('publish_debug').value
                and now - self.dbg_t >= 0.2):
            self.dbg_t = now
            pose = self.integ.pose
            img = cone_debug.draw(self.fol, Pose2D(pose.x, pose.y, pose.yaw),
                                  f'M8/{self.msub}')
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                m = CompressedImage()
                m.header.stamp = self.get_clock().now().to_msg()
                m.format = 'jpeg'
                m.data = jpg.tobytes()
                self.dbg_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mstate == 'M8_CONES':
            node.cmd_pub.publish(Twist())   # M7 중에는 avoid_node 토픽이라 건드리지 않는다
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
