"""라이다 장애물 노드.

/scan -> 전방 덩어리(클러스터) -> /obstacle/state (String JSON)
미션 상태(/mission/state)에 따라 /obstacle/cmd_vel을 낸다 (mission_node가 골라서 /cmd_vel로).

  M8_CONES   : 라바콘 회피 (cone_core.ConeFollower). 순서
                 sub ''       (접근, 차선 주행 중) 콘을 미리 추적만 한다. 차 앞 차로 안 콘 거리를 cone_ahead_mm로 보고
                 sub 'cones'  처음 보는 순간 구간 좌표계를 고정하고 콘 사이 경로를 따라간다
                 sub 'handover' 경로 끝(차로 중심, 구간 방향 직진)을 계속 따라가며 차선 노드가 차선을 다시 잡길 기다림
               마지막 콘을 지나면 /obstacle/state의 cones_done = true
               카메라: 콘 바닥 위치를 BEV 호모그래피로 이미지에 찍어 주황색인지 본다 (아니면 그 콘을 뺀다)
  M7_OBSTACLE: TODO. 차로 점유(lane_occupancy)로 빈 차로를 고르고 차선 변경.
               갈림길 방향(/mission/state의 fork_choice)에 맞는 쪽 차로로 끝내야 한다.
               방법 A: lane 노드의 target_offset_mm(±500)으로 옆 차로 중심을 따라가게 한다.
               방법 B: 오도메트리로 미리 짠 S자 경로를 재생한다.

구독: /scan, /mission/state, /lane/status, odom_topic, imu_topic, image_topic(카메라 확인용)
발행: /obstacle/cmd_vel, /obstacle/state, /obstacle/debug/compressed

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
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import String

from autorace_obstacle import cone_debug
from autorace_obstacle.cone_core import (ConeFollower, ConeParams, GroundProjector, Pose2D,
                                         PoseIntegrator, YawOffsetEstimator, orange_ratio, wrap)
from autorace_obstacle.scan_utils import cluster_points, lane_occupancy, roi, scan_to_points

# 이 상태들에서는 차선 방향이 도면 방향과 다를 수 있어 yaw 차이를 배우지 않는다
NO_LEARN_STATES = ('M6_ZIGZAG', 'M2_FORK', 'M8_CONES', 'M4_PARKING', 'FINISHED')


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
        state = {'clusters': clusters[:12], 'lane_occupancy': occ,
                 'front_min_mm': min(near) if near else None,
                 'm7_done': False, 'cones_done': False, 'cone_ahead_mm': None}
        if self.mstate == 'M8_CONES' and self.integ.ready:
            pose = self.integ.pose
            self.cands = self.fol.on_scan(pts, Pose2D(pose.x, pose.y, pose.yaw), self.lidar_xy)
            state['cones_done'] = self.fol.done
            state['cone_ahead_mm'] = self.fol.cone_ahead_mm(pose)
            state['cones'] = self.fol.state_dict()
        self.last = state
        self.state_pub.publish(String(data=json.dumps(state)))

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
        tw = Twist()
        if self.mstate == 'M8_CONES':
            if self.fol.corridor is None or not self.integ.ready:
                # 할 말이 없으면 아예 안 낸다 -> mission_node가 끊김으로 보고 차선 주행으로 대신
                # (접근 중이거나 odom/IMU가 없을 때 0 명령으로 차를 세우지 않기 위해)
                pass
            else:
                pose = self.integ.pose
                v, k = self.fol.control(Pose2D(pose.x, pose.y, pose.yaw))
                tw.linear.x, tw.angular.z = float(v), float(v * k)
                self.cmd_pub.publish(tw)
        else:
            if self.mstate == 'M7_OBSTACLE':
                pass   # TODO: 빈 차로 선택 + 차선 변경 (모듈 설명 참고)
            self.cmd_pub.publish(tw)
        now = time.monotonic()
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
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
