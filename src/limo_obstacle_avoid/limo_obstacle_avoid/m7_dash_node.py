#!/usr/bin/env python3
"""M7 점선 기준 회피 노드 (11/19 실험, 단독 실행). 판단은 m7_dash_core.M7Dash.

mission_node 없이 돈다: /cmd_vel 직접 (ackermann_bridge가 /limo/ack_cmd로). 나중에 합칠 때 cmd_topic:=/obstacle/cmd_vel.
차를 1|2차로 사이 점선 위, 시작선(커브 뒤 흰 가로선)에 똑바로 놓고 시작.

입력  /m7dash/enable (Bool)  true = 지금 자리·방향을 기준으로 시작, false = 정지
      /scan, /odom, /imu, image_topic (Image 또는 .../compressed), /lane/status (있으면 노란선 보조)
      /perception/detections (String JSON, 원본 detector_node) fork_a/fork_b 상자 높이 -> 표지판 가까우면 끝
출력  cmd_topic (Twist, 기본 /cmd_vel), /m7dash/state (String JSON), /m7dash/done (Bool)
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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
from std_msgs.msg import Bool, String

from autorace_lane.lane_core import BevConfig, remove_transverse
from autorace_lane.lane_node import image_msg_to_bgr
from limo_obstacle_avoid.m7_dash_core import DashParams, M7Dash, dash_points
from limo_obstacle_avoid.path_track import yaw_of


class M7DashNode(Node):
    def __init__(self):
        super().__init__('m7_dash_node')
        self.p = DashParams()
        for f in fields(DashParams):
            self.declare_parameter(f.name, getattr(self.p, f.name))
        for name, val in (('scan_topic', '/scan'), ('odom_topic', '/odom'), ('imu_topic', '/imu'),
                          ('image_topic', '/camera/color/image_raw'), ('lane_status_topic', '/lane/status'),
                          ('cmd_topic', '/cmd_vel'), ('bev_config', ''), ('white_v_min', 180), ('white_s_max', 60),
                          ('cam_hz', 10.0), ('odom_timeout', 0.3), ('scan_timeout', 0.5), ('range_min', 0.10),
                          ('range_max', 3.0), ('scan_yaw_offset_deg', 0.0), ('scan_flip', False),
                          ('start_y', 0.0), ('start_heading_deg', 0.0), ('dry_run', False),
                          ('det_topic', '/perception/detections'), ('sign_classes', ['fork_a', 'fork_b']),
                          ('sign_conf', 0.5)):
            self.declare_parameter(name, val)
        self._read_params()
        self.core = M7Dash(self.p)
        self.cfg = None
        path = self.gp('bev_config')
        if path:
            try:
                self.cfg = BevConfig.from_yaml(path)
            except (OSError, KeyError, ValueError) as e:
                self.get_logger().error(f'[M7 점선] BEV 못 읽음 ({path}): {e} -> 점선 보정 없이 (IMU·라이다만)')
        else:
            self.get_logger().warn('[M7 점선] bev_config 없음 -> 점선 보정 없이 (IMU·라이다만)')
        self.dry = bool(self.gp('dry_run'))
        out = self.gp('cmd_topic')
        self.pub_cmd = self.create_publisher(Twist, out + '_preview' if self.dry else out, 10)
        self.pub_state = self.create_publisher(String, '/m7dash/state', 10)
        self.pub_done = self.create_publisher(Bool, '/m7dash/done', 10)
        self.create_subscription(Bool, '/m7dash/enable', self.on_enable, 10)
        self.create_subscription(LaserScan, self.gp('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.gp('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, self.gp('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, self.gp('lane_status_topic'), self.on_lane, 10)
        self.create_subscription(String, self.gp('det_topic'), self.on_det, 10)
        img = self.gp('image_topic')
        if self.cfg is not None:
            if img.endswith('/compressed'):
                self.create_subscription(CompressedImage, img, self.on_compressed, qos_profile_sensor_data)
            else:
                self.create_subscription(Image, img, self.on_image, qos_profile_sensor_data)
        self.t_odom = self.t_scan = None
        self.t_cam = 0.0
        self.n_dash = 0
        self.active = False
        self.k = 0
        self.n_ev = 0
        self.create_timer(0.05, self.loop)
        self.get_logger().info(f'[M7 점선] 준비: 출력 {out}{" (dry_run)" if self.dry else ""}, 속도 {self.p.speed}, '
                               f'옆 여유 {100 * self.p.clear:.0f} cm, 끝 = '
                               + (f'표지판 상자 {self.p.end_sign_h_px:.0f} px 이상 {self.p.end_sign_frames}번 (x {self.p.end_min_x} m 뒤) / ' if self.p.end_by_sign else '')
                               + f'거리 {self.p.end_x} m, '
                               f'점선 {"켬" if self.cfg is not None else "끔"}. 시작: ros2 topic pub --once /m7dash/enable std_msgs/Bool "{{data: true}}"')

    def gp(self, name):
        return self.get_parameter(name).value

    def _read_params(self):
        for f in fields(DashParams):
            v, cur = self.gp(f.name), getattr(self.p, f.name)
            setattr(self.p, f.name, float(v) if isinstance(cur, float) else v)

    @staticmethod
    def stamp(m, fallback):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return t if t > 0 else fallback

    # ---------- 입력 ----------
    def on_imu(self, m):
        self.core.on_imu(self.stamp(m, time.monotonic()), m.angular_velocity.z)

    def on_odom(self, m):
        self.t_odom = time.monotonic()
        q = m.pose.pose.position
        self.core.on_odom(self.stamp(m, self.t_odom), q.x, q.y, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x)

    def on_scan(self, m):
        self.t_scan = time.monotonic()
        if not self.active:
            return
        off = math.radians(self.gp('scan_yaw_offset_deg'))
        flip = self.gp('scan_flip')
        rmin, rmax = self.gp('range_min'), self.gp('range_max')
        lx = self.p.lidar_x
        pts = []
        for i, r in enumerate(m.ranges):
            if math.isnan(r) or not (rmin <= r <= rmax):
                continue
            a = m.angle_min + i * m.angle_increment
            a = (-a if flip else a) + off
            pts.append((lx + r * math.cos(a), r * math.sin(a), a))
        pts.sort(key=lambda q: q[2])
        a0 = min(m.angle_min, m.angle_max) + off
        a1 = max(m.angle_min, m.angle_max) + off
        if pts:
            a0, a1 = max(a0, pts[0][2]), min(a1, pts[-1][2])
        self.core.on_scan(self.t_scan, pts, a0, a1)

    def on_lane(self, m):
        if not self.active:
            return
        try:
            self.core.on_lane(json.loads(m.data), time.monotonic())
        except (ValueError, TypeError, AttributeError):
            pass

    def on_det(self, m):
        """detector_node 형식 {"detections":[{"cls","conf","bbox":[x,y,w,h]}]} -> 표지판 상자 높이 최대값."""
        if not self.active:
            return
        try:
            dets = json.loads(m.data).get('detections', []) or []
        except (ValueError, AttributeError):
            return
        cls, conf = set(self.gp('sign_classes')), float(self.gp('sign_conf'))
        hs = []
        for d in dets:
            try:
                b = d.get('bbox') or []
                if d.get('cls') in cls and float(d.get('conf', 0.0)) >= conf and len(b) == 4:
                    hs.append(float(b[3]))
            except (AttributeError, TypeError, ValueError):
                continue
        self.core.on_sign(max(hs) if hs else None)

    def on_image(self, m):
        if self._cam_due():
            try:
                self._dash(image_msg_to_bgr(m))
            except ValueError as e:
                self.get_logger().warn(f'[M7 점선] 이미지: {e}', throttle_duration_sec=5.0)

    def on_compressed(self, m):
        if self._cam_due():
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                self._dash(img)

    def _cam_due(self):
        now = time.monotonic()
        if not self.active or now - self.t_cam < 1.0 / max(self.gp('cam_hz'), 0.1):
            return False
        self.t_cam = now
        return True

    def _dash(self, bgr):
        cfg = self.cfg
        if bgr.shape[1] != cfg.image_width or bgr.shape[0] != cfg.image_height:
            bgr = cv2.resize(bgr, (cfg.image_width, cfg.image_height))
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array([0, 0, int(self.gp('white_v_min'))], np.uint8),
                            np.array([179, int(self.gp('white_s_max')), 255], np.uint8))
        bev = remove_transverse(cfg.warp(white), 60.0 / cfg.mm_per_px_x)    # 가로선(시작선) 지움
        pts = dash_points(bev, cfg, self.core.dash_pred_mm, 1000.0 * self.p.dash_gate)
        self.n_dash += 1 if pts else 0
        self.core.on_dash(pts)

    def on_enable(self, m):
        if m.data and not self.active:
            if self.t_odom is None or self.t_scan is None or not self.core.fu.ready:
                self.get_logger().warn('[M7 점선] /odom·/scan 없음 또는 IMU 편향 측정 중(켜고 1초 정지) -> 시작 못 함')
                return
            self._read_params()
            self.core.p = self.p
            self.core.start(float(self.gp('start_y')), float(self.gp('start_heading_deg')))
            self.n_ev = 0
            self.active = True
            self.get_logger().info('▶ [M7 점선] 시작')
        elif not m.data and self.active:
            self.finish('수동 중단', done=False)

    # ---------- 주기 ----------
    def loop(self):
        if not self.active:
            return
        now = time.monotonic()
        if (self.t_odom is None or now - self.t_odom > self.gp('odom_timeout')
                or self.t_scan is None or now - self.t_scan > self.gp('scan_timeout')):
            self.pub_cmd.publish(Twist())
            self.get_logger().warn('[M7 점선] 센서 끊김 -> 정지', throttle_duration_sec=1.0)
            return
        v, kap = self.core.step(now)
        self._log_events()
        if self.core.state == 'DONE':
            self.finish('구간 끝', done=True)
            return
        tw = Twist()
        tw.linear.x = float(v)
        tw.angular.z = float(v * kap)
        self.pub_cmd.publish(tw)
        self.k += 1
        if self.k % 4 == 0:
            st = self.core.status()
            st.update(v=round(v, 3), kappa=round(kap, 2), dash_frames=self.n_dash)
            self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))

    def _log_events(self):
        for e in self.core.events[self.n_ev:]:
            self.get_logger().info(f'[M7 점선] {e}')
        self.n_ev = len(self.core.events)

    def finish(self, why, done):
        self.active = False
        self.pub_cmd.publish(Twist())
        self._log_events()
        st = self.core.status()
        st['state'] = 'DONE' if done else 'IDLE'
        self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if done:
            self.pub_done.publish(Bool(data=True))
        self.core.stop()
        self.get_logger().info(f'■ [M7 점선] {why}: {st["end"]} (보정 {st["fix"]}, 점선 본 프레임 {self.n_dash}, '
                               f'재판단 {st["n_rejudge"]}, 마지막 표지판 상자 {st["sign_h"]} px)')


def main(args=None):
    rclpy.init(args=args)
    n = M7DashNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            n.pub_cmd.publish(Twist())
        except Exception:
            pass
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
