#!/usr/bin/env python3
"""M7 점선 기준 회피 + M2 갈림길 진입 노드 (11/19). 판단은 m7_dash_core.M7Dash.

팀 주행(autorace.launch.py m7_impl:=dash, 기본): avoid_node 자리에서 replay·waypoint와 같은 토픽·파라미터로 돈다.
  obstacle_node(m7_link)가 3차선 시작선에서 section_length·start_heading_deg·start_y 등을 넣고 /avoid/enable true.
  끝나면 /avoid/done -> FSM M2 (갈래 A/B 차선 주행). 출력 /avoid/cmd_vel은 launch에서 /obstacle/cmd_vel로.
단독 시험(m7_dash.launch.py): /avoid/cmd_vel -> /cmd_vel, 차를 점선 위에 놓고(start_on_dash) /avoid/enable true.

입력  /avoid/enable (Bool)      true = 지금 자리·방향을 기준으로 시작, false = 정지
      /fork_direction (String)  A | B (FSM 투표, m7_link가 넘김). 안 오면 detector fork_a/fork_b 투표로
      /scan, /odom, /imu, image_topic (Image 또는 .../compressed), /lane/status (있으면 노란선 보조)
      /perception/detections (String JSON, 원본 detector_node)
출력  /avoid/cmd_vel (Twist), /avoid/active (Bool, 실행 중 20 Hz), /avoid/done (Bool), /avoid/state (String JSON)
M2 녹화: share/limo_obstacle_avoid/paths/m2_{1|2|3}-{A|B}.csv (replay와 같은 파일, m2_dir로 바꿈)
"""
import json
import math
import os
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
from limo_obstacle_avoid.m7_replay_core import load_m2
from limo_obstacle_avoid.path_track import yaw_of


def _share_paths():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('limo_obstacle_avoid'), 'paths')
    except Exception:                             # 패키지 설치 전 (소스에서 직접 실행)
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'paths')


class M7DashNode(Node):
    def __init__(self):
        super().__init__('avoid_node')
        self.p = DashParams()
        for f in fields(DashParams):
            self.declare_parameter(f.name, getattr(self.p, f.name))
        for name, val in (('scan_topic', '/scan'), ('odom_topic', '/odom'), ('imu_topic', '/imu'),
                          ('image_topic', '/camera/color/image_raw'), ('lane_status_topic', '/lane/status'),
                          ('cmd_topic', '/avoid/cmd_vel'), ('bev_config', ''), ('white_v_min', 180), ('white_s_max', 60),
                          ('cam_hz', 10.0), ('odom_timeout', 0.3), ('scan_timeout', 0.5), ('range_min', 0.10),
                          ('range_max', 3.0), ('scan_yaw_offset_deg', 0.0), ('scan_flip', False),
                          ('dry_run', False), ('m2_dir', ''),
                          ('det_topic', '/perception/detections'), ('sign_classes', ['fork_a', 'fork_b']),
                          ('sign_conf', 0.5), ('fork_vote_window', 8), ('fork_vote_need', 5)):
            self.declare_parameter(name, val)
        self._read_params()
        self.core = M7Dash(self.p)
        m2d = self.gp('m2_dir') or _share_paths()
        m2, m2src = load_m2(m2d, True)
        self.core.set_m2(m2, m2src)
        self.get_logger().info(f'[M7 점선] M2 녹화 {len(m2)}/6 ({m2d}): '
                               + ', '.join(f'{k}: {v}' for k, v in sorted(m2src.items())))
        if self.p.m2_replay and not m2:
            self.get_logger().error('[M7 점선] M2 녹화 파일 없음 -> 섬 끝까지 점선 주행 후 끝 (갈래 진입은 FSM 차선 주행)')
        self.fork_topic_got = False
        self.fork_votes = []
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
        self.pub_state = self.create_publisher(String, '/avoid/state', 10)
        self.pub_done = self.create_publisher(Bool, '/avoid/done', 10)
        self.pub_active = self.create_publisher(Bool, '/avoid/active', 10)
        self.create_subscription(Bool, '/avoid/enable', self.on_enable, 10)
        self.create_subscription(String, '/fork_direction', self.on_fork, 10)
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
                               + (f'M2 녹화 재생 (섬 앞 {self.p.m2_start_gap} m)' if self.p.m2_replay else
                                  f'표지판 상자 {self.p.end_sign_h_px:.0f} px' if self.p.end_by_sign else f'섬 + {self.p.done_margin} m')
                               + f', 점선 {"켬" if self.cfg is not None else "끔"}. 시작: /avoid/enable true')

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
        # /fork_direction(FSM)이 안 오면(단독 시험) 표지 투표로 갈래
        if not self.fork_topic_got:
            seen = set()
            for d in dets:
                try:
                    if d.get('cls') in ('fork_a', 'fork_b') and float(d.get('conf', 0.0)) >= conf:
                        seen.add('A' if d.get('cls') == 'fork_a' else 'B')
                except (AttributeError, TypeError, ValueError):
                    continue
            self.fork_votes.append(seen.pop() if len(seen) == 1 else None)
            del self.fork_votes[:-int(self.gp('fork_vote_window'))]
            for f in ('A', 'B'):
                if self.fork_votes.count(f) >= int(self.gp('fork_vote_need')) and self.core.exit_fork != f:
                    self.core.set_exit(f)
                    self.get_logger().info(f'[M7 점선] 표지 투표 -> 갈래 {f}')

    def on_fork(self, m):
        self.fork_topic_got = True
        old = self.core.exit_fork
        self.core.set_exit(m.data)
        if self.core.exit_fork != old:
            self.get_logger().info(f'[M7 점선] 갈림길 방향 {m.data} -> 갈래 {self.core.exit_fork}')

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
            self.core.start()
            self.n_ev = 0
            self.active = True
            self.get_logger().info(f'▶ [M7 점선] 시작 (섬까지 {self.p.section_length:.2f} m, 켤 때 방향 {self.p.start_heading_deg:+.1f}°, '
                                   f'옆 {100 * self.p.start_y:+.0f} cm{", 점선 위" if self.p.start_on_dash else ""}'
                                   f'{", M2만" if self.p.m2_test_start else ""}, 갈래 {self.core.exit_fork or "?"})')
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
            if not self.dry:
                self.pub_active.publish(Bool(data=True))
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
        if not self.dry:
            self.pub_active.publish(Bool(data=True))
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
        if not self.dry:
            self.pub_active.publish(Bool(data=False))
            if done:
                self.pub_done.publish(Bool(data=True))
        self.core.stop()
        self.core.exit_fork = None                # 다음 판은 새로 받음
        self.fork_topic_got = False
        self.fork_votes = []
        self.get_logger().info(f'■ [M7 점선] {why}: {st["end"]} (끝 차로 {st["now_lane"]}, 보정 {st["fix"]}, 점선 본 프레임 {self.n_dash}, '
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
