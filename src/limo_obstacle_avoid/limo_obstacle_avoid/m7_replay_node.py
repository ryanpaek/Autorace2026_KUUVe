#!/usr/bin/env python3
"""M7 녹화 경로 재생 회피 노드 (11/17). 예전 avoid_node 자리에서 같은 토픽으로 돈다 (launch m7_impl:=replay, 기본).

판단·동작은 m7_replay_core.M7Replay (모듈 설명에 우선순위·상태). 이 노드는 ROS 입출력만.

입력  /avoid/enable (Bool)      obstacle_node(m7_link)가 3차선 시작선에서 true. 켜기 직전에 section_length,
                                start_heading_deg, start_y 를 파라미터로 넣어 준다 (예전 avoid_node와 같음)
      /fork_direction (String)  A | B (FSM 갈림길 투표)
      /scan, /odom, /imu, /lane/status (String JSON)
출력  /avoid/cmd_vel (Twist)    launch에서 /obstacle/cmd_vel로 remap -> mission_node가 M7 동안 고름
      /avoid/active (Bool)      실행 중 20 Hz true (m7_link가 끊기면 정지시킴)
      /avoid/done (Bool)        끝 (m7_link -> FSM M2)
      /avoid/state (String)     JSON: state RUN, now_lane, target(끝 차로 -> m7_exit), gaps_mm, phase, why ...

녹화 경로: share/limo_obstacle_avoid/paths/m7_left.csv, m7_right.csv (tools/path_recorder.py로 녹화)
          한쪽만 있으면 거울상, 둘 다 없으면 기본 S자. 시작할 때 로그로 어떤 걸 쓰는지 알려 줌.
실시간 보기: ros2 topic echo /avoid/state
"""
import json
import math
import os
import time
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, load_change
from limo_obstacle_avoid.path_track import yaw_of


def _share_paths():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('limo_obstacle_avoid'), 'paths')
    except Exception:
        return ''


class M7ReplayNode(Node):
    def __init__(self):
        super().__init__('avoid_node')
        self.p = ReplayParams()
        for f in fields(ReplayParams):
            v = getattr(self.p, f.name)
            self.declare_parameter(f.name, v)
        d = _share_paths()
        self.declare_parameter('path_left', os.path.join(d, 'm7_left.csv') if d else '')
        self.declare_parameter('path_right', os.path.join(d, 'm7_right.csv') if d else '')
        for name, val in (('scan_topic', '/scan'), ('odom_topic', '/odom'), ('imu_topic', '/imu'),
                          ('lane_status_topic', '/lane/status'), ('odom_timeout', 0.3), ('scan_timeout', 0.5),
                          ('range_min', 0.10), ('range_max', 3.0), ('scan_yaw_offset_deg', 0.0), ('scan_flip', False),
                          ('dry_run', False)):
            self.declare_parameter(name, val)
        self._read_params()
        pl = self.gp('path_left') or (os.path.join(d, 'm7_left.csv') if d else '')
        pr = self.gp('path_right') or (os.path.join(d, 'm7_right.csv') if d else '')
        self.left, src_l = load_change(pl, pr, +1, self.p.lane_width)
        self.right, src_r = load_change(pr, pl, -1, self.p.lane_width)
        self.core = M7Replay(self.p, self.left, self.right)
        self.get_logger().info(f'[M7 재생] 왼쪽 경로: {src_l} / 오른쪽: {src_r} '
                               f'(옆 배율 {self.core.k_left:.2f}/{self.core.k_right:.2f}), 트리거 {self.p.trigger_mm:.0f} mm')
        # 11/17: 경로가 피하려는 장애물을 스스로 비켜 가는지 켤 때 점검 (안 되면 실차에서 늘 '옆 차로 막힘'으로 섬)
        chk = self.core.check_paths()
        for d, name in ((+1, '왼쪽'), (-1, '오른쪽')):
            if not chk[d]:
                self.get_logger().warn(
                    f'[M7 재생] ★ {name} 경로가 트리거 {self.p.trigger_mm:.0f} mm 앞 장애물에 닿음 -> 이 방향은 늘 막힘으로 판단됨. '
                    f'녹화 파일(paths/m7_left.csv, m7_right.csv)이 있는지, 녹화 시작 거리와 trigger_mm이 같은지 확인')
        self.dry = bool(self.gp('dry_run'))
        self.pub_cmd = self.create_publisher(Twist, '/avoid/cmd_vel_preview' if self.dry else '/avoid/cmd_vel', 10)
        self.pub_active = self.create_publisher(Bool, '/avoid/active', 10)
        self.pub_done = self.create_publisher(Bool, '/avoid/done', 10)
        self.pub_state = self.create_publisher(String, '/avoid/state', 10)
        self.create_subscription(Bool, '/avoid/enable', self.on_enable, 10)
        self.create_subscription(String, '/fork_direction', self.on_fork, 10)
        self.create_subscription(LaserScan, self.gp('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.gp('odom_topic'), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, self.gp('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, self.gp('lane_status_topic'), self.on_lane, 10)
        self.t_odom = self.t_scan = None
        self.active = False
        self.k = 0
        self.n_ev = 0
        self.create_timer(0.05, self.loop)

    def gp(self, name):
        return self.get_parameter(name).value

    def _read_params(self):
        for f in fields(ReplayParams):
            v = self.gp(f.name)
            cur = getattr(self.p, f.name)
            if isinstance(cur, float) and isinstance(v, int):
                v = float(v)
            setattr(self.p, f.name, v)

    @staticmethod
    def stamp(m, fallback):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return t if t > 0 else fallback

    # ---------- 입력 ----------
    def on_imu(self, m):
        la = getattr(m, 'linear_acceleration', None)
        self.core.on_imu(self.stamp(m, time.monotonic()), m.angular_velocity.z, la.x if la is not None else None)

    def on_odom(self, m):
        self.t_odom = time.monotonic()
        p = m.pose.pose.position
        self.core.on_odom(self.stamp(m, self.t_odom), p.x, p.y, yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x)

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
            if not (rmin <= r <= rmax):
                continue
            a = m.angle_min + i * m.angle_increment
            a = (-a if flip else a) + off
            pts.append((lx + r * math.cos(a), r * math.sin(a)))
        self.core.on_scan_points(self.t_scan, pts)

    def on_lane(self, m):
        if not self.active:
            return
        try:
            self.core.on_lane(json.loads(m.data), time.monotonic())
        except (ValueError, TypeError, AttributeError):
            pass

    def on_fork(self, m):
        self.core.set_exit(m.data)
        self.get_logger().info(f'[M7 재생] 갈림길 방향 {m.data} -> 출구 {self.core._exit_name()}')

    def on_enable(self, m):
        if m.data and not self.active:
            if self.t_odom is None or self.t_scan is None:
                self.get_logger().warn('[M7 재생] /odom 또는 /scan 없음 -> 시작 못 함')
                return
            self._read_params()           # obstacle_node가 방금 넣은 section_length 등
            self.core.p = self.p
            self.core.start()
            self.active = True
            self.get_logger().info(f'▶ [M7 재생] 시작 (섬까지 {self.p.section_length:.2f} m, '
                                   f'켤 때 방향 {self.p.start_heading_deg:+.1f}°, 출구 {self.core._exit_name()})')
        elif not m.data and self.active:
            self.finish('수동 중단', done=False)

    # ---------- 주기 ----------
    def loop(self):
        if not self.active:
            return
        now = time.monotonic()
        stale = (self.t_odom is None or now - self.t_odom > self.gp('odom_timeout')
                 or self.t_scan is None or now - self.t_scan > self.gp('scan_timeout'))
        if stale:
            self.pub_cmd.publish(Twist())
            if not self.dry:
                self.pub_active.publish(Bool(data=True))
            self.get_logger().warn('[M7 재생] 센서 끊김 -> 정지', throttle_duration_sec=1.0)
            return
        v, kap = self.core.step(now)
        for e in self.core.events[self.n_ev:]:
            self.get_logger().info(f'[M7 재생] {e}')
        self.n_ev = len(self.core.events)
        if self.core.state == 'DONE':
            self.finish('구간 통과', done=True)
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
            st.update(v=round(v, 3), kappa=round(kap, 2))
            self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))

    def finish(self, why, done):
        self.active = False
        self.core.events.append(f'끝 ({why})')
        for e in self.core.events[self.n_ev:]:
            self.get_logger().info(f'[M7 재생] {e}')
        self.n_ev = len(self.core.events)
        self.pub_cmd.publish(Twist())
        st = self.core.status()
        st['state'] = 'DONE' if done else 'IDLE'
        self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))
        if not self.dry:
            self.pub_active.publish(Bool(data=False))
            if done:
                self.pub_done.publish(Bool(data=True))
        self.core.stop()
        self.get_logger().info(f'■ [M7 재생] {why} (끝 차로 {self.core.lane})')


def main(args=None):
    rclpy.init(args=args)
    n = M7ReplayNode()
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
