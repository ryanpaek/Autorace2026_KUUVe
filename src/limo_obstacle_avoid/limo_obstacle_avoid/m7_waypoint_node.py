#!/usr/bin/env python3
"""M7 정차 LIMO 기준 웨이포인트 노드 (11/19 실험, launch m7_impl:=waypoint).

m7_replay_node와 같은 이름(avoid_node)·같은 토픽으로 돈다. 판단은 m7_waypoint_core.M7Waypoint.
M7만 한다: 3차선 끝(section_length - end_before)에서 지금 차로를 target으로 /avoid/done → m7_link → FSM M2.

입력  /avoid/enable (Bool)      obstacle_node(m7_link)가 3차선 시작선에서 true (켜기 직전 section_length·start_heading_deg·start_y 넣음)
      /fork_direction (String)  A | B → 끝 차로 목표 (A = +1 왼쪽, B = -1 오른쪽)
      /scan, /odom, /imu, /lane/status (yellow_l_mm / yellow_r_mm / yellow_f_mm)
출력  /avoid/cmd_vel (Twist)    launch에서 /obstacle/cmd_vel로 remap
      /avoid/active, /avoid/done (Bool), /avoid/state (String JSON: impl 'waypoint', plan, mem, why ...)

실시간 보기: ros2 topic echo /avoid/state
후진 켜기:   ros2 param set /avoid_node reverse_max 3
"""
import json
import math
import time
from dataclasses import fields

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String

from limo_obstacle_avoid.m7_waypoint_core import M7Waypoint, WaypointParams
from limo_obstacle_avoid.path_track import yaw_of


class M7WaypointNode(Node):
    def __init__(self):
        super().__init__('avoid_node')
        self.p = WaypointParams()
        for f in fields(WaypointParams):
            v = getattr(self.p, f.name)
            if isinstance(v, list):
                self.declare_parameter(f.name, [float(x) for x in v] or [0.0])
            else:
                self.declare_parameter(f.name, v)
        for name, val in (('scan_topic', '/scan'), ('odom_topic', '/odom'), ('imu_topic', '/imu'),
                          ('lane_status_topic', '/lane/status'), ('odom_timeout', 0.3), ('scan_timeout', 0.5),
                          ('range_min', 0.10), ('range_max', 3.0), ('scan_yaw_offset_deg', 0.0), ('scan_flip', False),
                          ('dry_run', False)):
            self.declare_parameter(name, val)
        self._read_params()
        self.core = M7Waypoint(self.p)
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
        self.get_logger().info(f'[M7 웨이포인트] 준비 (후진 {"끔" if self.p.reverse_max <= 0 else self.p.reverse_max}, '
                               f'굴려 보기 {self.p.roll_dist if self.p.rollout else 0:.1f} m, 옆 간격 {self.p.offset:.2f} m, '
                               f'정차 LIMO {self.p.obs_along:.2f} x {self.p.obs_across:.2f} m)')

    def gp(self, name):
        return self.get_parameter(name).value

    def _read_params(self):
        for f in fields(WaypointParams):
            v = self.gp(f.name)
            cur = getattr(self.p, f.name)
            if isinstance(cur, list):
                v = [float(x) for x in (v or [])]
                if len(v) == 1 and v[0] == 0.0:
                    v = []
            elif isinstance(cur, bool):
                v = bool(v)
            elif isinstance(cur, float) and isinstance(v, int):
                v = float(v)
            setattr(self.p, f.name, v)

    @staticmethod
    def stamp(m, fallback):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        return t if t > 0 else fallback

    # ---------- 입력 ----------
    def on_imu(self, m):
        self.core.on_imu(self.stamp(m, time.monotonic()), m.angular_velocity.z)

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
            if not (rmin <= r <= rmax) or math.isnan(r):
                continue
            a = m.angle_min + i * m.angle_increment
            a = (-a if flip else a) + off
            pts.append((lx + r * math.cos(a), r * math.sin(a), a))
        pts.sort(key=lambda q: q[2])
        a0 = min(m.angle_min, m.angle_max) + off
        a1 = max(m.angle_min, m.angle_max) + off
        if pts:   # 실제로 값이 나오는 각도 범위 (240° 라이다는 뒤쪽이 inf)
            a0, a1 = max(a0, pts[0][2]), min(a1, pts[-1][2])
        self.core.on_scan(time.monotonic(), pts, a0, a1)

    def on_lane(self, m):
        if not self.active:
            return
        try:
            self.core.on_lane(json.loads(m.data), time.monotonic())
        except (ValueError, TypeError, AttributeError):
            pass

    def on_fork(self, m):
        self.core.set_exit(m.data)
        self.get_logger().info(f'[M7 웨이포인트] 갈림길 방향 {m.data} -> 끝 차로 목표 {self.core.exit_lane()}')

    def on_enable(self, m):
        if m.data and not self.active:
            if self.t_odom is None or self.t_scan is None:
                self.get_logger().warn('[M7 웨이포인트] /odom 또는 /scan 없음 -> 시작 못 함')
                return
            self._read_params()           # obstacle_node가 방금 넣은 section_length 등
            self.core.p = self.p
            self.core.start()
            self.active = True
            self.get_logger().info(f'▶ [M7 웨이포인트] 시작 (섬까지 {self.p.section_length:.2f} m, '
                                   f'켤 때 방향 {self.p.start_heading_deg:+.1f}°, 옆 {100 * self.p.start_y:+.0f} cm)')
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
            self.get_logger().warn('[M7 웨이포인트] 센서 끊김 -> 정지', throttle_duration_sec=1.0)
            return
        v, kap = self.core.step(now)
        for e in self.core.events[self.n_ev:]:
            self.get_logger().info(f'[M7 웨이포인트] {e}')
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
        for e in self.core.events[self.n_ev:]:
            self.get_logger().info(f'[M7 웨이포인트] {e}')
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
        self.get_logger().info(f'■ [M7 웨이포인트] {why} (끝 차로 {self.core.target})')


def main(args=None):
    rclpy.init(args=args)
    n = M7WaypointNode()
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
