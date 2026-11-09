#!/usr/bin/env python3
"""
[5단계] 진짜 회피 노드 (미션 노드)

입력   /scan, /odom, /imu
       /avoid/enable  (std_msgs/Bool)    true = 미션 시작(이 순간 위치가 원점), false = 중단
       /fork_direction (std_msgs/String) 'A' / 'B'  — 갈림길 미션이 알려주면 그 쪽으로 빠져나감
출력   /avoid/cmd_vel  (Twist)  ← cmd_mux 가 /cmd_vel 로 넘겨줌
       /avoid/active   (Bool)   true 인 동안 cmd_mux 가 회피 명령을 우선 사용
       /avoid/done     (Bool)   true 한 번 = 끝, 차선추종에게 넘김
       /avoid/state    (String) 디버그용 한 줄 상태
       /avoid/markers  (MarkerArray) RViz

dry_run: true 이면 바퀴 명령을 /avoid/cmd_vel_preview 로만 내고 active 도 안 보냄
         → 손으로 밀거나 차선추종으로 달리면서 '판단'만 구경할 수 있음

안전장치: /odom 이나 /scan 이 끊기면(odom_timeout, scan_timeout) 그동안 정지 명령을 낸다.
          Ctrl+C 로 끄면 정지 명령과 active=false 를 보내고 끝난다.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base
from limo_obstacle_avoid.ros_common import (load_params, PoseInputs, make_marker, pt, make_twist,
                                            init_rclpy, shutdown_rclpy, declare)


class AvoidNode(Node):
    def __init__(self):
        super().__init__('avoid_node')
        self.p = load_params(self, AvoidParams())
        declare(self, 'scan_topic', '/scan')
        declare(self, 'dry_run', True)
        declare(self, 'enable_on_start', False)
        declare(self, 'marker_frame', 'odom')
        declare(self, 'rate', 20.0)
        declare(self, 'odom_timeout', 0.3)
        declare(self, 'scan_timeout', 0.5)
        self.t_scan = None
        self._stale = False

        self.inp = PoseInputs(self)
        self.inp.on_odom = self.on_odom
        self.pl = AvoidPlanner(self.p)
        self.active = False

        def _exit_param(params):           # ros2 param set /avoid_node exit_lane A 도 바로 반영
            from rcl_interfaces.msg import SetParametersResult
            for prm in params:
                if prm.name == 'exit_lane':
                    self.pl.set_exit(prm.value)
            return SetParametersResult(successful=True)
        self.add_on_set_parameters_callback(_exit_param)
        self.origin = None                    # (ox, oy, odom_yaw) — 마커용

        self.dry = self.get_parameter('dry_run').value
        cmd_topic = '/avoid/cmd_vel_preview' if self.dry else '/avoid/cmd_vel'
        self.pub_cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.pub_active = self.create_publisher(Bool, '/avoid/active', 10)
        self.pub_done = self.create_publisher(Bool, '/avoid/done', 10)
        self.pub_state = self.create_publisher(String, '/avoid/state', 10)
        self.pub_mk = self.create_publisher(MarkerArray, '/avoid/markers', 1)

        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/avoid/enable', self.on_enable, 10)
        self.create_subscription(String, '/fork_direction', self.on_fork, 10)
        # 카메라 차선 보정: 팀 lane_node 결과를 그대로 받아 옆 위치·방향을 맞춤 (lane_fix 파라미터로 끔)
        self.create_subscription(String, '/lane/status', self.on_lane, 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self.loop)
        self.create_timer(0.2, self.publish_markers)
        self._k = 0
        self._n_events = 0
        self.get_logger().info(f'avoid_node 준비 (dry_run={self.dry}, 명령 토픽={cmd_topic})')
        if self.get_parameter('enable_on_start').value:
            self._auto = self.create_timer(1.0, self._auto_enable)

    # ---------- 입력 ----------
    def _auto_enable(self):
        if self.inp.ready():
            self._auto.cancel()
            self.on_enable(Bool(data=True))

    def on_enable(self, m):
        if m.data:
            if self.active:
                return
            if not self.inp.ready():
                self.get_logger().warn('odom/imu 가 아직 없어서 시작 못 함 (드라이버, 토픽 이름 확인)')
                return
            if self.t_scan is None or time.monotonic() - self.t_scan > self.get_parameter('scan_timeout').value:
                self.get_logger().warn('/scan 이 안 들어와서 시작 못 함 (라이다 드라이버 확인)')
                return
            mode = self.inp.lock()
            ox, oy, yaw, _ = self.inp.get()
            self.pl.start(ox, oy, yaw, self.now())
            self.origin = (ox, oy, self.inp.odom[2])
            self.active = True
            self._n_events = 0
            self.get_logger().info(f'▶ 회피 미션 시작 (출구선호={self.pl.exit_pref}, 방향={mode})')
        else:
            if self.active:
                self.finish('수동 중단', done=False)

    def on_fork(self, m):
        self.p.exit_lane = m.data
        self.pl.set_exit(m.data)
        self.get_logger().info(f'갈림길 방향 받음: {m.data} → 출구 차선 {self.pl.exit_pref}')

    def on_lane(self, m):
        if not self.active:
            return
        try:
            self.pl.lane_fix(json.loads(m.data))
        except (ValueError, TypeError, AttributeError):
            pass

    def on_odom(self):
        if self.active:
            self.pl.pose.update(*self.inp.get())

    def on_scan(self, msg):
        self.t_scan = time.monotonic()
        if self.active:
            self.pl.update_scan(scan_to_base(msg.ranges, msg.angle_min, msg.angle_increment, self.p))

    def now(self):
        return time.monotonic()

    # ---------- 주기 ----------
    def loop(self):
        if not self.active:
            return
        # 센서가 끊기면 판단을 멈추고 정지 명령
        stale = (not self.inp.fresh(self.get_parameter('odom_timeout').value, 0.5)
                 or self.t_scan is None
                 or time.monotonic() - self.t_scan > self.get_parameter('scan_timeout').value)
        if stale:
            if not self._stale:
                self.get_logger().warn('센서 끊김(/odom·/imu·/scan) → 정지하고 기다림')
            self._stale = True
            self.pub_cmd.publish(Twist())
            if not self.dry:
                self.pub_active.publish(Bool(data=True))
            return
        if self._stale:
            self.get_logger().info('센서 다시 들어옴 → 계속')
            self._stale = False
        v, w = self.pl.step(self.now())

        for x, e in self.pl.events[self._n_events:]:
            self.get_logger().info(f'[x={x:.2f}m] {e}')
        self._n_events = len(self.pl.events)

        if self.pl.state == 'DONE':
            self.finish('구간 통과', done=True)
            return
        self.pub_cmd.publish(make_twist(self, v, w))
        if not self.dry:
            self.pub_active.publish(Bool(data=True))
        self._k += 1
        if self._k % 4 == 0:
            st = self.pl.status()
            st.update(v=round(v, 2), w=round(w, 2))
            self.pub_state.publish(String(data=json.dumps(st, ensure_ascii=False)))
            if self.pl.emergency or self.pl.blocked:
                self.get_logger().warn(f'정지 중: emergency={self.pl.emergency} blocked={self.pl.blocked}',
                                       throttle_duration_sec=1.0)

    def finish(self, why, done):
        self.active = False
        self.inp.unlock()
        self.pub_cmd.publish(Twist())
        self.pub_active.publish(Bool(data=False))
        if done:
            self.pub_done.publish(Bool(data=True))
        self.get_logger().info(f'■ 회피 종료: {why}  (최종 차선 {self.pl.target:+d}, '
                               f'x={self.pl.pose.x:.2f}, y={self.pl.pose.y:+.2f})')
        self.pl.state = 'IDLE'

    # ---------- RViz ----------
    def to_frame(self, X, Y):
        """미션 로컬 좌표 → odom 좌표"""
        ox, oy, th = self.origin
        Y = Y - self.p.start_lane * self.p.lane_width
        return ox + X * math.cos(th) - Y * math.sin(th), oy + X * math.sin(th) + Y * math.cos(th)

    def publish_markers(self):
        if self.origin is None:
            return
        p, fr = self.p, self.get_parameter('marker_frame').value
        st = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        L = p.section_length + p.island_length
        lines = make_marker(fr, 'lanes', 0, Marker.LINE_LIST, st, (0.8, 0.6, 0.1, 1.0))
        for k in range(-p.num_side_lanes, p.num_side_lanes + 2):
            yy = (k - 0.5) * p.lane_width
            lines.points += [pt(*self.to_frame(0, yy)), pt(*self.to_frame(L, yy))]
        ma.markers.append(lines)
        tgt = make_marker(fr, 'target', 0, Marker.LINE_STRIP, st, (0.1, 0.9, 0.2, 1.0), (0.04, 0, 0))
        yc = self.pl.center(self.pl.target)
        x0 = self.pl.pose.x
        tgt.points = [pt(*self.to_frame(x0, yc)), pt(*self.to_frame(x0 + 1.0, yc))]
        ma.markers.append(tgt)
        for i, o in enumerate(self.pl.obstacles(True)):
            col = (0.4, 0.4, 0.4, 0.5) if o.virtual else (1.0, 0.3, 0.1, 0.9)
            m = make_marker(fr, 'obs', i, Marker.LINE_STRIP, st, col, (0.03, 0, 0))
            xe = o.x_end(p.obs_min_len)
            for (a, b) in ((o.xmin, o.ymin), (xe, o.ymin), (xe, o.ymax), (o.xmin, o.ymax), (o.xmin, o.ymin)):
                m.points.append(pt(*self.to_frame(a, b), 0.05))
            ma.markers.append(m)
        self.pub_mk.publish(ma)


def main():
    init_rclpy()
    n = AvoidNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            try:
                n.pub_cmd.publish(Twist())
                n.pub_active.publish(Bool(data=False))
            except Exception:
                break
            time.sleep(0.02)
        shutdown_rclpy(n)


if __name__ == '__main__':
    main()
