#!/usr/bin/env python3
"""
[0단계] 가짜 트랙 (LIMO 없이 집에서)  — /scan /odom /imu /tf 를 흉내내고 /cmd_vel 로 움직임

  layout: single | zigzag6 | mixed7 | diag3   (sim_common.LAYOUTS 에서 수정)
  odom_scale: 1.04 → 오도메트리가 4% 더 길게 나오는 상황
  imu_drift_deg: IMU 방향이 서서히 틀어지는 상황
"""
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from limo_obstacle_avoid import sim_common as sc
from limo_obstacle_avoid.ros_common import make_marker, pt


def quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class FakeWorld(Node):
    def __init__(self):
        super().__init__('s0_fake_world')
        d = self.declare_parameter
        d('layout', 'mixed7'); d('lane_width', 0.55); d('start_lane', 0)
        d('lidar_x', 0.10); d('odom_scale', 1.0); d('imu_drift_deg', 0.0)
        d('scan_noise', 0.01); d('section_length', 4.3); d('island_length', 1.0)
        g = lambda n: self.get_parameter(n).value
        self.lw = g('lane_width')
        self.rects = sc.layout_rects(g('layout'), self.lw)
        self.segs = sc.rects_to_segments(self.rects)
        self.car = sc.SimCar(0.0, g('start_lane') * self.lw, 0.0)
        self.cmd = Twist()
        self.t = 0.0
        self.rnd = random.Random(0)

        self.create_subscription(Twist, '/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.p_odom = self.create_publisher(Odometry, '/odom', 10)
        self.p_imu = self.create_publisher(Imu, '/imu', 10)
        self.p_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.p_mk = self.create_publisher(MarkerArray, '/sim/world', 1)
        self.tfb = TransformBroadcaster(self)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = 'base_link', 'laser_link'
        st.transform.translation.x = g('lidar_x')
        st.transform.rotation.w = 1.0
        self.stb = StaticTransformBroadcaster(self)
        self.stb.sendTransform(st)

        self.create_timer(0.02, self.tick)      # 50Hz 차량/odom/imu
        self.create_timer(0.1, self.scan)       # 10Hz 라이다
        self.create_timer(1.0, self.world)
        self.get_logger().info(f"가짜 트랙 시작: layout={g('layout')}  장애물 {len(self.rects)}대")

    def tick(self):
        dt = 0.02
        self.t += dt
        drift = math.radians(self.get_parameter('imu_drift_deg').value) * min(1.0, self.t / 20.0)
        c = self.car
        c.step(self.cmd.linear.x, self.cmd.angular.z, dt,
               odom_scale=self.get_parameter('odom_scale').value, imu_drift=drift, rnd=self.rnd)
        now = self.get_clock().now().to_msg()
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = now, 'odom', 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = c.odom_x, c.odom_y
        qx, qy, qz, qw = quat(c.odom_yaw)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = self.cmd.linear.x
        o.twist.twist.angular.z = self.cmd.angular.z
        self.p_odom.publish(o)
        i = Imu()
        i.header.stamp, i.header.frame_id = now, 'imu_link'
        qx, qy, qz, qw = quat(c.imu_yaw)
        i.orientation.z, i.orientation.w = qz, qw
        i.angular_velocity.z = self.cmd.angular.z
        self.p_imu.publish(i)
        # RViz 는 '실제' 위치로 그려서 오차가 눈에 보이게
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = now, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = c.x, c.y
        qx, qy, qz, qw = quat(c.yaw)
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tfb.sendTransform(tf)

    def scan(self):
        c = self.car
        lx = c.x + self.get_parameter('lidar_x').value * math.cos(c.yaw)
        ly = c.y + self.get_parameter('lidar_x').value * math.sin(c.yaw)
        n = 360
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), 'laser_link'
        s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / n
        s.angle_max = s.angle_min + (n - 1) * s.angle_increment
        s.range_min, s.range_max = 0.1, 8.0
        s.ranges = [float(r) for r in sc.raycast(lx, ly, c.yaw, s.angle_min, s.angle_increment, n, self.segs,
                                                 noise=self.get_parameter('scan_noise').value, rnd=self.rnd)]
        self.p_scan.publish(s)

    def world(self):
        st = self.get_clock().now().to_msg()
        ma = MarkerArray()
        for k, r in enumerate(self.rects):
            m = make_marker('odom', 'limo', k, Marker.CUBE, st, (0.2, 0.2, 0.9, 0.8))
            m.pose.position = pt(r[0], r[1], 0.1)
            m.scale.x, m.scale.y, m.scale.z = r[2], r[3], 0.2
            ma.markers.append(m)
        L = self.get_parameter('section_length').value
        Li = L + self.get_parameter('island_length').value
        ln = make_marker('odom', 'track', 0, Marker.LINE_LIST, st, (1.0, 1.0, 1.0, 0.6))
        for k in (-1.5, -0.5, 0.5, 1.5):
            ln.points += [pt(-0.5, k * self.lw), pt(Li + 0.5, k * self.lw)]
        isl = make_marker('odom', 'track', 1, Marker.LINE_STRIP, st, (1.0, 0.7, 0.1, 1.0), (0.03, 0, 0))
        h = 0.3 * self.lw
        isl.points = [pt(L, 0), pt(L + 0.15, h), pt(Li - 0.15, h), pt(Li, 0), pt(Li - 0.15, -h),
                      pt(L + 0.15, -h), pt(L, 0)]
        ma.markers += [ln, isl]
        self.p_mk.publish(ma)


def main():
    rclpy.init()
    n = FakeWorld()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
