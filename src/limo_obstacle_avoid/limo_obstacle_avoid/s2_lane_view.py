#!/usr/bin/env python3
"""
[2단계] 차선별 장애물 거리 + "지금이면 어디로 피할까" 보기  — 로봇은 가만히

로봇을 가운데 차선 중앙에 똑바로 두고, 상자(또는 LIMO)를 각 차선에 놓아보세요.
출력 예)
  왼(+1) 1.25m | 가운데(0) 0.62m | 오(-1) 없음   → 판단: +1(왼쪽)으로 회피
RViz: Fixed Frame = base_link, MarkerArray /avoid/markers
"""
import copy
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base, INF
from limo_obstacle_avoid.ros_common import load_params, make_marker, pt, init_rclpy, shutdown_rclpy, declare

NAMES = {1: '왼(+1)', 0: '가운데(0)', -1: '오(-1)'}


class LaneView(Node):
    def __init__(self):
        super().__init__('s2_lane_view')
        self.p = load_params(self, AvoidParams())
        declare(self, 'scan_topic', '/scan')
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value,
                                 self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, '/avoid/markers', 1)
        self.k = 0

    def cb(self, msg):
        p = copy.copy(self.p)
        p.confirm_hits = 1                                  # 한 번만 보여도 인정(제자리 테스트)
        p.use_island = False                                # 제자리 테스트라 섬(가상 장애물)은 끔
        p.lane_align = False                                # 보이는 그대로 보기 (정렬 보정 끔)
        pl = AvoidPlanner(p)
        pl.start(0.0, 0.0, 0.0)
        pl.update_scan(scan_to_base(msg.ranges, msg.angle_min, msg.angle_increment, p))
        obs = pl.obstacles(True)
        cur = p.start_lane
        txt = []
        for lane in sorted(pl.lanes, reverse=True):
            f = pl.free_ahead(lane, obs)
            txt.append(f"{NAMES.get(lane, lane)} {'없음' if f == INF else f'{f:.2f}m'}")
        f_cur = pl.free_ahead(cur, obs)
        if f_cur >= p.trigger_dist:
            dec = f'직진 (내 차선 여유 {f_cur:.2f} ≥ trigger {p.trigger_dist})'
        else:
            nxt = pl.choose(cur, obs)
            dec = '갈 곳 없음 → 정지/대기' if nxt is None else f'{NAMES.get(nxt, nxt)} 으로 회피'
        if pl.emergency:
            dec += '  ⚠비상정지 박스 안에 물체'
        self.k += 1
        if self.k % 5 == 0:                                 # 2Hz 출력
            self.get_logger().info(' | '.join(txt) + '   → ' + dec)
        self.publish(pl, msg.header.stamp)

    def publish(self, pl, stamp):
        p = pl.p
        y0 = p.start_lane * p.lane_width                   # 로컬 → base_link 로 (y0 빼기)
        ma = MarkerArray()
        lines = make_marker('base_link', 'lanes', 0, Marker.LINE_LIST, stamp, (0.8, 0.6, 0.1, 1.0))
        for k in range(-p.num_side_lanes, p.num_side_lanes + 2):
            yy = (k - 0.5) * p.lane_width - y0
            lines.points += [pt(-0.5, yy), pt(p.roi_front, yy)]
        ma.markers.append(lines)
        clear = Marker(); clear.action = Marker.DELETEALL; clear.ns = 'obs'
        ma.markers.insert(0, clear)
        for i, o in enumerate(pl.obstacles(True)):
            m = make_marker('base_link', 'obs', i, Marker.CUBE, stamp,
                            (0.3, 0.3, 0.3, 0.5) if o.virtual else (1.0, 0.3, 0.1, 0.8))
            xe = o.x_end(p.obs_min_len)
            m.pose.position = pt((o.xmin + xe) / 2, (o.ymin + o.ymax) / 2 - y0, 0.1)
            m.scale.x, m.scale.y, m.scale.z = max(0.02, xe - o.xmin), max(0.02, o.ymax - o.ymin), 0.2
            ma.markers.append(m)
        self.pub.publish(ma)


def main():
    init_rclpy()
    n = LaneView()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        shutdown_rclpy(n)


if __name__ == '__main__':
    main()
