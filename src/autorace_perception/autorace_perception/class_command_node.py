"""YOLO + 현재 구간 → 행동 요청. 실제 제어·주차 실행은 팀원 노드의 책임."""

import json
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String

from autorace_perception.class_commands import MODE_CLASSES, ClassCommandRouter, V3_CLASSES


class ClassCommandNode(Node):
    def __init__(self):
        super().__init__('class_command_node')
        readonly = ParameterDescriptor(read_only=True)
        for name, value in (
                ('detections_topic', '/perception/detections'),
                ('commands_topic', '/perception/commands'),
                ('mission_topic', '/mission/state'),
                ('context_timeout_sec', 1.0)):
            self.declare_parameter(name, value, readonly)
        for name, value in (('conf_threshold', 0.7), ('yellow_action', 'none'),
                            ('vote_window', 10), ('vote_need', 7)):
            self.declare_parameter(name, value)
        for cls in V3_CLASSES:
            self.declare_parameter(f'min_bbox_height_px.{cls}', 0.0)
        for mode in MODE_CLASSES:                      # 미션별 투표 기준. 0 = vote_need
            self.declare_parameter(f'mode_vote_need.{mode}', 0)
        # 11/12 밤: 이 미션들은 최근 창이 다 차기 전에도 need번 보이면 바로 인정 (갈림길 표지를 늦게 정하던 것)
        self.declare_parameter('early_confirm_modes', ['M2_FORK', 'M7_OBSTACLE'], readonly)
        self.router = ClassCommandRouter()
        self.router.early_modes = set(self.get_parameter('early_confirm_modes').value)
        self.router.configure(
            self.get_parameter('conf_threshold').value,
            self.get_parameter('yellow_action').value,
            self.get_parameter('vote_window').value,
            self.get_parameter('vote_need').value,
            {cls: self.get_parameter(f'min_bbox_height_px.{cls}').value for cls in V3_CLASSES},
            {mode: self.get_parameter(f'mode_vote_need.{mode}').value for mode in MODE_CLASSES})
        self.context_timeout = self.get_parameter('context_timeout_sec').value
        if (isinstance(self.context_timeout, bool)
                or not isinstance(self.context_timeout, (int, float))
                or not math.isfinite(self.context_timeout) or self.context_timeout <= 0):
            raise ValueError('context_timeout_sec must be finite and positive')
        self._mode_at = self._ground_at = None
        self._slow_ground = False
        self._route_fixes = None
        self.dropped_frames = 0
        self.pub = self.create_publisher(String, self.get_parameter('commands_topic').value, 10)
        self.mode_sub = self.create_subscription(String, self.get_parameter('mission_topic').value,
                                                  self._on_mission, 10)
        self.sub = self.create_subscription(String, self.get_parameter('detections_topic').value,
                                             self._on_detections, 10)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.get_logger().info('class commands ready; waiting for /mission/state (no vehicle control)')

    def _on_mission(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('state'), str):
                raise ValueError('expected mission object with string state')
            if data['state'] != self.router.mode:
                self._ground_at = None
                self._slow_ground = False
                self._route_fixes = None
            self.router.set_mode(data['state'])
            now = time.monotonic()
            self._mode_at = now
            # 기존 RouteTracker가 카메라의 흰 선을 m5_slow_line과 새로 매칭한 경우만 인정.
            # last_fix는 과거 값을 계속 보내므로 fixes 변화 없이는 유효 시간을 늘리지 않는다.
            route = data.get('route')
            if isinstance(route, dict):
                fixes, fix = route.get('fixes'), route.get('last_fix')
                if type(fixes) is int and fixes >= 0:
                    if self._route_fixes is not None and fixes > self._route_fixes:
                        self._slow_ground = (isinstance(fix, dict)
                                             and fix.get('name') == 'm5_slow_line')
                        self._ground_at = now
                    elif self._route_fixes is not None and fixes < self._route_fixes:
                        self._ground_at = None
                    self._route_fixes = fixes
                else:
                    self._ground_at = self._route_fixes = None
            else:
                self._ground_at = self._route_fixes = None
        except (ValueError, TypeError) as exc:
            self.router.set_mode('')
            self._mode_at = self._ground_at = self._route_fixes = None
            self.get_logger().warning(f'invalid mission state: {exc}', throttle_duration_sec=5.0)

    def _on_parameters(self, parameters):
        values = dict(conf_threshold=self.router.conf_threshold,
                      yellow_action=self.router.yellow_action,
                      vote_window=self.router.vote_window, vote_need=self.router.vote_need)
        heights = dict(self.router.min_heights)
        per_mode = dict(self.router.mode_need)
        for parameter in parameters:
            if parameter.name in values:
                values[parameter.name] = parameter.value
            elif parameter.name.startswith('min_bbox_height_px.'):
                heights[parameter.name.split('.', 1)[1]] = parameter.value
            elif parameter.name.startswith('mode_vote_need.'):
                per_mode[parameter.name.split('.', 1)[1]] = parameter.value
        try:
            self.router.configure(**values, min_heights=heights, mode_need=per_mode)
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _on_detections(self, msg):
        now = time.monotonic()
        if self._mode_at is None or now - self._mode_at > self.context_timeout:
            self.router.set_mode('')
            self._ground_at = None
            self._route_fixes = None
        ground = (self._slow_ground and self._ground_at is not None
                  and now - self._ground_at <= self.context_timeout)
        previous_stamp = self.router.last_stamp
        try:
            frame = json.loads(msg.data)
            output = self.router.route(frame, slow_ground=ground)
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'invalid detections: {exc}', throttle_duration_sec=5.0)
            return
        if output is None:
            self.dropped_frames += 1
            reason = ('clock moved backwards: votes reset, existing traffic stop retained'
                      if frame['stamp'] < previous_stamp else 'duplicate frame ignored')
            self.get_logger().warning(
                f'{reason}; stamp={frame["stamp"]}, previous_stamp={previous_stamp}, '
                f'dropped={self.dropped_frames}', throttle_duration_sec=5.0)
            return
        self.pub.publish(String(data=json.dumps(output, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = ClassCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
