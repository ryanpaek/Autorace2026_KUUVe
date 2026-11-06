"""차선 추종 ROS2 노드 (초안).

구독
  image_topic (sensor_msgs/Image 또는 .../compressed면 CompressedImage)
  imu_topic   (sensor_msgs/Imu) -- 헤딩 유지에 yaw 사용
  odom_topic  (nav_msgs/Odometry) -- 차선 기억(memory_mm > 0)용 이동 거리
  /route/preview (std_msgs/String, JSON) -- 미션이 보내는 도면 앞쪽 곡률 (route_ff_* > 0일 때)
  /lane/config (std_msgs/String, JSON) -- 미션이 구간별 모드·속도·차로 폭을 보냄
발행
  /lane/cmd_vel (geometry_msgs/Twist) -- linear.x = v, angular.z = v * kappa
  /lane/status  (std_msgs/String, JSON) -- status, offset_mm, stoplines_mm 등
  /lane/debug/compressed (sensor_msgs/CompressedImage) -- BEV 디버그 그림

/cmd_vel로 직접 내보내지 않는다. 최종 /cmd_vel은 mission 노드(cmd_mux)만 낸다.

/lane/config 예:
  {"mode": "zigzag", "speed": 0.15, "lane_width_mm": 570}
  {"mode": "left_only"}                   # 갈림길 A
  {"mode": "heading", "yaw_ref": 1.57}    # 지정 yaw 유지
  키를 빼면 ROS 파라미터 값으로 돌아간다. LaneParams의 어떤 필드든 덮어쓸 수 있다.
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
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import String

from autorace_lane.lane_core import (MODES, BevConfig, LaneFollower, LaneParams,
                                     PoseIntegrator, draw_debug)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def image_msg_to_bgr(msg: Image):
    """cv_bridge 없이 sensor_msgs/Image를 BGR로 바꾼다 (Jetson numpy 충돌 회피)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    h, w, step = msg.height, msg.width, msg.step
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img.copy()
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(img, code)
    if enc in ('mono8', '8uc1'):
        img = buf.reshape(h, step)[:, :w]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ('yuv422', 'yuyv', 'yuv422_yuy2'):
        img = buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError(f'unsupported image encoding: {msg.encoding}')


class LaneNode(Node):
    def __init__(self):
        super().__init__('lane_node')
        # --- 노드 파라미터 ---
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('route_preview_timeout_sec', 0.5)
        self.declare_parameter('bev_config', '')
        self.declare_parameter('require_calibration', True)
        self.declare_parameter('base_speed', 0.30)      # m/s, 미션이 덮어씀
        self.declare_parameter('min_speed', 0.08)       # 구동 데드존 위 최소 속도
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('debug_every', 3)        # N프레임마다 디버그 이미지
        self.declare_parameter('image_timeout_sec', 0.5)
        self.declare_parameter('lane_mask_source', 'hsv')   # 'hsv' | 'model'
        self.declare_parameter('seg_model_path', '')        # lane_seg.onnx
        self.declare_parameter('seg_threshold', 0.5)
        # --- LaneParams 전부를 ROS 파라미터로 ---
        # dynamic_typing: `ros2 param set ... lookahead_mm 600`처럼 정수를 줘도 거부하지 않게
        defaults = LaneParams()
        for f in fields(LaneParams):
            self.declare_parameter(f.name, getattr(defaults, f.name),
                                   ParameterDescriptor(dynamic_typing=True))
        self.add_on_set_parameters_callback(self._on_params)

        path = self.get_parameter('bev_config').value
        self.cfg = self._load_bev(path)
        self.base_params = self._params_from_ros()
        self.overrides = {}
        self.speed_override = None
        self.follower = LaneFollower(self.cfg, self._effective_params(),
                                     mask_fn=self._load_masker())

        self.yaw = None
        self.pose = PoseIntegrator()
        self.route_preview = None            # (받은 시각, dict)
        self.frame_count = 0
        self.last_image_time = None
        self._last_warn = 0.0

        topic = self.get_parameter('image_topic').value
        if topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, topic, self._on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/lane/config', self._on_config, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, '/route/preview', self._on_preview, 10)

        self.cmd_pub = self.create_publisher(Twist, '/lane/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/lane/status', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/lane/debug/compressed', 2)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f'lane_node up: image={topic}, bev={path or "(none)"}, '
            f'calibrated={self.cfg.calibrated}, grid {self.cfg.bev_w}x{self.cfg.bev_h} '
            f'@ {self.cfg.mm_per_px_x:.2f}mm/px')

    # --- 설정 ---------------------------------------------------------------
    def _load_bev(self, path: str) -> BevConfig:
        if path:
            try:
                return BevConfig.from_yaml(path)
            except (OSError, KeyError, ValueError) as e:
                self.get_logger().error(f'BEV config load failed ({path}): {e}')
        self.get_logger().error('BEV 캘리브레이션 없음: bev_calib으로 만든 yaml을 '
                                'bev_config 파라미터로 주세요. 그 전까지 정지 명령만 냅니다.')
        return BevConfig(H=np.eye(3), calibrated=False)

    def _load_masker(self):
        if self.get_parameter('lane_mask_source').value != 'model':
            return None
        path = self.get_parameter('seg_model_path').value
        try:
            from autorace_lane.seg_infer import SegMasker
            m = SegMasker(path, float(self.get_parameter('seg_threshold').value))
            self.get_logger().info(f'lane mask: model {path} {m.size} on {m.providers}')
            return m
        except Exception as e:      # 모델이 없거나 깨지면 HSV로 대신 달린다
            self.get_logger().error(f'분할 모델 로드 실패 ({path}): {e} -> HSV로 대신합니다')
            return None

    def _params_from_ros(self) -> LaneParams:
        return LaneParams.from_dict({f.name: self.get_parameter(f.name).value
                                     for f in fields(LaneParams)})

    def _effective_params(self) -> LaneParams:
        d = self.base_params.as_dict()
        d.update(self.overrides)
        return LaneParams.from_dict(d)

    def _on_params(self, params):
        names = {f.name for f in fields(LaneParams)}
        changed = False
        for prm in params:
            if prm.name in names:
                setattr(self.base_params, prm.name, prm.value)
                changed = True
        if changed:
            self.follower.p = self._effective_params()
        return SetParametersResult(successful=True)

    def _on_config(self, msg: String):
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warning(f'/lane/config JSON error: {e}')
            return
        mode = cfg.get('mode', 'normal')
        if mode not in MODES:
            self.get_logger().warning(f'unknown lane mode {mode!r}, ignored')
            return
        names = {f.name for f in fields(LaneParams)}
        self.overrides = {k: v for k, v in cfg.items() if k in names}
        self.speed_override = cfg.get('speed')
        self.follower.p = self._effective_params()
        self.follower.set_mode(mode, yaw=self.yaw, yaw_ref=cfg.get('yaw_ref'),
                               fit_order=cfg.get('fit_order'))

    # --- 콜백 ---------------------------------------------------------------
    def _on_imu(self, msg: Imu):
        self.yaw = yaw_from_quaternion(msg.orientation)
        self.pose.on_imu_yaw(self.yaw)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose.on_odom(p.x, p.y, yaw_from_quaternion(msg.pose.pose.orientation),
                          msg.twist.twist.linear.x)

    def _on_preview(self, msg: String):
        try:
            self.route_preview = (time.monotonic(), json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def _on_compressed(self, msg: CompressedImage):
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self._process(bgr, msg.header)

    def _on_image(self, msg: Image):
        try:
            bgr = image_msg_to_bgr(msg)
        except ValueError as e:
            self._warn(str(e))
            return
        self._process(bgr, msg.header)

    def _process(self, bgr, header):
        self.last_image_time = time.monotonic()
        if not self.cfg.calibrated and self.get_parameter('require_calibration').value:
            self._publish_cmd(0.0, 0.0)
            self._warn('BEV 미캘리브레이션: 정지 명령만 발행 중')
            return
        speed = (self.speed_override if self.speed_override is not None
                 else self.get_parameter('base_speed').value)
        rp = self.route_preview
        fresh = (rp is not None and time.monotonic() - rp[0]
                 <= self.get_parameter('route_preview_timeout_sec').value)
        self.follower.route_k = rp[1].get('k') if fresh else None
        if fresh:
            self.follower.route_step_mm = float(rp[1].get('step_mm', 100.0))
        out = self.follower.process(bgr, yaw=self.yaw, speed=speed, pose=self.pose.pose,
                                    dist_mm=self.pose.dist_mm)
        v = 0.0
        if speed > 0.0 and out.speed_scale > 0.0:
            v = max(speed * out.speed_scale, self.get_parameter('min_speed').value)
        self._publish_cmd(v, v * out.kappa)

        st = out.status_dict()
        st['mode'] = self.follower.mode
        st['speed_cmd'] = v
        st['stamp'] = header.stamp.sec + header.stamp.nanosec * 1e-9
        self.status_pub.publish(String(data=json.dumps(st)))

        self.frame_count += 1
        if (self.get_parameter('publish_debug').value
                and self.frame_count % max(1, self.get_parameter('debug_every').value) == 0):
            img = draw_debug(out, self.cfg)
            ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                m = CompressedImage()
                m.header = header
                m.format = 'jpeg'
                m.data = jpg.tobytes()
                self.debug_pub.publish(m)

    def _publish_cmd(self, v: float, w: float):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def _watchdog(self):
        timeout = self.get_parameter('image_timeout_sec').value
        if self.last_image_time is None or time.monotonic() - self.last_image_time > timeout:
            self._publish_cmd(0.0, 0.0)
            self._warn('카메라 영상이 안 들어옴: 정지 명령 발행 중')

    def _warn(self, text: str):
        now = time.monotonic()
        if now - self._last_warn > 5.0:
            self.get_logger().warning(text)
            self._last_warn = now


def main(args=None):
    rclpy.init(args=args)
    node = LaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
