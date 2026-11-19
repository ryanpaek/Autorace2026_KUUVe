"""출발 전에 카메라 노출을 한 번 정해서 고정하는 노드 (11/19 '노란 실선이 흰색으로 날아감' 대응).

  ros2 run autorace_bringup exposure_node
  (sensors.launch.py / full.launch.py 가 기본으로 띄움. 단계: cam_exp_level:=0~4, 0 = 끔)

순서 (카메라가 켜지고 약 4초, 출발 전에 끝남):
  1. 자동 노출이 settle_s 초 동안 자리 잡게 둠
  2. 노출·게인·화이트밸런스와 바닥 밝기를 읽음
  3. 자동 노출·자동 화이트밸런스 끄고, 노출 = 읽은 값 * (목표 바닥 V / 지금 바닥 V)^2.2 로 고정
     (계산·단계 표는 exposure_core.py)
  4. 영상 구독을 끊고 대기 (주행 중 CPU 거의 0)
실패하면 자동 노출로 되돌림 (지금과 같음).

/camera/exposure_state (String JSON, transient_local -> 나중에 녹화를 시작해도 bag에 들어감)
"""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

from autorace_bringup.exposure_core import ExposureParams, fixed_exposure, floor_v

try:
    from orbbec_camera_msgs.srv import GetInt32, SetInt32
except ImportError:          # 노트북 등 orbbec 드라이버가 없는 곳
    GetInt32 = SetInt32 = None


def image_to_bgr(msg: Image):
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    h, w, step = msg.height, msg.width, msg.step
    if enc in ('bgr8', 'rgb8'):
        return buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)   # 바닥 V(=채널 최대)만 보므로 순서 무관
    if enc in ('yuv422', 'yuyv', 'yuv422_yuy2'):
        img = buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    return None


class ExposureNode(Node):
    def __init__(self):
        super().__init__('exposure_node')
        d = ExposureParams()
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_ns', '/camera')
        self.declare_parameter('settle_s', 3.0)
        self.declare_parameter('lock_wb', True)
        for k in ('level', 'fixed_exposure', 'min_exposure'):
            self.declare_parameter(k, int(getattr(d, k)))
        for k in ('target_floor_v', 'gamma', 'max_ratio', 'roi_top'):
            self.declare_parameter(k, float(getattr(d, k)))
        g = lambda k: self.get_parameter(k).value
        self.p = ExposureParams(**{k: g(k) for k in (
            'level', 'fixed_exposure', 'min_exposure', 'target_floor_v', 'gamma', 'max_ratio', 'roi_top')})
        self.settle_s = float(g('settle_s'))
        self.lock_wb = bool(g('lock_wb'))
        self.info = {'state': 'start', 'level': self.p.level, 'target_floor_v': self.p.target()}
        self.vs = []
        self.t_first = None
        self.sub = None
        self.cli = {}

        qos_l = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(String, '/camera/exposure_state', qos_l)

        if self.p.fixed_exposure <= 0 and self.p.target() is None:
            self._done('off', '단계 0: 카메라 자동 노출 그대로')
            return
        if GetInt32 is None:
            self._done('no_driver_msgs', 'orbbec_camera_msgs 없음 -> 자동 노출 그대로')
            return
        ns = g('camera_ns').rstrip('/')
        self.cli = {
            'get_exp': self.create_client(GetInt32, ns + '/get_color_exposure'),
            'set_exp': self.create_client(SetInt32, ns + '/set_color_exposure'),
            'get_gain': self.create_client(GetInt32, ns + '/get_color_gain'),
            'set_gain': self.create_client(SetInt32, ns + '/set_color_gain'),
            'get_wb': self.create_client(GetInt32, ns + '/get_white_balance'),
            'set_wb': self.create_client(SetInt32, ns + '/set_white_balance'),
            'ae': self.create_client(SetBool, ns + '/set_color_auto_exposure'),
            'awb': self.create_client(SetBool, ns + '/set_auto_white_balance'),
        }
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Image, g('image_topic'), self._on_image, qos)
        self.timer = self.create_timer(0.2, self._tick)
        self.started = False
        self.get_logger().info(f'exposure_node: 단계 {self.p.level} (바닥 V 목표 {self.p.target()}), '
                               f'{self.settle_s:.0f}초 뒤 노출 고정')

    def _on_image(self, msg):
        if self.t_first is None:
            self.t_first = time.monotonic()
        if time.monotonic() - self.t_first >= self.settle_s - 1.0:   # 마지막 1초 바닥 밝기만 모음
            bgr = image_to_bgr(msg)
            if bgr is not None:
                self.vs.append(floor_v(bgr, self.p))

    def _tick(self):
        if self.started or self.t_first is None or time.monotonic() - self.t_first < self.settle_s or not self.vs:
            return
        self.started = True
        self.timer.cancel()
        self.v0 = float(np.median(self.vs))
        self.destroy_subscription(self.sub)          # 이제 영상 안 봄
        self.sub = None
        self._call('get_exp', None, self._got_exp)

    # ---- 서비스 (순서대로) ----
    def _call(self, name, data, done):
        c = self.cli[name]
        if not c.service_is_ready():
            self._fail(f'서비스 없음: {c.srv_name} (카메라 노드가 켜져 있나?)')
            return
        req = c.srv_type.Request()
        if data is not None:
            req.data = data
        fut = c.call_async(req)

        def cb(f):
            r = f.result()
            if r is None or (hasattr(r, 'success') and not r.success):
                self._fail(f'{c.srv_name} 실패: {getattr(r, "message", "")}')
                return
            done(r)
        fut.add_done_callback(cb)

    def _got_exp(self, r):
        self.e0 = int(r.data)
        self._call('get_gain', None, self._got_gain)

    def _got_gain(self, r):
        self.gain = int(r.data)
        self._call('get_wb', None, self._got_wb)

    def _got_wb(self, r):
        self.wb = int(r.data)
        self.e = fixed_exposure(self.p, self.e0, self.v0)
        self.info.update(auto_exposure=self.e0, gain=self.gain, wb=self.wb, floor_v_at_start=round(self.v0, 1),
                         exposure=self.e)
        if self.e is None:
            self._fail(f'노출 계산 불가 (자동 노출 값 {self.e0}, 바닥 V {self.v0:.0f})')
            return
        self._call('ae', False, lambda r: self._call('set_exp', self.e, lambda r: self._call(
            'set_gain', self.gain, lambda r: self._lock_wb())))

    def _lock_wb(self):
        if not self.lock_wb:
            self._fixed()
            return
        self._call('awb', False, lambda r: self._call('set_wb', self.wb, lambda r: self._fixed()))

    def _fixed(self):
        self._done('fixed', f'노출 고정: 자동 {self.e0} -> {self.e} (출발선 바닥 V {self.v0:.0f}, '
                            f'목표 {self.p.target() or "-"}), 게인 {self.gain}, WB {self.wb}')

    def _fail(self, why):
        for n in ('ae', 'awb'):
            c = self.cli.get(n)
            if c is not None and c.service_is_ready():
                req = SetBool.Request()
                req.data = True
                c.call_async(req)
        self._done('failed', f'{why} -> 자동 노출로 되돌림', error=True)

    def _done(self, state, text, error=False):
        self.info['state'] = state
        (self.get_logger().error if error else self.get_logger().info)(f'exposure_node: {text}')
        m = String()
        m.data = json.dumps(self.info, ensure_ascii=False)
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ExposureNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
