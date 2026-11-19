"""카메라 노출을 바닥 밝기로 맞추는 노드 (11/19 '노란 실선이 흰색으로 날아감' 대응).

  ros2 run autorace_bringup exposure_node
  (sensors.launch.py / full.launch.py 가 기본으로 띄움. 끄려면 cam_ae:=false)

순서:
  1. 카메라 자동 노출이 settle_s 초 동안 자리 잡게 둠
  2. 그때 노출·게인·화이트밸런스를 읽고, 자동 노출·자동 화이트밸런스를 끔 (게인·WB는 그 값으로 고정)
  3. period_s 마다 화면 아래쪽(바닥) 밝기를 보고 target_floor_v 가 되게 /camera/set_color_exposure
서비스가 없거나 실패하면 아무것도 안 바꾸고 카메라 자동 노출 그대로 둠 (주행에는 영향 없음).

/camera/exposure_state (String JSON, 1 Hz): {"state", "exp", "gain", "wb", "floor_v", "target"}
-> bag에 넣으면 다음부터 노출 값을 로그로 볼 수 있음 (11/19 bag에는 노출 값이 없었음).
"""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool

from autorace_bringup.exposure_core import ExposureController, ExposureParams, floor_v

try:
    from orbbec_camera_msgs.srv import GetInt32, SetInt32
except ImportError:          # 노트북 등 orbbec 드라이버가 없는 곳
    GetInt32 = SetInt32 = None


def image_to_bgr(msg: Image):
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, np.uint8)
    h, w, step = msg.height, msg.width, msg.step
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return img[..., ::-1] if enc == 'rgb8' else img       # V(=max)만 보므로 채널 순서는 상관없지만 맞춰 둠
    if enc in ('yuv422', 'yuyv', 'yuv422_yuy2'):
        img = buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)
    return None


class ExposureNode(Node):
    def __init__(self):
        super().__init__('exposure_node')
        d = ExposureParams()
        self.declare_parameter('enable', True)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_ns', '/camera')
        self.declare_parameter('settle_s', 3.0)
        self.declare_parameter('period_s', 0.5)
        self.declare_parameter('lock_wb', True)
        for k in ('target_floor_v', 'deadband_v', 'gamma', 'kp', 'max_up', 'max_down', 'roi_top', 'roi_side'):
            self.declare_parameter(k, float(getattr(d, k)))
        self.declare_parameter('min_exposure', d.min_exposure)
        self.declare_parameter('max_exposure', d.max_exposure)
        g = lambda k: self.get_parameter(k).value
        self.p = ExposureParams(**{k: g(k) for k in (
            'target_floor_v', 'deadband_v', 'gamma', 'kp', 'max_up', 'max_down', 'roi_top', 'roi_side',
            'min_exposure', 'max_exposure')})

        self.state = 'off' if not g('enable') else 'wait_image'
        self.fv = None
        self.exp = self.gain = self.wb = None
        self.ctrl = None
        self.busy = False
        self.t_first = None
        self.settle_s = float(g('settle_s'))
        self.lock_wb = bool(g('lock_wb'))

        self.pub = self.create_publisher(String, '/camera/exposure_state', 10)
        self.create_timer(1.0, self._publish)
        if self.state == 'off':
            self.get_logger().info('exposure_node: enable=false, 카메라 자동 노출 그대로')
            return
        if GetInt32 is None:
            self.state = 'no_driver_msgs'
            self.get_logger().warn('orbbec_camera_msgs 없음 -> 카메라 자동 노출 그대로')
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
        self.create_subscription(Image, g('image_topic'), self._on_image, qos)
        self.create_timer(float(g('period_s')), self._tick)
        self.get_logger().info(f'exposure_node: 바닥 V 목표 {self.p.target_floor_v:.0f}, 자동 노출 {self.settle_s:.0f}초 뒤 잠금')

    # ---------- 입력 ----------
    def _on_image(self, msg):
        bgr = image_to_bgr(msg)
        if bgr is None:
            return
        self.fv = floor_v(bgr, self.p)
        if self.t_first is None:
            self.t_first = time.monotonic()

    # ---------- 서비스 (한 번에 하나씩) ----------
    def _call(self, name, data, done):
        c = self.cli[name]
        if not c.service_is_ready():
            self._fail(f'서비스 없음: {c.srv_name}')
            return
        req = c.srv_type.Request()
        if data is not None:
            req.data = data
        self.busy = True
        fut = c.call_async(req)

        def cb(f):
            self.busy = False
            r = f.result()
            if r is None or (hasattr(r, 'success') and not r.success):
                self._fail(f'{c.srv_name} 실패: {getattr(r, "message", "")}')
                return
            done(r)
        fut.add_done_callback(cb)

    def _fail(self, why):
        self.busy = False
        if self.state != 'failed':
            self.get_logger().error(f'exposure_node: {why} -> 자동 노출로 되돌리고 손 뗌')
            self.state = 'failed'
            for n in ('ae', 'awb'):                 # 잠그다 실패했으면 원래대로
                if self.cli[n].service_is_ready():
                    req = SetBool.Request()
                    req.data = True
                    self.cli[n].call_async(req)

    # ---------- 순서 ----------
    def _tick(self):
        if self.busy or self.state in ('off', 'failed', 'no_driver_msgs'):
            return
        if self.state == 'wait_image':
            if self.t_first is not None and time.monotonic() - self.t_first >= self.settle_s:
                self.state = 'read'
                self._call('get_exp', None, self._got_exp)
            return
        if self.state == 'control' and self.fv is not None:
            new = self.ctrl.next_exposure(self.fv, self.exp)
            if new is not None:
                self._call('set_exp', new, lambda r, n=new: self._set_exp_done(n))

    def _got_exp(self, r):
        self.exp = int(r.data)
        if self.exp <= 0:
            self._fail(f'노출 값 이상: {self.exp}')
            return
        self._call('get_gain', None, self._got_gain)

    def _got_gain(self, r):
        self.gain = int(r.data)
        self._call('get_wb', None, self._got_wb)

    def _got_wb(self, r):
        self.wb = int(r.data)
        self.get_logger().info(f'자동 노출이 고른 값: 노출 {self.exp} 게인 {self.gain} WB {self.wb} (바닥 V {self.fv:.0f})')
        self._call('ae', False, lambda r: self._call('set_exp', self.exp, lambda r: self._call(
            'set_gain', self.gain, lambda r: self._lock_wb())))

    def _lock_wb(self):
        if not self.lock_wb:
            self._start_control()
            return
        self._call('awb', False, lambda r: self._call('set_wb', self.wb, lambda r: self._start_control()))

    def _start_control(self):
        self.ctrl = ExposureController(self.p, self.exp)
        self.state = 'control'
        self.get_logger().info(f'잠금 완료. 노출 {self.p.min_exposure}~{self.ctrl.max_exp} 범위에서 바닥 V {self.p.target_floor_v:.0f}로 맞춤')

    def _set_exp_done(self, new):
        self.get_logger().info(f'노출 {self.exp} -> {new} (바닥 V {self.fv:.0f}, 목표 {self.p.target_floor_v:.0f})',
                               throttle_duration_sec=1.0)
        self.exp = new

    def _publish(self):
        m = String()
        m.data = json.dumps({'state': self.state, 'exp': self.exp, 'gain': self.gain, 'wb': self.wb,
                             'floor_v': None if self.fv is None else round(self.fv, 1),
                             'target': self.p.target_floor_v})
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
