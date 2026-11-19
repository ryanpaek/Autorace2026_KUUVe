"""노란 선 옆 거리 노드 (11/19). lane_node와 따로 돌고, 따로 발행만 한다.

구독  image_topic (Image 또는 .../compressed)
발행  /lane/yellow_side (std_msgs/String JSON)
  {"l_mm", "r_mm", "offset_mm", "src", "y_mm", "raw_l", "raw_r", "n_l", "n_r", "n_bridge",
   "l_ok", "r_ok", "proc_ms", "t"}
  y_mm: 잰 띠의 앞 거리 (기본 약 900 mm). n_bridge: 옅은 노랑 다리로 잡은 줄 수
  l_mm/r_mm: 필터 거친 차 중심 -> 왼쪽/오른쪽 노란 선 거리 (못 보면 null)
  offset_mm: 차로 중심에서 밀린 양 (오른쪽 +, 양쪽 선 다 보일 때만). src: both | L | R | none
  offset_one_mm: 한쪽 선 + 반폭 257.5로 추정 (참고용, 오프라인 오차 큼)

아무 노드도 아직 이 값으로 조향하지 않는다 (보기·기록용).
켜기: ros2 launch autorace_bringup autorace.launch.py yellow_side:=true
"""

import json
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from autorace_lane.lane_core import BevConfig
from autorace_lane.lane_node import image_msg_to_bgr
from autorace_lane.yellow_side import YellowSide

HSV_KEYS = ('yellow_h_min', 'yellow_h_max', 'yellow_s_min', 'yellow_v_min')


def lane_hsv_from_file(path: str):
    """params.yaml의 lane_node 노란색 임계값을 그대로 읽는다 (두 노드 값이 어긋나지 않게)."""
    with open(path, 'r', encoding='utf-8') as f:
        d = yaml.safe_load(f) or {}
    p = (d.get('lane_node') or {}).get('ros__parameters') or {}
    return {k: int(p[k]) for k in HSV_KEYS if k in p}


class YellowSideNode(Node):
    def __init__(self):
        super().__init__('yellow_side_node')
        dp = self.declare_parameter
        dp('image_topic', '/camera/color/image_raw')
        dp('bev_config', '')
        dp('lane_params_file', '')        # 주면 lane_node의 노란색 임계값을 여기서 읽음
        dp('every_n', 3)                  # 3이면 카메라 30 fps 중 10 fps만 처리
        # 11/19: 550~750은 화각 밖 (550 mm 앞 보이는 옆 폭 -188~+220 mm). 800 mm부터 +-340 mm 보임
        dp('near_from_mm', 800.0)         # 띠: 제어 기준점에서 앞쪽 이 거리부터
        dp('near_to_mm', 1000.0)          #     이 거리까지
        dp('row_step', 2)
        dp('min_run_mm', 8.0)
        dp('max_run_mm', 120.0)
        dp('min_rows', 5)
        dp('half_width_mm', 257.5)        # 한쪽만 보일 때 반폭 (직선 515 mm / 2)
        dp('window_s', 0.5)
        dp('max_jump_mm', 50.0)
        dp('lo_mm', 120.0)
        dp('hi_mm', 450.0)
        dp('bridge_mm', 60.0)             # 옅은 노랑 다리 (직전 선 +-이 안만). 0 = 끔
        dp('pale_s_min', 20)              # lane_node white_bridge_pale_*와 같은 값
        dp('pale_v_min', 200)
        for k, v in zip(HSV_KEYS, (15, 40, 80, 80)):
            dp(k, v)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        hsv = {k: int(g(k)) for k in HSV_KEYS}
        lp = g('lane_params_file')
        if lp:
            try:
                hsv.update(lane_hsv_from_file(lp))
            except (OSError, yaml.YAMLError) as e:
                self.get_logger().warn(f'lane_params_file 읽기 실패 ({lp}): {e} -> 자체 값 사용')
        path = g('bev_config')
        if not path:
            raise RuntimeError('bev_config 필요 (launch가 bev_limo.yaml을 넣어 줌)')
        cfg = BevConfig.from_yaml(path)
        self.ys = YellowSide(cfg, hsv=tuple(hsv[k] for k in HSV_KEYS),
                             near_from_mm=float(g('near_from_mm')), near_to_mm=float(g('near_to_mm')),
                             row_step=int(g('row_step')), min_run_mm=float(g('min_run_mm')),
                             max_run_mm=float(g('max_run_mm')), min_rows=int(g('min_rows')),
                             half_width_mm=float(g('half_width_mm')), window_s=float(g('window_s')),
                             max_jump_mm=float(g('max_jump_mm')), lo_mm=float(g('lo_mm')),
                             hi_mm=float(g('hi_mm')), bridge_mm=float(g('bridge_mm')),
                             pale_s_min=int(g('pale_s_min')), pale_v_min=int(g('pale_v_min')))
        self.every_n = max(1, int(g('every_n')))
        self.count = 0
        self.pub = self.create_publisher(String, '/lane/yellow_side', 10)
        topic = g('image_topic')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        if topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, topic, self._on_compressed, qos)
        else:
            self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info(
            f'yellow_side up: image={topic}, band {g("near_from_mm"):.0f}~{g("near_to_mm"):.0f}mm '
            f'({len(self.ys.sampler.rows)} rows), every_n={self.every_n}, hsv={hsv}')

    def _skip(self) -> bool:
        self.count += 1
        return (self.count % self.every_n) != 0

    def _on_image(self, msg: Image):
        if self._skip():
            return
        bgr = image_msg_to_bgr(msg)
        if bgr is not None:
            self._process(bgr)

    def _on_compressed(self, msg: CompressedImage):
        if self._skip():
            return
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self._process(bgr)

    def _process(self, bgr):
        t0 = time.monotonic()
        d = self.ys.step(t0, bgr)
        d['proc_ms'] = round((time.monotonic() - t0) * 1000.0, 2)
        d['t'] = round(time.time(), 3)
        for k in ('l_mm', 'r_mm', 'offset_mm', 'offset_one_mm', 'raw_l', 'raw_r', 'sd_l', 'sd_r', 'y_mm'):
            if d.get(k) is not None:
                d[k] = round(d[k], 1)
        self.pub.publish(String(data=json.dumps(d)))


def main(args=None):
    rclpy.init(args=args)
    node = YellowSideNode()
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
