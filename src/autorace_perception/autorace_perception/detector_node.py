"""표지판·신호등 인식 노드.

model_path 확장자로 백엔드를 고른다.
  .onnx -> onnxruntime (D-Racer와 같은 방식, dracer-yolo 레포의 export 사용)
  .pt   -> ultralytics (pip install ultralytics)
  ''    -> 모델 없이 빈 결과만 낸다 (나머지 스택을 먼저 돌려 볼 때)

발행 /perception/detections (String JSON)
  {"stamp": 123.4, "infer_ms": 18.2,
   "detections": [{"cls": "red", "conf": 0.91, "bbox": [x, y, w, h]}]}
cls 이름은 미션 FSM이 쓰는 이름과 같아야 한다 (class_names 파라미터 순서 = 학습 라벨 순서).
  red, yellow, green, pedestrian, fork_a, fork_b, slow, bump, parking
최신 프레임만 infer_hz로 처리한다 (밀린 프레임은 버림).
"""

import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from autorace_perception.yolo_onnx import decode, letterbox, to_tensor

DEFAULT_CLASSES = ['red', 'yellow', 'green', 'pedestrian', 'fork_a', 'fork_b',
                   'slow', 'bump', 'parking']


def image_msg_to_bgr(msg):
    from autorace_lane.lane_node import image_msg_to_bgr as conv
    return conv(msg)


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        p = self.declare_parameter
        p('image_topic', '/camera/color/image_raw')
        p('model_path', '')
        p('class_names', DEFAULT_CLASSES)
        p('input_size', 320)
        p('conf_threshold', 0.5)
        p('nms_threshold', 0.45)
        p('infer_hz', 10.0)
        p('publish_debug', True)
        # onnxruntime 스레드 수. 0이면 기본값(코어 전부). 11/09 통주행: 기본값이면 LIMO(6코어)에서
        # 장당 536ms + 차선 노드 19Hz -> 9Hz로 떨어짐. 2로 제한해 나머지 코어를 차선·센서에 남긴다
        p('ort_threads', 2)
        self.names = [str(c) for c in self.get_parameter('class_names').value]
        self.size = int(self.get_parameter('input_size').value)
        self.latest = None
        self.backend = None
        self._load_model(self.get_parameter('model_path').value)

        topic = self.get_parameter('image_topic').value
        if topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, topic, self._on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/perception/detections', 10)
        self.debug_pub = self.create_publisher(CompressedImage,
                                               '/perception/debug/compressed', 2)
        self.create_timer(1.0 / float(self.get_parameter('infer_hz').value), self._on_timer)

    def _load_model(self, path):
        if not path:
            self.get_logger().warning('model_path가 비어 있음: 빈 인식 결과만 발행합니다')
            return
        if path.endswith('.onnx'):
            import onnxruntime as ort
            so = ort.SessionOptions()
            n = int(self.get_parameter('ort_threads').value)
            if n > 0:
                so.intra_op_num_threads = n
                so.inter_op_num_threads = 1
                so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            self.session = ort.InferenceSession(path, sess_options=so,
                                                providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.backend = 'onnx'
        else:
            from ultralytics import YOLO
            self.model = YOLO(path)
            self.backend = 'ultralytics'
        self.get_logger().info(f'model loaded ({self.backend}): {path}'
                               f' (ort_threads={self.get_parameter("ort_threads").value})')

    def _on_image(self, msg):
        try:
            self.latest = (image_msg_to_bgr(msg), msg.header)
        except ValueError as e:
            self.get_logger().warning(str(e), throttle_duration_sec=5.0)

    def _on_compressed(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            self.latest = (img, msg.header)

    def _on_timer(self):
        if self.latest is None:
            return
        frame, header = self.latest
        self.latest = None
        t0 = time.perf_counter()
        dets = []
        conf = float(self.get_parameter('conf_threshold').value)
        try:
            if self.backend == 'onnx':
                canvas, scale, px, py = letterbox(frame, self.size)
                out = self.session.run(None, {self.input_name: to_tensor(canvas)})[0]
                dets = decode(out, self.names, frame.shape[:2], scale, px, py, self.size,
                              conf, float(self.get_parameter('nms_threshold').value))
            elif self.backend == 'ultralytics':
                res = self.model.predict(frame, imgsz=self.size, conf=conf, verbose=False)[0]
                for b in res.boxes:
                    x1, y1, x2, y2 = [float(v) for v in b.xyxy[0].tolist()]
                    dets.append({'cls': res.names[int(b.cls)], 'conf': round(float(b.conf), 4),
                                 'bbox': [int(x1), int(y1), int(x2 - x1), int(y2 - y1)]})
        except Exception as e:      # 한 프레임 실패로 노드가 죽지 않게
            self.get_logger().error(f'inference failed: {e}', throttle_duration_sec=5.0)
            return
        dt = (time.perf_counter() - t0) * 1000.0
        stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
        self.pub.publish(String(data=json.dumps(
            {'stamp': stamp, 'infer_ms': round(dt, 1), 'detections': dets})))
        if self.get_parameter('publish_debug').value and dets:
            dbg = frame.copy()
            for d in dets:
                x, y, w, h = d['bbox']
                cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(dbg, f"{d['cls']} {d['conf']:.2f}", (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            ok, jpg = cv2.imencode('.jpg', dbg, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                m = CompressedImage()
                m.header, m.format, m.data = header, 'jpeg', jpg.tobytes()
                self.debug_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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
