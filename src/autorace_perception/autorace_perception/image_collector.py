"""학습 데이터·캘리브레이션 사진 수집기.

예)
  # 5프레임마다 저장, 최대 2000장
  ros2 run autorace_perception image_collector --ros-args -p save_dir:=~/autorace_images/day1
  # 캘리브레이션용 한 장
  ros2 run autorace_perception image_collector --ros-args -p every_n:=1 -p max_images:=1

조명을 바꿔 가며, 광고 영상 모니터를 신호등 옆에 켜 두고, 표지판 거리·각도를 다양하게 찍을 것.
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('save_dir', '~/autorace_images')
        self.declare_parameter('every_n', 5)
        self.declare_parameter('max_images', 2000)
        self.declare_parameter('ext', 'jpg')
        self.dir = os.path.expanduser(self.get_parameter('save_dir').value)
        os.makedirs(self.dir, exist_ok=True)
        self.n_seen = 0
        self.n_saved = 0
        self.prefix = time.strftime('%m%d_%H%M%S')
        topic = self.get_parameter('image_topic').value
        if topic.endswith('/compressed'):
            self.create_subscription(CompressedImage, topic, self._on_compressed,
                                     qos_profile_sensor_data)
        else:
            self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f'saving every {self.get_parameter("every_n").value} frames '
                               f'to {self.dir}')

    def _on_image(self, msg):
        from autorace_lane.lane_node import image_msg_to_bgr
        self._save(image_msg_to_bgr(msg))

    def _on_compressed(self, msg):
        self._save(cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR))

    def _save(self, img):
        if img is None:
            return
        self.n_seen += 1
        if self.n_seen % max(1, self.get_parameter('every_n').value):
            return
        ext = self.get_parameter('ext').value
        path = os.path.join(self.dir, f'{self.prefix}_{self.n_saved:05d}.{ext}')
        cv2.imwrite(path, img)
        self.n_saved += 1
        if self.n_saved >= self.get_parameter('max_images').value:
            self.get_logger().info(f'saved {self.n_saved} images, done')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ImageCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
