#!/usr/bin/env python3
"""녹화(bag)의 카메라 영상을 JPG 폴더로 꺼낸다. (LIMO에서 실행 - ROS2 필요)

ROS가 없는 노트북(Windows)에서도 offline_replay는 이미지 폴더를 입력으로 받으므로,
주말에 차 없이 튜닝하려면 이 폴더를 가져가면 된다.

  python3 ~/autorace_ws/src/autorace_bringup/scripts/bag_to_images.py ~/bags/<폴더> [--every 2]
  -> ~/bags/<폴더>_frames/000000.jpg ... (번호 = offline_replay의 프레임 번호와 같음, every=1일 때)

노트북에서:
  python -m autorace_lane.offline_replay --bev bev_limo.yaml --input <폴더>_frames --out out --set lane_color=yellow
"""
import argparse
import glob
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/camera/color/image_raw/compressed')
    ap.add_argument('--every', type=int, default=1, help='N장마다 저장 (학습용이면 3~5 권장)')
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    try:
        import cv2
        import numpy as np
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        sys.exit(f'ROS2 환경을 source 하세요 ({e})')
    path = os.path.expanduser(a.bag).rstrip('/')
    out = os.path.expanduser(a.out) if a.out else path + '_frames'
    os.makedirs(out, exist_ok=True)
    storage = 'mcap' if glob.glob(os.path.join(path, '*.mcap')) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if a.topic not in types:
        sys.exit(f'{a.topic} 없음: {sorted(types)}')
    mt = get_message(types[a.topic])
    i = saved = 0
    while r.has_next():
        name, data, _ = r.read_next()
        if name != a.topic:
            continue
        if i % max(1, a.every) == 0:
            m = deserialize_message(data, mt)
            if types[a.topic].endswith('CompressedImage'):
                img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
            else:
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
                if m.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(os.path.join(out, f'{i:06d}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
            saved += 1
        i += 1
    print(f'{saved}장 -> {out}')


if __name__ == '__main__':
    main()
