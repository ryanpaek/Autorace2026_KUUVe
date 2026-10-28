"""차 없이 차선 알고리즘을 돌려 보는 도구.

입력: 이미지 폴더, 동영상 파일, 또는 rosbag2 폴더 (ROS2가 깔린 곳에서만)
출력: 프레임별 디버그 이미지(카메라 | BEV)와 결과 CSV

예)
  ros2 run autorace_lane offline_replay --bev src/autorace_bringup/config/bev_limo.yaml \
      --input ~/bags/run1 --topic /camera/color/image_raw --out ~/replay/run1
  python3 -m autorace_lane.offline_replay --bev bev.yaml --input frames/ --out out/ \
      --set lane_color=yellow lookahead_mm=600
파라미터를 바꿔 가며 같은 입력을 다시 돌려 보면 튜닝 전후를 비교할 수 있다 (D-Racer의 A/B 리플레이).
"""

import argparse
import csv
import glob
import os
import sys

import cv2
import numpy as np

from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams, draw_debug


def iter_folder(path):
    files = sorted(f for ext in ('png', 'jpg', 'jpeg')
                   for f in glob.glob(os.path.join(path, f'*.{ext}')))
    for f in files:
        img = cv2.imread(f)
        if img is not None:
            yield os.path.basename(f), img


def iter_video(path):
    cap = cv2.VideoCapture(path)
    i = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        yield f'frame_{i:05d}', img
        i += 1


def iter_bag(path, topic):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from autorace_lane.lane_node import image_msg_to_bgr
    except ImportError as e:
        sys.exit(f'rosbag 읽기에는 ROS2 환경이 필요합니다 ({e})')
    storage = 'mcap' if glob.glob(os.path.join(path, '*.mcap')) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f'topic {topic} not in bag: {sorted(types)}')
    msg_type = get_message(types[topic])
    i = 0
    while reader.has_next():
        name, data, t = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        if types[topic].endswith('CompressedImage'):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            img = image_msg_to_bgr(msg)
        yield f'{t}', img
        i += 1


def parse_sets(items):
    out = {}
    for s in items or []:
        k, v = s.split('=', 1)
        for cast in (int, float):
            try:
                v = cast(v)
                break
            except ValueError:
                pass
        if v in ('true', 'false'):
            v = v == 'true'
        out[k] = v
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bev', required=True)
    ap.add_argument('--input', required=True)
    ap.add_argument('--topic', default='/camera/color/image_raw')
    ap.add_argument('--out', required=True)
    ap.add_argument('--mode', default='normal')
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--set', nargs='*', help='LaneParams 덮어쓰기 key=value')
    ap.add_argument('--every', type=int, default=1, help='N프레임마다 이미지 저장')
    ap.add_argument('--model', default=None, help='차선 분할 ONNX (없으면 HSV)')
    a = ap.parse_args(argv)

    cfg = BevConfig.from_yaml(os.path.expanduser(a.bev))
    params = LaneParams.from_dict({**LaneParams().as_dict(), **parse_sets(a.set)})
    masker = None
    if a.model:
        from autorace_lane.seg_infer import SegMasker
        masker = SegMasker(os.path.expanduser(a.model))
    f = LaneFollower(cfg, params, mask_fn=masker)
    f.set_mode(a.mode)
    src = os.path.expanduser(a.input)
    if os.path.isdir(src) and (glob.glob(os.path.join(src, '*.db3'))
                               or glob.glob(os.path.join(src, '*.mcap'))):
        frames = iter_bag(src, a.topic)
    elif os.path.isdir(src):
        frames = iter_folder(src)
    else:
        frames = iter_video(src)

    out_dir = os.path.expanduser(a.out)
    os.makedirs(out_dir, exist_ok=True)
    n = 0
    with open(os.path.join(out_dir, 'result.csv'), 'w', newline='') as fcsv:
        w = csv.writer(fcsv)
        w.writerow(['frame', 'status', 'source', 'kappa', 'offset_mm', 'heading_rad',
                    'path_kappa', 'low_conf', 'stopline_mm'])
        for name, img in frames:
            out = f.process(img, speed=a.speed)
            w.writerow([name, out.status, out.source, f'{out.kappa:.4f}',
                        '' if out.offset_mm is None else f'{out.offset_mm:.1f}',
                        '' if out.heading_rad is None else f'{out.heading_rad:.4f}',
                        '' if out.path_kappa is None else f'{out.path_kappa:.3f}',
                        int(out.low_conf),
                        out.stoplines_mm[0] if out.stoplines_mm else ''])
            if n % max(1, a.every) == 0:
                cam = cv2.resize(img, (cfg.image_width, cfg.image_height))
                dbg = draw_debug(out, cfg)
                scale = cam.shape[0] / dbg.shape[0]
                dbg = cv2.resize(dbg, (int(dbg.shape[1] * scale), cam.shape[0]))
                cv2.imwrite(os.path.join(out_dir, f'{n:05d}.jpg'), np.hstack([cam, dbg]))
            n += 1
    print(f'{n} frames -> {out_dir}')


if __name__ == '__main__':
    main()
