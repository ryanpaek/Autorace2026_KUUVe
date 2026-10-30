#!/usr/bin/env python3
"""주행 녹화(bag)에서 차선 노드 판단과 실제 차 움직임을 시간순 CSV로 뽑는다. (LIMO에서 실행)

  python3 ~/autorace_ws/src/autorace_bringup/scripts/bag_lane_log.py ~/bags/<폴더> [--every 3]

열: t(초), status, source, kappa(명령, 왼쪽+), path_kappa(차선 곡률, 왼쪽+), offset_mm,
    steer_deg(/limo/ack_cmd 조향각, 왼쪽+), yaw_rate_cmd(v*kappa), yaw_rate_imu(실제), speed_odom
명령 곡률과 실제 회전(yaw_rate_imu)의 시간차를 보면 조향 지연을 알 수 있다.
결과는 <bag폴더>/lane_log.csv 로도 저장된다.
"""
import argparse
import bisect
import csv
import glob
import json
import math
import os
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--every', type=int, default=3, help='화면에는 N줄마다 출력 (CSV는 전부)')
    a = ap.parse_args()
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        sys.exit(f'ROS2 환경을 source 하세요 ({e})')
    path = os.path.expanduser(a.bag)
    storage = 'mcap' if glob.glob(os.path.join(path, '*.mcap')) else 'sqlite3'
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = ('/lane/status', '/limo/ack_cmd', '/imu', '/odom')
    msgs = {k: get_message(types[k]) for k in want if k in types}
    lane, steer, imu, odom = [], [], [], []
    t0 = None
    while r.has_next():
        name, data, t = r.read_next()
        if name not in msgs:
            continue
        t = t * 1e-9
        t0 = t if t0 is None else t0
        m = deserialize_message(data, msgs[name])
        if name == '/lane/status':
            lane.append((t, json.loads(m.data)))
        elif name == '/limo/ack_cmd':
            steer.append((t, math.degrees(m.steering_angle)))
        elif name == '/imu':
            imu.append((t, m.angular_velocity.z))
        elif name == '/odom':
            odom.append((t, m.twist.twist.linear.x))

    def near(seq, t):
        if not seq:
            return None
        i = bisect.bisect_left([s[0] for s in seq], t) if len(seq) < 50 else None
        if i is None:
            ts = near.cache.setdefault(id(seq), [s[0] for s in seq])
            i = bisect.bisect_left(ts, t)
        i = min(max(i, 0), len(seq) - 1)
        return seq[i][1]
    near.cache = {}

    out_path = os.path.join(path, 'lane_log.csv')
    cols = ['t', 'status', 'source', 'model', 'kappa', 'path_kappa', 'offset_mm', 'steer_deg',
            'yaw_rate_cmd', 'yaw_rate_imu', 'speed_odom']
    print(' '.join(cols))
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i, (t, st) in enumerate(lane):
            v = near(odom, t) or 0.0
            row = [round(t - t0, 2), st.get('status'), st.get('source'), st.get('model') or '',
                   round(st.get('kappa', 0.0), 2),
                   '' if st.get('path_kappa') is None else round(st['path_kappa'], 2),
                   '' if st.get('offset_mm') is None else round(st['offset_mm']),
                   '' if near(steer, t) is None else round(near(steer, t), 1),
                   round(st.get('speed_cmd', 0.0) * st.get('kappa', 0.0), 2),
                   '' if near(imu, t) is None else round(near(imu, t), 2),
                   round(v, 2)]
            w.writerow(row)
            if i % max(1, a.every) == 0:
                print(' '.join(str(c) for c in row))
    print(f'-> {out_path}')


if __name__ == '__main__':
    main()
