"""bag 회귀 시험: 같은 녹화를 여러 파라미터 묶음으로 한 번에 돌려 숫자로 비교한다.

차는 녹화 때 실제로 간 길을 그대로 가므로(개루프) "그 길에서 차선을 얼마나 잘 잡았나"를 본다.
  - 놓침: 선이 하나도 안 잡힌(NONE) 프레임, hold/stop으로 버틴 프레임 비율
  - 짧음: 중앙선이 lookahead까지 안 닿은(low_conf) 비율
  - 좌우 뒤집힘: 한쪽 선만 보이다가 바로 반대쪽 선으로 바뀐 횟수 (엉뚱한 선을 잡을 때 늘어남)
  - 목표점 튐: 목표점 좌우 위치가 한 프레임에 120mm 넘게 바뀐 횟수
  - 흔들림: 곡률 명령 프레임 간 변화 중앙값
  - 코너 선행(s): IMU로 찾은 코너마다, 이 설정의 곡률 명령이 실제 회전보다 얼마나 먼저 섰나 (+가 빠름)

예)
  python3 -m autorace_lane.lane_eval --bev src/autorace_bringup/config/bev_limo.yaml \\
      --bags ~/bags/1106_*base_03 --t0 3 --t1 180 \\
      --cfg base: --cfg mem:memory_mm=600 --cfg mem_slow:memory_mm=600,preview_slowdown=0.25
  출력: 표 + --out 폴더에 설정별 프레임 CSV
"""

import argparse
import csv
import glob
import math
import os
import sys

import cv2
import numpy as np

from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams, PoseIntegrator
from autorace_lane.lane_node_util import yaw_of
from autorace_lane.offline_replay import parse_sets


def _cast_cfg(text):
    name, _, rest = text.partition(':')
    items = [kv for kv in rest.split(',') if kv]
    return name or 'cfg', parse_sets(items)


def read_bag(path, topic_hint=None, t0=None, t1=None):
    """(rel_t, kind, data) 를 시간 순으로. kind = 'img' | 'odom' | 'imu' | 'cmd'"""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    try:
        ts.register(get_types_from_msg(
            'float32 steering_angle\nfloat32 steering_angle_velocity\nfloat32 speed\n'
            'float32 acceleration\nfloat32 jerk', 'ackermann_msgs/msg/AckermannDrive'))
    except Exception:
        pass
    from pathlib import Path
    with AnyReader([Path(path)], default_typestore=ts) as r:
        topics = {c.topic: c for c in r.connections}
        img_topic = topic_hint
        if img_topic is None:
            for cand in ('/camera/color/image_raw', '/camera/color/image_raw/compressed'):
                if cand in topics:
                    img_topic = cand
                    break
        if img_topic not in topics:
            sys.exit(f'{path}: 영상 토픽 없음 ({sorted(topics)})')
        want = [c for c in r.connections if c.topic in (img_topic, '/odom', '/imu', '/limo/ack_cmd')]
        start = r.start_time
        for c, t, raw in r.messages(connections=want):
            rel = (t - start) * 1e-9
            if t0 is not None and rel < t0:
                continue
            if t1 is not None and rel > t1:
                break
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/odom':
                yield rel, 'odom', m
            elif c.topic == '/imu':
                yield rel, 'imu', m
            elif c.topic == '/limo/ack_cmd':
                yield rel, 'cmd', m
            else:
                if c.msgtype.endswith('CompressedImage'):
                    img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
                else:
                    img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
                    if m.encoding == 'rgb8':
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                yield rel, 'img', img


def find_corners(t, yaw_rate, v, k_min=0.8, min_len=1.0):
    """실제 곡률 |gyro/v| > k_min 이 min_len초 넘게 이어진 구간의 시작 시각들."""
    k = np.where(v > 0.08, np.abs(yaw_rate) / np.maximum(v, 0.08), 0.0)
    k = np.convolve(k, np.ones(5) / 5, mode='same')
    on = k > k_min
    out, i = [], 0
    while i < len(on):
        if on[i]:
            j = i
            while j < len(on) and on[j]:
                j += 1
            if t[j - 1] - t[i] >= min_len:
                out.append((t[i], float(np.sign(np.median(yaw_rate[i:j])))))
            i = j
        else:
            i += 1
    return out


def evaluate(bag, cfg, sets, speed, t0, t1, model=None, out_dir=None):
    pose = PoseIntegrator()
    masker = None
    if model:
        from autorace_lane.seg_infer import SegMasker
        masker = SegMasker(os.path.expanduser(model))
    fol = {n: LaneFollower(cfg, LaneParams.from_dict({**LaneParams().as_dict(), **d}),
                           mask_fn=masker) for n, d in sets}
    rows = {n: [] for n, _ in sets}
    imu_t, imu_w, odom_t, odom_v = [], [], [], []
    yaw = None
    for rel, kind, m in read_bag(bag, t0=t0, t1=t1):
        if kind == 'imu':
            yaw = yaw_of(m.orientation)
            pose.on_imu_yaw(yaw)
            imu_t.append(rel)
            imu_w.append(m.angular_velocity.z)
        elif kind == 'odom':
            pose.on_odom(m.pose.pose.position.x, m.pose.pose.position.y,
                         yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x)
            odom_t.append(rel)
            odom_v.append(m.twist.twist.linear.x)
        elif kind == 'img':
            for n, f in fol.items():
                o = f.process(m, yaw=yaw, speed=speed, pose=pose.pose, dist_mm=pose.dist_mm)
                rows[n].append((rel, o.status, o.source, o.kappa, int(o.low_conf),
                                o.target_mm[0] if o.target_mm else float('nan'), o.mem_pts))
    imu_t, imu_w = np.array(imu_t), np.array(imu_w)
    v_at = np.interp(imu_t, odom_t, odom_v) if odom_t else np.zeros_like(imu_t)
    corners = find_corners(imu_t, imu_w, v_at) if len(imu_t) else []
    res = {}
    for n, rs in rows.items():
        if not rs:
            continue
        t = np.array([r[0] for r in rs])
        st = [r[1] for r in rs]
        src = [r[2] for r in rs]
        k = np.array([r[3] for r in rs])
        low = np.array([r[4] for r in rs])
        tx = np.array([r[5] for r in rs])
        N = len(rs)
        flips = sum(1 for a, b in zip(st, st[1:]) if {a, b} == {'L', 'R'})
        dtx = np.abs(np.diff(tx))
        jumps = int(np.nansum(dtx > 120.0))
        leads = []
        for tc, sgn in corners:
            # 이 설정의 명령이 같은 방향으로 |k| > 0.6을 처음 넘은 시각 (코너 2초 전부터)
            w = (t >= tc - 2.0) & (t <= tc + 1.0) & (np.sign(k) == sgn) & (np.abs(k) > 0.6)
            leads.append(round(float(tc - t[w][0]), 2) if w.any() else None)
        res[n] = {
            'frames': N,
            'none%': 100.0 * st.count('NONE') / N,
            'hold%': 100.0 * sum(s in ('hold', 'stop') for s in src) / N,
            'mem%': 100.0 * src.count('memory') / N,
            'low%': 100.0 * low.mean(),
            'flips': flips,
            'jumps': jumps,
            'jitter': float(np.median(np.abs(np.diff(k)))) if N > 2 else 0.0,
            'lead_s': leads,
        }
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            with open(os.path.join(out_dir, f'{os.path.basename(bag.rstrip("/"))}_{n}.csv'),
                      'w', newline='') as fh:
                w = csv.writer(fh)
                w.writerow(['t', 'status', 'source', 'kappa', 'low_conf', 'tgt_x', 'mem_pts'])
                w.writerows(rs)
    return res, corners


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bev', required=True)
    ap.add_argument('--bags', nargs='+', required=True)
    ap.add_argument('--cfg', action='append', default=None,
                    help='이름:key=val,key=val (여러 번). 이름만 쓰면 기본값')
    ap.add_argument('--speed', type=float, default=0.3)
    ap.add_argument('--t0', type=float, default=None, help='bag 시작 후 몇 초부터')
    ap.add_argument('--t1', type=float, default=None, help='몇 초까지')
    ap.add_argument('--model', default=None)
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    cfg = BevConfig.from_yaml(os.path.expanduser(a.bev))
    sets = [_cast_cfg(c) for c in (a.cfg or ['base:'])]
    bags = []
    for b in a.bags:
        bags += sorted(glob.glob(os.path.expanduser(b))) or [b]
    for bag in bags:
        res, corners = evaluate(bag, cfg, sets, a.speed, a.t0, a.t1, a.model, a.out)
        print(f'\n== {bag}  (IMU 코너 {len(corners)}개: '
              + ', '.join(f'{tc:.1f}s{"L" if s > 0 else "R"}' for tc, s in corners) + ')')
        print(f'{"설정":14s} {"프레임":>6s} {"NONE%":>6s} {"hold%":>6s} {"mem%":>5s} {"low%":>5s} '
              f'{"뒤집힘":>5s} {"튐":>4s} {"흔들림":>6s}  코너 선행(s)')
        for n, r in res.items():
            print(f'{n:14s} {r["frames"]:6d} {r["none%"]:6.1f} {r["hold%"]:6.1f} {r["mem%"]:5.1f} '
                  f'{r["low%"]:5.1f} {r["flips"]:5d} {r["jumps"]:4d} {r["jitter"]:6.3f}  '
                  f'{r["lead_s"]}')


if __name__ == '__main__':
    main()
