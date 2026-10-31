"""조명을 바꿔도 차선(노랑)을 찾는지 HSV와 학습 모델을 비교한다 (픽셀 IoU).

기준은 원래 조명에서 만든 라벨이다. 같은 사진을 어둡게·밝게·색온도·대비·그림자·반사광으로
바꿔서 HSV와 모델이 각각 얼마나 기준과 맞는지 본다. 학습 때와 같은 검증 분할(시드 0, 15%)을 쓴다.

  python eval_lighting.py --data lane_dataset --model runs/v1/lane_seg.onnx --n 200
"""

import argparse
import glob
import os
import random
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'autorace_lane'))
from autorace_lane.lane_core import LaneParams, color_masks  # noqa: E402
from autorace_lane.seg_infer import SegMasker  # noqa: E402

KINDS = ['none', 'dark', 'bright', 'warm', 'cool', 'lowcontrast', 'shadow', 'glare']


def perturb(img, kind, rng):
    f = img.astype(np.float32)
    if kind == 'dark':
        f = 255 * (f / 255) ** 1.8 * 0.6
    elif kind == 'bright':
        f = f * 1.7 + 30
    elif kind == 'warm':
        f = f * np.array([0.65, 0.9, 1.25])
    elif kind == 'cool':
        f = f * np.array([1.3, 1.05, 0.7])
    elif kind == 'lowcontrast':
        f = (f - 128) * 0.45 + 128
    elif kind == 'shadow':
        w = f.shape[1]
        x0 = rng.randint(0, w // 2)
        f[:, x0:x0 + w // 2] *= 0.35
    elif kind == 'glare':
        h, w = f.shape[:2]
        yy, xx = np.mgrid[0:h, 0:w]
        for _ in range(3):
            cx, cy, r = rng.randint(0, w), rng.randint(h // 2, h), rng.randint(40, 110)
            f += 255 * 0.9 * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * r * r))[..., None]
    return np.clip(f, 0, 255).astype(np.uint8)


def iou(a, b):
    u = (a | b).sum()
    return (a & b).sum() / u if u else 1.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--n', type=int, default=200)
    ap.add_argument('--ignore-top', type=float, default=0.3)
    a = ap.parse_args(argv)
    pairs = []
    for f in sorted(glob.glob(os.path.join(a.data, 'images', '*'))):
        m = os.path.join(a.data, 'masks', os.path.splitext(os.path.basename(f))[0] + '.png')
        if os.path.exists(m):
            pairs.append((f, m))
    random.Random(0).shuffle(pairs)
    val = pairs[:max(2, int(len(pairs) * 0.15))][:a.n]
    sm = SegMasker(a.model)
    p = LaneParams(lane_color='yellow')
    rng = random.Random(1)
    res = {k: ([], []) for k in KINDS}
    for fi, fm in val:
        img, lab = cv2.imread(fi), cv2.imread(fm)
        if lab.shape[:2] != img.shape[:2]:
            lab = cv2.resize(lab, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        ref = (lab[..., 2] > 127) & (lab[..., 1] > 127) & (lab[..., 0] < 128)
        top = int(img.shape[0] * a.ignore_top)
        ref[:top] = False
        for k in KINDS:
            im = img if k == 'none' else perturb(img, k, rng)
            hs = color_masks(im, p)[0] > 0
            md = sm.predict(im)[0] > 0
            hs[:top] = md[:top] = False
            res[k][0].append(iou(hs, ref))
            res[k][1].append(iou(md, ref))
    print(f'노란 차선 IoU (기준 = 원래 조명 라벨), 검증 {len(val)}장')
    print(f"{'조명':12s} {'HSV':>6s} {'모델':>6s}")
    for k in KINDS:
        print(f'{k:12s} {np.mean(res[k][0]):6.3f} {np.mean(res[k][1]):6.3f}')


if __name__ == '__main__':
    main()
