"""수집한 사진에 HSV로 차선 마스크를 자동으로 만든다 (학습 라벨 초안).

조명이 괜찮은 사진에서는 HSV가 거의 맞는 마스크를 만든다. 틀린 사진은
  - 지우거나 (dataset/images와 dataset/masks에서 같은 이름 둘 다),
  - 그림판·GIMP로 masks/*.png를 고친다 (흰색 255 = 노면 표시, 검정 0 = 배경).
조명이 나쁜 사진(어둡거나 반사가 심한)은 HSV가 틀리기 쉬우니, 같은 장면을 조명 좋을 때
찍은 사진으로 라벨을 만들고 학습 때 밝기·반사 증강으로 나쁜 조명을 흉내 내는 편이 빠르다.

예)
  python3 autolabel.py --images ~/autorace_images/day1 --out ~/lane_dataset \
      --every 2 --ignore-top 0.35 --white-v-min 170
결과
  ~/lane_dataset/images/*.jpg   학습 입력 (원본 복사)
  ~/lane_dataset/masks/*.png    라벨 (0/255)
  ~/lane_dataset/preview/*.jpg  확인용 (초록 = 라벨). 이걸 넘겨 보며 틀린 것을 고른다
"""

import argparse
import glob
import os
import shutil
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'autorace_lane'))
from autorace_lane.lane_core import LaneParams, color_masks  # noqa: E402


def clean(mask, min_area):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = np.zeros_like(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', required=True, nargs='+', help='사진 폴더 (여러 개 가능)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--every', type=int, default=1, help='N장마다 하나')
    ap.add_argument('--dedupe', type=float, default=4.0,
                    help='직전에 고른 사진과 평균 밝기 차가 이보다 작으면 건너뜀 (0=끔)')
    ap.add_argument('--ignore-top', type=float, default=0.35,
                    help='위쪽 이 비율은 배경으로 라벨 (트랙 밖 벽·사람)')
    ap.add_argument('--min-area', type=int, default=30)
    ap.add_argument('--color', default='both', choices=['white', 'yellow', 'both'])
    ap.add_argument('--white-v-min', type=int, default=180)
    ap.add_argument('--white-s-max', type=int, default=60)
    ap.add_argument('--yellow-h-min', type=int, default=15)
    ap.add_argument('--yellow-h-max', type=int, default=40)
    a = ap.parse_args(argv)

    p = LaneParams(lane_color=a.color, white_v_min=a.white_v_min, white_s_max=a.white_s_max,
                   yellow_h_min=a.yellow_h_min, yellow_h_max=a.yellow_h_max)
    out = os.path.expanduser(a.out)
    for sub in ('images', 'masks', 'preview'):
        os.makedirs(os.path.join(out, sub), exist_ok=True)

    files = []
    for d in a.images:
        d = os.path.expanduser(d)
        files += sorted(f for ext in ('jpg', 'jpeg', 'png')
                        for f in glob.glob(os.path.join(d, f'*.{ext}')))
    kept, last_small = 0, None
    for i, f in enumerate(files):
        if i % max(1, a.every):
            continue
        img = cv2.imread(f)
        if img is None:
            continue
        small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (64, 48)).astype(np.float32)
        if a.dedupe > 0 and last_small is not None and np.abs(small - last_small).mean() < a.dedupe:
            continue
        last_small = small
        lane, _ = color_masks(img, p)
        lane[:int(lane.shape[0] * a.ignore_top)] = 0
        lane = clean(lane, a.min_area)
        stem = f'{os.path.basename(os.path.dirname(f))}_{os.path.splitext(os.path.basename(f))[0]}'
        shutil.copy(f, os.path.join(out, 'images', stem + os.path.splitext(f)[1]))
        cv2.imwrite(os.path.join(out, 'masks', stem + '.png'), lane)
        prev = img.copy()
        prev[lane > 0] = (0.4 * prev[lane > 0] + 0.6 * np.array([0, 255, 0])).astype(np.uint8)
        cv2.imwrite(os.path.join(out, 'preview', stem + '.jpg'), prev)
        kept += 1
    print(f'{len(files)} files -> {kept} labeled in {out}')


if __name__ == '__main__':
    main()
