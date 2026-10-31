"""라벨 폴더를 학습 크기(320x240)로 줄여 여러 개의 작은 zip으로 묶는다 (Drive 업로드·Colab용).

모델 입력이 320x240이라 원본(640x480)을 올릴 필요가 없다. 용량이 약 1/8로 준다.
각 zip은 images/, masks/ 를 그대로 담고 있어서 Colab에서 여러 개를 같은 폴더에 풀면 합쳐진다.

  python pack_dataset.py --data lane_dataset --out packs --max-mb 25
  -> packs/lane_dataset_part0.zip, part1.zip, ...
"""

import argparse
import glob
import io
import os
import zipfile

import cv2

SIZE = (320, 240)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--max-mb', type=float, default=25.0)
    ap.add_argument('--quality', type=int, default=92)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    pairs = []
    for f in sorted(glob.glob(os.path.join(a.data, 'images', '*'))):
        stem = os.path.splitext(os.path.basename(f))[0]
        m = os.path.join(a.data, 'masks', stem + '.png')
        if os.path.exists(m):
            pairs.append((stem, f, m))
    part, size, z, n_total = 0, 0, None, 0
    limit = a.max_mb * 1024 * 1024 * 0.97
    for stem, fi, fm in pairs:
        img = cv2.resize(cv2.imread(fi), SIZE, interpolation=cv2.INTER_AREA)
        lab = cv2.resize(cv2.imread(fm, cv2.IMREAD_UNCHANGED), SIZE, interpolation=cv2.INTER_NEAREST)
        ok1, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
        ok2, png = cv2.imencode('.png', lab)
        add = len(jpg) + len(png)
        if z is None or size + add > limit:
            if z is not None:
                z.close()
                print(f'part{part - 1}: {size / 2**20:.1f} MB')
            z = zipfile.ZipFile(os.path.join(a.out, f'lane_dataset_part{part}.zip'), 'w',
                                zipfile.ZIP_STORED)
            part, size = part + 1, 0
        z.writestr(f'lane_dataset/images/{stem}.jpg', jpg.tobytes())
        z.writestr(f'lane_dataset/masks/{stem}.png', png.tobytes())
        size += add
        n_total += 1
    if z is not None:
        z.close()
        print(f'part{part - 1}: {size / 2**20:.1f} MB')
    print(f'{n_total} pairs -> {part} zip(s) in {a.out}')


if __name__ == '__main__':
    main()
