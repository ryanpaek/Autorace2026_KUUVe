"""수집한 사진에 HSV로 노면 표시 라벨 초안을 만든다 (2클래스: 노란 차선 / 흰 표시).

왜 2클래스인가
  모의 맵은 차로 경계가 노란 실선이고, 정지선·횡단보도·마름모·'천천히' 글자·3차선 점선이 흰색이다.
  차선 노드는 구간마다 노랑만 볼지(일반), 노랑+흰색을 볼지(3차선) 고르고, 정지선은 흰색에서 따로 찾는다.
  한 클래스로 섞으면 마름모·글자가 차선 후보로 들어가 쌍 선택이 흔들린다.

왜 매트 위 표시만 남기는가
  매트 밖 회색 바닥, 흰 벽·캐비닛, 상자도 HSV '흰색'·'노랑'에 걸린다. 덩어리마다 바로 바깥
  테두리가 대부분 검정 천이면 매트 위 표시로 보고 남긴다.

라벨 파일 (masks/*.png, 컬러): 노랑 (0,255,255) = 노란 차선, 흰색 (255,255,255) = 흰 표시, 검정 = 배경.
  그림판으로 고칠 때 이 두 색으로 칠하면 된다 (스포이트로 기존 색을 찍어 쓰면 정확).

예)
  python autolabel.py --images frames/run1 frames/run2 --out lane_dataset --every 1
결과
  lane_dataset/images/*.jpg   학습 입력 (원본 복사)
  lane_dataset/masks/*.png    라벨
  lane_dataset/preview/*.jpg  확인용 (노랑 = 노란 차선, 하늘 = 흰 표시)
  lane_dataset/contact/*.jpg  미리보기 20장씩 한 장으로 (빨리 넘겨 보기용)
"""

import argparse
import glob
import os
import shutil

import cv2
import numpy as np

YELLOW_BGR = (0, 255, 255)
WHITE_BGR = (255, 255, 255)


def clean(mask, min_area):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = np.zeros_like(mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255
    return out


def dark_mask(hsv, a):
    """매트(검정 천)로 보이는 화소. 방마다 밝기가 달라서 화면 아래쪽 밝기 기준으로 문턱을 올린다."""
    h = hsv.shape[0]
    v_bottom = float(np.median(hsv[int(h * 0.7):, :, 2]))
    v_max = max(a.mat_v_max, min(170, v_bottom + 45))
    return (hsv[..., 2] <= v_max) & (hsv[..., 1] <= a.mat_s_max)


def keep_on_mat(mask, dark, a, frac):
    """덩어리마다 바로 바깥 테두리가 대부분 매트(어두움)인 것만 남긴다.

    매트 위 표시는 사방이 검정 천이고, 매트 밖 회색 바닥·벽·상자는 주변도 밝다.
    """
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = np.zeros_like(mask)
    H, W = mask.shape[:2]
    r = a.ring_px
    k = np.ones((2 * r + 1, 2 * r + 1), np.uint8)
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < a.min_area:
            continue
        x0, y0, x1, y1 = max(0, x - r), max(0, y - r), min(W, x + w + r), min(H, y + h + r)
        comp = (lab[y0:y1, x0:x1] == i).astype(np.uint8)
        ring = (cv2.dilate(comp, k) > 0) & (comp == 0) & (mask[y0:y1, x0:x1] == 0)
        if ring.sum() == 0:
            continue
        if dark[y0:y1, x0:x1][ring].mean() >= frac:
            out[y0:y1, x0:x1][comp > 0] = 255
    return out


def make_labels(img, a):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    dark = dark_mask(hsv, a)
    yellow = cv2.inRange(hsv, np.array([a.yellow_h_min, a.yellow_s_min, a.yellow_v_min], np.uint8),
                         np.array([a.yellow_h_max, 255, 255], np.uint8))
    white = cv2.inRange(hsv, np.array([0, 0, a.white_v_min], np.uint8),
                        np.array([179, a.white_s_max, 255], np.uint8))
    white[yellow > 0] = 0
    top = int(img.shape[0] * a.ignore_top)
    for m in (yellow, white):
        m[:top] = 0
    op = np.ones((3, 3), np.uint8)
    # 노란 선은 매트 가장자리(바닥과 맞닿음)에도 있어 기준을 낮춘다. 상자 같은 갈색은 바닥·잔디 위라 걸러진다
    yellow = keep_on_mat(cv2.morphologyEx(yellow, cv2.MORPH_OPEN, op), dark, a, a.ring_dark_frac_yellow)
    white = keep_on_mat(cv2.morphologyEx(white, cv2.MORPH_OPEN, op), dark, a, a.ring_dark_frac)
    white = drop_far_floor(white, a)
    return yellow, white, dark


def drop_far_floor(white, a):
    """화면 좌우 끝에 닿은 채 지평선 근처에 있는 흰 덩어리 = 매트 너머 바닥 띠. 지운다."""
    n, lab, stats, cents = cv2.connectedComponentsWithStats(white, connectivity=8)
    H, W = white.shape[:2]
    for i in range(1, n):
        x, y, w, h, _ = stats[i]
        touches_side = x <= 1 or x + w >= W - 1
        if touches_side and cents[i][1] < a.far_floor_y * H:
            white[lab == i] = 0
    return white


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', required=True, nargs='+', help='사진 폴더 (여러 개 가능)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--every', type=int, default=1, help='N장마다 하나')
    ap.add_argument('--dedupe', type=float, default=4.0,
                    help='직전 저장 사진과 평균 밝기차가 이보다 작으면 건너뜀 (정지 중 중복 제거)')
    ap.add_argument('--ignore-top', type=float, default=0.3,
                    help='화면 위쪽 이 비율은 라벨에서 뺀다 (벽·천장)')
    ap.add_argument('--min-area', type=int, default=30)
    ap.add_argument('--white-v-min', type=int, default=170)
    ap.add_argument('--white-s-max', type=int, default=60)
    ap.add_argument('--yellow-h-min', type=int, default=14)
    ap.add_argument('--yellow-h-max', type=int, default=40)
    ap.add_argument('--yellow-s-min', type=int, default=70)
    ap.add_argument('--yellow-v-min', type=int, default=90)
    ap.add_argument('--mat-v-max', type=int, default=110, help='매트(검정 천) 밝기 상한')
    ap.add_argument('--mat-s-max', type=int, default=70)
    ap.add_argument('--min-mat-frac', type=float, default=0.55,
                    help='화면 아래 40%% 중 매트 비율이 이보다 작으면(차가 트랙 밖) 건너뜀')
    ap.add_argument('--far-floor-y', type=float, default=0.62,
                    help='좌우 끝에 닿은 흰 덩어리의 중심이 이 높이(비율)보다 위면 매트 너머 바닥으로 본다')
    ap.add_argument('--ring-px', type=int, default=6, help='덩어리 바깥 테두리 두께(px)')
    ap.add_argument('--ring-dark-frac-yellow', type=float, default=0.35)
    ap.add_argument('--ring-dark-frac', type=float, default=0.75,
                    help='테두리 중 매트(어두움) 비율이 이 이상이면 매트 위 표시로 본다')
    a = ap.parse_args(argv)

    out = os.path.expanduser(a.out)
    for sub in ('images', 'masks', 'preview', 'contact'):
        os.makedirs(os.path.join(out, sub), exist_ok=True)

    files = []
    for d in a.images:
        d = os.path.expanduser(d)
        files += sorted(f for ext in ('jpg', 'jpeg', 'png')
                        for f in glob.glob(os.path.join(d, '**', f'*.{ext}'), recursive=True))
    kept, skipped, last_small, sheet, n_sheet = 0, 0, None, [], 0
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
        yellow, white, dark = make_labels(img, a)
        if dark[int(dark.shape[0] * 0.6):].mean() < a.min_mat_frac:
            skipped += 1
            continue
        rel = os.path.relpath(f, os.path.commonpath([os.path.expanduser(d) for d in a.images]))
        stem = os.path.splitext(rel)[0].replace(os.sep, '_').replace('/', '_')
        shutil.copy(f, os.path.join(out, 'images', stem + os.path.splitext(f)[1].lower()))
        lab = np.zeros_like(img)
        lab[yellow > 0] = YELLOW_BGR
        lab[white > 0] = WHITE_BGR
        cv2.imwrite(os.path.join(out, 'masks', stem + '.png'), lab)
        prev = img.copy()
        prev[yellow > 0] = (0.3 * prev[yellow > 0] + 0.7 * np.array([0, 230, 255])).astype(np.uint8)
        prev[white > 0] = (0.3 * prev[white > 0] + 0.7 * np.array([255, 200, 0])).astype(np.uint8)
        cv2.imwrite(os.path.join(out, 'preview', stem + '.jpg'), prev)
        t = cv2.resize(prev, (320, 240))
        cv2.putText(t, stem[-28:], (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        sheet.append(t)
        if len(sheet) == 20:
            cv2.imwrite(os.path.join(out, 'contact', f'{n_sheet:03d}.jpg'),
                        np.vstack([np.hstack(sheet[r:r + 5]) for r in range(0, 20, 5)]))
            sheet, n_sheet = [], n_sheet + 1
        kept += 1
    if sheet:
        while len(sheet) % 5:
            sheet.append(np.zeros_like(sheet[0]))
        cv2.imwrite(os.path.join(out, 'contact', f'{n_sheet:03d}.jpg'),
                    np.vstack([np.hstack(sheet[r:r + 5]) for r in range(0, len(sheet), 5)]))
    print(f'{len(files)} files -> {kept} labeled in {out} (트랙 밖 {skipped}장 제외)')


if __name__ == '__main__':
    main()
