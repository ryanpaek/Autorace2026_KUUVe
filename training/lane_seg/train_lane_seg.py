"""차선(노면 표시) 분할 모델 학습 -> ONNX 내보내기.

모델: MobileNetV2 인코더 + 작은 U-Net 디코더, 입력 320x240, 출력 1채널 로짓.
VRAM 4GB에 맞춘 기본값: batch 8, 혼합정밀도(AMP). 부족하면 --batch 4.

데이터 폴더 (autolabel.py 결과)
  DATA/images/*.jpg|png, DATA/masks/*.png (같은 이름, 0/255)

예)
  pip install torch torchvision onnx          # CUDA 버전은 pytorch.org 안내대로
  python3 train_lane_seg.py --data ~/lane_dataset --out runs/v1 --pretrained
  # 결과: runs/v1/best.pt, runs/v1/lane_seg.onnx, runs/v1/val_preview/*.jpg
  # LIMO: params.yaml에서 lane_mask_source: model, seg_model_path: <lane_seg.onnx 경로>

조명 증강을 세게 건다 (밝기, 대비, 감마, 색온도, 채도, 그림자, 반사광, 블러, 노이즈).
대회장 조명을 모르니 이 증강이 HSV보다 나은 이유의 대부분이다.
"""

import argparse
import glob
import os
import random
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'autorace_lane'))
from autorace_lane.seg_infer import DEFAULT_SIZE, preprocess  # noqa: E402


# --------------------------------------------------------------------------
# 모델
# --------------------------------------------------------------------------
class Up(nn.Module):
    def __init__(self, c_in, c_skip, c_out):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(c_in + c_skip, c_out, 3, padding=1, bias=False),
            nn.BatchNorm2d(c_out), nn.ReLU(inplace=True),
            nn.Conv2d(c_out, c_out, 3, padding=1, bias=False),
            nn.BatchNorm2d(c_out), nn.ReLU(inplace=True))

    def forward(self, x, skip):
        x = F.interpolate(x, size=skip.shape[-2:], mode='bilinear', align_corners=False)
        return self.conv(torch.cat([x, skip], 1))


class LaneUNet(nn.Module):
    """MobileNetV2 특징 (stride 2/4/8/16/32) + U-Net 디코더."""

    def __init__(self, pretrained=False):
        super().__init__()
        weights = 'DEFAULT' if pretrained else None
        feats = torchvision.models.mobilenet_v2(weights=weights).features
        self.s2, self.s4, self.s8 = feats[:2], feats[2:4], feats[4:7]
        self.s16, self.s32 = feats[7:14], feats[14:18]
        self.up16 = Up(320, 96, 96)
        self.up8 = Up(96, 32, 48)
        self.up4 = Up(48, 24, 32)
        self.up2 = Up(32, 16, 24)
        self.head = nn.Conv2d(24, 1, 1)

    def forward(self, x):
        h, w = x.shape[-2:]
        f2 = self.s2(x)
        f4 = self.s4(f2)
        f8 = self.s8(f4)
        f16 = self.s16(f8)
        f32 = self.s32(f16)
        y = self.up2(self.up4(self.up8(self.up16(f32, f16), f8), f4), f2)
        return F.interpolate(self.head(y), size=(h, w), mode='bilinear', align_corners=False)


# --------------------------------------------------------------------------
# 데이터 + 조명 증강
# --------------------------------------------------------------------------
def light_augment(img: np.ndarray, rng: random.Random) -> np.ndarray:
    """이미지만 바꾸는 조명 증강 (라벨은 그대로)."""
    f = img.astype(np.float32)
    # 밝기·대비·감마
    f = f * rng.uniform(0.5, 1.5) + rng.uniform(-40, 40)
    f = np.clip(f, 0, 255)
    f = 255.0 * (f / 255.0) ** rng.uniform(0.6, 1.6)
    # 색온도 (B, R 채널 따로)
    f[..., 0] *= rng.uniform(0.75, 1.25)
    f[..., 2] *= rng.uniform(0.75, 1.25)
    img = np.clip(f, 0, 255).astype(np.uint8)
    # 채도
    if rng.random() < 0.7:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
        hsv[..., 1] = np.clip(hsv[..., 1] * rng.uniform(0.5, 1.5), 0, 255)
        img = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    h, w = img.shape[:2]
    # 그림자 (큰 다각형을 어둡게)
    if rng.random() < 0.5:
        pts = np.array([[rng.randint(0, w), rng.randint(0, h)] for _ in range(5)], np.int32)
        m = np.zeros((h, w), np.uint8)
        cv2.fillPoly(m, [cv2.convexHull(pts)], 1)
        img = img.astype(np.float32)
        img[m > 0] *= rng.uniform(0.35, 0.75)
        img = img.astype(np.uint8)
    # 반사광 (흐린 흰 타원) — 반사를 차선으로 착각하지 않게
    if rng.random() < 0.5:
        glare = np.zeros((h, w), np.float32)
        cv2.ellipse(glare, (rng.randint(0, w), rng.randint(0, h)),
                    (rng.randint(w // 20, w // 4), rng.randint(h // 30, h // 8)),
                    rng.uniform(0, 180), 0, 360, 1.0, -1)
        glare = cv2.GaussianBlur(glare, (0, 0), rng.uniform(5, 20)) * rng.uniform(80, 220)
        img = np.clip(img.astype(np.float32) + glare[..., None], 0, 255).astype(np.uint8)
    if rng.random() < 0.3:
        img = cv2.GaussianBlur(img, (0, 0), rng.uniform(0.5, 1.5))
    if rng.random() < 0.3:
        noise = np.random.default_rng(rng.randint(0, 1 << 30)).normal(0, rng.uniform(3, 12), img.shape)
        img = np.clip(img + noise, 0, 255).astype(np.uint8)
    return img


def geo_augment(img, mask, rng):
    """좌우 반전 + 작은 회전·이동·확대 (이미지와 라벨을 같이)."""
    if rng.random() < 0.5:
        img, mask = img[:, ::-1].copy(), mask[:, ::-1].copy()
    if rng.random() < 0.6:
        h, w = img.shape[:2]
        M = cv2.getRotationMatrix2D((w / 2, h / 2), rng.uniform(-6, 6), rng.uniform(0.9, 1.1))
        M[:, 2] += (rng.uniform(-0.05, 0.05) * w, rng.uniform(-0.05, 0.05) * h)
        img = cv2.warpAffine(img, M, (w, h), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        mask = cv2.warpAffine(mask, M, (w, h), flags=cv2.INTER_NEAREST, borderValue=0)
    return img, mask


def find_pairs(data_dir):
    pairs = []
    for f in sorted(glob.glob(os.path.join(data_dir, 'images', '*'))):
        m = os.path.join(data_dir, 'masks', os.path.splitext(os.path.basename(f))[0] + '.png')
        if os.path.exists(m):
            pairs.append((f, m))
    return pairs


class LaneDataset(torch.utils.data.Dataset):
    def __init__(self, pairs, size, train):
        self.pairs, self.size, self.train = pairs, size, train
        self.rng = random.Random(0 if not train else None)

    def __len__(self):
        return len(self.pairs)

    def __getitem__(self, i):
        fi, fm = self.pairs[i]
        img = cv2.resize(cv2.imread(fi), self.size, interpolation=cv2.INTER_AREA)
        mask = cv2.resize(cv2.imread(fm, cv2.IMREAD_GRAYSCALE), self.size,
                          interpolation=cv2.INTER_NEAREST)
        if self.train:
            img, mask = geo_augment(img, mask, self.rng)
            img = light_augment(img, self.rng)
        x = torch.from_numpy(preprocess(img, self.size))
        y = torch.from_numpy((mask > 127).astype(np.float32))[None]
        return x, y


# --------------------------------------------------------------------------
# 학습
# --------------------------------------------------------------------------
def dice_loss(logits, y, eps=1.0):
    p = torch.sigmoid(logits)
    inter = (p * y).sum((1, 2, 3))
    return (1 - (2 * inter + eps) / (p.sum((1, 2, 3)) + y.sum((1, 2, 3)) + eps)).mean()


@torch.no_grad()
def evaluate(model, loader, device):
    model.eval()
    inter = union = 0.0
    for x, y in loader:
        x, y = x.to(device), y.to(device)
        p = (model(x) > 0).float()
        inter += (p * y).sum().item()
        union += ((p + y) > 0).float().sum().item()
    return inter / max(union, 1.0)


def export_onnx(model, path, size):
    model.eval().cpu()
    dummy = torch.zeros(1, 3, size[1], size[0])
    torch.onnx.export(model, dummy, path, input_names=['images'], output_names=['logits'],
                      opset_version=17, dynamo=False)
    print(f'ONNX -> {path}')


def save_previews(model, pairs, out_dir, size, n=12):
    os.makedirs(out_dir, exist_ok=True)
    model.eval().cpu()
    with torch.no_grad():
        for fi, fm in pairs[:n]:
            img = cv2.resize(cv2.imread(fi), size)
            logit = model(torch.from_numpy(preprocess(img, size))[None])[0, 0].numpy()
            pred = logit > 0
            gt = cv2.resize(cv2.imread(fm, cv2.IMREAD_GRAYSCALE), size,
                            interpolation=cv2.INTER_NEAREST) > 127
            vis = img.copy()
            vis[gt & ~pred] = (0, 0, 255)      # 빨강: 놓친 곳
            vis[pred & ~gt] = (255, 0, 0)      # 파랑: 잘못 잡은 곳
            vis[pred & gt] = (0, 255, 0)       # 초록: 맞춘 곳
            cv2.imwrite(os.path.join(out_dir, os.path.basename(fi)), np.hstack([img, vis]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', required=True)
    ap.add_argument('--out', default='runs/lane_seg')
    ap.add_argument('--epochs', type=int, default=60)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--val-ratio', type=float, default=0.15)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--pretrained', action='store_true', help='ImageNet 사전학습 인코더 (권장)')
    ap.add_argument('--device', default='auto')
    ap.add_argument('--export-only', default=None, help='best.pt 경로: 학습 없이 ONNX만')
    a = ap.parse_args(argv)

    size = DEFAULT_SIZE
    os.makedirs(a.out, exist_ok=True)
    device = ('cuda' if torch.cuda.is_available() else 'cpu') if a.device == 'auto' else a.device
    model = LaneUNet(pretrained=a.pretrained and not a.export_only)
    if a.export_only:
        model.load_state_dict(torch.load(a.export_only, map_location='cpu'))
        export_onnx(model, os.path.join(a.out, 'lane_seg.onnx'), size)
        return

    pairs = find_pairs(os.path.expanduser(a.data))
    if len(pairs) < 10:
        sys.exit(f'라벨 쌍이 너무 적음: {len(pairs)}')
    random.Random(0).shuffle(pairs)
    n_val = max(2, int(len(pairs) * a.val_ratio))
    val_pairs, train_pairs = pairs[:n_val], pairs[n_val:]
    print(f'train {len(train_pairs)} / val {len(val_pairs)}, device {device}')

    train_dl = torch.utils.data.DataLoader(LaneDataset(train_pairs, size, True), a.batch,
                                           shuffle=True, num_workers=a.workers, drop_last=True)
    val_dl = torch.utils.data.DataLoader(LaneDataset(val_pairs, size, False), a.batch,
                                         num_workers=a.workers)
    model.to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    steps = a.epochs * max(1, len(train_dl))
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=steps)
    use_amp = device.startswith('cuda')
    scaler = torch.amp.GradScaler('cuda', enabled=use_amp)
    # 노면 표시는 화소의 몇 %뿐이라 양성 가중치를 준다
    bce = nn.BCEWithLogitsLoss(pos_weight=torch.tensor([5.0], device=device))

    best = -1.0
    for ep in range(a.epochs):
        model.train()
        t0, total = time.time(), 0.0
        for x, y in train_dl:
            x, y = x.to(device), y.to(device)
            with torch.autocast(device_type='cuda', dtype=torch.float16, enabled=use_amp):
                logits = model(x)
                loss = bce(logits, y) + dice_loss(logits.float(), y)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            total += loss.item()
        iou = evaluate(model, val_dl, device)
        mark = ''
        if iou > best:
            best = iou
            torch.save(model.state_dict(), os.path.join(a.out, 'best.pt'))
            mark = ' *best'
        print(f'epoch {ep + 1}/{a.epochs} loss {total / max(1, len(train_dl)):.4f} '
              f'val IoU {iou:.3f} ({time.time() - t0:.0f}s){mark}')
        if use_amp and ep == 0:
            print(f'  GPU memory peak {torch.cuda.max_memory_allocated() / 2**30:.2f} GB')

    model.load_state_dict(torch.load(os.path.join(a.out, 'best.pt'), map_location='cpu'))
    export_onnx(model, os.path.join(a.out, 'lane_seg.onnx'), size)
    save_previews(model, val_pairs, os.path.join(a.out, 'val_preview'), size)
    print(f'best val IoU {best:.3f}')


if __name__ == '__main__':
    main()
