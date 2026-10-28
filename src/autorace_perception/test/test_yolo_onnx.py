"""ONNX 디코딩 테스트 (onnxruntime 없이 가짜 출력으로)."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_perception.yolo_onnx import decode, letterbox  # noqa: E402

NAMES = ['red', 'yellow', 'green']


def test_letterbox_scale_and_pad():
    canvas, scale, px, py = letterbox(np.zeros((480, 640, 3), np.uint8), 320)
    assert canvas.shape == (320, 320, 3)
    assert scale == pytest.approx(0.5)
    assert (px, py) == (0, 40)


def test_decode_end2end_unletterboxes():
    # 320 입력에서 (100,100)-(140,180) 상자, 원본 640x480 (scale 0.5, pad_y 40)
    out = np.array([[[100, 100, 140, 180, 0.9, 2], [0, 0, 10, 10, 0.1, 0]]], np.float32)
    dets = decode(out, NAMES, (480, 640), 0.5, 0, 40, 320)
    assert len(dets) == 1
    assert dets[0]['cls'] == 'green'
    assert dets[0]['bbox'] == [200, 120, 80, 160]


def test_decode_raw_with_nms():
    a = np.zeros((7, 3), np.float32)        # (4+nc, A) 레이아웃
    a[:, 0] = [160, 160, 20, 40, 0.8, 0.1, 0.1]
    a[:, 1] = [161, 160, 20, 40, 0.7, 0.1, 0.1]  # 거의 같은 상자 -> NMS로 제거
    a[:, 2] = [50, 50, 10, 10, 0.1, 0.2, 0.1]    # 낮은 점수
    dets = decode(a[None], NAMES, (320, 320), 1.0, 0, 0, 320, conf_th=0.5)
    assert len(dets) == 1 and dets[0]['cls'] == 'red'
