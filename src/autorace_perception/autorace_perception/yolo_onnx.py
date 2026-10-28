"""YOLO ONNX 전처리·후처리 (ROS 의존성 없음).

D-Racer detection_node.py(ryanpaek/D-Racer-Kit)의 레터박스·디코딩을 함수로 떼어 옮겼다.
raw 레이아웃([1, 4+nc, A] 또는 [1, A, 4+nc], NMS 필요)과
end2end 레이아웃([1, N, 6] = x1,y1,x2,y2,conf,cls, NMS 포함, 예: YOLO26)을 모두 처리한다.
"""

from typing import List, Sequence, Tuple

import cv2
import numpy as np


def letterbox(frame: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
    h0, w0 = frame.shape[:2]
    scale = min(size / h0, size / w0)
    nw, nh = int(round(w0 * scale)), int(round(h0 * scale))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    pad_x, pad_y = (size - nw) // 2, (size - nh) // 2
    canvas[pad_y:pad_y + nh, pad_x:pad_x + nw] = cv2.resize(frame, (nw, nh))
    return canvas, scale, pad_x, pad_y


def to_tensor(canvas: np.ndarray) -> np.ndarray:
    """BGR -> RGB, /255, NCHW float32 (Ultralytics 전처리와 같음)."""
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.ascontiguousarray(np.transpose(rgb, (2, 0, 1))[np.newaxis])


def decode(out: np.ndarray, class_names: Sequence[str], orig_hw, scale: float,
           pad_x: int, pad_y: int, input_size: int, conf_th: float = 0.5,
           nms_th: float = 0.45) -> List[dict]:
    """모델 출력 -> [{'cls', 'conf', 'bbox': [x, y, w, h] (원본 px)}]."""
    nc = len(class_names)
    preds = np.squeeze(out)
    if preds.ndim == 1:
        preds = preds[None, :]
    if preds.ndim != 2:
        return []
    r, c = preds.shape
    if c == 4 + nc:
        layout = 'raw'
    elif r == 4 + nc:
        preds, layout = preds.T, 'raw'
    elif c == 6:
        layout = 'end2end'
    elif r == 6:
        preds, layout = preds.T, 'end2end'
    else:
        preds, layout = (preds.T if r < c else preds), 'raw'

    dets = []
    if layout == 'end2end':
        for row in preds:
            conf = float(row[4])
            if conf < conf_th:
                continue
            x1, y1 = (row[0] - pad_x) / scale, (row[1] - pad_y) / scale
            x2, y2 = (row[2] - pad_x) / scale, (row[3] - pad_y) / scale
            dets.append(_make(int(row[5]), conf, x1, y1, x2 - x1, y2 - y1, orig_hw, class_names))
        return dets

    boxes = preds[:, :4].astype(np.float32)
    scores = preds[:, 4:4 + nc]
    ids = np.argmax(scores, axis=1)
    confs = scores[np.arange(len(ids)), ids]
    keep = confs >= conf_th
    boxes, confs, ids = boxes[keep], confs[keep], ids[keep]
    if len(boxes) == 0:
        return []
    if float(boxes.max()) <= 2.0:          # 정규화 좌표로 내보낸 모델
        boxes *= input_size
    x = (boxes[:, 0] - boxes[:, 2] / 2 - pad_x) / scale
    y = (boxes[:, 1] - boxes[:, 3] / 2 - pad_y) / scale
    w, h = boxes[:, 2] / scale, boxes[:, 3] / scale
    idxs = cv2.dnn.NMSBoxes([[float(a), float(b), float(c_), float(d)]
                             for a, b, c_, d in zip(x, y, w, h)],
                            confs.tolist(), conf_th, nms_th)
    for i in (np.array(idxs).flatten().tolist() if len(idxs) else []):
        dets.append(_make(int(ids[i]), float(confs[i]), x[i], y[i], w[i], h[i],
                          orig_hw, class_names))
    return dets


def _make(cls_id, conf, x, y, w, h, orig_hw, class_names):
    h0, w0 = orig_hw
    xi = int(max(0, min(round(float(x)), w0 - 1)))
    yi = int(max(0, min(round(float(y)), h0 - 1)))
    wi = int(max(0, min(round(float(w)), w0 - xi)))
    hi = int(max(0, min(round(float(h)), h0 - yi)))
    name = class_names[cls_id] if 0 <= cls_id < len(class_names) else f'class_{cls_id}'
    return {'cls': name, 'conf': round(float(conf), 4), 'bbox': [xi, yi, wi, hi]}
