"""차선 분할 모델(ONNX) 추론 + 학습·추론 공통 전처리.

학습(training/lane_seg/train_lane_seg.py)과 추론이 같은 preprocess()를 쓰므로
정규화·입력 크기가 어긋날 일이 없다. numpy/cv2만 쓰고 onnxruntime은 SegMasker에서만 부른다.

모델: 입력 [1, 3, H, W] float32 (RGB, ImageNet 정규화), 출력 [1, 1, H, W] 로짓.
클래스는 '노면 표시(차선·정지선 등 흰/노란 선)' 하나. 차선과 가로선의 구분은
지금처럼 BEV 단계(가로선 제거, 정지선 검출)가 맡는다.
"""

import numpy as np
import cv2

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
DEFAULT_SIZE = (320, 240)   # (W, H)


def preprocess(bgr: np.ndarray, size=DEFAULT_SIZE) -> np.ndarray:
    """BGR 이미지 -> [3, H, W] float32 (RGB, 정규화)."""
    img = cv2.resize(bgr, size, interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    return np.ascontiguousarray(((rgb - MEAN) / STD).transpose(2, 0, 1))


class SegMasker:
    """LaneFollower(mask_fn=...)에 넘기는 콜러블. bgr -> (마스크, 마스크)."""

    def __init__(self, onnx_path: str, threshold: float = 0.5):
        import onnxruntime as ort
        avail = ort.get_available_providers()
        prefer = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
        providers = [p for p in prefer if p in avail] or avail
        self.sess = ort.InferenceSession(onnx_path, providers=providers)
        self.input_name = self.sess.get_inputs()[0].name
        shape = self.sess.get_inputs()[0].shape          # [1, 3, H, W]
        self.size = (int(shape[3]), int(shape[2]))
        self.logit_th = float(np.log(threshold / (1.0 - threshold)))
        self.providers = self.sess.get_providers()

    def predict(self, bgr: np.ndarray) -> np.ndarray:
        """원본 크기의 uint8 마스크 (255 = 노면 표시)."""
        x = preprocess(bgr, self.size)[None]
        logit = self.sess.run(None, {self.input_name: x})[0][0, 0]
        mask = (logit > self.logit_th).astype(np.uint8) * 255
        return cv2.resize(mask, (bgr.shape[1], bgr.shape[0]), interpolation=cv2.INTER_NEAREST)

    def __call__(self, bgr: np.ndarray):
        m = self.predict(bgr)
        return m, m
