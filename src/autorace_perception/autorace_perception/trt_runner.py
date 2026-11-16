"""YOLO를 GPU(TensorRT)로 돌리기 (11/16).

리모(JetPack 6.2): tensorrt 10.3 + torch 2.8(CUDA) 가 이미 있음 -> pycuda 없이 torch 텐서를 GPU 버퍼로 씀.
엔진(.engine)은 리모에서 한 번 만들어 둠 (tools/build_trt.sh, trtexec --fp16). 다른 기계에서 만든 엔진은 못 씀.

choose_backend: 엔진이 있고 GPU가 되면 'trt', 아니면 'onnx'(CPU, ort_threads 제한).
"""
import os
from typing import Optional, Tuple

import numpy as np


def engine_path_for(model_path: str, engine_path: str = '') -> str:
    """engine_path가 비면 model_path 옆 같은 이름 .engine"""
    if engine_path:
        return engine_path
    root, _ = os.path.splitext(model_path or '')
    return root + '.engine' if root else ''


def choose_backend(model_path: str, engine_path: str, use_gpu: bool,
                   exists=os.path.exists, gpu_ok: Optional[bool] = None) -> Tuple[str, str, str]:
    """(backend, path, why). backend: 'trt' | 'onnx' | 'ultralytics' | ''"""
    if not model_path and not engine_path:
        return '', '', 'model_path 비어 있음'
    eng = engine_path_for(model_path, engine_path)
    why = ''
    if use_gpu:
        if not eng or not exists(eng):
            why = f'엔진 없음 ({eng}) -> tools/build_trt.sh 로 만들기'
        else:
            if gpu_ok is None:
                gpu_ok = trt_available()
            if gpu_ok:
                return 'trt', eng, 'GPU TensorRT'
            why = 'tensorrt/torch CUDA 못 씀'
    else:
        why = 'use_gpu false'
    if model_path.endswith('.onnx'):
        return 'onnx', model_path, 'CPU: ' + why
    if model_path.endswith('.pt'):
        return 'ultralytics', model_path, why
    return '', model_path, why


def trt_available() -> bool:
    try:
        import tensorrt  # noqa: F401
        import torch
        return bool(torch.cuda.is_available())
    except Exception:       # noqa: BLE001
        return False


class TrtRunner:
    """입력 1개(1x3xSxS float32) -> 첫 출력 numpy. onnxruntime session.run(...)[0] 와 같은 모양."""

    def __init__(self, engine_path: str, logger=None):
        import tensorrt as trt
        import torch
        self.torch = torch
        self._log = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(self._log).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f'엔진 읽기 실패 (다른 TensorRT 버전/기계에서 만든 엔진?): {engine_path}')
        self.ctx = self.engine.create_execution_context()
        self.dev = torch.device('cuda')
        self.stream = torch.cuda.Stream()
        tdt = {trt.float32: torch.float32, trt.float16: torch.float16,
               trt.int32: torch.int32, trt.int8: torch.int8}
        self.inputs, self.outputs = [], []
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            buf = torch.empty(shape, dtype=tdt[self.engine.get_tensor_dtype(name)], device=self.dev)
            self.ctx.set_tensor_address(name, buf.data_ptr())
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.inputs.append((name, buf))
            else:
                self.outputs.append((name, buf))
        self.input_shape = self.inputs[0][1].shape

    def __call__(self, x: np.ndarray) -> np.ndarray:
        torch = self.torch
        _, buf = self.inputs[0]
        with torch.cuda.stream(self.stream):
            buf.copy_(torch.from_numpy(np.ascontiguousarray(x)).to(self.dev, non_blocking=True))
            self.ctx.execute_async_v3(self.stream.cuda_stream)
            out = self.outputs[0][1].float().cpu()
        self.stream.synchronize()
        return out.numpy()
