"""11/16: GPU(TensorRT) 고르기 - 엔진 없거나 GPU 안 되면 CPU(onnx)로."""
from autorace_perception.trt_runner import choose_backend, engine_path_for

M = '/m/autorace_yolo_v45.onnx'


def test_engine_path_next_to_onnx():
    assert engine_path_for(M) == '/m/autorace_yolo_v45.engine'
    assert engine_path_for(M, '/x/a.engine') == '/x/a.engine'


def test_gpu_when_engine_and_cuda():
    assert choose_backend(M, '', True, exists=lambda p: True, gpu_ok=True)[:2] == ('trt', '/m/autorace_yolo_v45.engine')


def test_cpu_fallbacks():
    assert choose_backend(M, '', True, exists=lambda p: False, gpu_ok=True)[0] == 'onnx'
    assert choose_backend(M, '', True, exists=lambda p: True, gpu_ok=False)[0] == 'onnx'
    assert choose_backend(M, '', False, exists=lambda p: True, gpu_ok=True)[0] == 'onnx'
    assert choose_backend('', '', True)[0] == ''
