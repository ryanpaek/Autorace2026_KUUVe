"""v4.5 인식/발행 변경. ROS 수송 대신 API 모의 객체 사용."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2  # noqa: F401  # sys.modules 모의 블록 밖에서 먼저 불러 단독 실행 시 numpy 재로딩 오류를 막는다
import numpy  # noqa: F401

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/autorace_perception'))
from autorace_perception.class_commands import ClassCommandRouter


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def detector_module():
    stubs = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'rcl_interfaces.msg', 'sensor_msgs.msg', 'std_msgs.msg')}
    stubs['rclpy.node'].Node = object
    stubs['rclpy.qos'].qos_profile_sensor_data = None
    for module, names in (
        ('rcl_interfaces.msg', ('ParameterDescriptor', 'SetParametersResult')),
        ('sensor_msgs.msg', ('Image', 'CompressedImage')),
        ('std_msgs.msg', ('String',))):
        for name in names:
            setattr(stubs[module], name, SimpleNamespace)
    with patch.dict(sys.modules, stubs):
        return load_file('detector_v45_test', ROOT / 'src/autorace_perception/autorace_perception/detector_node.py')


class V45Contract(unittest.TestCase):
    def test_yellow_event_is_not_a_motion_command_or_red_release(self):
        r = ClassCommandRouter(vote_window=1, vote_need=1)
        r.set_mode('M1_TRAFFIC')
        def frame(t, cls):
            return r.route({'stamp': t, 'detections': [{'cls': cls, 'conf': 0.9}]})
        y = frame(1, 'yellow')
        self.assertEqual(y['commands'], [])
        self.assertEqual(y['signals'][0]['signal'], 'YELLOW_OBSERVED')
        frame(2, 'red')
        y = frame(3, 'yellow')
        self.assertTrue(y['traffic_stop_active'])
        self.assertEqual(y['commands'][0]['command'], 'PAUSE_UNTIL_GREEN')
        self.assertEqual(y['signals'][0]['action'], 'none')
        self.assertFalse(frame(4, 'green')['traffic_stop_active'])

    def test_yellow_event_requires_vote_and_current_observation(self):
        r = ClassCommandRouter(vote_window=3, vote_need=2)
        r.set_mode('M1_TRAFFIC')
        for t in (1, 2):
            self.assertEqual(r.route({'stamp': t, 'detections': [{'cls': 'yellow', 'conf': .9}]})['signals'], [])
        self.assertTrue(r.route({'stamp': 3, 'detections': [{'cls': 'yellow', 'conf': .9}]})['signals'])
        self.assertEqual(r.route({'stamp': 4, 'detections': []})['signals'], [])

    def test_conf_validation_rejects_invalid_values(self):
        cls = detector_module().DetectorNode
        node = cls.__new__(cls)
        for key in ('conf_threshold', 'nms_threshold'):
            for value in (-.1, 1.1, float('nan'), float('inf'), True, '0.7'):
                self.assertFalse(node._on_parameters([SimpleNamespace(name=key, value=value)]).successful)
            self.assertTrue(node._on_parameters([SimpleNamespace(name=key, value=.7)]).successful)

    def test_wrong_fixed_onnx_shape_fails_at_startup(self):
        cls = detector_module().DetectorNode
        node = cls.__new__(cls)
        node.size = 320
        node.get_parameter = lambda name: SimpleNamespace(value=2)
        node.get_logger = Mock(return_value=Mock())
        ort = ModuleType('onnxruntime')
        ort.SessionOptions = SimpleNamespace
        ort.ExecutionMode = SimpleNamespace(ORT_SEQUENTIAL=0)
        session = SimpleNamespace(get_inputs=lambda: [SimpleNamespace(name='images', shape=[1, 3, 640, 640])])
        ort.InferenceSession = lambda *a, **kw: session
        with patch.dict(sys.modules, {'onnxruntime': ort}):
            with self.assertRaisesRegex(ValueError, 'does not match'):
                node._load_model('model.onnx')
            node.size = 640
            node._load_model('model.onnx')
            self.assertEqual(node.backend, 'onnx')

    def test_profile_tool_preserves_every_non_yolo_parameter(self):
        import yaml
        tool = load_file('profile_v45_test', ROOT / 'tools/configure_yolo_v45.py')
        text = (ROOT / 'src/autorace_bringup/config/params.yaml').read_text()
        for size in (320, 640):
            output = tool.configured_text(text, '/tmp/model.onnx', size, .65)
            before, after = yaml.safe_load(text), yaml.safe_load(output)
            for key in before:
                if key not in ('detector_node', 'class_command_node'):
                    self.assertEqual(before[key], after[key])
            for key in ('infer_hz', 'ort_threads', 'class_names'):
                self.assertEqual(before['detector_node']['ros__parameters'][key],
                                 after['detector_node']['ros__parameters'][key])
            self.assertEqual(after['detector_node']['ros__parameters']['input_size'], size)
            self.assertEqual(after['class_command_node']['ros__parameters']['conf_threshold'], .65)
        self.assertIn('ped_wait_sec: 3.5', output)


if __name__ == '__main__':
    unittest.main()
