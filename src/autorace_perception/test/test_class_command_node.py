"""ROS API 모의 객체로 구독 → JSON 변환 → 발행 경계를 검증한다. 실제 DDS 시험은 아니다."""

import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def load_node():
    class FakeNode:
        def __init__(self, name):
            self.name, self.params = name, {}
            self.publisher = Mock()
            self.logger = Mock()

        def declare_parameter(self, name, value, *args):
            self.params[name] = value

        def get_parameter(self, name):
            return SimpleNamespace(value=self.params[name])

        def create_publisher(self, msg_type, topic, depth):
            self.output_topic = topic
            return self.publisher

        def create_subscription(self, msg_type, topic, callback, depth):
            self.input_topic, self.callback = topic, callback
            return Mock()

        def add_on_set_parameters_callback(self, callback):
            self.parameter_callback = callback

        def get_logger(self):
            return self.logger

    rclpy = ModuleType('rclpy')
    rclpy.shutdown = Mock()
    node_module, msg_module, interfaces = [ModuleType(n) for n in
                                          ('rclpy.node', 'std_msgs.msg', 'rcl_interfaces.msg')]
    node_module.Node = FakeNode
    msg_module.String = SimpleNamespace
    interfaces.ParameterDescriptor = SimpleNamespace
    interfaces.SetParametersResult = SimpleNamespace
    modules = {'rclpy': rclpy, 'rclpy.node': node_module,
               'std_msgs': ModuleType('std_msgs'), 'std_msgs.msg': msg_module,
               'rcl_interfaces': ModuleType('rcl_interfaces'), 'rcl_interfaces.msg': interfaces}
    path = Path(__file__).resolve().parents[1] / 'autorace_perception' / 'class_command_node.py'
    spec = importlib.util.spec_from_file_location('command_node_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module, rclpy


class ClassCommandNodeTests(unittest.TestCase):
    def setUp(self):
        self.module, self.rclpy = load_node()
        self.clock = patch.object(self.module.time, 'monotonic', return_value=100.0).start()
        self.addCleanup(patch.stopall)
        self.node = self.module.ClassCommandNode()
        self.node.router.configure(0.5, 'none', 1, 1)
        self.mission('M1_TRAFFIC')

    def mission(self, mode, fixes=0, line=None):
        self.node._on_mission(SimpleNamespace(data=json.dumps({
            'state': mode, 'route': {'fixes': fixes, 'last_fix': {'name': line}}})))

    def send(self, stamp, cls):
        self.node._on_detections(SimpleNamespace(data=json.dumps(
            {'stamp': stamp, 'detections': [{'cls': cls, 'conf': 0.9}]})))
        return json.loads(self.node.publisher.publish.call_args.args[0].data)

    def test_red_to_green_does_not_shutdown(self):
        self.assertEqual(self.node.input_topic, '/perception/detections')
        self.assertEqual(self.node.output_topic, '/perception/commands')
        self.assertTrue(self.send(1, 'red')['traffic_stop_active'])
        self.assertFalse(self.send(2, 'green')['traffic_stop_active'])
        self.rclpy.shutdown.assert_not_called()

    def test_unknown_mode_and_malformed_mission_disable_output(self):
        self.mission('WAIT_START')
        self.assertEqual(self.send(1, 'red')['commands'], [])
        for data in ('[]', '{}', 'null', 'broken'):
            self.node._on_mission(SimpleNamespace(data=data))
            self.assertEqual(self.node.router.mode, '')
        self.assertEqual(self.send(2, 'parking')['commands'], [])

    def test_mode_heartbeat_timeout_blocks_commands_and_requires_new_votes(self):
        self.assertTrue(self.send(1, 'red')['traffic_stop_active'])
        self.clock.return_value = 102.0
        self.assertEqual(self.send(2, 'red')['commands'], [])
        self.mission('M1_TRAFFIC')
        self.assertTrue(self.send(3, 'red')['traffic_stop_active'])

    def test_ground_uses_new_route_match_not_cached_last_fix(self):
        self.mission('M5_BUMP', fixes=20, line='m5_slow_line')
        self.assertEqual(self.send(1, 'slow')['commands'], [])
        self.mission('M5_BUMP', fixes=21, line='m5_slow_line')
        self.assertEqual(self.send(2, 'slow')['commands'][0]['command'], 'SLOW')
        self.clock.return_value = 102.0
        self.mission('M5_BUMP', fixes=21, line='m5_slow_line')
        self.assertEqual(self.send(3, 'slow')['commands'], [])
        self.mission('M5_BUMP', fixes=22, line='m5_slow_line')
        self.assertEqual(self.send(4, 'slow')['commands'][0]['command'], 'SLOW')

    def test_other_line_route_reset_and_missing_route_remove_ground_evidence(self):
        self.mission('M5_BUMP', fixes=1)
        self.mission('M5_BUMP', fixes=2, line='m5_slow_line')
        self.mission('M5_BUMP', fixes=3, line='m3_stop_line')
        self.assertEqual(self.send(1, 'slow')['commands'], [])
        self.mission('M5_BUMP', fixes=4, line='m5_slow_line')
        self.mission('M5_BUMP', fixes=0, line='m5_slow_line')
        self.assertEqual(self.send(2, 'slow')['commands'], [])
        self.mission('M5_BUMP', fixes=1, line='m5_slow_line')
        self.node._on_mission(SimpleNamespace(data='{"state":"M5_BUMP","route":null}'))
        self.assertEqual(self.send(3, 'slow')['commands'], [])

    def test_mode_change_drops_previous_ground_evidence(self):
        self.mission('M5_BUMP', fixes=1)
        self.mission('M5_BUMP', fixes=2, line='m5_slow_line')
        self.mission('M6_ZIGZAG', fixes=2, line='m5_slow_line')
        self.mission('M5_BUMP', fixes=2, line='m5_slow_line')
        self.assertEqual(self.send(1, 'slow')['commands'], [])

    def test_malformed_detections_do_not_publish_or_kill_callback(self):
        for data in ('broken', '[]', 'null', '{"stamp":1,"detections":{}}'):
            self.node._on_detections(SimpleNamespace(data=data))
        self.node.publisher.publish.assert_not_called()
        self.assertTrue(self.send(1, 'red')['traffic_stop_active'])

    def test_duplicate_is_dropped_and_clock_reset_recovers_without_restart(self):
        self.send(100, 'red')
        self.send(100, 'green')
        self.assertEqual(self.node.publisher.publish.call_count, 1)
        self.send(1, 'green')
        self.assertIn('votes reset', self.node.logger.warning.call_args.args[0])
        self.assertEqual(self.node.dropped_frames, 2)
        self.assertTrue(self.node.router.traffic_stop_active)
        self.assertFalse(self.send(2, 'green')['traffic_stop_active'])

    def test_default_node_waits_for_ten_frames(self):
        self.node = self.module.ClassCommandNode()
        self.mission('M4_PARKING')
        for stamp in range(1, 10):
            self.assertEqual(self.send(stamp, 'parking')['commands'], [])
        self.assertEqual(self.send(10, 'parking')['commands'][0]['command'], 'START_PARKING')

    def test_runtime_vote_height_yellow_and_atomic_rejection(self):
        params = {'vote_window': 4, 'vote_need': 3, 'yellow_action': 'pause',
                  'min_bbox_height_px.slow': 50.0}
        result = self.node.parameter_callback([SimpleNamespace(name=k, value=v) for k, v in params.items()])
        self.assertTrue(result.successful)
        self.assertEqual(self.node.router.min_heights['slow'], 50)
        for stamp in range(1, 5):
            out = self.send(stamp, 'yellow')
        self.assertTrue(out['traffic_stop_active'])
        for values in ({'conf_threshold': 0.8, 'vote_need': 5},
                       {'conf_threshold': 0.8, 'yellow_action': 'invalid'}):
            result = self.node.parameter_callback([SimpleNamespace(name=k, value=v) for k, v in values.items()])
            self.assertFalse(result.successful)
            self.assertEqual(self.node.router.conf_threshold, 0.5)


if __name__ == '__main__':
    unittest.main()
