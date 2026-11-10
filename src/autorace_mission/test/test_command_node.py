"""실제 MissionNode 콜백→Inputs→FSM 연결 검증. ROS 통신만 모의한다."""
import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/autorace_mission'))
from autorace_mission.command_inbox import CommandInbox
from autorace_mission.fsm import MissionFSM


def load_node():
    modules = {}
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs.msg', 'nav_msgs.msg',
                 'rcl_interfaces.msg', 'sensor_msgs.msg', 'std_msgs.msg'):
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = object
    modules['rclpy.qos'].qos_profile_sensor_data = None
    modules['rclpy.qos'].QoSProfile = SimpleNamespace
    modules['rclpy.qos'].DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
    modules['rclpy.qos'].ReliabilityPolicy = SimpleNamespace(RELIABLE=1)
    for module, names in (('geometry_msgs.msg', ['Twist']), ('nav_msgs.msg', ['Odometry']),
                          ('rcl_interfaces.msg', ['ParameterDescriptor']), ('sensor_msgs.msg', ['Imu']),
                          ('std_msgs.msg', ['Bool', 'Empty', 'Float32', 'String'])):
        for name in names:
            setattr(modules[module], name, SimpleNamespace)
    path = ROOT / 'src/autorace_mission/autorace_mission/mission_node.py'
    spec = importlib.util.spec_from_file_location('mission_node_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


class CommandNodeTests(unittest.TestCase):
    def node(self, mode):
        module = load_node()
        node = module.MissionNode.__new__(module.MissionNode)
        node.fsm = MissionFSM({'start_state': mode})
        node.command_inbox = CommandInbox()
        node.use_class_commands = True
        node.route_active = False
        node.dist_mm, node.yaw, node.det = 0, None, {}
        node.new_det, node.start_flag = False, False
        node.lane, node.obstacle, node.parking = {}, {}, {}
        node.get_logger = Mock(return_value=Mock())
        return node

    def send(self, node, command='START_PARKING', request='same', new=False):
        node._on_commands(SimpleNamespace(data=json.dumps({
            'schema_version': 2, 'mode': node.fsm.state,
            'commands': [{'command': command, 'request_id': request, 'new_request': new}]})))
        return node.fsm.step(node._inputs())

    def test_actual_callback_repeats_never_reenter_parking(self):
        node = self.node('M4_PARKING')
        self.assertEqual(self.send(node).source, 'parking')
        started = (node.fsm.sub_t, node.fsm.sub_dist)
        for n in range(100):
            node.dist_mm = n
            # 재전송에 new_request=true가 있어도 진행 중인 동작은 재시작하지 않는다.
            self.assertEqual(self.send(node, request=str(n), new=True).source, 'parking')
            self.assertEqual((node.fsm.sub_t, node.fsm.sub_dist), started)
        node.parking = {'done': True}
        self.send(node)
        self.assertEqual(node.fsm.state, 'FINISHED')
        self.assertEqual(self.send(node).source, 'stop')

    def test_actual_callback_repeated_green_and_missing_input_preserve_go(self):
        node = self.node('M1_TRAFFIC')
        self.assertEqual(self.send(node, 'RESUME_NORMAL').source, 'lane')
        started = node.fsm.sub_t
        for n in range(100):
            node.dist_mm = n
            self.assertEqual(self.send(node, 'RESUME_NORMAL').source, 'lane')
            self.assertEqual(node.fsm.sub_t, started)
        node._on_commands(SimpleNamespace(data=json.dumps({
            'schema_version': 2, 'mode': 'M1_TRAFFIC', 'commands': []})))
        self.assertEqual(node.fsm.step(node._inputs()).source, 'lane')
        self.assertEqual(node.fsm.sub_t, started)


class M7ExitLaneTests(unittest.TestCase):
    def test_m2_gets_exit_lane_from_topic_when_obstacle_state_lacks_it(self):
        module = load_node()
        node = module.MissionNode.__new__(module.MissionNode)
        node.fsm = MissionFSM({'start_state': 'M2_FORK'})
        node.obstacle = {}
        node.m7_exit_lane = {'lane': 3, 'fork': 'B'}
        self.assertEqual(node._obstacle_input()['m7_exit'], 'B')
        node.obstacle = {'m7_exit': 'A'}                 # /obstacle/state 값이 있으면 그대로
        self.assertEqual(node._obstacle_input()['m7_exit'], 'A')
        node.fsm = MissionFSM({'start_state': 'M5_BUMP'})
        node.obstacle = {}
        self.assertNotIn('m7_exit', node._obstacle_input())


if __name__ == '__main__':
    unittest.main()
