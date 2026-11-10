"""YOLO 투표 → 명령 수신 → 실제 미션 FSM의 실행 지속 검증 (ROS 불필요)."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src/autorace_mission'), str(ROOT / 'src/autorace_perception')]
from autorace_perception.class_commands import ClassCommandRouter
from autorace_mission.command_inbox import CommandInbox
from autorace_mission.fsm import Inputs, MissionFSM


class Pipeline:
    def __init__(self, mode, **params):
        self.router = ClassCommandRouter()
        self.router.set_mode(mode)
        self.inbox = CommandInbox()
        self.fsm = MissionFSM(dict(start_state=mode, **params))
        self.stamp = 0

    def tick(self, cls=None, dist=0, ground=False, **kw):
        self.stamp += 1
        packet = self.router.route({'stamp': self.stamp, 'detections': [] if cls is None else [
            {'cls': cls, 'conf': 0.9}]}, slow_ground=ground)
        self.inbox.receive(packet, self.fsm.state)
        return self.fsm.step(Inputs(t=self.stamp * 0.2, dist_mm=dist,
                                   confirmed_classes=dict(self.inbox.classes), **kw))


class CommandIntegrationTests(unittest.TestCase):
    def test_parking_waits_for_vote_and_repeats_do_not_reset_execution(self):
        p = Pipeline('M4_PARKING')
        for _ in range(9):
            self.assertEqual(p.tick('parking').source, 'lane')
            self.assertEqual(p.fsm.sub, '')
        self.assertEqual(p.tick('parking', dist=30).source, 'parking')
        started = (p.fsm.sub_t, p.fsm.sub_dist)
        for dist in range(40, 140):
            self.assertEqual(p.tick('parking', dist=dist).source, 'parking')
            self.assertEqual((p.fsm.sub_t, p.fsm.sub_dist), started)
        # 표지가 사라져도 이미 실행 중인 주차를 취소하거나 다시 시작하지 않는다.
        for _ in range(12):
            self.assertEqual(p.tick(dist=140).source, 'parking')
        p.tick('parking', dist=150, parking={'done': True})
        self.assertEqual(p.fsm.state, 'FINISHED')
        for _ in range(12):
            self.assertEqual(p.tick('parking', dist=150).source, 'stop')
            self.assertEqual(p.fsm.state, 'FINISHED')

    def test_green_repeats_and_disappearance_do_not_reset_departure_distance(self):
        p = Pipeline('M1_TRAFFIC', leave_mm=600)
        for _ in range(10):
            p.tick('green')
        self.assertEqual(p.fsm.sub, 'go')
        started = (p.fsm.sub_t, p.fsm.sub_dist)
        for dist in range(1, 300):
            self.assertEqual(p.tick('green', dist=dist).source, 'lane')
            self.assertEqual((p.fsm.sub_t, p.fsm.sub_dist), started)
        for dist in range(300, 600):
            self.assertEqual(p.tick(dist=dist).source, 'lane')
            self.assertEqual((p.fsm.sub_t, p.fsm.sub_dist), started)
        p.tick(dist=600)
        self.assertEqual(p.fsm.state, 'M4_PARKING')

    def test_red_stop_and_release_are_not_rearmed_by_repeated_messages(self):
        p = Pipeline('M1_TRAFFIC', green_go_delay_sec=0, stop_brake_mm=0)
        for _ in range(9):
            p.tick('red')
            self.assertEqual(p.fsm.sub, '')
        p.tick('red', lane={'crosswalk_lines_mm': [0]})
        self.assertEqual(p.fsm.sub, 'stopped')
        stopped = (p.fsm.sub_t, id(p.fsm.stop))
        for _ in range(20):
            self.assertEqual(p.tick('red').source, 'stop')
            self.assertEqual((p.fsm.sub_t, id(p.fsm.stop)), stopped)
        for _ in range(6):
            self.assertEqual(p.tick('green').source, 'stop')
        self.assertEqual(p.tick('green').source, 'lane')
        self.assertEqual(p.fsm.sub, 'go')
        departed = p.fsm.sub_t
        for _ in range(20):
            p.tick('green')
            self.assertEqual(p.fsm.sub_t, departed)

    def test_slow_repeated_requests_do_not_reset_pass_distance(self):
        p = Pipeline('M5_BUMP', bump_slow_mm=300)
        for _ in range(9):
            self.assertEqual(p.tick('slow', ground=True).lane['speed'], p.fsm.p['base_speed'])
        self.assertEqual(p.tick('slow', dist=20, ground=True).lane['speed'], p.fsm.p['slow_speed'])
        started = (p.fsm.sub_t, p.fsm.sub_dist)
        for dist in range(21, 320):
            p.tick('slow', dist=dist, ground=True)
            self.assertEqual((p.fsm.sub_t, p.fsm.sub_dist), started)
        p.tick('slow', dist=320, ground=True)
        self.assertEqual(p.fsm.state, 'M6_ZIGZAG')

    def test_raw_detections_cannot_bypass_confirmed_command_path(self):
        fsm = MissionFSM({'start_state': 'M5_BUMP'})
        for stamp in range(30):
            result = fsm.step(Inputs(t=stamp, dist_mm=0, confirmed_classes={},
                                    detections={'slow': 1.0}, new_detection_frame=True))
            self.assertEqual(result.lane['speed'], fsm.p['base_speed'])
        self.assertEqual(fsm.sub, '')

    def test_first_start_packet_can_be_lost_without_losing_parking_request(self):
        p = Pipeline('M4_PARKING')
        for stamp in range(10):
            packet = p.router.route({'stamp': stamp, 'detections': [{'cls': 'parking', 'conf': 0.9}]})
        self.assertTrue(packet['commands'][0]['new_request'])
        held = p.router.route({'stamp': 10, 'detections': [{'cls': 'parking', 'conf': 0.9}]})
        self.assertFalse(held['commands'][0]['new_request'])
        p.inbox.receive(held, 'M4_PARKING')
        result = p.fsm.step(Inputs(t=2, dist_mm=0, confirmed_classes=p.inbox.classes))
        self.assertEqual(result.source, 'parking')

    def test_wrong_mode_and_old_requests_after_mode_transition_are_ignored(self):
        p = Pipeline('M4_PARKING')
        for _ in range(10):
            p.tick('parking')
        p.inbox.set_mode('FINISHED')
        self.assertFalse(p.inbox.receive({'schema_version': 2, 'mode': 'M4_PARKING',
                                         'commands': []}, 'FINISHED'))
        self.assertEqual(p.inbox.classes, {})


if __name__ == '__main__':
    unittest.main()
