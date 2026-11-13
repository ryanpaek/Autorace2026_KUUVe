"""실제 구간별 명령 규약. ROS 없이 unittest로 실행."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autorace_perception.class_commands import (  # noqa: E402
    ClassCommandRouter, MODE_CLASSES, TRAINING_LABELS, V3_CLASSES, canonical_class,
)


def frame(stamp, *classes, confidence=0.9, height=None):
    detections = [{'cls': cls, 'conf': confidence} for cls in classes]
    if height is not None:
        for detection in detections:
            detection['bbox'] = [0, 0, 50, height]
    return {'stamp': stamp, 'detections': detections}


def router(mode='M1_TRAFFIC', window=1, need=1, **kwargs):
    result = ClassCommandRouter(vote_window=window, vote_need=need, **kwargs)
    result.set_mode(mode)
    return result


def commands(output):
    return [c['command'] for c in output['commands']]


class ClassCommandTests(unittest.TestCase):
    def test_all_class_mappings_in_allowed_modes(self):
        expected = {'red': 'PAUSE_UNTIL_GREEN', 'green': 'RESUME_NORMAL',
                    'slow': 'SLOW', 'bump': 'CROSS_BUMP_SLOWLY',
                    'pedestrian': 'STOP_AT_PEDESTRIAN_LINE', 'parking': 'START_PARKING',
                    'fork_a': 'SELECT_ROUTE', 'fork_b': 'SELECT_ROUTE'}
        for mode, classes in MODE_CLASSES.items():
            for cls in classes:
                with self.subTest(mode=mode, cls=cls):
                    output = router(mode).route(frame(1, cls), slow_ground=True)
                    self.assertEqual(commands(output), [] if cls == 'yellow' else [expected[cls]])

    def test_off_mode_objects_cannot_accumulate_votes_or_commands(self):
        for mode, allowed in MODE_CLASSES.items():
            r = router(mode)
            for stamp in range(1, 15):
                output = r.route(frame(stamp, *(c for c in V3_CLASSES if c not in allowed)),
                                 slow_ground=True)
                self.assertEqual(commands(output), [])
                self.assertEqual(output['confirmed_classes'], [])

    def test_missing_unknown_and_inactive_modes_disable_commands(self):
        for mode in ('', 'M9', 'WAIT_START', 'FINISHED', 'LANE_ONLY'):
            self.assertEqual(commands(router(mode).route(frame(1, *V3_CLASSES))), [])
        self.assertEqual(commands(ClassCommandRouter().route(frame(1, 'red'))), [])

    def test_mode_change_clears_stop_and_old_votes(self):
        r = router()
        r.route(frame(1, 'red'))
        r.set_mode('M4_PARKING')
        self.assertFalse(r.traffic_stop_active)
        self.assertEqual(commands(r.route(frame(2, 'red', 'parking'))), ['START_PARKING'])
        r.set_mode('M1_TRAFFIC')
        self.assertEqual(commands(r.route(frame(3))), [])

    def test_repeated_mode_heartbeat_does_not_reset_votes(self):
        r = router(window=10, need=7)
        for stamp in range(10):
            r.set_mode('M1_TRAFFIC')
            out = r.route(frame(stamp, 'red'))
        self.assertTrue(out['traffic_stop_active'])

    def test_training_labels_are_normalized(self):
        for raw, cls in zip(TRAINING_LABELS, V3_CLASSES):
            self.assertEqual(canonical_class(raw), cls)
            mode = next(m for m, classes in MODE_CLASSES.items() if cls in classes)
            left = router(mode).route(frame(1, raw), slow_ground=True)
            right = router(mode).route(frame(1, cls), slow_ground=True)
            for output in (left, right):
                for command in output['commands']:
                    command.pop('request_id')
            self.assertEqual(left, right)

    def test_red_waits_at_line_through_missing_and_yellow_until_green(self):
        r = router()
        first = r.route(frame(1, 'red'))['commands'][0]
        self.assertEqual(first['stop_target'], 'before_stop_line')
        for stamp, classes in ((2, ()), (3, ('yellow',))):
            out = r.route(frame(stamp, *classes))
            self.assertTrue(out['traffic_stop_active'])
            self.assertTrue(out['commands'][0]['held'])
        self.assertEqual(commands(r.route(frame(4, 'green'))), ['RESUME_NORMAL'])
        self.assertEqual(commands(r.route(frame(5))), [])  # 미검출은 정지 요청이 아님

    def test_red_wins_over_simultaneous_green(self):
        self.assertEqual(commands(router().route(frame(1, 'red', 'green'))), ['PAUSE_UNTIL_GREEN'])

    def test_yellow_default_tracks_detection_without_command(self):
        out = router().route(frame(1, 'yellow'))
        self.assertEqual(out['confirmed_classes'], ['yellow'])
        self.assertEqual(commands(out), [])

    def test_yellow_pause_is_optional_and_disabling_does_not_release_stop(self):
        r = router(yellow_action='pause')
        self.assertTrue(r.route(frame(1, 'yellow'))['traffic_stop_active'])
        r.configure(0.5, 'none')
        self.assertTrue(r.route(frame(2))['traffic_stop_active'])
        self.assertFalse(r.route(frame(3, 'green'))['traffic_stop_active'])

    def test_slow_requires_current_sign_and_ground_confirmation(self):
        r = router('M5_BUMP', window=3, need=2)
        for stamp in range(3):
            self.assertEqual(commands(r.route(frame(stamp, 'slow'))), [])
        self.assertEqual(commands(r.route(frame(3, 'slow'), slow_ground=True)), ['SLOW'])
        self.assertEqual(commands(r.route(frame(4), slow_ground=True)), [])
        self.assertEqual(commands(r.route(frame(5, 'slow'), slow_ground=False)), [])

    def test_bump_does_not_require_slow_sign_or_ground(self):
        self.assertEqual(commands(router('M5_BUMP').route(frame(1, 'bump'))), ['CROSS_BUMP_SLOWLY'])

    def test_pedestrian_policy_and_route_choice_delegate_execution(self):
        for mode in ('M1_TRAFFIC', 'M3_PEDESTRIAN'):
            cmd = router(mode).route(frame(1, 'pedestrian'))['commands'][0]
            self.assertEqual(cmd['traffic_policy'], 'obey_if_present')
            self.assertNotIn('stop_margin_mm', cmd)
        for cls, choice in (('fork_a', 'A'), ('fork_b', 'B')):
            cmd = router('M2_FORK').route(frame(1, cls))['commands'][0]
            self.assertEqual(cmd['route'], choice)
            self.assertNotIn('angle', cmd)

    def test_fork_conflict_holds_choice(self):
        out = router('M7_OBSTACLE').route(frame(1, 'fork_a', 'fork_b'))
        self.assertEqual(out['conflicts'], ['fork_direction'])
        self.assertEqual(commands(out), [])

    def test_repeated_parking_is_same_level_request_without_timing(self):
        r = router('M4_PARKING')
        for stamp in range(3):
            cmd = r.route(frame(stamp, 'parking'))['commands'][0]
            self.assertEqual(cmd['command'], 'START_PARKING')
            self.assertFalse({'speed', 'distance', 'duration', 'restart'} & cmd.keys())

    def test_parking_request_id_survives_disappearance_and_tuning(self):
        r = router('M4_PARKING')
        first = r.route(frame(1, 'parking'))['commands'][0]
        self.assertTrue(first['new_request'])
        r.route(frame(2))
        r.configure(0.6, 'none')
        repeated = r.route(frame(3, 'parking'))['commands'][0]
        self.assertFalse(repeated['new_request'])
        self.assertEqual(first['request_id'], repeated['request_id'])
        r.set_mode('M1_TRAFFIC')
        r.set_mode('M4_PARKING')
        fresh = r.route(frame(4, 'parking'))['commands'][0]
        self.assertTrue(fresh['new_request'])
        self.assertNotEqual(first['request_id'], fresh['request_id'])

    def test_green_is_held_request_until_new_red_episode(self):
        r = router()
        first = r.route(frame(1, 'green'))['commands'][0]
        for stamp in (2, 3):
            repeated = r.route(frame(stamp, 'green'))['commands'][0]
            self.assertFalse(repeated['new_request'])
            self.assertEqual(first['request_id'], repeated['request_id'])
        r.route(frame(4))
        self.assertFalse(r.route(frame(5, 'green'))['commands'][0]['new_request'])
        r.route(frame(6, 'red'))
        released = r.route(frame(7, 'green'))['commands'][0]
        self.assertTrue(released['new_request'])
        self.assertNotEqual(first['request_id'], released['request_id'])

    def test_minimum_height_filters_before_voting_and_keeps_larger_box(self):
        r = router('M5_BUMP')
        heights = dict(r.min_heights, slow=50.0)
        r.configure(0.5, 'none', min_heights=heights)
        for h in (None, 49):
            self.assertEqual(commands(r.route(frame(1 if h is None else 2, 'slow', height=h),
                                              slow_ground=True)), [])
        sample = frame(3, 'slow', height=50, confidence=0.7)
        sample['detections'].append({'cls': 'slow', 'conf': 0.99, 'bbox': [0, 0, 1, 1]})
        cmd = r.route(sample, slow_ground=True)['commands'][0]
        self.assertEqual(cmd['bbox'][3], 50)
        self.assertEqual(cmd['confidence'], 0.7)

    def test_invalid_boxes_fail_closed_only_when_size_filter_enabled(self):
        for bbox in (None, [], [0, 0, -1, 3], [0, 0, 1, float('nan')]):
            sample = frame(1, 'parking')
            sample['detections'][0]['bbox'] = bbox
            self.assertNotIn('bbox', router('M4_PARKING').route(sample)['commands'][0])
            r = router('M4_PARKING')
            r.configure(0.5, 'none', min_heights=dict(r.min_heights, parking=1.0))
            self.assertEqual(commands(r.route(sample)), [])

    def test_bad_frames_and_entries_do_not_crash_or_advance_state(self):
        r = router()
        for sample in (None, [], {}, {'stamp': 1, 'detections': {}},
                       *[frame(t, 'red') for t in (True, -1, float('nan'), float('inf'), 10**400)]):
            with self.assertRaises(ValueError):
                r.route(sample)
        sample = frame(1)
        sample['detections'] = [None, [], {'cls': []}, {'cls': 'unknown', 'conf': 1},
                               *[{'cls': 'green', 'conf': x} for x in
                                 (True, None, '0.9', float('nan'), 10**400, -1, 1.1, 0.49)]]
        self.assertEqual(commands(r.route(sample)), [])

    def test_invalid_parameters_are_atomic(self):
        r = router()
        for window, need in ((0, 0), (10, 11), (10, 0), (True, 1), (10, 7.0)):
            with self.assertRaises(ValueError):
                r.configure(0.8, 'none', window, need)
            self.assertEqual(r.conf_threshold, 0.5)
        for height in (-1, float('nan'), True, '50'):
            with self.assertRaises(ValueError):
                r.configure(0.8, 'none', min_heights=dict(r.min_heights, red=height))
            self.assertEqual(r.conf_threshold, 0.5)


class TemporalVoteTests(unittest.TestCase):
    def test_full_window_then_seven_nonconsecutive_hits(self):
        r = router('M4_PARKING', window=10, need=7)
        for stamp in range(1, 11):
            out = r.route(frame(stamp, *('parking',) if stamp not in (3, 6, 9) else ()))
            if stamp < 10:
                self.assertEqual(commands(out), [])
        self.assertEqual(out['vote']['counts']['parking'], 7)
        self.assertEqual(commands(out), ['START_PARKING'])

    def test_six_hits_and_single_false_positive_do_not_trigger(self):
        for hits in (1, 6):
            r = router(window=10, need=7)
            for stamp in range(1, 15):
                out = r.route(frame(stamp, *('red',) if stamp <= hits else ()))
                self.assertEqual(commands(out), [])

    def test_duplicates_and_many_boxes_do_not_multiply_votes(self):
        r = router(window=10, need=7)
        r.route(frame(1, *(['red'] * 20)))
        for _ in range(20):
            self.assertIsNone(r.route(frame(1, 'red')))
        for stamp in range(2, 11):
            out = r.route(frame(stamp))
        self.assertEqual(out['vote']['counts']['red'], 1)
        self.assertEqual(commands(out), [])

    def test_only_confident_detections_count(self):
        r = router(window=10, need=7)
        for stamp in range(10):
            out = r.route(frame(stamp, 'red', confidence=0.5 if stamp < 6 else 0.49))
        self.assertEqual(out['vote']['counts']['red'], 6)
        self.assertEqual(commands(out), [])

    def test_green_votes_before_red_cannot_release_new_stop(self):
        r = router(window=10, need=7)
        for stamp in range(1, 11):
            r.route(frame(stamp, 'green'))
        for stamp in range(11, 18):
            out = r.route(frame(stamp, 'red', 'green'))
        self.assertTrue(out['traffic_stop_active'])
        self.assertEqual(out['vote']['counts']['green'], 0)
        for stamp in range(18, 24):
            self.assertTrue(r.route(frame(stamp, 'green'))['traffic_stop_active'])
        self.assertFalse(r.route(frame(24, 'green'))['traffic_stop_active'])

    def test_backwards_clock_recollects_votes_keeps_stop_and_recovers(self):
        r = router(window=10, need=7)
        for stamp in range(100, 110):
            r.route(frame(stamp, 'red'))
        self.assertIsNone(r.route(frame(0, 'green')))
        self.assertTrue(r.traffic_stop_active)
        for stamp in range(1, 10):
            self.assertTrue(r.route(frame(stamp, 'green'))['traffic_stop_active'])
        self.assertFalse(r.route(frame(10, 'green'))['traffic_stop_active'])
        self.assertEqual(commands(r.route(frame(11))), [])

    def test_runtime_vote_and_height_change_clear_votes_but_keep_stop(self):
        r = router()
        r.route(frame(1, 'red'))
        r.configure(0.8, 'none', 4, 3, dict(r.min_heights, green=20.0))
        for stamp in range(2, 5):
            self.assertTrue(r.route(frame(stamp, 'green', height=25))['traffic_stop_active'])
        self.assertFalse(r.route(frame(5, 'green', height=25))['traffic_stop_active'])

    def test_old_votes_expire_but_no_automatic_resume_on_disappearance(self):
        r = router('M5_BUMP', window=10, need=7)
        for stamp in range(1, 11):
            r.route(frame(stamp, 'bump', height=50))
        for stamp in (11, 12, 13):
            cmd = r.route(frame(stamp))['commands'][0]
            self.assertTrue(cmd['held'])
            self.assertEqual(cmd['source_stamp'], 10)
        self.assertEqual(commands(r.route(frame(14))), [])


if __name__ == '__main__':
    unittest.main()


def test_fork_confirmed_early_without_full_window():
    """11/12 밤: 갈림길(M2·M7)은 10프레임 창이 다 차기 전에도 need(3)번 보면 바로 SELECT_ROUTE."""
    from autorace_perception.class_commands import ClassCommandRouter
    r = ClassCommandRouter(vote_window=10, vote_need=7)
    need = dict.fromkeys(r.mode_need, 0)
    need.update({'M2_FORK': 3, 'M7_OBSTACLE': 3})
    r.configure(0.5, 'none', 10, 7, mode_need=need)
    r.early_modes = {'M2_FORK', 'M7_OBSTACLE'}
    r.set_mode('M7_OBSTACLE')
    outs = [r.route({'stamp': float(k + 1), 'detections': [{'cls': 'fork_a', 'conf': 0.9}]}) for k in range(3)]
    assert not any(c['command'] == 'SELECT_ROUTE' for c in outs[1]['commands'])
    assert any(c['command'] == 'SELECT_ROUTE' and c['route'] == 'A' for c in outs[2]['commands'])
    r2 = ClassCommandRouter(vote_window=10, vote_need=3)       # early 없으면 예전처럼 창이 찰 때까지
    r2.set_mode('M1_TRAFFIC')
    outs = [r2.route({'stamp': float(k + 1), 'detections': [{'cls': 'red', 'conf': 0.9}]}) for k in range(3)]
    assert 'red' not in outs[2]['confirmed_classes']
