"""ROS 없이 도는 갈림길 판단 시험.  python3 -m pytest -q test"""
import json

from autorace_fork.fork_core import ForkVote, parse_detections, parse_sign_text, plan


def det(*items):
    return json.dumps({'detections': [{'cls': c, 'conf': cf, 'bbox': [0, 0, 10, h]} for c, cf, h in items]})


def test_parse_detections():
    assert parse_detections(det(('fork_a', 0.9, 40))) == {'A'}
    assert parse_detections(det(('fork_b', 0.3, 40))) == set()               # 신뢰도 낮음
    assert parse_detections(det(('fork_b', 0.9, 10)), min_box_h=20) == set()  # 너무 작음(멀리)
    assert parse_detections(det(('red', 0.9, 40))) == set()
    assert parse_detections('깨진 json') == set()


def test_parse_text():
    assert parse_sign_text(' A ') == {'A'} and parse_sign_text('fork_b') == {'B'} and parse_sign_text('?') == set()


def test_vote_needs_majority():
    v = ForkVote(8, 5)
    for _ in range(4):
        assert v.push({'B'}) is None
    assert v.push({'B'}) == 'B'
    v.reset()
    for _ in range(5):
        v.push({'A', 'B'})                       # 둘 다 보이면 결정 안 함
    assert v.result() is None


def test_plan_lane_shift_counts():
    # 1차선(+1)에 있을 때 A 는 유지, B 는 오른쪽 2차선
    assert plan('A', {'now_lane': 1, 'target': 1})['need_lanes'] == 0
    s = plan('B', {'now_lane': 1, 'target': 1, 'exit_plan': 'go', 'exit_rem_m': 2.3, 'exit_need_m': 1.52})
    assert s['need_lanes'] == -2 and '2차선 오른쪽' in s['text'] and '이동 시작' in s['text']
    # 가운데(0)면 어느 쪽이든 1차선
    assert plan('A', {'now_lane': 0, 'target': 0})['need_lanes'] == 1
    assert plan('B', {'now_lane': 0, 'target': 0})['need_lanes'] == -1
    # 거리 부족
    assert '거리 부족' in plan('A', {'now_lane': -1, 'target': -1, 'exit_plan': 'too_late'})['text']
    # 회피 노드 상태 없음 / 결정 전
    assert plan('A', {})['need_lanes'] is None and plan(None, {'now_lane': 0})['text'] == '표지 판단 전'


def test_min_box_h_uses_xywh_like_detector():
    """detector_node bbox = [x, y, w, h]. 화면 아래쪽(y 큼)에 있어도 높이 h로 거른다."""
    from autorace_fork.fork_core import parse_detections
    det = json.dumps({'detections': [{'cls': 'fork_a', 'conf': 0.9, 'bbox': [100, 300, 40, 50]}]})
    assert parse_detections(det, min_box_h=40.0) == {'A'}
    assert parse_detections(det, min_box_h=60.0) == set()
