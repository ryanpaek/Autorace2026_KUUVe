"""11/20 M1 신호등: 정지선에 섰다가 초록에 출발할 때 처음 m1_go_heading_mm는 차선 모드 heading.

실차 증상(캡틴): 빨강·노랑에 정지선에서 잘 서고 초록도 검출되는데 출발을 안 함.
코드 추정 원인: 서 있는 동안 차선 노드가 '선 놓침'을 세다가 정지(source=stop)에 걸리고, 차선 모드가 normal 그대로라 안 풀림.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.fsm import Inputs, MissionFSM  # noqa: E402


def _run(params, yaw=0.7, arrival_cls=None):
    """도면 정지선 3000 mm. 빨강으로 서서 3초 기다린 뒤 초록. 차는 지시 속도대로 움직인다."""
    fsm = MissionFSM({'start_state': 'M1_TRAFFIC', 'stop_brake_mm': 0.0,
                      'route_points': {'m1_stop_line': 3000.0}, **params})
    t, d, cmd, stop_t = 0.0, 2000.0, None, None
    log = []
    for _ in range(2000):
        v = 0.0 if cmd is None or cmd.source == 'stop' else cmd.lane.get('speed', 0.0)
        t += 0.05
        d += v * 1000.0 * 0.05
        if fsm.sub == 'stopped' and stop_t is None:
            stop_t = t
        if arrival_cls is not None:
            cls = arrival_cls
        else:
            cls = {'green': 1} if stop_t is not None and t - stop_t > 3.0 else {'red': 1}
        cmd = fsm.step(Inputs(t=t, dist_mm=d, route_s=d, yaw=yaw, confirmed_classes=cls))
        log.append((fsm.state, fsm.sub, d, cmd.source, dict(cmd.lane)))
        if fsm.state != 'M1_TRAFFIC':
            break
    return fsm, log


def test_go_from_stop_uses_heading_then_lane():
    fsm, log = _run({})
    assert fsm.state == 'M4_PARKING'
    go = [(d, lane) for st, sub, d, src, lane in log if st == 'M1_TRAFFIC' and sub == 'go']
    d0 = go[0][0]
    assert go[0][1]['mode'] == 'heading' and go[0][1]['yaw_ref'] == 0.7
    assert go[0][1]['speed'] > 0.0
    # 처음 300 mm는 heading, 그 뒤는 normal (차선)
    assert all(lane['mode'] == 'heading' for d, lane in go if d - d0 < 300.0 - 1.0)
    assert any(lane['mode'] == 'normal' for d, lane in go if d - d0 >= 300.0)
    assert 'M1 go heading done -> lane' in fsm.log


def test_go_heading_off_keeps_old_behavior():
    fsm, log = _run({'m1_go_heading_mm': 0.0})
    assert fsm.state == 'M4_PARKING'
    go = [lane for st, sub, d, src, lane in log if st == 'M1_TRAFFIC' and sub == 'go']
    assert go and all(lane['mode'] == 'normal' for lane in go)


def test_go_heading_without_yaw_lets_lane_node_pick():
    fsm, log = _run({}, yaw=None)
    assert fsm.state == 'M4_PARKING'
    go = [lane for st, sub, d, src, lane in log if st == 'M1_TRAFFIC' and sub == 'go']
    assert go[0]['mode'] == 'heading' and 'yaw_ref' not in go[0]


def test_green_on_arrival_pass_stays_on_lane():
    """달리면서 초록으로 통과(서지 않음)하면 차선이 살아 있으니 heading 안 씀."""
    fsm, log = _run({'stop_on_green_arrival': False}, arrival_cls={'green': 1})
    assert fsm.state == 'M4_PARKING'
    assert all(src != 'stop' for st, sub, d, src, lane in log)
    go = [lane for st, sub, d, src, lane in log if st == 'M1_TRAFFIC' and sub == 'go']
    assert go and all(lane['mode'] == 'normal' for lane in go)
