"""11/20 M1 신호등 출발 안 함: 차선 노드 쪽 동작 확인 (차선 코드는 안 고침, 미션 노드 m1_go_heading_mm의 근거).

서 있는 동안 선이 안 보이면 lost_frames가 쌓여 source=stop -> 속도를 줘도(normal 그대로) 계속 stop.
차선 모드를 heading으로 바꾸면 초기화되고 방향 유지로 속도가 나간다.
"""

import synth
from autorace_lane.lane_core import LaneFollower


def _step(f, cfg, speed, yaw=0.5):
    b = synth.blank(cfg)
    return f.process_bev(b, b.copy(), yaw=yaw, speed=speed)


def test_lost_while_stopped_then_normal_stays_stopped():
    cfg = synth.make_cfg()
    f = LaneFollower(cfg)
    for _ in range(f.p.lost_hold_frames + 5):         # 정지선에서 서서 기다림 (선 안 보임)
        out = _step(f, cfg, 0.0)
    out = _step(f, cfg, 0.3)                           # 초록: 미션은 normal 0.3을 줌
    assert out.source == 'stop' and out.speed_scale == 0.0


def test_switch_to_heading_resets_and_moves_then_normal_holds():
    cfg = synth.make_cfg()
    f = LaneFollower(cfg)
    for _ in range(f.p.lost_hold_frames + 5):
        _step(f, cfg, 0.0)
    f.set_mode('heading', yaw=0.5, yaw_ref=0.5)
    out = _step(f, cfg, 0.3)
    assert out.source == 'heading' and out.speed_scale > 0.0
    f.set_mode('normal', yaw=0.5)                      # 30 cm 뒤 차선으로: 다시 초기화 -> 바로 stop 아님
    out = _step(f, cfg, 0.3)
    assert out.source != 'stop' and out.speed_scale > 0.0
