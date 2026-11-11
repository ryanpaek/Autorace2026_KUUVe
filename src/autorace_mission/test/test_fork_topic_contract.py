"""갈림길 표지 토픽 형식 계약 (ROS 불필요, 11/11).

detector_node가 내는 /perception/detections JSON(yolo_onnx._make와 같은 dict) 한 줄을
  ① class_command_node(ClassCommandRouter) -> /perception/commands JSON -> CommandInbox -> FSM fork_choice
  ② obstacle_node -> M7Link.on_detections (M7 끄기: 표지 확인·화면 옆으로 나감)
  ③ autorace_fork.parse_detections (use_fork_node:=true일 때)
가 모두 같은 뜻으로 읽는지 본다. JSON 직렬화를 거쳐 실제 토픽과 같게 한다."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'src' / p) for p in ('autorace_mission', 'autorace_perception',
                                                  'autorace_obstacle', 'autorace_fork')]
from autorace_perception.class_commands import ClassCommandRouter       # noqa: E402
from autorace_perception.yolo_onnx import _make                          # noqa: E402
from autorace_mission.command_inbox import CommandInbox                  # noqa: E402
from autorace_mission.fsm import Inputs, MissionFSM                      # noqa: E402
from autorace_obstacle.m7_link import M7Link, M7LinkParams               # noqa: E402
from autorace_fork.fork_core import parse_detections                     # noqa: E402

import yaml                                                              # noqa: E402

PARAMS_NAMES = yaml.safe_load((ROOT / 'src/autorace_bringup/config/params.yaml').read_text())[
    'detector_node']['ros__parameters']['class_names']                  # 실제 launch가 쓰는 순서


def detector_msg(stamp, cls_id, x, w=40, h=50):
    """detector_node._on_timer(onnx)와 같은 메시지 문자열."""
    d = _make(cls_id, 0.83, x, 200, w, h, (480, 640), PARAMS_NAMES)
    return json.dumps({'stamp': stamp, 'infer_ms': 120.0, 'detections': [d]})


def test_params_class_order_maps_fork_ids():
    assert PARAMS_NAMES[3] == 'fork_a' and PARAMS_NAMES[4] == 'fork_b'
    assert json.loads(detector_msg(1.0, 4, 300))['detections'][0]['cls'] == 'fork_b'


def test_detector_to_fsm_fork_choice_in_m7_with_mode_vote():
    router = ClassCommandRouter(vote_window=10, vote_need=7)
    need = dict.fromkeys(router.mode_need, 0)
    need.update({'M7_OBSTACLE': 7, 'M2_FORK': 7})                     # params.yaml mode_vote_need
    router.configure(0.5, 'none', 10, 7, mode_need=need)
    router.set_mode('M7_OBSTACLE')
    inbox = CommandInbox()
    fsm = MissionFSM({'start_state': 'M7_OBSTACLE'})
    got = []
    for k in range(10):
        msg = detector_msg(1.0 + 0.2 * k, 4, 300)                     # fork_b
        out = router.route(json.loads(msg))
        packet = json.loads(json.dumps(out, allow_nan=False))        # /perception/commands 그대로
        inbox.receive(packet, fsm.state)
        fsm.step(Inputs(t=0.2 * k, dist_mm=10.0 * k, confirmed_classes=dict(inbox.classes)))
        got.append(fsm.fork_choice)
    assert got[:9] == [None] * 9 and got[9] == 'B'                   # 10프레임 다 차야 확정
    cmd = [c for c in packet['commands'] if c['command'] == 'SELECT_ROUTE']
    assert cmd and cmd[0]['route'] == 'B' and packet['mode'] == 'M7_OBSTACLE'


def test_detector_bbox_reaches_m7_off_sign_edge_logic():
    """M7Link는 bbox를 [x, y, w, h]로 읽어 화면 옆(640 px)에 닿았는지 본다."""
    link = M7Link(M7LinkParams(off_mode='sign', off_sign_edge_px=10.0))
    link.in_m7 = True
    for k in range(3):
        link.on_detections(json.loads(detector_msg(1.0 + k, 3, 300))['detections'])
    assert link.sign_seen and not link.sign_edge
    link.on_detections(json.loads(detector_msg(5.0, 3, 610, w=60))['detections'])   # 오른쪽 끝에서 잘림
    assert link.sign_edge


def test_detector_bbox_height_in_fork_node():
    msg = detector_msg(1.0, 3, 300, h=50)
    assert parse_detections(msg, min_box_h=45.0) == {'A'}
    assert parse_detections(msg, min_box_h=55.0) == set()
