"""ROS 메시지 칸 타입 시험 (ROS 없이).

실제 ROS(rclpy) 메시지는 float 칸에 int를 넣으면 AssertionError를 낸다.
11/08 실차에서 make_marker(..., scale=(0.04, 0, 0))의 0 때문에 회피 노드가 출발 0.2초 만에 죽었다.
ROS가 없으면 같은 규칙을 지키는 흉내 메시지로 make_marker를 돌려 본다.
"""
import importlib
import sys
import types

import pytest


class _Strict:
    """지정한 칸에 float만 받는 메시지 흉내 (ROS 생성 코드와 같은 검사)."""
    _float_fields = ()

    def __setattr__(self, name, value):
        if name in self._float_fields:
            assert isinstance(value, float), f"The '{name}' field must be of type 'float'"
        object.__setattr__(self, name, value)


def _fake_ros_modules():
    class Vector3(_Strict):
        _float_fields = ('x', 'y', 'z')

    class ColorRGBA(_Strict):
        _float_fields = ('r', 'g', 'b', 'a')

    class Quaternion(_Strict):
        _float_fields = ('x', 'y', 'z', 'w')

    class Pose:
        def __init__(self):
            self.orientation = Quaternion()

    class Header:
        frame_id = ''
        stamp = None

    class Marker:
        ADD, LINE_STRIP, LINE_LIST, CUBE, DELETEALL = 0, 4, 5, 1, 3

        def __init__(self):
            self.header, self.pose = Header(), Pose()
            self.scale, self.color = Vector3(), ColorRGBA()
            self.points = []

    class Point(_Strict):
        _float_fields = ('x', 'y', 'z')

    mods = {}
    for name, attrs in {
        'rcl_interfaces': {}, 'rcl_interfaces.msg': {'SetParametersResult': object},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'Point': Point},
        'visualization_msgs': {}, 'visualization_msgs.msg': {'Marker': Marker},
    }.items():
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        mods[name] = m
    return mods


@pytest.fixture
def ros_common(monkeypatch):
    try:
        import rclpy  # noqa: F401  실제 ROS가 있으면 진짜 메시지로 시험
    except ImportError:
        for name, mod in _fake_ros_modules().items():
            monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.delitem(sys.modules, 'limo_obstacle_avoid.ros_common', raising=False)
    return importlib.import_module('limo_obstacle_avoid.ros_common')


def test_make_marker_accepts_int_scale_and_color(ros_common):
    m = ros_common.make_marker('odom', 'target', 0, 4, None, (0.1, 0.9, 0.2, 1), (0.04, 0, 0))
    assert (m.scale.x, m.scale.y, m.scale.z) == (0.04, 0.0, 0.0)
    assert m.color.a == 1.0


def test_marker_calls_in_nodes_do_not_crash(ros_common):
    """노드들이 실제로 쓰는 인자 모양 그대로."""
    for color, scale in [((0.1, 0.9, 0.2, 1.0), (0.04, 0, 0)),     # s5 target
                         ((1.0, 0.3, 0.1, 0.9), (0.03, 0, 0)),     # s5 obs, s0 island
                         ((0.8, 0.6, 0.1, 1.0), (0.02, 0.02, 0.02))]:
        ros_common.make_marker('odom', 'x', 0, 4, None, color, scale)
    p = ros_common.pt(1, 0)
    assert (p.x, p.y, p.z) == (1.0, 0.0, 0.0)
