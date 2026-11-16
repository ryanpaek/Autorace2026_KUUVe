"""M7 재생 방식 시험 (ROS 없이): 핵심 로직 + 노드 입출력 (가짜 rclpy)."""
import importlib
import math
import os
import sys
import types

import pytest

from limo_obstacle_avoid.m7_replay_core import (M7Replay, ReplayParams, default_change, fit_lateral, load_change,
                                                mirror)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))


def _sim():
    return importlib.import_module('m7_replay_sim')


def test_fit_lateral_scales_end_to_lane_width():
    segs = default_change(1, lane_width=0.48)
    out, k = fit_lateral(segs, 0.52)
    assert out[-1].y[-1] == pytest.approx(0.52, abs=1e-6)
    assert k == pytest.approx(0.52 / 0.48, rel=1e-6)


def test_load_change_falls_back_to_mirror_then_default(tmp_path):
    segs, src = load_change(None, None, -1, 0.52)
    assert '기본' in src and segs[-1].y[-1] == pytest.approx(-0.52, abs=1e-6)
    assert mirror(default_change(1))[-1].y[-1] == pytest.approx(-0.52, abs=1e-6)


def test_choose_prefers_exit_side_when_both_free():
    p = ReplayParams(exit_lane='B')
    m = M7Replay(p, default_change(1), default_change(-1))
    m.start()
    m.pts = []
    d, why = m.choose(0, 1.2)
    assert d == -1 and '출구' in why


def test_choose_rejects_side_with_obstacle_on_path():
    p = ReplayParams(exit_lane='A')
    m = M7Replay(p, default_change(1), default_change(-1))
    m.start()
    # 왼쪽 차로 x 1.8 m(녹화 동작이 끝나는 곳)에 물체 -> 왼쪽 경로가 닿음, 오른쪽은 빔
    m.pts = [(1.8 + 0.01 * i, 0.52, 0, 0) for i in range(10)]
    d, why = m.choose(0, 1.2)
    assert d == -1


def test_island_is_virtual_obstacle_for_late_change():
    p = ReplayParams(section_length=1.0)
    m = M7Replay(p, default_change(1), default_change(-1))
    m.start()
    m.pts = []
    assert m.path_hit(1, 0.5) is not None      # 1.3 m 걸리는 변경을 섬 0.5 m 앞에서 시작 -> 섬 침범
    assert m.path_hit(1, -1.0) is None


@pytest.mark.parametrize('i,fork', [(0, 'A'), (1, 'B'), (3, 'A')])
def test_sim_layouts_pass(i, fork):
    S = _sim()
    import random
    random.seed(99)
    left = S.record_left()
    S.TRIGGER_MM = 700.0
    r = S.run(S.LAYOUTS[i], left, mirror(left), seed=0, fork=fork)
    assert r['ok'] and r['island'] == 0 and r['clear_cm'] > 1.0
    assert r['fork_ok']


# ---------------- 노드 (가짜 rclpy) ----------------
class _F:
    _floats = ()

    def __setattr__(self, k, v):
        if k in self._floats:
            assert isinstance(v, float), k
        object.__setattr__(self, k, v)


def _fake_ros():
    class V3(_F):
        _floats = ('x', 'y', 'z')

        def __init__(self):
            object.__setattr__(self, 'x', 0.0)
            object.__setattr__(self, 'y', 0.0)
            object.__setattr__(self, 'z', 0.0)

    class Twist:
        def __init__(self):
            self.linear, self.angular = V3(), V3()

    class Bool:
        def __init__(self, data=False):
            self.data = data

    class String:
        def __init__(self, data=''):
            self.data = data

    class Param:
        def __init__(self, v):
            self.value = v

    class Node:
        def __init__(self, name):
            self._p, self.subs, self.pubs, self.timers = {}, {}, {}, []

        def declare_parameter(self, k, v):
            self._p.setdefault(k, v)

        def get_parameter(self, k):
            return Param(self._p[k])

        def create_publisher(self, typ, topic, q):
            out = self.pubs.setdefault(topic, [])

            class P:
                def publish(self_, m):
                    out.append(m)
            return P()

        def create_subscription(self, typ, topic, cb, q):
            self.subs[topic] = cb

        def create_timer(self, dt, cb):
            self.timers.append(cb)

        def get_logger(self):
            class L:
                def info(self, *a, **k): pass
                def warn(self, *a, **k): pass
            return L()

    mods = {
        'rclpy': types.SimpleNamespace(init=lambda **k: None, ok=lambda: True, shutdown=lambda: None),
        'rclpy.node': types.SimpleNamespace(Node=Node),
        'rclpy.qos': types.SimpleNamespace(qos_profile_sensor_data=None),
        'geometry_msgs': types.ModuleType('geometry_msgs'),
        'geometry_msgs.msg': types.SimpleNamespace(Twist=Twist),
        'nav_msgs': types.ModuleType('nav_msgs'), 'nav_msgs.msg': types.SimpleNamespace(Odometry=object),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.SimpleNamespace(Imu=object, LaserScan=object),
        'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': types.SimpleNamespace(Bool=Bool, String=String),
    }
    return mods


def test_node_runs_and_publishes_float_twist(monkeypatch):
    try:
        import rclpy  # noqa: F401
        pytest.skip('실제 ROS 환경에서는 실차로 확인')
    except ImportError:
        pass
    for k, v in _fake_ros().items():
        monkeypatch.setitem(sys.modules, k, v)
    monkeypatch.delitem(sys.modules, 'limo_obstacle_avoid.m7_replay_node', raising=False)
    mod = importlib.import_module('limo_obstacle_avoid.m7_replay_node')
    n = mod.M7ReplayNode()
    S = types.SimpleNamespace
    t = [0.0]

    def hdr():
        return S(stamp=S(sec=int(t[0]), nanosec=int((t[0] % 1) * 1e9)))
    x = [0.0]

    def feed():
        t[0] += 0.05
        n.subs['/imu'](S(header=hdr(), angular_velocity=S(z=0.0)))
        n.subs['/odom'](S(header=hdr(), pose=S(pose=S(position=S(x=x[0], y=0.0), orientation=S(x=0, y=0, z=0, w=1))),
                          twist=S(twist=S(linear=S(x=0.0)))))
        ranges = [float('inf')] * 360
        ranges[180] = 1.0        # 정면 1 m (angle_min -pi)
        n.subs['/scan'](S(ranges=ranges, angle_min=-math.pi, angle_increment=math.radians(1.0)))
    for _ in range(30):
        feed()
    n.subs['/avoid/enable'](S(data=True))
    assert n.active
    for _ in range(10):
        feed()
        for cb in n.timers:
            cb()
    cmds = n.pubs['/avoid/cmd_vel']
    assert cmds and isinstance(cmds[-1].linear.x, float)
    assert n.pubs['/avoid/active'][-1].data is True
    assert any(isinstance(m.data, str) and '"state": "RUN"' in m.data for m in n.pubs['/avoid/state'])


def _m(**kw):
    m = M7Replay(ReplayParams(**kw), default_change(1), default_change(-1))
    m.start()
    m.pts = []
    return m


def _lane(off_mm=0.0, h=0.0, lid=None, left=None, right=None, status='BOTH'):
    return {'status': status, 'offset_mm': off_mm, 'heading_rad': h, 'lane_id': lid,
            'line_left': left, 'line_right': right}


Y = {'color': 'yellow', 'dashed': False}
W = {'color': 'white', 'dashed': True}
WS = {'color': 'white', 'dashed': False}   # 트랙 밖 밝은 바닥을 흰 실선으로 잡은 것


def test_camera_fixes_lateral_offset():
    m = _m()
    for _ in range(30):
        m.on_lane(_lane(off_mm=-40.0, left=W, right=W, lid=0))     # 차로 중심이 오른쪽 4 cm = 차가 왼쪽 4 cm
    assert m.pose()[1] == pytest.approx(0.04, abs=0.005)


def test_camera_rejects_wrong_line_types():
    m = _m(start_lane=1)
    for _ in range(30):
        m.on_lane(_lane(off_mm=100.0, left=WS, right=Y))          # 1차선인데 (흰 실선, 노란선) = 노란선 밖을 차로로 봄
    assert abs(m.pose()[1] - 0.52) < 0.06 or m.pose()[1] > 0.52   # 차로 안쪽으로 끌려가지 않음
    assert m.cam_type_rej + m.outside_n >= 0


def test_outside_yellow_moves_estimate_out():
    m = _m(start_lane=1)
    for _ in range(3):
        m.on_lane(_lane(off_mm=0.0, left=WS, right=Y))
    assert m.pose()[1] > 0.52 + 0.25            # 노란선 바깥으로 봄 -> 차로 중심으로 돌아가는 조향이 나옴


def test_lane_id_corrects_lane_after_votes():
    m = _m()
    for _ in range(6):
        m.on_lane(_lane(off_mm=0.0, lid=-1, left=W, right=Y))
    assert m.lane == -1 and m.pose()[1] == pytest.approx(-0.52, abs=0.03)


def test_relock_when_far_off_consistently():
    m = _m()
    for _ in range(12):
        m.on_lane(_lane(off_mm=-200.0, left=W, right=W, lid=0))    # 20 cm 차이 (문 15 cm 밖)
    assert m.pose()[1] > 0.08
