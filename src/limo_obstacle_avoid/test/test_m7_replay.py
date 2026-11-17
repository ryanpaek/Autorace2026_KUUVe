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


# ── 11/17: 트리거 전에는 차선 기반 조향만 + 미리 계획 + 차로 번호 ──
def _cruising(lane=0):
    from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, default_change, mirror
    left = default_change(+1, 0.52)
    # 기본 S자(녹화 파일 없음)는 546 mm에서 시작하면 피하려는 장애물에 닿아 늘 '막힘' -> 시험은 900 mm
    m = M7Replay(ReplayParams(start_lane=lane, trigger_mm=900.0), left, mirror(left))
    m.start()
    m.scan_t = 0.0
    return m


def test_cruise_steers_by_lane_node_only():
    """트리거 전(CRUISE): 차선 노드 곡률을 그대로 씀. 차선이 끊기거나 놓치면 그동안만 위치 추정"""
    m = _cruising()
    m.on_lane({'status': 'BOTH', 'kappa': 0.37, 'source': 'lane', 'lane_id': 0}, 1.0)
    v, k = m._cruise_cmd(1.1)
    assert m.steer_src == 'lane' and abs(k - 0.37) < 1e-9 and v > 0
    v, k = m._cruise_cmd(2.0)                               # 0.9초 동안 차선 소식 없음
    assert m.steer_src == 'pose'
    m.on_lane({'status': 'NONE', 'kappa': 0.0, 'source': 'hold'}, 2.0)
    m._cruise_cmd(2.1)
    assert m.steer_src == 'pose'                            # 선 놓침
    m.on_lane({'status': 'BOTH', 'kappa': 0.2, 'source': 'lane', 'lane_id': 1}, 3.0)
    m._cruise_cmd(3.1)
    assert m.steer_src == 'pose'                            # 카메라가 다른 차로라고 함 -> 그 곡률 안 씀


def test_plan_ahead_before_trigger():
    """장애물이 트리거(546 mm)보다 멀 때도 어디 있고 어느 차로로 빠질지 미리 계산 (조향은 그대로 직진)"""
    m = _cruising()
    pts = [(1.5 + 0.01 * i, -0.10 + 0.02 * j) for i in range(3) for j in range(11)]     # 2차선 1.5 m 앞
    pts += [(1.3 + 0.01 * i, 0.52 - 0.10 + 0.02 * j) for i in range(3) for j in range(11)]   # 1차선도 막힘
    m.on_scan_points(0.0, pts)
    v, k = m.step(1.0)
    assert m.state == 'CRUISE' and v > 0                    # 아직 재생 안 함
    assert m.plan['obstacle_mm'] is not None and 1300 < m.plan['obstacle_mm'] < 1500
    assert m.plan['to_lane'] == -1 and m.plan['to_lane_no'] == 3, m.plan    # 1차선은 막혀 3차선으로
    assert m.status()['plan'] == m.plan


def test_lane_number_and_outer_lane_swap_needs_full_window():
    """지금 몇 차선인지 늘 상태에 있음. 1 <-> 3차선(두 칸)은 카메라가 7번 모두 같아야 바꿈, 한 칸은 5번"""
    m = _cruising(lane=1)
    assert m.status()['lane_no'] == 1
    for i in range(6):                                      # 카메라가 3차선(-1)이라고 6번 (7번 미만)
        m.on_lane({'status': 'L', 'lane_id': -1}, 0.1 * i)
    assert m.lane == 1 and m.status()['lane_agree'] is False
    for i in range(7):
        m.on_lane({'status': 'BOTH', 'lane_id': -1, 'offset_mm': 0.0, 'heading_rad': 0.0}, 1.0 + 0.1 * i)
    assert m.lane == -1 and m.status()['lane_no'] == 3


def test_check_paths_flags_default_curve_at_546():
    """녹화 파일 없이 기본 S자를 546 mm에서 시작하면 피하려는 장애물에 닿음 -> 켤 때 점검이 알려 줌 (900 mm면 됨)"""
    from limo_obstacle_avoid.m7_replay_core import M7Replay, ReplayParams, default_change, mirror
    left = default_change(+1, 0.52)
    bad = M7Replay(ReplayParams(trigger_mm=546.0), left, mirror(left)).check_paths()
    ok = M7Replay(ReplayParams(trigger_mm=900.0), left, mirror(left)).check_paths()
    assert bad == {1: False, -1: False} and ok == {1: True, -1: True}


# ── 11/17: 후진 없음 (재생 전후 가운데 맞추기·물러나기 빼고 차선 주행을 믿음) ──
def test_no_reverse_starts_replay_without_align_even_off_center():
    """가운데에서 6 cm·5° 벗어나 있어도 서서 앞뒤로 맞추지 않고 바로 재생 (예전: STOP -> ALIGN 후진)"""
    import math as _m
    m = _cruising()
    m._shift(ddy=0.06, ddpsi=_m.radians(5.0))
    assert not m.aligned()
    pts = [(0.10 + 0.88 + 0.01 * i, -0.10 + 0.02 * j) for i in range(3) for j in range(11)]   # 라이다 880 mm 앞 2차선
    m.on_scan_points(0.0, pts)
    v, k = m.step(1.0)
    assert m.state == 'EXEC' and v > 0
    old = _cruising()
    old.p.no_reverse, old.p.align_before = False, True
    old._shift(ddy=0.06, ddpsi=_m.radians(5.0))
    old.on_scan_points(0.0, pts)
    old.step(1.0)
    assert old.state == 'STOP'                              # 예전: 서서 가운데 맞추기로


def test_no_reverse_never_backs_off_when_blocked():
    """앞도 옆도 막혀 오래 서 있어도 물러나지 않고 서서 다시 봄"""
    m = _cruising()
    pts = []
    for yc in (0.0, 0.52, -0.52):
        pts += [(0.10 + 0.80 + 0.01 * i, yc - 0.10 + 0.02 * j) for i in range(3) for j in range(11)]
    m.on_scan_points(0.0, pts)
    for i in range(80):                                     # 8초
        v, k = m.step(1.0 + 0.1 * i)
        assert v >= 0.0 and m.state != 'ALIGN'
    assert '후진 안 함' in m.why
