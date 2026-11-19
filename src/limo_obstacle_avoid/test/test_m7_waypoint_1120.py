"""11/20 웨이포인트: 노란선 옆 위치 보정 · 끝 처리 · 15 s 막기."""
import math

from limo_obstacle_avoid.m7_waypoint_core import M7Waypoint, WaypointParams


def _core(**kw):
    p = WaypointParams(**kw)
    c = M7Waypoint(p)
    c.start()
    return c


def test_yellow_fix_pulls_y_to_line():
    c = _core()
    assert abs(c.pose()[1]) < 1e-9
    y_true = -0.25                       # 실제는 오른쪽으로 25 cm (켤 때 옆 0으로 잡음)
    d = y_true + c.p.yellow_line_y       # 오른쪽 노란선까지 (psi 0)
    for i in range(30):
        c.t_last = 0.04 * i
        c.on_lane({'yellow_r_mm': 1000 * d, 'yellow_f_mm': 550.0}, 0.04 * i)
    assert abs(c.pose()[1] - y_true) < 0.02
    assert c.yfix_cnt > 0


def test_yellow_fix_ignores_far_off_and_blocked_view():
    c = _core()
    for i in range(10):
        c.on_lane({'yellow_r_mm': 1000 * (0.775 + 0.9)}, 0.04 * i)   # 0.9 m 차이 = 잘못 본 선
    assert abs(c.pose()[1]) < 1e-9
    c.front_gap = 0.3                    # 바로 앞 LIMO가 화면 가림
    for i in range(10):
        c.on_lane({'yellow_r_mm': 500.0}, 0.04 * i)
    assert abs(c.pose()[1]) < 1e-9


def test_end_waits_for_limo_ahead_then_finishes():
    c = _core(section_length=4.0, end_before=1.0)
    c.n_scan = 5
    c.fu.x = 3.05                        # 끝 거리(3.0) 지남
    c.mem = [{'x': 3.6, 'y': 0.52, 'n': 5, 'seen': 0.0}]
    c.step(1.0)
    assert c.state == 'RUN'              # 1 m 안에 LIMO -> 계속
    c.mem = []
    c.step(1.05)
    assert c.state == 'DONE'


def test_end_hard_limit():
    c = _core(section_length=4.0, end_before=1.0, end_extra=1.0)
    c.n_scan = 5
    c.fu.x = 4.05                        # 끝 거리 + 1.0 넘음
    c.mem = [{'x': 4.3, 'y': 0.0, 'n': 5, 'seen': 0.0}]
    c.step(1.0)
    assert c.state == 'DONE'


def test_x_end_extends_past_last_limo():
    c = _core(section_length=4.8, end_before=1.0)
    assert c._x_end([]) == 3.8
    xe = c._x_end([{'x': 4.4, 'y': 0.52}])
    assert 3.8 < xe <= 3.8 + c.p.end_extra
