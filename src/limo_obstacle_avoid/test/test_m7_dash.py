"""m7_dash_core 검사 (ROS 없음)."""
import math
import types

import numpy as np

from limo_obstacle_avoid.m7_dash_core import DashParams, M7Dash, dash_points


def _core(**kw):
    c = M7Dash(DashParams(**kw))
    c.state = 'RUN'
    return c


def _obs(c, side, x0, w=0.32):
    """차로 중심에 정확히 놓인 LIMO (앞면 폭 w)."""
    yc = side * 0.26
    c.mem.append({'side': side, 'x0': x0, 'x1': x0 + (0.54 - w), 'y0': yc - w / 2, 'y1': yc + w / 2,
                  'w': w, 'n': 5, 'seen': 0.0})


def test_no_obstacle_follows_dash():
    c = _core()
    c._replan()
    assert c.y_ref(1.0) == 0.0


def test_lane2_goes_right_lane1_goes_left():
    c = _core()
    _obs(c, +1, 1.0)               # 2차로(가운데, 왼쪽) → 오른쪽으로
    _obs(c, -1, 3.0)               # 1차로(오른쪽, 안쪽) → 왼쪽으로
    c._replan()
    (a1, b1, y1, _), (a2, b2, y2, _) = c.holds
    assert y1 < 0 < y2
    # 옆 여유: 안쪽 모서리(0.26-0.16=0.10) - 반폭 - clear
    assert abs(y1 - (0.10 - 0.115 - c.p.clear)) < 1e-9
    assert abs(y2 + y1) < 1e-9
    assert c.y_ref((a1 + b1) / 2) == y1
    assert a2 - b1 >= 2 * c.p.shift_len and c.y_ref(b1 + c.p.shift_len + 0.01) == 0.0   # 빈 칸 길면 점선 복귀


def test_consecutive_goes_straight_across():
    c = _core()
    _obs(c, +1, 1.0)
    _obs(c, -1, 2.0)               # 약 1 m 간격 → 점선 복귀 없이 바로 반대쪽
    c._replan()
    (_, b1, y1, _), (a2, _, y2, _) = c.holds
    assert a2 - b1 < 2 * c.p.shift_len
    ys = [c.y_ref(b1 + (a2 - b1) * u / 10) for u in range(11)]
    assert all(ys[i] <= ys[i + 1] + 1e-12 for i in range(10))      # 단조 (중간에 되돌아가지 않음)
    assert ys[0] == y1 and abs(ys[-1] - y2) < 1e-9


def test_lengthwise_obstacle_less_offset_more_length():
    c = _core()
    _obs(c, +1, 1.0, w=0.22)       # 세로: 폭 좁고 길이 김
    c._replan()
    a, b, y, _ = c.holds[0]
    assert abs(y - (0.26 - 0.11 - 0.115 - c.p.clear)) < 1e-9
    assert abs(b - (1.0 + 0.32 + c.p.hold_rear + c.p.clear_x)) < 1e-9


def test_both_sides_same_x_is_blocked():
    c = _core()
    _obs(c, +1, 1.0)
    _obs(c, -1, 1.1)
    c._replan()
    assert c.block_x is not None and len(c.holds) == 1


def test_straddling_obstacle_is_blocked():
    c = _core()
    c.mem.append({'side': 0, 'x0': 1.0, 'x1': 1.2, 'y0': -0.1, 'y1': 0.1, 'w': 0.2, 'n': 3, 'seen': 0.0})
    c._replan()
    assert c.block_x == 1.0


def test_beside_obstacle_never_steers_toward_it():
    c = _core()
    _obs(c, +1, 1.0)
    _obs(c, -1, 1.9)
    c._replan()
    c.n_scan = 5
    a1, b1, y1, _ = c.holds[0]
    c.fu.x, c.fu.y, c.offset = b1 - 0.02, 0.0, (0.0, y1, 0.0)   # 아직 첫 장애물 옆
    _, k = c.step(0.0)
    assert k <= 1e-9               # 왼쪽(장애물 쪽)으로 안 꺾음


def test_dash_points_and_heading_sign():
    cfg = types.SimpleNamespace(mm_per_px_x=4.0, mm_per_px_y=4.0, vehicle_col=200.0, bev_h=400,
                                ref_to_bev_bottom_mm=300.0)
    cfg.row_to_y_mm = lambda r: 300.0 + (399 - r) * 4.0
    cfg.col_to_x_mm = lambda col: (col - 200.0) * 4.0
    m = np.zeros((400, 400), np.uint8)
    for r in range(400):                       # 오른쪽 40 mm, 앞으로 갈수록 오른쪽 (기울기 0.05)
        f = cfg.row_to_y_mm(r)
        col = int(round(200 + (40 + 0.05 * (f - 300)) / 4))
        m[r, col - 2:col + 3] = 255
    m[:, 330:335] = 255                         # 옆 점선 (520 mm 오른쪽) — 걸러져야 함
    pts = dash_points(m, cfg, lambda f: 0.0, 120.0)
    assert len(pts) > 10 and all(abs(x - (40 + 0.05 * (f - 300))) < 8 for f, x in pts)
    c = _core()
    c.n_scan = 5
    c.on_dash(pts)
    _, y, psi = c.pose()
    assert y > 0                               # 점선이 오른쪽에 보임 = 내가 왼쪽
    assert psi > 0                             # 앞으로 갈수록 오른쪽 = 내가 왼쪽을 봄


def test_dash_ignored_near_obstacle():
    c = _core()
    _obs(c, +1, 0.6)
    c.on_dash([(400.0, 50.0)] * 6)
    assert c.n_fix['dash'] == 0


def test_sim_first_competition_layout():
    from limo_obstacle_avoid.m7_dash_sim import run
    for orient in ('side', 'long'):
        r = run(1, orient, jitter=0.0)
        assert r['success'], r['why']
        assert r['gap_cm'] > 3.0


def test_sign_ends_only_after_min_x_and_frames():
    c = _core(end_by_sign=True, end_sign_h_px=60.0, end_sign_frames=3, end_min_x=2.5)
    c.n_scan = 5
    for _ in range(3):
        c.on_sign(80.0)
    c.step(0.0)
    assert c.state == 'RUN'                    # 2.5 m 전엔 표지판 봐도 안 끝남
    c.offset = (3.0, 0.0, 0.0)
    c.on_sign(None)                            # 끊기면 다시 셈
    c.on_sign(80.0)
    c.step(0.05)
    assert c.state == 'RUN'
    c.on_sign(80.0)
    c.on_sign(80.0)
    c.step(0.1)
    assert c.state == 'DONE' and '표지판' in c.end_why


def test_distance_cap_ends_without_sign():
    c = _core(end_x=4.6)
    c.n_scan = 5
    c.offset = (4.7, 0.0, 0.0)
    c.step(0.0)
    assert c.state == 'DONE' and '거리' in c.end_why


def test_arc_clear_sees_obstacle_on_turn_not_beside():
    c = _core()
    c.pts = [(0.0, 0.14)]                       # 바로 옆 2.5 cm -> 직진도, 그쪽으로 꺾어도(회전 안쪽) 안 닿음
    assert c.arc_clear(0.0) == math.inf and c.arc_clear(1.8) == math.inf
    c.pts = [(0.35, 0.20)]                      # 앞 왼쪽: 직진은 비켜 감, 왼쪽으로 꺾으면 닿음
    assert c.arc_clear(0.0) == math.inf
    assert c.arc_clear(1.8) < 0.3
    c.pts = [(0.35, 0.0)]                       # 앞 15 cm
    assert abs(c.arc_clear(0.0) - 0.14) < 0.03


def test_stop_then_rejudge_relaxes_and_resumes():
    c = _core(recover_wait=0.2)
    c.n_scan = 5
    c.pts = [(0.25, 0.0)]                       # 바로 앞 5 cm -> 정지
    t = 0.0
    for _ in range(6):
        t += 0.05
        c.step(t)
    assert c.n_rejudge == 1 and c.rj_t is not None
    c.pts = []                                  # 다시 보니 비었음
    for _ in range(10):
        t += 0.05
        v, _ = c.step(t)
    assert v > 0                                # 다시 감
    c.pts = [(0.25, 0.0)]
    for _ in range(30):
        t += 0.05
        c.step(t)
    assert c.relax >= 1                         # 같은 자리 재판단 -> 여유 줄임
    c.relax = 10
    assert c._eff(c.p.clear, c.p.clear_min) == c.p.clear_min


# ---------- 팀 주행 통합 (m7_impl:=dash) ----------
def test_start_positions_team_frame():
    c = M7Dash(DashParams())
    c.start()
    assert abs(c.pose()[1] - 0.26) < 1e-9                  # 가운데 차로 중심 = 점선 기준 +0.26
    st = c.status()
    assert st['now_lane'] == 0 and abs(st['y']) < 1e-9 and st['state'] == 'RUN'
    c = M7Dash(DashParams(start_on_dash=True))
    c.start()
    assert abs(c.pose()[1]) < 1e-9
    c = M7Dash(DashParams(m2_test_start=True, start_lane_no=3))
    c.start()
    x, y, _ = c.pose()
    assert abs(x - c.m2_x()) < 1e-9 and abs(y + 0.26) < 1e-9 and c.status()['now_lane'] == -1


def test_base_goes_dash_then_fork_lane():
    import os
    from limo_obstacle_avoid.m7_replay_core import load_m2
    c = M7Dash(DashParams())
    c.set_m2(*load_m2(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'paths')))
    c.start()
    assert abs(c.base(0.0) - 0.26) < 1e-9 and abs(c.base(1.5)) < 1e-9
    c.set_exit('B')
    assert abs(c.base(c.m2_x()) + 0.26) < 1e-9             # B: 오른쪽 차로 중심
    c.set_exit('A')
    assert abs(c.base(c.m2_x()) - 0.26) < 1e-9             # A: 가운데 차로 중심 (2-A 녹화 시작 자리)


def test_m7_link_reports_fork_lane():
    import os
    import sys
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'autorace_obstacle'))
    from autorace_obstacle.m7_link import M7Link
    from limo_obstacle_avoid.m7_dash_sim import run
    for fork in ('A', 'B'):
        r = run(1, 'long', jitter=0.0, integrated=True, fork=fork)
        assert r['success'], (fork, r['why'], r['status'].get('end'))
        link = M7Link()
        link.phase = 'running'
        link.on_avoid_state(r['status'])
        link.on_avoid_done(True)
        link.on_avoid_state(r['status'])
        assert link.exit_fork == fork and link.report['fork'] == fork
