"""라바콘 로직 단위 테스트 (`python3 -m pytest src/autorace_obstacle/test`)."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_obstacle.cone_core import (ConeFollower, ConeParams, ConeTrack,  # noqa: E402
                                         ConeTracker,
                                         GroundProjector, Pose2D, PoseIntegrator,
                                         YawOffsetEstimator, advance, cone_candidates,
                                         find_gates, orange_ratio, plan_path, pursue, wrap)
from autorace_obstacle.scan_utils import scan_to_points  # noqa: E402

BEV_YAML = os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup', 'config',
                        'bev_limo.yaml')


def scan_of(objs, n=720, noise=0.0, seed=0):
    """objs: [(x, y, r)] 원 / [('box', x, y, w, d)] 사각형 (차량 좌표 mm)."""
    rng = np.random.default_rng(seed)
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    ranges = np.full(n, np.inf)
    for i, a in enumerate(angles):
        d = np.array([math.cos(a), math.sin(a)])
        for o in objs:
            if o[0] == 'box':
                _, x, y, w, dep = o
                for t in np.linspace(0, 3000, 3001):        # 단순 광선 진행
                    q = d * t
                    if abs(q[0] - x) <= dep / 2 and abs(q[1] - y) <= w / 2:
                        ranges[i] = min(ranges[i], t / 1000.0)
                        break
            else:
                x, y, r = o
                c = np.array([x, y])
                b = d @ c
                disc = b * b - (c @ c - r * r)
                if disc >= 0 and b > 0:
                    ranges[i] = min(ranges[i], (b - math.sqrt(disc)) / 1000.0)
    ranges = ranges + rng.normal(0, noise / 1000.0, n)
    return scan_to_points(ranges, -math.pi, 2 * math.pi / n, 0.05, 8.0)


def test_candidates_keep_cones_drop_limo_box():
    p = ConeParams()
    pts = scan_of([(800, 200, 30), (1200, -250, 30), ('box', 1000, 0, 220, 320)])
    c = cone_candidates(pts, p)
    xy = sorted((round(k['x'], -1), round(k['y'], -1)) for k in c)
    assert len(c) == 2, c
    assert xy[0] == pytest.approx((800, 200), abs=15)      # 보이는 면 + 반지름 -> 중심
    assert xy[1] == pytest.approx((1200, -250), abs=15)


def test_candidates_ignore_self_body_and_240_scan():
    """라이다 바로 옆 차체 점은 버리고, 240도 스캔(뒤 120도 없음)에서도 앞 콘은 그대로 잡는다."""
    p = ConeParams()
    n = 480                                            # 0.5도 x 480 = 240도
    angles = -math.radians(120) + np.arange(n) * math.radians(0.5)
    ranges = np.full(n, np.inf)
    ranges[(angles > math.radians(100)) & (angles < math.radians(103))] = 0.08   # 차체 모서리
    c = np.array([900.0, 150.0])
    for i, a in enumerate(angles):
        d = np.array([math.cos(a), math.sin(a)])
        b = d @ c
        disc = b * b - (c @ c - 30.0 ** 2)
        if disc >= 0 and b > 0:
            ranges[i] = (b - math.sqrt(disc)) / 1000.0
    pts = scan_to_points(ranges, angles[0], math.radians(0.5), 0.0, 8.0)
    got = cone_candidates(pts, p)
    assert len(got) == 1 and got[0]['x'] == pytest.approx(900.0, abs=15)


def test_tracker_keeps_cone_outside_fov():
    """차 옆을 지나 뒤로 간 콘(시야 밖)은 안 보여도 지우지 않는다."""
    p = ConeParams()
    tr = ConeTracker(p)
    for _ in range(4):
        tr.update(np.array([[600.0, 250.0]]), Pose2D())
    for _ in range(p.drop_misses_confirmed + 10):
        tr.update(np.zeros((0, 2)), Pose2D(900.0, 0.0, 0.0))   # 콘이 뒤차축 300mm 뒤
    assert len(tr.ok_tracks()) == 1


def test_tracker_confirm_and_drop_tentative():
    p = ConeParams()
    tr = ConeTracker(p)
    pose = Pose2D()
    for _ in range(3):
        tr.update(np.array([[800.0, 100.0]]), pose)
    assert len(tr.ok_tracks()) == 1
    tr.update(np.array([[900.0, -300.0]]), pose)        # 한 번 보인 잡음
    for _ in range(p.drop_misses):
        tr.update(np.array([[800.0, 100.0]]), pose)
    assert len(tr.tracks) == 1                            # 잡음은 버려지고 확정 콘만 남음


def test_tracker_keeps_occluded_cone():
    p = ConeParams()
    tr = ConeTracker(p)
    pose = Pose2D()
    for _ in range(4):
        tr.update(np.array([[600.0, 0.0], [1200.0, 0.0]]), pose)
    for _ in range(p.drop_misses_confirmed + 5):         # 뒤 콘이 앞 콘에 가려 안 보임
        tr.update(np.array([[600.0, 0.0]]), pose)
    assert len(tr.ok_tracks()) == 2


def test_camera_veto():
    p = ConeParams()
    tr = ConeTracker(p)
    for _ in range(3):
        tr.update(np.array([[800.0, 0.0]]), Pose2D())
    t = tr.tracks[0]
    for _ in range(p.cam_min_checks):
        t.add_camera(False, p)
    assert t.rejected and not tr.ok_tracks()
    for _ in range(4 * p.cam_min_checks):                 # 나중에 계속 주황으로 보이면 되살림
        t.add_camera(True, p)
    assert not t.rejected


def _corridor(width=500, x0=500, x1=2100, step=400, yc=0.0):
    xs = np.arange(x0, x1 + 1, step)
    return np.array([(x, yc + width / 2) for x in xs] + [(x, yc - width / 2) for x in xs])


def test_plan_straight_corridor_center_and_clearance():
    p = ConeParams()
    cones = _corridor(500, yc=40.0)
    plan = plan_path(cones, y_lane=0.0, p=p)
    assert len(plan.gates) >= 5 and not plan.singles       # 대각선 쌍도 중점은 가운데
    inside = plan.path[(plan.path[:, 0] > 600) & (plan.path[:, 0] < 2000)]
    assert np.abs(inside[:, 1] - 40.0).max() < 15.0
    d = np.hypot(plan.path[:, None, 0] - cones[None, :, 0],
                 plan.path[:, None, 1] - cones[None, :, 1])
    assert d.min() >= 245.0 - 1.0                        # 통로 반폭 250 - 잡음


def test_plan_narrow_gate_stays_centered():
    """반폭 175 < 최소 여유 205: 양쪽에서 똑같이 밀려 가운데에 머문다."""
    p = ConeParams()
    cones = np.array([(1000.0, 175.0), (1000.0, -175.0)])
    plan = plan_path(cones, 0.0, p)
    k = int(np.argmin(np.abs(plan.path[:, 0] - 1000.0)))
    assert abs(plan.path[k, 1]) < 10.0


def test_gate_rejects_same_row_diagonal():
    """같은 줄 콘이 휘어 대각선으로 놓여도 쌍으로 묶지 않는다."""
    p = ConeParams()
    cones = np.array([(1000.0, 250.0), (1300.0, 560.0)])
    assert find_gates(cones, p) == []


def test_outside_decoy_is_ignored():
    p = ConeParams()
    cones = np.vstack([_corridor(460), [[1300.0, -470.0]]])   # 오른쪽 줄 바깥 기둥
    plan = plan_path(cones, 0.0, p)
    k = int(np.argmin(np.abs(plan.path[:, 0] - 1300.0)))
    assert abs(plan.path[k, 1]) < 20.0


def test_slalom_center_cones_pass_one_side():
    p = ConeParams()
    cones = np.array([(800.0, 0.0), (1500.0, 10.0), (2100.0, -10.0)])
    plan = plan_path(cones, 0.0, p)
    assert {s for _, s in plan.singles} == {'L'}            # 전부 콘의 오른쪽으로
    for c in cones:
        k = int(np.argmin(np.abs(plan.path[:, 0] - c[0])))
        assert plan.path[k, 1] < c[1] - p.min_pass_mm + 5


def test_one_side_row_offsets_by_half_width():
    p = ConeParams()
    cones = np.array([(x, 250.0) for x in (600.0, 1000.0, 1400.0)])   # 왼쪽 줄만
    plan = plan_path(cones, 0.0, p)
    k = int(np.argmin(np.abs(plan.path[:, 0] - 1000.0)))
    assert plan.path[k, 1] == pytest.approx(0.0, abs=20)


def test_pursue_sign_and_advance():
    path = np.array([[x, 200.0] for x in np.arange(0, 2000, 20.0)])
    k, tgt, _ = pursue(path, Pose2D(0, 0, 0), 300.0)
    assert k > 0                                             # 경로가 왼쪽 -> 왼쪽(+)
    q = advance(Pose2D(0, 0, 0), math.pi / 2 * 500.0, 2.0)  # R 0.5m 원 1/4
    assert (q.x, q.y) == pytest.approx((500.0, 500.0), abs=1.0)
    assert q.yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_follower_done_after_last_cone():
    p = ConeParams(done_min_progress_mm=1000.0)
    f = ConeFollower(p)
    f.start(Pose2D(), 0.0)
    for i, (x, y) in enumerate(_corridor(500, x0=600, x1=1400)):
        f.tracker.tracks.append(ConeTrack(i, float(x), float(y), hits=5, confirmed=True))
    f.replan()
    f.control(Pose2D(1400.0, 0.0, 0.0))
    assert not f.done
    f.control(Pose2D(1400.0 + p.done_margin_mm + 10, 0.0, 0.0))
    assert f.done and 'last cone' in f.done_reason


def test_yaw_offset_estimator_wraps():
    e = YawOffsetEstimator(min_samples=5)
    for k in range(10):
        e.add(math.pi - 0.01 + 0.002 * (k % 3), 0.0, 0.0)     # ±pi 경계 근처
        e.add(-math.pi + 0.01, 0.0, 0.0)
    off = e.offset()
    assert abs(wrap(off - math.pi)) < 0.02


def test_pose_integrator_uses_imu_heading():
    pi = PoseIntegrator()
    pi.on_imu(math.pi / 2)
    pi.on_odom(0.0, 0.0)
    for k in range(1, 11):
        pi.on_odom(0.1 * k, 0.0, 0.2)        # 드라이버 odom은 x로 1m (방향 틀림)
    assert (pi.pose.x, pi.pose.y) == pytest.approx((0.0, 1000.0), abs=1.0)


def test_ground_projector_matches_bev():
    """콘 바닥점을 이미지로 찍고 다시 BEV 호모그래피로 보내면 원래 BEV 위치로 돌아온다."""
    import yaml
    with open(BEV_YAML, encoding='utf-8') as fh:
        bev = yaml.safe_load(fh)['bev']
    gp = GroundProjector(bev)
    H = np.array(bev['homography']).reshape(3, 3)
    for x, y in [(700.0, 0.0), (1000.0, 200.0), (1500.0, -300.0)]:
        u, v = gp.project(x, y)
        q = H @ np.array([u, v, 1.0])
        col, row = q[0] / q[2], q[1] / q[2]
        assert col == pytest.approx(bev['vehicle_col'] - y / bev['mm_per_px_x'], abs=0.01)
        assert row == pytest.approx(bev['bev_h'] - 1 - (x - bev['ref_to_bev_bottom_mm'])
                                    / bev['mm_per_px_y'], abs=0.01)
    box = gp.cone_box(1000.0, 0.0, 50.0)
    assert box is not None and box[2] > box[0] and box[3] > box[1]


def test_orange_ratio_box():
    hsv = np.zeros((100, 100, 3), np.uint8)
    hsv[40:60, 40:60] = (8, 200, 200)                  # 주황
    hsv[0:20, 0:20] = (28, 200, 200)                   # 노랑 (차선)
    assert orange_ratio(hsv, (40, 40, 59, 59)) == pytest.approx(1.0)
    assert orange_ratio(hsv, (0, 0, 19, 19)) == pytest.approx(0.0)
