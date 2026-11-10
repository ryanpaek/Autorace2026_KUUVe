"""parking_node 시험: 계산 함수 + 가상 트랙 한 바퀴 (ROS 없이).

가상 트랙 좌표 (mm): x = 진행 방향, 0 = 주차 앞 흰 선의 뒤쪽 가장자리. y = 왼쪽 +, 0 = 주차 구역 위 테두리.
칸: 500 x 600, 선 20, 흰 선 -> 주차 구역 440 (2026Track 도면). 정차 리모는 칸 가운데라고 가정.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission import parking_node as pn  # noqa: E402

WB, L_BODY, W_BODY, R_MIN = 210.0, 322.0, 220.0, 530.0
REAR = (L_BODY - WB) / 2.0
LIDAR_AHEAD = 47.0            # 라이다가 앞바퀴 축보다 47mm 앞 (노드 기본값 front_axle_from_lidar_mm -47과 같게)


def cell(i):
    """칸 i(1~6)의 (x0, x1, y_top, y_bot). 앞줄 1 2 3, 뒷줄 4 5 6."""
    col, row = (i - 1) % 3, (i - 1) // 3
    x0 = 460 + col * 520
    y0 = -20 - row * 620
    return x0, x0 + 500, y0, y0 - 600


def parked_box(i):
    """칸 가운데에 앞뒤로 선 정차 리모 (x0, x1, y0, y1)."""
    x0, x1, yt, yb = cell(i)
    cx, cy = (x0 + x1) / 2, (yt + yb) / 2
    return cx - W_BODY / 2, cx + W_BODY / 2, cy - L_BODY / 2, cy + L_BODY / 2


def corners(x, y, th):
    """뒷바퀴축 (x, y, th) 차의 네 모서리."""
    c, s = math.cos(th), math.sin(th)
    return [(x + dx * c - dy * s, y + dx * s + dy * c)
            for dx in (-REAR, WB + REAR) for dy in (-W_BODY / 2, W_BODY / 2)]


def raycast(px, py, th, boxes, n=360, fov_deg=360.0):
    """사각형들에 대한 가짜 라이다 (m). fov_deg 밖은 inf (실측: 리모 덩어리가 잡히는 건 약 ±105도까지)."""
    out = []
    for i in range(n):
        a = -math.pi + i * 2 * math.pi / n
        if abs(a) > math.radians(fov_deg / 2.0):
            out.append(math.inf)
            continue
        dx, dy = math.cos(th + a), math.sin(th + a)
        best = math.inf
        for b in boxes:
            tmin, tmax, ok = -math.inf, math.inf, True
            for o, d, lo, hi in ((px, dx, b[0], b[1]), (py, dy, b[2], b[3])):
                if abs(d) < 1e-12:
                    ok = ok and lo <= o <= hi
                else:
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if ok and tmax >= max(tmin, 0.0) and tmin > 0.0:
                best = min(best, tmin)
        out.append(best / 1000.0)
    return out, -math.pi, 2 * math.pi / n


def hits(x, y, th, boxes):
    """차 테두리가 정차 리모와 닿았나 (테두리 점 검사)."""
    cs = corners(x, y, th)
    for p, q in ((cs[0], cs[1]), (cs[1], cs[3]), (cs[3], cs[2]), (cs[2], cs[0])):
        for k in range(21):
            px, py = p[0] + (q[0] - p[0]) * k / 20, p[1] + (q[1] - p[1]) * k / 20
            if any(b[0] <= px <= b[1] and b[2] <= py <= b[3] for b in boxes):
                return True
    return False


# 가로선 (y 위쪽 가장자리, 아래쪽): 차로·주차 구역 경계, 앞줄·뒷줄 사이, 바깥 테두리
H_LINES = (0.0, -620.0, -1240.0)


def camera_lines(fx, fy, th):
    """가짜 카메라: 칸 쪽(남쪽)을 볼 때 앞바퀴 -> 가로선 가까운 가장자리 거리 (BEV 350~1900mm만)."""
    dev = th + math.pi / 2
    if abs(dev) > math.radians(25):
        return []
    out = [(fy - yl) / math.cos(dev) for yl in H_LINES]
    return sorted(d for d in out if 350.0 <= d <= 1900.0)


def drive(parked, target, overrides=None, y0=240.0, fov_deg=360.0, camera=False, log=None, hide=()):
    """주차 논리(ParkingV2)를 가상 트랙에서 끝까지 돌린다 -> (칸 안 모서리 수, 충돌, 결과).

    hide: 라이다가 못 보는 리모의 parked 안 순서 (충돌 검사에는 그대로 들어감)
    """
    p = dict(pn.DEFAULTS)
    p.update(overrides or {})
    logic = pn.ParkingV2(p)
    boxes = [parked_box(i) for i in parked]
    x, y, th, dist, t, dt, offset, hit = -900.0, y0, 0.0, 0.0, 0.0, 0.05, 0.0, False
    info = {'done': False, 'result': ''}
    for _ in range(int(60 / dt)):
        fx, fy = x + WB * math.cos(th), y + WB * math.sin(th)
        seen = [b for i, b in enumerate(boxes) if i not in hide]
        rng, amin, ainc = raycast(fx + LIDAR_AHEAD * math.cos(th), fy + LIDAR_AHEAD * math.sin(th), th, seen,
                                  fov_deg=fov_deg)
        cl = pn.scan_clusters(rng, amin, ainc, 0.05, 8.0, p['front_axle_from_lidar_mm'], 0.0,
                              max_dist_mm=max(1500.0, p['side_far_mm'] + 200.0))
        bl = pn.front_blocked(rng, amin, ainc, 0.05, 8.0, p['front_axle_from_lidar_mm'], 0.0,
                              p['front_bumper_from_axle_mm'], p['front_stop_mm'], 120.0)
        d_line = -40.0 - fx                                   # 앞바퀴 -> 흰 선 가까운 가장자리
        lines = [d_line] if 60.0 <= d_line <= 700.0 and abs(th) < 0.3 else []
        lane_w = 0.18 * max(-1.89, min(1.89, 0.004 * (240.0 + offset - y) - 2.0 * th))   # 가짜 차선 노드
        v, k, info = logic.step({'t': t, 'dist_mm': dist, 'yaw': th, 'lane_v': 0.18, 'lane_w': lane_w,
                                 'stoplines_mm': lines, 'clusters': cl, 'blocked': bl,
                                 'end_lines_mm': camera_lines(fx, fy, th) if camera else []})
        if log is not None:
            log.append(dict(info))
        if logic.lane_cfg:
            offset = logic.lane_cfg.get('target_offset_mm', 0.0)
        kap = 0.0 if abs(v) < 1e-6 else (lane_w if k is None else v * k) / v
        kap = max(-1 / R_MIN, min(1 / R_MIN, kap))           # 실제 차 최대 곡률
        d = v * 1000 * dt
        if abs(kap) < 1e-9:
            x, y = x + d * math.cos(th), y + d * math.sin(th)
        else:
            th2 = th + d * kap
            x += (math.sin(th2) - math.sin(th)) / kap
            y -= (math.cos(th2) - math.cos(th)) / kap
            th = th2
        dist += abs(d)
        t += dt
        hit = hit or hits(x, y, th, boxes)
        if info['done']:
            break
    x0, x1, yt, yb = cell(target)
    n_in = sum(1 for cx, cy in corners(x, y, th) if x0 < cx < x1 and yb < cy < yt)
    return n_in, hit, info['result']


# ---------------------------------------------------------------- 계산 함수
def test_find_pair_two_cells_apart():
    pair = pn.find_pair([(100.0, -500.0), (1140.0, -510.0)], 1300.0, 1040.0, 150.0)
    assert pair is not None
    mx, my, vx, vy = pair
    assert math.isclose(mx, 620.0) and math.isclose(my, -505.0)
    assert vx > 0.99                                          # 짝 방향 = 차 앞쪽


def test_find_pair_rejects_wrong_spacing_and_rows():
    assert pn.find_pair([(100.0, -500.0), (620.0, -500.0)], 1300.0, 1040.0, 150.0) is None    # 칸 1개 간격
    assert pn.find_pair([(100.0, -500.0), (1140.0, -1100.0)], 1300.0, 1040.0, 150.0) is None  # 다른 줄
    assert pn.find_pair([(100.0, -500.0)], 1300.0, 1040.0, 150.0) is None                     # 한 대만


def test_cell_line_from_neighbors_points_forward():
    mx, my, ux, uy = pn.cell_line_from_neighbors([(-20.0, -300.0), (-20.0, 300.0)])
    assert math.isclose(mx, -20.0) and math.isclose(my, 0.0)
    assert ux > 0.99 and abs(uy) < 1e-9


def test_pursuit_turns_toward_line():
    line = (0.0, 0.0, 1.0, 0.0)                              # x축 방향 선
    assert pn.pursuit_kappa_on_line(0.0, 100.0, 0.0, line, 250.0) < 0                 # 선이 오른쪽 -> 오른쪽으로
    assert pn.pursuit_kappa_on_line(0.0, -100.0, 0.0, line, 250.0) > 0
    # 후진: 선이 오른쪽 뒤에 있으면 오른쪽 조향 (오른쪽 회전 중심을 돌며 뒤-오른쪽으로 감)
    assert pn.pursuit_kappa_on_line(0.0, 100.0, 0.0, line, 250.0, reverse=True) < 0
    assert pn.pursuit_kappa_on_line(0.0, -100.0, 0.0, line, 250.0, reverse=True) > 0


def test_neighbors_abeam_passed():
    assert pn.neighbors_abeam([(-50.0, -300.0), (-50.0, 300.0)], passed_ok=True) == (True, True)
    assert pn.neighbors_abeam([(-50.0, -300.0), (-50.0, 300.0)], passed_ok=False) == (False, False)
    assert pn.neighbors_abeam([(200.0, -300.0), (10.0, 300.0)]) == (False, True)


def test_scan_clusters_keeps_limo_drops_wall():
    limo = (400.0, 620.0, -500.0, -178.0)                    # 라이다 기준 오른쪽 앞 리모
    wall = (-2000.0, 2000.0, 700.0, 720.0)                   # 왼쪽 긴 벽
    rng, amin, ainc = raycast(0.0, 0.0, 0.0, [limo, wall], n=720)
    cl = pn.scan_clusters(rng, amin, ainc, 0.05, 8.0, front_axle_from_lidar_mm=0.0)
    assert len(cl) == 1
    assert 380.0 < cl[0][0] < 640.0 and cl[0][1] < 0


def test_kappa_full_right_saturates_bridge():
    # 브리지(축거 0.2~0.222, 최대 0.40rad)가 최대 조향으로 자르는 값이어야 "끝까지 꺾기"가 된다
    for wb in (0.2, 0.222):
        assert abs(math.atan(wb * pn.KAPPA_FULL_RIGHT)) > 0.40


# ---------------------------------------------------------------- 가상 트랙 한 바퀴
def test_park_front_row_cell2():
    n_in, hit, result = drive([1, 3], 2)
    assert (n_in, hit, result) == (4, False, 'PARKED')


def test_park_back_row_cell5():
    n_in, hit, result = drive([4, 6], 5, {'target_row': 'auto'})   # 11/10부터 기본은 2번 고정
    assert (n_in, hit, result) == (4, False, 'PARKED')


def test_late_turn_recovers_by_realign():
    # 40cm 늦게 꺾어 앞이 막혀도 재정렬로 5번 칸에 들어간다 (예전 실패 사례)
    n_in, hit, _ = drive([4, 6], 5, {'turn_lead_mm': -80.0, 'target_row': 'auto'})
    assert n_in == 4 and not hit


def test_disabled_realign_still_parks_nominal():
    n_in, hit, result = drive([1, 3], 2, {'max_realign': 0})
    assert (n_in, hit, result) == (4, False, 'PARKED')


# ---------------------------------------------------------------- 11/09 park_02: 라이다 시야가 좁을 때
NARROW = 210.0    # 실측: 꺾은 뒤 리모 덩어리가 약 ±105도 밖에서 사라짐 (park_02 66.3초)


def test_narrow_fov_front_row_parks_by_memory():
    """꺾은 뒤 왼쪽 리모가 시야 밖으로 빠져도 기억한 칸 가운데 선으로 선다 (예전: 1.1m 직진 후 실패)."""
    log = []
    n_in, hit, result = drive([1, 3], 2, fov_deg=NARROW, log=log)
    assert (n_in, hit) == (4, False) and result.startswith('PARKED')
    assert log[-1]['row'] == 'front'


def test_narrow_fov_back_row_parks():
    log = []
    n_in, hit, result = drive([4, 6], 5, {'target_row': 'auto'}, fov_deg=NARROW, log=log)
    assert (n_in, hit) == (4, False) and result.startswith('PARKED')
    assert log[-1]['row'] == 'back'


def test_camera_end_line_stops_in_right_row():
    """옆면 정지를 끄고(side_max_mm 50) 카메라만 주면 칸 끝 선으로 그 줄 칸 안에 선다. 앞줄=줄 사이 선, 뒷줄=테두리."""
    for parked, target in (([1, 3], 2), ([4, 6], 5)):
        log = []
        n_in, hit, result = drive(parked, target, {'side_max_mm': 50.0, 'target_row': 'auto'}, fov_deg=NARROW,
                                  camera=True, log=log)
        assert (n_in, hit) == (4, False), (parked, n_in, result)
        assert log[-1]['stop_how'] == 'camera', (parked, log[-1]['stop_how'])


def test_cap_keeps_car_inside_cell_without_side_or_camera():
    """옆면도 카메라도 없으면 짝(깊이 보정)으로 칸 가운데를 조금 지나서 멈춤 -> 칸 밖으로 안 나감."""
    for parked, target in (([1, 3], 2), ([4, 6], 5)):
        log = []
        n_in, hit, result = drive(parked, target, {'side_max_mm': 50.0, 'target_row': 'auto'}, fov_deg=NARROW, log=log)
        assert (n_in, hit) == (4, False), (parked, n_in, result)
        assert log[-1]['stop_how'] == 'cap', (parked, log[-1]['stop_how'])


# ---------------------------------------------------------------- 11/10 대회 배치: 1·3·4·5·6에 정차 리모, 목표 2번
FIVE = [1, 3, 4, 5, 6]


def test_cell_line_ignores_diagonal_back_row():
    """2번 칸에 들어가는 중 1·3번(옆, 조금 뒤)과 4·6번(대각선 앞)이 다 보여도 1·3번을 고른다."""
    cl = [(-300.0, -400.0), (120.0, -420.0), (120.0, -60.0), (260.0, 310.0), (-250.0, 360.0)]
    mx, my, ux, uy = pn.cell_line_from_neighbors(cl)
    assert math.isclose(mx, -275.0) and math.isclose(my, -20.0)


def test_five_limos_cell2():
    for fov, cam in ((360.0, False), (NARROW, False), (NARROW, True)):
        log = []
        n_in, hit, result = drive(FIVE, 2, fov_deg=fov, camera=cam, log=log)
        assert (n_in, hit) == (4, False) and result.startswith('PARKED'), (fov, cam, n_in, result)
        assert log[-1]['row'] == 'front'


def test_five_limos_cell2_when_limo1_missed():
    """1번을 못 보면 4·6번이 짝이 된다. 앞뒤 위치는 같고 깊이는 한 줄 옮겨 써서 2번에 들어간다 (auto면 5번으로 감)."""
    n_in, hit, result = drive(FIVE, 2, fov_deg=NARROW, camera=True, hide=(0,))
    assert (n_in, hit) == (4, False) and result.startswith('PARKED'), (n_in, result)


def test_five_limos_cell2_turn_timing_off():
    for lead in (520.0, 120.0, -80.0):          # 20cm 일찍 / 20cm 늦게 / 40cm 늦게
        n_in, hit, result = drive(FIVE, 2, {'turn_lead_mm': lead}, fov_deg=NARROW, camera=True)
        assert (n_in, hit) == (4, False), (lead, n_in, result)
