"""parking_node 시험: 계산 함수 + 가상 트랙 한 바퀴 (ROS 없이).

가상 트랙 좌표 (mm): x = 진행 방향, 0 = 주차 앞 흰 선의 뒤쪽 가장자리. y = 왼쪽 +, 0 = 주차 구역 위 테두리.
칸: 500 x 600, 선 20, 흰 선 -> 주차 구역 440 (2026Track 도면). 정차 리모는 칸 가운데라고 가정.
"""
import math
import os
import pytest
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


def drive(parked, target, overrides=None, y0=240.0, fov_deg=360.0, camera=False, log=None, hide=(), out=None,
          r_min=None, steer_lag_mm=0.0):
    """주차 논리(ParkingV2)를 가상 트랙에서 끝까지 돌린다 -> (칸 안 모서리 수, 충돌, 결과).

    hide: 라이다가 못 보는 리모의 parked 안 순서 (충돌 검사에는 그대로 들어감)
    """
    p = dict(pn.DEFAULTS)
    p.update(overrides or {})
    logic = pn.ParkingV2(p)
    boxes = [parked_box(i) for i in parked]
    x, y, th, dist, t, dt, offset, hit = -900.0, y0, 0.0, 0.0, 0.0, 0.05, 0.0, False
    info = {'done': False, 'result': ''}
    kap_prev = 0.0
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
                                 'end_lines_mm': camera_lines(fx, fy, th) if camera else [],
                                 'points': pn.scan_points(rng, amin, ainc, 0.05, 8.0, p['front_axle_from_lidar_mm']),
                                 'points_far': pn.scan_points(rng, amin, ainc, 0.05, 8.0, p['front_axle_from_lidar_mm'],
                                                              max_dist_mm=p['fixed_detect_range_mm']),
                                 'clusters_pair': pn.scan_clusters(rng, amin, ainc, 0.05, 8.0, p['front_axle_from_lidar_mm'], 0.0,
                                                                   max_dist_mm=max(1500.0, p['side_far_mm'] + 200.0),
                                                                   x_mode='extent', min_size_mm=p['car_min_size_mm']),
                                 'clusters_box': pn.scan_clusters(rng, amin, ainc, 0.05, 8.0, p['front_axle_from_lidar_mm'], 0.0,
                                                                  max_dist_mm=1500.0, x_mode='box')})
        if log is not None:
            log.append(dict(info))
        if logic.lane_cfg:
            offset = logic.lane_cfg.get('target_offset_mm', 0.0)
        kap = 0.0 if abs(v) < 1e-6 else (lane_w if k is None else v * k) / v
        rm = R_MIN if r_min is None else r_min               # 11/15: 실차 최대 조향 회전반경 (1114 bag 실측 약 640)
        kap = max(-1 / rm, min(1 / rm, kap))                 # 실제 차 최대 곡률
        if steer_lag_mm > 0.0:
            # 조향 서보가 따라오는 시간 (11/15): 끝에서 끝(2/R)까지 steer_lag_mm / 0.12 m/s 초.
            # 브리지는 속도 0.02 m/s 미만이면 조향을 0으로 보낸다 -> 섰다 출발하면 바퀴가 0부터 다시 꺾임
            want = kap if abs(v) >= 0.02 else 0.0
            rate = (2.0 / rm) / (steer_lag_mm / 120.0)       # 1/mm per s
            step = rate * dt
            kap = kap_prev + max(-step, min(step, want - kap_prev))
        kap_prev = kap
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
    if out is not None:
        out.update(pose=(x, y, th), logic=logic, start=(-900.0, y0))
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


def test_wait_line_ignores_line_until_armed():
    """미션 노드가 주차 표지/앞 선 근처를 알리기 전엔 흰 선을 봐도 주차를 시작하지 않는다."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS))
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'stoplines_mm': [300.0], 'end_lines_mm': [],
            'clusters': [], 'blocked': False}
    for i in range(40):
        _, _, info = logic.step({**base, 't': i * 0.05, 'dist_mm': i * 20.0, 'armed': False})
    assert info['phase'] == 'WAIT_LINE'
    _, _, info = logic.step({**base, 't': 2.0, 'dist_mm': 800.0, 'armed': True})
    assert info['phase'] == 'FOLLOW'


def test_wait_line_starts_on_pair_without_line():
    """표지를 흰 선 뒤에 확정해 선을 놓쳐도, armed 상태에서 1·3번 짝을 연속으로 보면 주차 모드로 간다."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS))
    pair = [(500.0, -420.0), (-540.0, -420.0)]          # 오른쪽 같은 줄, 사이 1040mm
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'stoplines_mm': [], 'end_lines_mm': [],
            'clusters': pair, 'blocked': False}
    _, _, info = logic.step({**base, 't': 0.0, 'dist_mm': 0.0, 'armed': False})
    for i in range(1, 5):                                 # armed 전: 짝이 보여도 안 켜짐
        _, _, info = logic.step({**base, 't': i * 0.05, 'dist_mm': i * 5.0, 'armed': False})
    assert info['phase'] == 'WAIT_LINE'
    for i in range(5, 7):                                 # armed 후 2번: 아직
        _, _, info = logic.step({**base, 't': i * 0.05, 'dist_mm': i * 5.0, 'armed': True})
    assert info['phase'] == 'WAIT_LINE'
    _, _, info = logic.step({**base, 't': 0.4, 'dist_mm': 40.0, 'armed': True})
    assert info['phase'] != 'WAIT_LINE' and info['trigger'] == 'pair'


def test_wait_line_single_limo_does_not_trigger():
    """커브 등에 리모 한 대만 서 있으면 짝이 안 생겨 주차 모드로 안 간다."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS))
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'stoplines_mm': [], 'end_lines_mm': [],
            'clusters': [(300.0, -420.0)], 'blocked': False, 'armed': True}
    for i in range(20):
        _, _, info = logic.step({**base, 't': i * 0.05, 'dist_mm': i * 20.0})
    assert info['phase'] == 'WAIT_LINE'


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


def test_col5_centerline_found_and_followed():
    """꺾은 뒤 정면 5번 리모로 2번 칸 세로 가운데선을 구한다 (윤서 아이디어 11/10). 라이다 위치가 5cm 틀려도 따라감."""
    for lidar in (-47.0, 3.0):
        out = {}
        n_in, hit, result = drive(FIVE, 2, {'front_axle_from_lidar_mm': lidar}, fov_deg=NARROW, camera=True, out=out)
        assert (n_in, hit) == (4, False), (lidar, n_in, result)
        lg = out['logic']
        assert lg.col5 is not None and lg.col5_n >= 3
        col5_x = lg.col5[0] + out['start'][0]                 # 자기 위치 좌표 -> 가상 트랙 좌표
        assert abs(col5_x - 1230.0) < 20.0, (lidar, col5_x)  # 2번 칸 가운데 x = 1230
        x, y, th = out['pose']
        assert abs(x + 105.0 * math.cos(th) - 1230.0) < 40.0     # 차체 중심 옆 위치


def test_pair_ignores_small_objects():
    """칸 사이·앞의 가는 기둥(표지판, 사람 다리)은 짝 후보가 아니다. 리모 면(220mm)은 남는다."""
    pole = (285.0, 315.0, -515.0, -485.0)            # 오른쪽 50cm, 굵기 3cm 기둥 (xmin, xmax, ymin, ymax)
    limo = (790.0, 1010.0, -760.0, -440.0)           # 리모 220 x 320
    rng, amin, ainc = raycast(0.0, 0.0, 0.0, [pole, limo], n=1440)
    small = pn.scan_clusters(rng, amin, ainc, 0.05, 8.0, 0.0, 0.0)
    big = pn.scan_clusters(rng, amin, ainc, 0.05, 8.0, 0.0, 0.0, min_size_mm=pn.DEFAULTS['car_min_size_mm'])
    assert len(small) == 2 and len(big) == 1
    assert big[0][0] > 700.0


def test_sign_post_does_not_trigger_early_turn():
    """11/12 실차: 입구 오른쪽 주차 표지판이 옆을 지나가도 EXTRA(일찍 꺾기)로 가지 않는다."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS))
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'end_lines_mm': [], 'blocked': False, 'armed': True}
    logic.step({**base, 't': 0.0, 'dist_mm': 0.0, 'stoplines_mm': [], 'clusters': []})
    _, _, info = logic.step({**base, 't': 0.1, 'dist_mm': 400.0, 'stoplines_mm': [200.0], 'clusters': []})
    assert info['phase'] == 'FOLLOW'
    phases = set()
    for i in range(40):                       # 표지판: 앞 300mm -> 뒤로 지나감 (오른쪽 400mm)
        x = 300.0 - i * 20.0
        _, _, info = logic.step({**base, 't': 0.2 + i * 0.1, 'dist_mm': 400.0 + i * 20.0, 'stoplines_mm': [],
                                 'clusters': [(x, -400.0)], 'clusters_pair': [], 'clusters_single': []})
        phases.add(info['phase'])
    assert phases == {'FOLLOW'}


def test_single_fallback_ignores_small_objects_when_enabled():
    """예비 규칙을 켜도 리모보다 작은 덩어리(clusters_single에 없음)로는 꺾지 않는다."""
    p = dict(pn.DEFAULTS); p['single_fallback'] = True
    logic = pn.ParkingV2(p)
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'end_lines_mm': [], 'blocked': False, 'armed': True}
    logic.step({**base, 't': 0.0, 'dist_mm': 0.0, 'stoplines_mm': [], 'clusters': []})
    logic.step({**base, 't': 0.1, 'dist_mm': 400.0, 'stoplines_mm': [200.0], 'clusters': []})
    phases = set()
    for i in range(40):
        x = 300.0 - i * 20.0
        _, _, info = logic.step({**base, 't': 0.2 + i * 0.1, 'dist_mm': 400.0 + i * 20.0, 'stoplines_mm': [],
                                 'clusters': [(x, -400.0)], 'clusters_pair': [], 'clusters_single': []})
        phases.add(info['phase'])
    assert 'EXTRA' not in phases


def _direct_logic(**over):
    p = dict(pn.DEFAULTS); p['skip_wait_line'] = True; p.update(over)
    return pn.ParkingV2(p)


def test_skip_wait_line_goes_follow_without_line():
    """11/12: skip_wait_line이면 흰 선·짝 없이 바로 FOLLOW (trigger=direct), 왼쪽 붙기 시작."""
    logic = _direct_logic()
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'end_lines_mm': [], 'blocked': False, 'armed': False,
            'stoplines_mm': [], 'clusters': []}
    _, _, info = logic.step({**base, 't': 0.0, 'dist_mm': 0.0})
    assert info['phase'] == 'FOLLOW' and info['trigger'] == 'direct'
    assert logic.lane_cfg == {'target_offset_mm': pn.DEFAULTS['target_offset_mm']}


def test_direct_no_pair_stops_after_follow_max_direct():
    """짝 없이 follow_max_direct_mm(3.5m)를 가면 FAIL_NO_CAR. 그 전(2.5m)에는 안 멈춤."""
    logic = _direct_logic()
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'end_lines_mm': [], 'blocked': False,
            'stoplines_mm': [], 'clusters': []}
    info = None
    for i in range(0, 200):
        d = i * 20.0
        _, _, info = logic.step({**base, 't': i * 0.1, 'dist_mm': d})
        if d == 3000.0:
            assert info['phase'] == 'FOLLOW'
        if info['phase'] == 'DONE':
            break
    assert info['result'] == 'FAIL_NO_CAR' and 3480.0 <= d <= 3560.0


def test_blocked_before_pair_waits_instead_of_ending():
    """짝 찾기 전에 앞이 막히면 BLOCKED로 끝내지 않고 서서 기다렸다가 다시 감."""
    logic = _direct_logic()
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'end_lines_mm': [], 'stoplines_mm': [], 'clusters': []}
    logic.step({**base, 't': 0.0, 'dist_mm': 0.0, 'blocked': False})
    for i in range(1, 10):
        v, _, info = logic.step({**base, 't': i * 0.1, 'dist_mm': 100.0, 'blocked': True})
        assert info['phase'] == 'FOLLOW' and v == 0.0
    v, _, info = logic.step({**base, 't': 1.1, 'dist_mm': 100.0, 'blocked': False})
    assert info['phase'] == 'FOLLOW' and v > 0.0


def test_empty_filtered_pair_list_does_not_fall_back_to_raw():
    """11/12 버그: 크기로 거른 목록(clusters_pair)이 비면 거르기 전 목록으로 돌아가 기둥 + 1번이 짝이 됐음."""
    logic = _direct_logic()
    base = {'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'end_lines_mm': [], 'stoplines_mm': [], 'blocked': False}
    raw = [(200.0, -420.0), (1240.0, -420.0)]      # 기둥(작음) + 1번, 사이 1040mm -> 거르기 전이면 짝
    for i in range(10):
        _, _, info = logic.step({**base, 't': i * 0.1, 'dist_mm': i * 10.0, 'clusters': raw, 'clusters_pair': []})
    assert info['phase'] == 'FOLLOW' and logic.pair_line is None


# ---------------------------------------------------------------- 11/12 1번 모서리 기준 꺾기 + 5번 앞 22cm 정지
NEW_1112 = {'skip_wait_line': True, 'car_min_size_mm': 150.0, 'turn_ref': 'corner', 'turn_from_corner_mm': 140.0,
            'stop_from_col5_mm': 220.0, 'max_realign': 0, 'guard_stop_mm': 20.0, 'pullout_mm': 500.0, 'pause_sec': 0.3}


def test_corner_turn_and_col5_stop_parks_in_one_go():
    """1번 모서리로 꺾는 시점을 잡고 5번 앞 22cm에 서면, 재정렬 없이 한 번에 2번 칸 안 (대회 배치, 시야 ±105도)."""
    log, out = [], {}
    n_in, hit, res = drive([1, 3, 4, 5, 6], 2, NEW_1112, fov_deg=210.0, log=log, out=out)
    assert (n_in, hit, res) == (4, False, 'PARKED')
    assert any(i.get('turn_ref') == 'corner' for i in log)
    assert log[-1]['realigns'] == 0 and log[-1]['stop_how'] == 'col5'
    x, y, th = out['pose']
    bumper_y = y + (105.0 + 161.0) * math.sin(th)            # 앞범퍼 (차가 -y로 들어감)
    assert bumper_y > -620.0 + 30.0                           # 칸 끝 선(-620)보다 3cm 이상 앞


def test_corner_turn_point_ignores_lidar_mount_error():
    """라이다 위치 값이 5cm 틀려도 모서리 기준이면 꺾는 위치가 같다 (짝 중점 평균은 5cm 넘게 달라짐)."""
    def turn_x(ov):
        seen = []
        orig = pn.ParkingV2._enter

        def rec(self, phase, inp):
            if phase == 'PAUSE' and not seen:
                seen.append(self.px)                          # 꺾기 직전 차 위치 (출발점 기준 실제 이동 거리)
            orig(self, phase, inp)
        pn.ParkingV2._enter = rec
        try:
            drive([1, 3, 4, 5, 6], 2, ov, fov_deg=210.0)
        finally:
            pn.ParkingV2._enter = orig
        return seen[0] if seen else None
    a = turn_x(NEW_1112)
    b = turn_x(dict(NEW_1112, front_axle_from_lidar_mm=-97.0))
    c = turn_x(dict(NEW_1112, turn_ref='pair'))
    d = turn_x(dict(NEW_1112, turn_ref='pair', front_axle_from_lidar_mm=-97.0))
    assert None not in (a, b, c, d)
    assert abs(a - b) <= 20.0 < abs(c - d)


def test_measure_corner_from_points():
    """1번 상자 차로 쪽 면 점들의 진행 방향 끝 = 모서리 (칸 가운데 기준 약 -410)."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS, turn_ref='corner'))
    logic.px, logic.py = 0.0, 0.0
    logic.pair_line = (1000.0, -600.0, 0.0, -1.0)              # 2번 칸 가운데 (1000, -600), 깊이 방향 -y
    # 앞바퀴 = (210, 0). 1번 중심 x = 480, 차로 쪽 면 y = -600 + 161 = -439, 폭 220 -> x 370~590
    pts = [(x - 210.0, -439.0) for x in range(380, 591, 15)] + [(590.0 - 210.0, -439.0 - d) for d in range(20, 200, 30)]
    sc = logic._measure_corner({'yaw': 0.0, 'points': pts})
    assert sc is not None and abs(sc - (-410.0)) <= 15.0


def test_mission_sends_parking_lane_offset():
    """11/12: 미션 노드가 주차 노드에 넘긴 뒤 차선 설정에 왼쪽 오프셋을 넣는다 (1초마다 다시 보내도 유지)."""
    from autorace_mission.fsm import Inputs, MissionFSM
    fsm = MissionFSM({'start_state': 'M4_PARKING', 'm4_direct': True, 'm4_target_offset_mm': 50.0})
    cmd = fsm.step(Inputs(t=0.0, dist_mm=0.0))
    assert cmd.source == 'parking' and cmd.lane.get('target_offset_mm') == 50.0


def test_target_turn_parks_and_ignores_lidar_mount_error():
    """11/13 target: 목표 = 1·3번 중심선 ∩ 5번 중심선. 한 번에 주차, 라이다 위치 값이 5cm 틀려도 꺾는 위치 같음."""
    ov = dict(NEW_1112, turn_ref='target', pair_tol_mm=230.0)
    log = []
    n_in, hit, res = drive([1, 3, 4, 5, 6], 2, ov, fov_deg=210.0, log=log)
    assert (n_in, hit, res) == (4, False, 'PARKED')
    assert any(str(i.get('turn_ref', '')).startswith('target_') for i in log)
    orig = pn.ParkingV2._enter

    def turn_px(o):
        seen = []

        def rec(self, phase, inp):
            if phase == 'PAUSE' and not seen:
                seen.append(self.px)
            orig(self, phase, inp)
        pn.ParkingV2._enter = rec
        try:
            drive([1, 3, 4, 5, 6], 2, o, fov_deg=210.0)
        finally:
            pn.ParkingV2._enter = orig
        return seen[0]
    assert abs(turn_px(ov) - turn_px(dict(ov, front_axle_from_lidar_mm=-97.0))) <= 20.0


# ---------------------------------------------------------------- 11/13 캡틴 경로 (path)
PATH_1113 = dict(NEW_1112, turn_ref='path', pair_tol_mm=230.0, stop_from_col5_mm=300.0, stop_turn_deg=85.0,
                 stop_hard_mm=200.0)


def test_path_parks_with_settle_and_move_to_q():
    """인식 -> SETTLE(정지) -> MOVE_Q -> PAUSE -> TURN -> 앞범퍼 R에서 정지, 칸 안, 재정렬 없음."""
    log, out = [], {}
    n_in, hit, res = drive([1, 3, 4, 5, 6], 2, PATH_1113, fov_deg=210.0, log=log, out=out)
    assert (n_in, hit, res) == (4, False, 'PARKED')
    phases = [i['phase'] for i in log]
    for ph in ('SETTLE', 'MOVE_Q', 'PAUSE', 'TURN'):
        assert ph in phases
    assert phases.index('SETTLE') < phases.index('MOVE_Q') < phases.index('PAUSE') < phases.index('TURN')
    assert str(log[-1]['turn_ref']).startswith('path_') and log[-1]['realigns'] == 0
    x, y, th = out['pose']
    assert y + (105.0 + 161.0) * math.sin(th) > -620.0 + 30.0        # 앞범퍼가 칸 끝 선 3cm 이상 앞


def test_path_move_q_reverses_when_past_q():
    """Q를 지나쳐 섰으면 후진으로 돌아온다."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS, **PATH_1113))
    logic.pair_line = (0.0, -600.0, 0.0, -1.0)                       # 칸 가운데 선: x = 0, 깊이 -y
    logic.q_s = -529.0
    logic.px, logic.py = -300.0, 0.0                                 # 뒷바퀴 s = -300 -> Q보다 229mm 지나침
    logic.phase, logic.t_phase, logic.d_phase = 'MOVE_Q', 0.0, 0.0
    v, k, info = logic.step({'t': 0.1, 'dist_mm': 0.0, 'yaw': 0.0, 'lane_v': 0.2, 'lane_w': 0.0, 'stoplines_mm': [],
                             'end_lines_mm': [], 'clusters': [], 'blocked': False})
    assert info['phase'] == 'MOVE_Q' and v < 0 and k == 0.0


def test_path_done_logs_once_flag():
    """주차 완료 로그는 한 번만 (노드가 done_logged로 막음)."""
    logic = pn.ParkingV2(dict(pn.DEFAULTS))
    assert logic.done_logged is False


# --- 11/13 실차: 양옆 정중앙, 앞 15cm, 후진 15cm, 전진<->후진 0.5초, 옆 감시에 걸리면 후진 뒤 다시 -----------------

def _yaml_parking(path_mode=True):
    """params.yaml 주차 값. path_mode: 11/16부터 기본이 fixed라, 예전 경로 방식 시험은 turn_ref만 path로 둔다."""
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', '..', 'autorace_bringup', 'config', 'params.yaml')
    d = yaml.safe_load(open(path, encoding='utf-8'))['parking_node']['ros__parameters']
    out = {k: v for k, v in d.items() if k in pn.DEFAULTS}
    if path_mode:
        out['turn_ref'] = 'path'
    return out


def test_corridor_mid_between_inner_faces():
    pts = [(100.0, 410.0), (120.0, 430.0), (90.0, -410.0), (110.0, -420.0), (100.0, -415.0)]
    x, y = pn.corridor_mid(pts, 0.0, 170.0)
    assert x == 85.0 and y == pytest.approx(0.0)
    assert pn.corridor_mid([(100.0, 410.0), (110.0, 420.0)], 0.0, 170.0) is None          # 한쪽만
    assert pn.corridor_mid([(100.0, 120.0), (110.0, 130.0), (100.0, -410.0), (110.0, -420.0)], 0.0, 170.0) is None  # 폭 530: 다른 물체


def test_params_yaml_values_from_captain():
    p = _yaml_parking()
    assert p['front_stop_mm'] == 150.0 and p['back_mm'] == 150.0 and p['switch_pause_sec'] == 0.5
    assert p['guard_back'] and p['center_on_sides'] is False and p['follow_col5'] is False   # 11/13 캡틴: 원을 접선까지
    assert p['center_check'] is True       # 11/14 캡틴: 다시 켬 (끝에서 선 밟음). 11/13 15:49엔 끔
    q = _yaml_parking(path_mode=False)
    assert q['turn_ref'] == 'fixed' and q['fixed_move_mm'] == 820.0 and q['fixed_turn_mm'] == 0.0 and q['fixed_back_mm'] == 50.0   # 11/16 실차 870, 11/18 현장 -50, 11/18 캡틴 5cm 후진


def _step_inp(t, dist, blocked=False, yaw=0.0):
    return {'t': t, 'dist_mm': dist, 'yaw': yaw, 'lane_v': 0.18, 'lane_w': 0.0, 'stoplines_mm': [], 'clusters': [],
            'blocked': blocked, 'end_lines_mm': [], 'points': [], 'clusters_pair': [], 'clusters_box': []}


def test_blocked_pauses_then_backs_back_mm_then_pauses_again():
    p = dict(pn.DEFAULTS, switch_pause_sec=0.5, back_mm=150.0)
    lg = pn.ParkingV2(p)
    lg._enter('TURN', _step_inp(0.0, 0.0))
    lg.yaw_turn0 = 0.0
    v, _, _ = lg.step(_step_inp(0.05, 0.0, blocked=True))
    assert lg.phase == 'SWITCH' and v == 0.0
    v, _, _ = lg.step(_step_inp(0.05 + 0.5 - 0.45, 0.0))
    assert lg.phase == 'SWITCH' and v == 0.0                       # 0.5초 전: 아직 멈춤
    v, _, _ = lg.step(_step_inp(0.30, 0.0))
    assert lg.phase == 'SWITCH' and -0.03 <= v < 0.0               # 11/15: 끝 0.4초는 후진 조향을 미리 (아주 느리게)
    v, _, _ = lg.step(_step_inp(0.56, 0.0))
    assert lg.phase == 'BACK' and v < 0.0
    v, _, _ = lg.step(_step_inp(1.5, 100.0))
    assert lg.phase == 'BACK' and v < 0.0                          # 15cm 전
    v, _, _ = lg.step(_step_inp(2.0, 151.0))
    assert lg.phase == 'SWITCH' and v == 0.0                       # 15cm 후진 끝 -> 다시 멈춤
    lg.step(_step_inp(2.6, 151.0))
    assert lg.phase == 'TURN'


def test_side_guard_backs_up_instead_of_ending():
    """11/13 실차: 꺾는 중 옆 리모 1.9cm -> GUARD_STOP으로 칸에 걸친 채 끝. 이제 후진 뒤 다시 (3번까지)."""
    p = dict(pn.DEFAULTS, guard_stop_mm=20.0, max_realign=0, guard_back=True, guard_back_max=3, switch_pause_sec=0.5)
    lg = pn.ParkingV2(p)
    lg._enter('TURN', _step_inp(0.0, 0.0))
    lg.yaw_turn0 = 0.0
    lg._cell = lambda: (0.0, 0.0, 1.0, 0.0)
    lg._side_gap = lambda inp: 19.0
    lg.step(_step_inp(0.1, 0.0, yaw=math.radians(30)))
    assert lg.phase == 'SWITCH' and lg.switch_to == 'BACK' and lg.err[0] == 'guard_back'
    lg.guard_backs = 3
    lg._enter('TURN', _step_inp(1.0, 0.0))
    lg.step(_step_inp(1.1, 0.0, yaw=math.radians(30)))
    assert lg.result == 'GUARD_STOP'                               # 횟수를 다 쓰면 예전처럼


@pytest.mark.parametrize('lead', [153.0, 193.0, 353.0, 433.0])
def test_centers_between_side_limos_even_with_mistimed_turn(lead):
    """꺾는 시점이 틀려도(실차 조정 전) 양옆 리모 정중앙에 다시 맞춰 네 모서리 안, 충돌 없음.
    예전(params 11/13 오전): 옆 오차 최대 95mm, 433이면 모서리 3개만."""
    o = _yaml_parking()
    o['turn_lead_lidar_mm'] = lead
    out = {}
    n_in, hit, result = drive(FIVE, 2, o, fov_deg=210.0, camera=True, out=out)
    x, y, th = out['pose']
    x0, x1, _, _ = cell(2)
    lat = x + 105.0 * math.cos(th) - (x0 + x1) / 2.0
    assert (n_in, hit) == (4, False) and result.startswith('PARKED'), (lead, n_in, result)
    assert abs(lat) <= 35.0, (lead, lat)


def test_all_defaults_are_ros_parameter_types():
    """11/13: 기본값이 ROS 파라미터로 등록 가능한 모양이어야 함 (((a,b),(c,d))면 노드가 켜지자마자 죽음)."""
    ok_scalar = (bool, int, float, str)
    for k, v in pn.DEFAULTS.items():
        if isinstance(v, list):
            assert v and len({type(x) for x in v}) == 1 and isinstance(v[0], ok_scalar), k
        else:
            assert isinstance(v, ok_scalar), k


@pytest.mark.parametrize('ov', [{'arc_radius_mm': 569.0}, {'arc_radius_mm': 489.0}, {'front_axle_from_lidar_mm': -97.0}])
def test_center_check_works_without_center_on_sides(ov):
    """11/14: center_on_sides 끔(원은 접선까지) + center_check 켬이면 끝에서 양옆 정중앙을 재서 15mm 넘으면 다시 넣는다.
    예전엔 center_on_sides일 때만 재서 center_check가 아무 일도 안 했다 (회전반경 -4cm: 가운데서 +48mm 그대로)."""
    o = dict(_yaml_parking(), **ov)
    o['center_on_sides'], o['center_check'] = False, True
    o['turn_fix'] = False          # 11/15: 도는 중 바로잡기를 켜면 처음부터 가운데라 다시 맞추기가 안 일어남 -> 이 시험은 끔
    o['center_tol_mm'] = 15.0      # 11/16: 실차 값은 50(±20% 마진). 다시 맞추기 동작 자체를 보려고 예전 15
    out, log = {}, []
    n_in, hit, result = drive(FIVE, 2, o, fov_deg=210.0, camera=True, out=out, log=log)
    x, y, th = out['pose']
    x0, x1, _, _ = cell(2)
    lat = x + 105.0 * math.cos(th) - (x0 + x1) / 2.0
    assert (n_in, hit) == (4, False) and result.startswith('PARKED')
    assert abs(lat) <= 20.0, lat
    assert any((i.get('last_err') or (None,))[0] == 'center' for i in log)



@pytest.mark.parametrize('r_true,lag', [(600.0, 0.0), (640.0, 120.0), (700.0, 120.0), (530.0, 120.0)])
def test_turn_fix_keeps_center_when_real_radius_differs(r_true, lag):
    """11/15 (1114 bag): 실제 최대 조향 반경, 조향 서보 지연이 달라도 가운데 다시 맞추기 없이(center_check 끔)
    도는 중 바로잡기 + 미리 꺾기로 칸 안(모서리 4개). 예전(529, 바로잡기 없음)은 R 640에서 +118~125 mm(모서리 3개)."""
    o = dict(_yaml_parking(), center_check=False)
    out = {}
    n_in, hit, result = drive(FIVE, 2, o, fov_deg=210.0, camera=True, out=out, r_min=r_true, steer_lag_mm=lag)
    x, y, th = out['pose']
    x0, x1, _, _ = cell(2)
    lat = x + 105.0 * math.cos(th) - (x0 + x1) / 2.0
    assert (n_in, hit) == (4, False) and result.startswith('PARKED')
    assert abs(lat) <= 80.0, lat      # 11/16: 고치는 문턱 50 mm·70%만 고침 -> 첫 진입만으로 ±80 안 (남은 건 center_check ±50이 맡음)


def test_turn_fix_backs_with_left_lock_when_overshooting():
    """계획 반경 529인데 실제 700: 도는 중 왼쪽 최대 조향 후진(끊어 넣기)이 일어난다."""
    o = dict(_yaml_parking(), center_check=False, arc_radius_mm=529.0)
    log = []
    drive(FIVE, 2, o, fov_deg=210.0, camera=True, log=log, r_min=700.0)
    fixes = [i['last_err'] for i in log if i.get('last_err') and i['last_err'][0] == 'turn_fix_back']
    assert fixes and fixes[0][1] > 35
    assert any(i['phase'] == 'BACK' for i in log)


def test_back_uses_opposite_10_percent_steering():
    """11/14: 주차 중 앞이 막히거나 옆 감시로 BACK 후진할 때, 직전 전진 조향(오른쪽 -)의 반대 방향(+) 10% 조향."""
    p = dict(pn.DEFAULTS, switch_pause_sec=0.0, back_mm=150.0, back_reverse_scale=0.10)
    lg = pn.ParkingV2(p)
    lg._enter('TURN', _step_inp(0.0, 0.0))
    lg.yaw_turn0 = 0.0
    v_fwd, k_fwd, _ = lg.step(_step_inp(0.05, 10.0, blocked=False))
    assert v_fwd > 0.0 and k_fwd == pn.KAPPA_FULL_RIGHT
    lg.step(_step_inp(0.10, 20.0, blocked=True))                   # TURN -> BACK 전환 틱
    assert lg.phase == 'BACK'
    v_back, k_back, _ = lg.step(_step_inp(0.15, 20.0, blocked=True))
    assert lg.phase == 'BACK' and v_back < 0.0
    assert k_back == pytest.approx(-0.10 * pn.KAPPA_FULL_RIGHT, abs=1e-6)




def test_center_check_margin_and_short_pullout():
    """11/16: 정중앙 ±50 mm면 끝(다시 안 함). 다시 할 때 후진은 짧게(320 mm, 오차로 안 늘림)."""
    o = dict(_yaml_parking(), turn_fix=False, presteer_sec=0.0, arc_radius_mm=529.0)
    log, out = [], {}
    n_in, hit, result = drive(FIVE, 2, o, fov_deg=210.0, camera=True, out=out, log=log, r_min=640.0)
    assert (n_in, hit) == (4, False) and result.startswith('PARKED')
    x, y, th = out['pose']
    x0, x1, _, _ = cell(2)
    assert abs(x + 105.0 * math.cos(th) - (x0 + x1) / 2.0) <= 55.0
    pulls = [i for i in log if i['phase'] == 'PULLOUT']
    assert pulls                                          # 120 mm 넘게 벗어나 한 번은 다시 함
    errs = [i['last_err'] for i in log if i.get('last_err') and i['last_err'][0] == 'center']
    assert all(abs(e[1]) > 50 for e in errs)              # 50 안이면 다시 안 함


# ---- 11/16 캡틴 고정 거리 방식 (turn_ref: fixed) ----
# 시험 트랙(흰 선 -> 주차 구역 500mm)은 실제 트랙과 달라 직진 거리를 940으로 맞춰 논리만 시험한다.
# 실제 값 1040은 캡틴이 실제 트랙에서 앞범퍼를 선에 대고 잰 값 (11/16).
FIXED = {'turn_ref': 'fixed', 'front_stop_mm': 150.0, 'fixed_move_mm': 940.0, 'fixed_turn_mm': 0.0,   # 시험 트랙은 87도로 끝
         'fixed_cells_move_mm': [420.0, 940.0, 1460.0]}                       # 시험 트랙 칸 간격 520


@pytest.mark.parametrize('r_min', [560.0, 590.0, 620.0, 650.0])
@pytest.mark.parametrize('lag', [0.0, 120.0])
def test_fixed_parks_in_cell2(r_min, lag):
    """주차선 정지 -> 104cm -> 최대 조향 87도: 실제 반경 560~650mm, 서보 지연 0~120mm 모두 2번 칸 안, 충돌 없음."""
    n_in, hit, res = drive([1, 3, 5], 2, FIXED, y0=290.0, r_min=r_min, steer_lag_mm=lag)
    assert n_in == 4 and not hit and res == 'PARKED_FIXED'


def test_fixed_stops_at_line_then_moves_1040():
    out, log = {}, []
    drive([1, 3, 5], 2, FIXED, y0=290.0, r_min=590.0, out=out, log=log)
    lg = out['logic']
    assert abs(lg.turn_at["line_mm"] - 56.0) <= 10.0       # 앞범퍼가 선에 닿게 (앞바퀴 5.6cm 앞)
    assert lg.q_s == pytest.approx(940.0, abs=15.0)
    phases = [i['phase'] for i in log]
    assert phases.index('F_PAUSE') < phases.index('F_MOVE') < phases.index('F_TURN')


def test_fixed_turn_mm_ends_turn_early():
    """fixed_turn_mm 400 (각도 끔): 반경 590이면 약 39도에서 끝 -> 칸에 덜 들어감 (캡틴 '40cm' 값 확인용)."""
    out = {}
    drive([1, 3, 5], 2, dict(FIXED, fixed_turn_mm=400.0, fixed_turn_deg=0.0), y0=290.0, r_min=590.0, out=out)
    assert 30.0 <= -math.degrees(out['pose'][2]) <= 45.0


def test_fixed_defaults_captain_values():
    assert pn.DEFAULTS['fixed_move_mm'] == 870.0 and pn.DEFAULTS['fixed_turn_mm'] == 0.0 and pn.DEFAULTS['fixed_line_target_mm'] == 56.0
    assert pn.DEFAULTS['fixed_line_target_mm'] == pn.DEFAULTS['front_bumper_from_axle_mm']


def test_fixed_no_line_fails_and_stops():
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED))
    v, d, info = 0.18, 0.0, None
    for i in range(2000):
        v, k, info = lg.step({'t': i * 0.05, 'dist_mm': d, 'yaw': 0.0, 'lane_v': 0.18, 'lane_w': 0.0,
                              'stoplines_mm': [], 'clusters': [], 'blocked': False})
        d += v * 50.0
        if info['done']:
            break
    assert info['result'] == 'FAIL_NO_LINE' and v == 0.0
    assert d == pytest.approx(pn.DEFAULTS['fixed_line_max_mm'], abs=20.0)


def _fixed_lines(first_lines, line_at):
    """앞바퀴 기준 선 거리를 직접 주는 1차원 시험: first_lines = 시작 때 보이는 다른 선들(mm), line_at = 주차선 거리."""
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED))
    d, info, stop_d = 0.0, None, None
    for i in range(3000):
        ls = sorted([x - d for x in first_lines + [line_at] if 60.0 <= x - d <= 700.0])
        v, k, info = lg.step({'t': i * 0.05, 'dist_mm': d, 'yaw': 0.0, 'lane_v': 0.18, 'lane_w': 0.0,
                              'stoplines_mm': ls, 'clusters': [], 'blocked': False})
        if lg.phase == 'F_PAUSE' and stop_d is None:
            stop_d = d
        d += v * 50.0
        if info['done']:
            break
    return lg, stop_d


def test_fixed_practice_line_30cm_ahead_is_caught():
    """런 가이드: 주차 앞 선 30cm 앞(앞범퍼)에 놓고 set_state -> 앞바퀴 기준 약 356mm. 예전 ignore 300이면 지나침."""
    lg, stop_d = _fixed_lines([], 356.0)
    assert stop_d == pytest.approx(356.0 - 56.0, abs=15.0)


def test_fixed_full_run_skips_crosswalk_end_line():
    """통주행 M4 시작: 횡단보도 끝 선 약 120mm 앞, 주차선 약 1950mm 앞 -> 횡단보도 선에서 서지 않음."""
    lg, stop_d = _fixed_lines([120.0], 1950.0)
    assert stop_d == pytest.approx(1950.0 - 56.0, abs=15.0)


def _fixed_lane_lost(line_at, lost_below, over=None, blocked_until=-1.0):
    """11/20 simcheck M4: 선이 lost_below(mm)보다 가까워지면 차선 노드가 선을 놓쳐 lane_v = 0 (선도 카메라 밖).
    이때도 주차선 fixed_line_target_mm 앞에서 서야 함. blocked_until(초)까지는 앞에 물체."""
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED, **(over or {})))
    d, info, stop_d, th = 0.0, None, None, 0.0
    for i in range(3000):
        rel = line_at - d
        ls = [rel] if lost_below <= rel <= 700.0 else []
        lane_v = 0.18 if rel > lost_below else 0.0
        v, k, info = lg.step({'t': i * 0.05, 'dist_mm': d, 'yaw': th, 'lane_v': lane_v, 'lane_w': 0.0,
                              'stoplines_mm': ls, 'clusters': [], 'blocked': i * 0.05 < blocked_until})
        if lg.phase == 'F_PAUSE' and stop_d is None:
            stop_d = d
            break
        d += v * 50.0
        th += (k or 0.0) * v * 0.05 if v else 0.0
        if info['done']:
            break
    return lg, stop_d


def test_fixed_lane_lost_after_line_still_reaches_line():
    """시뮬 재현: 선 380 mm 앞에서 차선이 멈춤(속도 0) -> 예전엔 F_LINE에서 영원히 정지. 이제 선까지 감."""
    lg, stop_d = _fixed_lane_lost(616.0, 380.0)
    assert stop_d == pytest.approx(616.0 - 56.0, abs=15.0)
    assert lg._info()['creep_n'] >= 1


def test_fixed_lane_lost_creep_off_keeps_old_behaviour():
    lg, stop_d = _fixed_lane_lost(616.0, 380.0, over={'fixed_line_creep': False})
    assert stop_d is None and lg.phase == 'F_LINE'


def test_fixed_lane_lost_creep_waits_for_front_object():
    """선 놓친 뒤 앞에 물체(라이다 blocked)면 기다렸다가 사라지면 감."""
    lg, stop_d = _fixed_lane_lost(616.0, 380.0, blocked_until=8.0)
    assert stop_d == pytest.approx(616.0 - 56.0, abs=15.0)


def test_fixed_stale_line_at_arm_is_not_caught():
    """시뮬 재현: 켜지는 순간 낡은 선 1636 mm가 0.4초 남아 있다가 사라지고, 진짜 선은 400 mm 앞 -> 진짜 선에서 섬."""
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED))
    d, stop_d = 0.0, None
    for i in range(3000):
        t = i * 0.05
        # 0.4초 동안 새 메시지 없음(번호 그대로, 낡은 선) -> 그 뒤 메시지마다 번호 +1, 진짜 선 400 mm
        ls = [1636.0] if t < 0.4 else sorted(x for x in [400.0 - d] if 60.0 <= x <= 700.0)
        seq = 10 if t < 0.4 else 10 + i
        v, k, info = lg.step({'t': t, 'dist_mm': d, 'yaw': 0.0, 'lane_v': 0.18, 'lane_w': 0.0,
                              'stoplines_mm': ls, 'clusters': [], 'blocked': False, 'lane_seq': seq})
        if lg.phase == 'F_PAUSE':
            stop_d = d
            break
        d += v * 50.0
    assert stop_d == pytest.approx(400.0 - 56.0, abs=15.0)


def test_fixed_practice_line_caught_while_standing():
    """런 가이드 자리(선 352 mm, 카메라 아래 한계 근처): 서 있는 동안 새 메시지 2개면 바로 잡음 (움직일 필요 없음)."""
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED))
    d, stop_d = 0.0, None
    for i in range(400):
        rel = 352.0 - d
        ls = [rel] if rel >= 340.0 else []                  # 1 cm만 가도 카메라 밖
        v, k, info = lg.step({'t': i * 0.05, 'dist_mm': d, 'yaw': 0.0, 'lane_v': 0.0 if i < 4 else 0.18, 'lane_w': 0.0,
                              'stoplines_mm': ls, 'clusters': [], 'blocked': False, 'lane_seq': i})
        if lg.phase == 'F_PAUSE':
            stop_d = d
            break
        d += v * 50.0
    assert stop_d == pytest.approx(352.0 - 56.0, abs=15.0)


def test_fixed_no_line_seen_lane_stop_does_not_creep():
    """선을 아직 못 잡았으면(end_d 없음) 차선이 멈출 때 같이 멈춤 (아무 데로나 가지 않음)."""
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED))
    for i in range(40):
        v, k, info = lg.step({'t': i * 0.05, 'dist_mm': 0.0, 'yaw': 0.0, 'lane_v': 0.0, 'lane_w': 0.0,
                              'stoplines_mm': [], 'clusters': [], 'blocked': False})
    assert v == 0.0 and lg.phase == 'F_LINE'


# ---- 11/17 공지: 빈 칸 랜덤 (한 칸만 비움) -> 선에서 라이다로 빈 칸 찾기 ----
@pytest.mark.parametrize('empty', [1, 2, 3])
@pytest.mark.parametrize('r_min', [590.0, 620.0])
def test_fixed_finds_empty_cell_and_parks(empty, r_min):
    parked = [i for i in range(1, 7) if i != empty]          # 나머지 다섯 칸은 모두 막음
    out, log = {}, []
    n_in, hit, res = drive(parked, empty, FIXED, y0=290.0, r_min=r_min, steer_lag_mm=60.0, out=out, log=log)
    lg = out['logic']
    assert lg.fixed_cell == empty and lg.cell_src == 'agree' and lg.cell_a == empty and lg.cell_b == empty
    assert n_in == 4 and not hit and res == 'PARKED_FIXED'


def test_fixed_force_cell_overrides_detection():
    out = {}
    drive([1, 3, 4, 5, 6], 2, dict(FIXED, fixed_force_cell=2), y0=290.0, r_min=590.0, out=out)
    assert out['logic'].fixed_cell == 2 and out['logic'].cell_src == 'force'


def test_pick_cell_a_b_rules():
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **FIXED))
    lg.cell_frames, lg.detect_extra = 10, False
    lg.cell_cnt, lg.cell_occ = [100, 0, 80], [10, 0, 10]        # 둘 다 2번
    assert lg._pick_cell() == 2 and lg.cell_src == 'agree'
    lg.cell_cnt, lg.cell_occ = [100, 0, 80], [10, 0, 0]         # A: 2·3번 둘 다 빔 -> B
    assert lg._pick_cell() == 2 and lg.cell_src == 'a_fail_b'
    lg.cell_cnt, lg.cell_occ = [100, 5, 0], [10, 0, 10]         # A 2번, B 3번 -> 처음엔 더 봄
    assert lg._pick_cell() is None
    lg.detect_extra = True
    assert lg._pick_cell() == 3 and lg.cell_src == 'mismatch_b'


def test_fixed_backs_up_5cm_straight_after_turn():
    """11/18 캡틴: 다 돈 뒤 바퀴 펴고 5cm 똑바로 후진 -> PARKED_FIXED. 후진 중 조향 0."""
    out, log = {}, []
    n_in, hit, res = drive([1, 3, 4, 5, 6], 2, FIXED, y0=290.0, r_min=590.0, out=out, log=log)
    lg = out['logic']
    assert res == 'PARKED_FIXED' and n_in == 4 and not hit
    assert lg.back_done_mm == pytest.approx(50.0, abs=8.0)
    ph = [i['phase'] for i in log]
    assert ph.index('F_TURN') < ph.index('F_BPAUSE') < ph.index('F_BACK')


def test_fixed_back_zero_skips():
    out, log = {}, []
    drive([1, 3, 4, 5, 6], 2, dict(FIXED, fixed_back_mm=0.0), y0=290.0, r_min=590.0, out=out, log=log)
    assert 'F_BACK' not in [i['phase'] for i in log]


# ---- 11/19 캡틴: 가짜 선(횡단보도 끝 선) 거르기 ----
def _fixed_two_lines(fake_at, real_at, empty=2, need=2):
    """1차원 시험: 앞바퀴 기준 선 두 개. 진짜 선 근처에서만 1·2·3번 중 빈 칸 빼고 점을 준다."""
    lg = pn.ParkingV2(dict(pn.DEFAULTS, **dict(FIXED, fixed_need_occ=need)))
    d, info, stops = 0.0, None, []
    for i in range(4000):
        ls = sorted([x - d for x in (fake_at, real_at) if 60.0 <= x - d <= 700.0])
        pts = []
        if abs((real_at - d) - 56.0) < 80.0:                     # 진짜 선에 서 있을 때만 리모가 보임
            for c in range(3):
                if c + 1 != empty:
                    x0, x1, y0, y1 = lg._cell_box(c)
                    pts += [((x0 + x1) / 2 + k * 15.0, -450.0) for k in range(-3, 4)]
        prev = lg.phase
        v, k, info = lg.step({'t': i * 0.05, 'dist_mm': d, 'yaw': 0.0, 'lane_v': 0.18, 'lane_w': 0.0,
                              'stoplines_mm': ls, 'clusters': [], 'blocked': False, 'points_far': pts})
        if lg.phase == 'F_PAUSE' and prev != 'F_PAUSE':
            stops.append(round(d))
        d += abs(v) * 50.0
        if info['done'] or lg.phase == 'F_MOVE':
            break
    return lg, stops


def test_fixed_rejects_crosswalk_line_then_parks_at_real_line():
    lg, stops = _fixed_two_lines(fake_at=400.0, real_at=2300.0, empty=1)
    assert len(stops) == 2 and lg.rejects == 1
    assert stops[1] == pytest.approx(2300.0 - 56.0, abs=20.0)
    assert lg.fixed_cell == 1


def test_fixed_need_occ_zero_keeps_old_behavior():
    lg, stops = _fixed_two_lines(fake_at=400.0, real_at=2300.0, need=0)
    assert len(stops) == 1 and getattr(lg, 'rejects', 0) == 0


# ---- 11/19 공지: 1·2행 랜덤 -> 뒷칸 되살림 ----
@pytest.mark.parametrize('front,back', [(1, 4), (2, 5), (3, 6)])
def test_fixed_back_row_target(front, back):
    parked = [i for i in range(1, 7) if i not in (front, back)]
    out, log = {}, []
    n_in, hit, res = drive(parked, back, FIXED, y0=290.0, r_min=590.0, out=out, log=log)
    lg = out['logic']
    assert lg.fixed_cell == front and lg.row == 'back'
    assert res == 'PARKED_FIXED_BACK' and n_in == 4 and not hit
    assert 'F_BACK' not in [i['phase'] for i in log]          # 뒷칸은 5cm 후진 안 함 (보류)


def test_fixed_front_row_still_backs_5cm():
    out, log = {}, []
    n_in, hit, res = drive([1, 3, 4, 5, 6], 2, FIXED, y0=290.0, r_min=590.0, out=out, log=log)
    ph = [i['phase'] for i in log]
    assert out['logic'].row == 'front' and res == 'PARKED_FIXED' and n_in == 4 and not hit
    assert ph.index('F_CHECK') < ph.index('F_BACK')
