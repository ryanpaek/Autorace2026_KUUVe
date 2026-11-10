"""통주행 미션 로직 시뮬레이션 (ROS 없이). 출발부터 주차까지 상태 전환이 어디서·왜 일어나는지 본다.

진짜 코드를 그대로 돌리는 것:
  MissionFSM(fsm.py), RouteTracker(route.py), ClassCommandRouter + 노드의 서행 노면선 규칙(class_commands.py),
  CommandInbox, 파라미터는 config/params.yaml의 mission_node 값.
가짜(모형)로 넣는 것 — 실차와 다를 수 있음:
  차선 주행(경로 중심선을 완벽히 따라감), 카메라 흰 선(앞바퀴 350~1500mm 앞, 85% 프레임, ±10mm),
  YOLO(표지 보이는 구간에서 90% 프레임), IMU 방지턱(위치는 도면에 없어 서행 선 + 600mm로 가정),
  M7 회피 노드(섬 앞에서 m7_done만 줌), 라바콘(양쪽 통로, 400mm 간격), 주차(앞 선을 지나면 끝난 것으로).

  python3 tools/course_sim.py                 # 기본 + 실패 상황 몇 개
  python3 tools/course_sim.py --scenario all  # 같은 것
"""
import argparse
import os
import random
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for pkg in ('autorace_mission', 'autorace_perception'):
    sys.path.insert(0, os.path.join(ROOT, 'src', pkg))

from autorace_mission.command_inbox import CommandInbox  # noqa: E402
from autorace_mission.fsm import DEFAULT_PARAMS, Inputs, MissionFSM  # noqa: E402
from autorace_mission.route import RouteMap, RouteTracker  # noqa: E402
from autorace_perception.class_commands import ClassCommandRouter  # noqa: E402

CFG = os.path.join(ROOT, 'src', 'autorace_bringup', 'config')
DT = 0.05
WHEELBASE = 200.0


def load_params():
    y = yaml.safe_load(open(os.path.join(CFG, 'params.yaml')))
    mp = y['mission_node']['ros__parameters']
    cp = y.get('class_command_node', {}).get('ros__parameters', {})
    return {k: v for k, v in mp.items() if k in DEFAULT_PARAMS}, mp, cp


def sign_windows(P):
    """표지가 카메라에 보이는 앞바퀴 위치 구간 (가정). 클래스 -> [(from, to)]."""
    return {
        'slow': [(P['m5_slow_line'] - 1500, P['m5_slow_line'] + 100)],
        'bump': [(P['m5_slow_line'] - 1500, P['m5_slow_line'] + 100)],
        'pedestrian': [(P['m3_stop_line'] - 1500, P['m3_stop_line'])],
        'fork_b': [(P['m7_island_tip'] - 2500, P['m7_island_tip'] - 200)],
        'parking': [(P['m1_crosswalk_end'], P['m4_parking_to'])],
    }


def run(name, miss=(), odom_scale=1.0, jump_at=None, camera=True, seed=1, verbose=True, guard_source=True,
        m7_reverse=None, signed_odom=True):
    """m7_reverse=(횟수, m): M7 회피 중 막혀서 이만큼 후진 (실차 11/10). signed_odom: 후진을 거리에서 빼는지 (mission_node 11/10부터 뺌)"""
    rnd = random.Random(seed)
    route = RouteMap.from_yaml(os.path.join(CFG, 'track_route.yaml'))
    P = dict(route.points)
    fsm_p, mp, cp = load_params()
    fsm_p.update(route_points=route.points, m7_external_avoid=True)
    fsm = MissionFSM(fsm_p)
    tracker = RouteTracker(route, gate_mm=mp.get('route_gate_mm', 250.0), gain=mp.get('route_gain', 0.6))
    router = ClassCommandRouter(cp.get('conf_threshold', 0.5), cp.get('yellow_action', 'none'),
                                cp.get('vote_window', 10), cp.get('vote_need', 7))
    inbox = CommandInbox()
    win = sign_windows(P)
    lms = route.landmarks
    bump_s = P['m5_slow_line'] + 600.0
    cones = [(s, side) for s in range(int(P['m8_start_line'] + 300), int(P['m8_end_line'] - 100), 400)
             for side in (-260, 260)]

    t, s, odo, jumped = 0.0, 0.0, 0.0, 0.0
    route_fixes, ground_at, slow_ground = None, None, False
    det_mode = None
    red_until = None
    events, log_seen = [], 0
    stop_at = {}
    tracker.reset(0.0, 0.0)
    started = False
    rev_left, rev_m, rev_run = (m7_reverse or (0, 0.0))[0], (m7_reverse or (0, 0.0))[1], 0.0
    rev_at = None
    for k in range(int(900 / DT)):
        # --- 카메라 흰 선 (차선 노드) -------------------------------------------------
        lines, xw = [], []
        if camera and rnd.random() < 0.85:
            for lm in lms:
                d = lm['s'] - s
                if 350.0 <= d <= 1500.0:
                    dd = d + rnd.gauss(0, 10)
                    lines.append(dd)
                    if lm.get('kind') == 'crosswalk':
                        xw.append(dd)
        if jump_at is not None and not jumped and s >= jump_at:
            jumped = 7_500_000.0
        dist = odo * odom_scale + jumped
        if guard_source:          # mission_node._on_odom: 한 메시지 200mm 넘게 튀면 버림
            dist = odo * odom_scale
        route_s = tracker.update(dist, lines if lines or camera else None, xw)
        # --- YOLO 5Hz -> 명령 노드 ----------------------------------------------------
        st = tracker.state()
        if fsm.state != det_mode:
            det_mode, ground_at, slow_ground, route_fixes = fsm.state, None, False, None
        router.set_mode(fsm.state)
        f = st.get('fixes')
        if route_fixes is not None and f > route_fixes:
            slow_ground = (st.get('last_fix') or {}).get('name') == 'm5_slow_line'
            ground_at = t
        route_fixes = f
        if k % 4 == 0:
            dets = []
            for cls, ws in win.items():
                if cls in miss:
                    continue
                if any(a <= s <= b for a, b in ws) and rnd.random() < 0.9:
                    dets.append({'cls': cls, 'conf': 0.8, 'bbox': [0, 0, 40, 40]})
            if fsm.state == 'M1_TRAFFIC' and P['m1_stop_line'] - 2000 <= s <= P['m1_stop_line'] + 300:
                if fsm.sub == 'stopped' and red_until is None:
                    red_until = t + 3.0
                light = 'green' if (red_until is not None and t >= red_until) else 'red'
                if light not in miss and rnd.random() < 0.9:
                    dets.append({'cls': light, 'conf': 0.8, 'bbox': [0, 0, 40, 60]})
            ground = slow_ground and ground_at is not None and t - ground_at <= 1.0
            out = router.route({'stamp': t, 'detections': dets}, slow_ground=ground)
            if out is not None:
                inbox.receive(out, fsm.state)
        inbox.set_mode(fsm.state)
        # --- 그 밖의 노드 모형 ------------------------------------------------------
        tilt = 0.6 if (s - 15 < bump_s <= s or s - 15 < bump_s + WHEELBASE <= s) else 0.05
        obstacle = {}
        if fsm.state == 'M7_OBSTACLE':
            obstacle = {'m7_active': s >= P['m7_start_line'],
                        'm7_done': s >= P['m7_island_tip'] - 300, 'm7_exit': 'B'}
        if fsm.state == 'M8_CONES':
            ahead = [cs - s for cs, _ in cones if 0 < cs - s <= 1500]
            obstacle = {'cone_any_ahead_mm': min(ahead) if ahead else None, 'cones_done': False,
                        'cone_ahead_mm': None}
        lane = {'status': 'NONE' if P['m8_start_line'] < s < P['m8_end_line'] + 200 else 'BOTH',
                'stoplines_mm': lines}
        parking = {'done': fsm.state == 'M4_PARKING' and fsm.parking_armed and s > P['m4_parking_from'] + 300}
        inp = Inputs(t=t, dist_mm=dist, route_s=route_s, lane=lane, obstacle=obstacle, parking=parking,
                     confirmed_classes=dict(inbox.classes), new_detection_frame=(k % 4 == 0),
                     start=not started, tilt_rate=tilt)
        started = True
        prev = (fsm.state, fsm.sub)
        cmd = fsm.step(inp)
        if (fsm.state, fsm.sub) != prev:
            why = fsm.log[-1] if fsm.log else ''
            events.append((t, s, f'{fsm.state}/{fsm.sub}', why))
        if cmd.source == 'stop' and fsm.state in ('M3_PEDESTRIAN', 'M1_TRAFFIC'):
            stop_at.setdefault(fsm.state, s)
        v = 0.0 if cmd.source == 'stop' else (cmd.lane or {}).get('speed', fsm.p['base_speed'])
        if cmd.source == 'obstacle':
            v = fsm.p['base_speed']
        if fsm.state == 'M7_OBSTACLE' and rev_left > 0 and s >= P['m7_start_line'] + 1200.0 and rev_at is None:
            rev_at, rev_run = s, 0.0
        if rev_at is not None:                       # 회피 노드가 막혀서 온 길로 후진 중
            v = -0.15
            rev_run += 0.15 * DT
            if rev_run >= rev_m:
                rev_left -= 1
                rev_at = None
                if rev_left > 0:                     # 다시 1.2m 앞까지 갔다가 또 막힘
                    s_block = s + rev_m * 1000.0
                    pass
        s += v * 1000.0 * DT
        odo += (v if signed_odom else abs(v)) * 1000.0 * DT
        t += DT
        if fsm.state == 'FINISHED':
            break

    def where(x):
        best = min(list(P.items()) + [(lm['name'], lm['s']) for lm in lms], key=lambda kv: abs(kv[1] - x))
        return f'{best[0]} {x - best[1]:+.0f}mm'

    if verbose:
        print(f'\n=== {name} ===')
        for t_, s_, st_, why in events:
            print(f'{t_:6.1f}s  s={s_:7.0f} ({where(s_):<26}) -> {st_:<22} {why}')
        for k_, v_ in stop_at.items():
            line = 'm3_stop_line' if k_ == 'M3_PEDESTRIAN' else 'm1_stop_line'
            print(f'  정지 {k_}: 앞바퀴가 정지선 {v_ - P[line]:+.0f}mm (규정 ±50)')
        fb = [m for m in fsm.log if 'FALLBACK' in m]
        print(f'  결과: {fsm.state}, {t:.0f}s, 갈림길={fsm.fork_choice}, 예비경로 사용 {len(fb)}건'
              + (''.join(f'\n    - {m}' for m in fb)))
    return fsm, events, stop_at, t


SCENARIOS = {
    'nominal': {},
    'signs_missed': {'miss': ('slow', 'bump', 'pedestrian', 'fork_b', 'parking')},
    'slow_sign_only': {'miss': ('bump',)},
    'odom_jump': {'jump_at': 15000.0},
    'odom_5pct_long': {'odom_scale': 1.05},
    'no_camera_lines': {'camera': False},
    'm7_rollback_3x1.5m': {'m7_reverse': (3, 1.5)},
    'm7_rollback_old_unsigned': {'m7_reverse': (3, 1.5), 'signed_odom': False},
}

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='all', choices=['all'] + list(SCENARIOS))
    a = ap.parse_args()
    for n, kw in SCENARIOS.items():
        if a.scenario in ('all', n):
            run(n, **kw)
