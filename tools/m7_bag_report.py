#!/usr/bin/env python3
"""통주행 bag에서 M7(3차선 장애물)·M2(갈림길)에 무슨 일이 있었는지 시간순으로 뽑고, 의심 원인을 표시한다.

ROS 없이 python3만으로 돈다 (rosbag2 기본 저장 형식 sqlite3 .db3를 직접 읽음).
record_bag.sh로 녹화한 폴더를 넣는다:

  python3 ~/autorace_ws/tools/m7_bag_report.py ~/bags/1111_153000_full1
  python3 ~/autorace_ws/tools/m7_bag_report.py ~/bags/1111_153000_full1 --all     # M7·M2 밖 상태 변화도

보는 것:
  - 미션 상태 전환 (언제, 도면 위치 s, 길 잃음 여부)
  - M7: 회피를 켰나(/avoid/enable), 켠 이유·위치·앞 거리, 회피 노드가 실제로 돌았나(/avoid/active),
        멈춤(hold)·탈출(후진)·비상, 끈 이유(표지/회피 완료/강제), 끝 차로
  - 갈림길 표지를 언제 정했나(fork_choice), M7 끝 차로와 맞나
  - 센서·YOLO 주기 (CPU가 밀리면 라이다·YOLO·차선 주기가 떨어진다)
"""
import argparse
import glob
import json
import os
import sqlite3
import struct
import sys

STR_TOPICS = ('/mission/state', '/obstacle/state', '/avoid/state', '/m7/exit_lane', '/m7/lane',
              '/fork_direction', '/perception/detections', '/lane/status')
BOOL_TOPICS = ('/avoid/active', '/avoid/enable', '/avoid/done')
RATE_TOPICS = ('/scan', '/perception/detections', '/lane/status', '/avoid/state', '/obstacle/state',
               '/mission/state', '/odom')


def cdr_string(data: bytes) -> str:
    """std_msgs/String CDR: 4바이트 머리 + uint32 길이(널 포함) + 글자."""
    little = data[1] == 1
    n = struct.unpack('<I' if little else '>I', data[4:8])[0]
    return data[8:8 + max(0, n - 1)].decode('utf-8', 'replace')


def cdr_bool(data: bytes) -> bool:
    return bool(data[4])


def open_bag(path):
    dbs = sorted(glob.glob(os.path.join(path, '*.db3'))) if os.path.isdir(path) else [path]
    if not dbs:
        mc = glob.glob(os.path.join(path, '*.mcap'))
        sys.exit(f'.db3 파일이 없음: {path}' + (' (mcap 형식은 지원 안 함)' if mc else ''))
    return dbs


def read(dbs):
    """-> {topic: [(t_sec, value)]}  String은 JSON이면 dict, Bool은 bool, 나머지는 None (주기만 셈)."""
    out = {}
    for db in dbs:
        con = sqlite3.connect(db)
        topics = {tid: name for tid, name in con.execute('SELECT id, name FROM topics')}
        want = {tid: n for tid, n in topics.items() if n in STR_TOPICS + BOOL_TOPICS + RATE_TOPICS}
        for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp'):
            name = want.get(tid)
            if name is None:
                continue
            t = ts * 1e-9
            if name in BOOL_TOPICS:
                v = cdr_bool(data)
            elif name in STR_TOPICS and name not in ('/perception/detections', '/lane/status'):
                s = cdr_string(data)
                try:
                    v = json.loads(s)
                except ValueError:
                    v = s
            else:
                v = None
            out.setdefault(name, []).append((t, v))
        con.close()
    return out


def rate(msgs, t0, t1):
    n = sum(1 for t, _ in msgs if t0 <= t <= t1)
    return n / max(1e-6, t1 - t0)


def max_gap(msgs, t0, t1):
    ts = [t for t, _ in msgs if t0 <= t <= t1]
    if len(ts) < 2:
        return None
    return max(b - a for a, b in zip(ts, ts[1:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--all', action='store_true', help='M7·M2 밖 상태 변화도 출력')
    a = ap.parse_args()
    d = read(open_bag(a.bag))
    if '/mission/state' not in d:
        sys.exit('/mission/state 가 bag에 없음 (record_bag.sh로 녹화했는지 확인)')
    T0 = d['/mission/state'][0][0]

    def ts(t):
        return f'{t - T0:7.1f}s'

    warn = []
    # --- 미션 상태 전환 ---------------------------------------------------------
    print('=== 미션 상태 전환 ===')
    segs, last = [], None
    for t, m in d['/mission/state']:
        if not isinstance(m, dict):
            continue
        key = (m.get('state'), m.get('sub'))
        if key != last:
            r = m.get('route') or {}
            s = r.get('s_mm')
            print(f'{ts(t)}  {key[0]}/{key[1] or ""}  s={s if s is None else round(s)}  '
                  f'lost={r.get("lost")}  fork={m.get("fork_choice")}  src={m.get("source")}  {m.get("note") or ""}')
            if last is None or key[0] != last[0]:
                segs.append([key[0], t, None])
                if len(segs) > 1:
                    segs[-2][2] = t
            last = key
    if segs:
        segs[-1][2] = d['/mission/state'][-1][0]
    m7 = [s for s in segs if s[0] == 'M7_OBSTACLE']
    m2 = [s for s in segs if s[0] == 'M2_FORK']
    if not m7:
        warn.append('M7_OBSTACLE 상태가 한 번도 없음: FSM이 M7에 안 들어감 (M3 끝 조건, 상태 건너뜀 확인)')

    # --- M7 상세 --------------------------------------------------------------
    for st, t0, t1 in m7:
        print(f'\n=== M7 {ts(t0)} ~ {ts(t1)} ({t1 - t0:.1f}s) ===')
        en = [(t, v) for t, v in d.get('/avoid/enable', []) if t0 - 1 <= t <= t1 + 1]
        act = [(t, v) for t, v in d.get('/avoid/active', []) if t0 - 1 <= t <= t1 + 1]
        print(f'/avoid/enable: {[(ts(t), v) for t, v in en] or "없음"}')
        if not any(v for _, v in en):
            warn.append('M7 동안 /avoid/enable true 없음: obstacle_node가 회피를 안 켬 '
                        '(m7_avoid:=true로 실행했나, 켜기 문턱·라이다 앞 거리 아래 표 확인)')
        if not any(v for _, v in act):
            warn.append('M7 동안 /avoid/active true 없음: 회피 노드가 안 돎 '
                        '(avoid_node 떴나: ros2 node list, odom/imu/scan 없다는 경고 확인)')
        gap = max_gap([x for x in act if x[1]], t0, t1)
        if gap is not None and gap > 0.5:
            warn.append(f'/avoid/active 가 {gap:.1f}s 끊김 (CPU 밀림 또는 회피 노드 멈춤 -> m7_hold)')
        # obstacle/state m7_* 변화
        prev = None
        for t, o in d.get('/obstacle/state', []):
            if not (t0 <= t <= t1 + 0.5) or not isinstance(o, dict):
                continue
            k = {x: o.get('m7_' + x) for x in ('phase', 'trigger', 'how', 'off_by', 'hold', 'hold_kind', 'exit', 'sign', 'lane')}
            if k != prev:
                print(f'{ts(t)}  phase={k["phase"]} trig={k["trigger"]} pos={o.get("m7_pos_mm")} '
                      f'how={k["how"]} front={o.get("m7_front_mm")} sign={k["sign"]} off={k["off_by"]} '
                      f'hold={k["hold"]}/{k["hold_kind"]} lane={k["lane"]} exit={k["exit"]}')
                prev = k
                if k['how'] == 'distance':
                    warn.append(f'{ts(t)} M7 위치를 진입 후 거리로 계산 (도면 길 잃음 + 시작선 카메라 못 봄) -> 켜기·끄기 위치 오차')
                if k['hold']:
                    warn.append(f'{ts(t)} m7_hold ({k["hold_kind"]}): 회피 노드 소식 끊김으로 정지')
        # 라이다는 봤는데 안 켰나 / 라이다가 못 봤나
        seen_gate = seen_any = None
        for t, o in d.get('/obstacle/state', []):
            if not (t0 <= t <= t1) or not isinstance(o, dict) or o.get('m7_phase') != 'idle':
                continue
            f, pos = o.get('m7_front_mm'), o.get('m7_pos_mm')
            if f is not None and f <= 1000:
                seen_any = seen_any or (t, f, pos)
                if pos is not None and pos < -250 and seen_gate is None:
                    seen_gate = (t, f, pos)
        if seen_gate:
            warn.append(f'{ts(seen_gate[0])} 라이다가 앞 {seen_gate[1]} mm 장애물을 봤지만 위치 {seen_gate[2]} mm '
                        f'(시작선 -250 전)라 안 켬: 위치가 늦게 잡힘 (how 확인)')
        if seen_any is None and en and not any(o.get('m7_trigger') == 'obstacle' for _, o in
                                                d.get('/obstacle/state', []) if isinstance(o, dict) and t0 <= _ <= t1):
            warn.append('M7 대기 중 라이다 앞 차로(차 중심 ±200 mm)에 1 m 안 장애물을 한 번도 못 봄: '
                        '라이다 각도·장착 위치(laser_yaw, laser_x_mm) 또는 장애물 위치 확인 (예비 위치로 켜졌을 것)')
        # 회피 노드
        rec, em = 0, 0
        for t, s in d.get('/avoid/state', []):
            if t0 <= t <= t1 and isinstance(s, dict):
                if s.get('recover_n', 0) > rec:
                    rec = s['recover_n']
                    print(f'{ts(t)}  [avoid] 탈출 {rec}번째 (rollback={s.get("rollback")} x={s.get("x")} '
                          f'y={s.get("y")} target={s.get("target")} hit={s.get("hit_m")})')
                em += bool(s.get('emergency'))
        if rec >= 3:
            warn.append(f'M7에서 탈출(후진) {rec}번: 막힘 반복')
        # 주기
        print('주기 (Hz, M7 동안 / 전체):', ', '.join(
            f'{tp} {rate(d.get(tp, []), t0, t1):.1f}/{rate(d.get(tp, []), T0, d["/mission/state"][-1][0]):.1f}'
            for tp in RATE_TOPICS if tp in d))
        for tp, need in (('/scan', 7.0), ('/perception/detections', 3.0), ('/lane/status', 10.0)):
            if tp in d and rate(d[tp], t0, t1) < need:
                warn.append(f'M7 동안 {tp} {rate(d[tp], t0, t1):.1f} Hz (< {need}): CPU 밀림 의심')
        last_ms = [m for t, m in d['/mission/state'] if t0 <= t <= t1 and isinstance(m, dict)]
        if last_ms and last_ms[-1].get('fork_choice') is None:
            warn.append('M7이 끝날 때까지 갈림길 표지를 못 정함 (fork_choice 없음): 회피가 아무 차로로 끝남')
        last_o = [o for t, o in d.get('/obstacle/state', []) if t0 <= t <= t1 + 0.5 and isinstance(o, dict)]
        if last_o and not last_o[-1].get('m7_done'):
            warn.append('M7을 m7_done 없이 벗어남: FSM 강제 종료(섬 + 0.5 m / 거리) 또는 상태 점프')

    for st, t0, t1 in m2:
        print(f'\n=== M2 {ts(t0)} ~ {ts(t1)} ===')
        ex = [(ts(t), v.get('lane') if isinstance(v, dict) else v, v.get('fork') if isinstance(v, dict) else None,
               v.get('at') if isinstance(v, dict) else None)
              for t, v in d.get('/m7/exit_lane', []) if t0 - 3 <= t <= t1]
        print(f'/m7/exit_lane: {ex or "없음"}')
        ms = [m for t, m in d['/mission/state'] if t0 <= t <= t1 and isinstance(m, dict)]
        fc = ms[-1].get('fork_choice') if ms else None
        print(f'갈림길 결정: {fc},  M7 끝 차로(m7_exit_lane): {ms[-1].get("m7_exit_lane") if ms else None}')
        if ex and fc and ex[-1][2] and ex[-1][2] != fc:
            warn.append(f'M2: 표지 {fc}인데 M7은 {ex[-1][2]} 차로에서 끝남 (섬을 가로지름)')

    if a.all:
        print('\n(--all) 전체 상태 전환은 위 첫 표 참고')
    print('\n=== 의심 원인 ===')
    for w in warn or ['특이 사항 없음']:
        print(' -', w)


if __name__ == '__main__':
    main()
