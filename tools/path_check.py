"""기록 CSV 점검 (ROS 필요 없음).

  python3 path_check.py m7_left.csv            # 요약
  python3 path_check.py m7_left.csv --png      # m7_left.png 그림도 저장 (matplotlib 있으면)

보는 것: 기록 길이·시간·평균 속도, 끝 위치/방향(재생 기준 vs odom), 최대 곡률(차 한계 1.89),
전진/후진 구간(후진은 그림에서 점선), odom·IMU 회전 부호가 같은지, odom 속도값 부호.
"""
import argparse
import math

from path_common import MAX_CURV, load_segments, read_rows, wrap


def f(r, k):
    try:
        return float(r[k]) if r.get(k, '') != '' else None
    except (KeyError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--png', action='store_true')
    a = ap.parse_args()
    rows = read_rows(a.csv)
    if not rows:
        print('빈 파일')
        return
    new = 'xs' in rows[0]
    print(f'{a.csv}: {len(rows)}줄, {f(rows[-1], "t"):.1f} s, ' + ('새 형식(xs 있음)' if new else '예전 형식(xs 없음) → 다시 기록 권장'))

    # 속도
    vf = [f(r, 'vx') for r in rows if f(r, 'vx') is not None and f(r, 'vx') > 0.02]
    vr = [-f(r, 'vx') for r in rows if f(r, 'vx') is not None and f(r, 'vx') < -0.02]
    if vf or vr:
        print('기록 평균 속도: ' + (f'전진 {sum(vf) / len(vf):.3f} m/s' if vf else '전진 없음')
              + (f', 후진 {sum(vr) / len(vr):.3f} m/s' if vr else '')
              + '  → 재생 speed / --rev_speed를 이 값에 맞추면 기록과 가장 비슷')

    # 부호: odom 방향 변화 vs IMU 자이로 적분 (돈 구간만)
    turn_o = turn_i = 0.0
    ok = bad = 0
    prev = None
    for r in rows:
        t, yaw, gz = f(r, 't'), f(r, 'yaw'), f(r, 'gz')
        if None in (t, yaw):
            continue
        if (f(r, 'cmd_vx') or 0) < -0.02 or (f(r, 'vx') or 0) < -0.02:     # 후진 중엔 비교 안 함
            prev = None
            turn_o = turn_i = 0.0
            continue
        if prev and gz is not None:
            dt = t - prev[0]
            if 0 < dt < 0.5:
                turn_o += wrap(yaw - prev[1])
                turn_i += gz * dt
                if abs(turn_o) > 0.35 or abs(turn_i) > 0.35:
                    if abs(turn_o) > 0.1 and abs(turn_i) > 0.1:
                        if (turn_o > 0) == (turn_i > 0):
                            ok += 1
                        else:
                            bad += 1
                    turn_o = turn_i = 0.0
        prev = (t, yaw)
    if 'gz' in rows[0]:
        print(f'회전 부호 (odom 방향 vs IMU 자이로): 같음 {ok}번 / 반대 {bad}번' + ('  ← ★ 반대! 센서 부호 확인' if bad > ok else ''))
    # 속도 부호: odom 위치 변화 방향 vs 속도값
    vb = vo = 0
    for p, r in zip(rows, rows[1:]):
        x0, y0, h0, x1, y1, vx = f(p, 'x'), f(p, 'y'), f(p, 'yaw'), f(r, 'x'), f(r, 'y'), f(r, 'vx')
        if None in (x0, y0, h0, x1, y1, vx) or abs(vx) < 0.05:
            continue
        dx, dy = x1 - x0, y1 - y0
        if math.hypot(dx, dy) < 0.001:
            continue
        fwd = math.cos(math.atan2(dy, dx) - h0) >= 0
        if fwd == (vx > 0):
            vo += 1
        else:
            vb += 1
    print(f'속도 부호 (위치 변화 vs 속도값): 맞음 {vo} / 반대 {vb}' + ('  ← ★ 속도값 부호 뒤집힘' if vb > vo else ''))

    # 경로
    for pose in (['fused', 'odom'] if new else ['imu', 'odom']):
        try:
            segs, info = load_segments(a.csv, pose)
        except ValueError as e:
            print(e)
            continue
        if not segs:
            print(f'[{pose}] 경로 점 없음')
            continue
        if info.get('note'):
            print(f"[{pose}] 알림: {info['note']}")
        kmax = max(abs(k) for g in segs for k in g.k)
        ally = [y for g in segs for y in g.y]
        last = segs[-1]
        print(f'[{pose:5s}] 구간 ' + ' → '.join(f'{"전진" if g.dir > 0 else "후진"} {g.length:.2f}m' for g in segs)
              + f', 끝 x={last.x[-1]:.3f} y={last.y[-1]:+.3f} 방향 {math.degrees(last.h[-1] - segs[0].h[0]):+.1f}°, '
              f'옆으로 {min(ally):+.2f}~{max(ally):+.2f} m, 최대 곡률 {kmax:.2f}'
              + (' ← 차 한계 근처(너무 급함)' if kmax > 0.9 * MAX_CURV else ''))
        for j, g in enumerate(segs):
            print(f'         {j + 1}. {"전진" if g.dir > 0 else "후진"} {g.length:.2f} m: '
                  f'({g.x[0]:+.2f}, {g.y[0]:+.2f}) → ({g.x[-1]:+.2f}, {g.y[-1]:+.2f}), 차 방향 {math.degrees(g.h[-1] - segs[0].h[0]):+.0f}°')

    if a.png:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib 없음 → 그림 생략')
            return
        fig, ax = plt.subplots(figsize=(8, 4))
        for pose, c in ((('fused', 'tab:green'), ('odom', 'tab:gray')) if new else (('imu', 'tab:green'), ('odom', 'tab:gray'))):
            try:
                segs, _ = load_segments(a.csv, pose)
                for j, g in enumerate(segs):
                    ax.plot(g.x, g.y, color=c, ls='-' if g.dir > 0 else '--', label=pose if j == 0 else None)
            except ValueError:
                pass
        for yl in (-0.52, 0, 0.52):
            ax.axhline(yl, color='k', lw=0.5, ls=':')
        ax.set_aspect('equal')
        ax.set_xlabel('x (m, 출발 방향)')
        ax.set_ylabel('y (m, 왼쪽 +)')
        ax.legend()
        out = a.csv.rsplit('.', 1)[0] + '.png'
        fig.savefig(out, dpi=110, bbox_inches='tight')
        print(f'그림 저장: {out}')


if __name__ == '__main__':
    main()
