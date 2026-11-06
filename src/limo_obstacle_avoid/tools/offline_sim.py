#!/usr/bin/env python3
"""
offline_sim.py — LIMO / ROS 없이 회피 로직만 시험  (윈도우·우분투 둘 다 됨, 필요: matplotlib)

  python3 tools/offline_sim.py                       # 기본 배치(mixed7)
  python3 tools/offline_sim.py --layout zigzag6 --exit B
  python3 tools/offline_sim.py --all                 # 모든 배치 + 오차 넣어서 한 번에
  python3 tools/offline_sim.py --trigger 1.2 --lookahead 0.7   # 파라미터 바꿔보기

결과: 터미널 요약 + PDF 그림(sim_<배치>.pdf)
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from limo_obstacle_avoid.avoid_core import AvoidParams, AvoidPlanner, scan_to_base   # noqa: E402
from limo_obstacle_avoid import sim_common as sc                                     # noqa: E402


def run(layout, p, exit_lane='auto', odom_scale=1.0, imu_drift_deg=0.0, jitter=0.0,
        noise=0.01, seed=0, verbose=True):
    rnd = random.Random(seed)
    p.exit_lane = exit_lane
    rects = sc.layout_rects(layout, p.lane_width, jitter=jitter, seed=seed)
    segs = sc.rects_to_segments(rects)
    car = sc.SimCar(0.0, p.start_lane * p.lane_width, 0.0)
    pl = AvoidPlanner(p)
    pl.start(car.odom_x, car.odom_y, car.imu_yaw, 0.0)

    n_beam, a_min = 360, -math.pi
    a_inc = 2 * math.pi / n_beam
    dt, t = 0.05, 0.0
    path, foot, worst = [], [], float('inf')
    collided = False
    stuck = 0
    for k in range(int(60 / dt)):
        drift = math.radians(imu_drift_deg) * min(1.0, t / 20.0)
        pl.pose.update(car.odom_x, car.odom_y, car.imu_yaw)
        if k % 2 == 0:   # 라이다 10Hz
            lx = car.x + p.lidar_x * math.cos(car.yaw)
            ly = car.y + p.lidar_x * math.sin(car.yaw)
            rng = sc.raycast(lx, ly, car.yaw, a_min, a_inc, n_beam, segs, noise=noise, rnd=rnd)
            # 라이다는 lidar 위치 기준이므로 scan_to_base 의 lidar_x 와 맞음
            pl.update_scan(scan_to_base(rng, a_min, a_inc, p))
        v, w = pl.step(t)
        car.step(v, w, dt, odom_scale=odom_scale, imu_drift=drift, rnd=rnd)
        t += dt
        g = sc.min_gap(car.footprint(), rects)
        worst = min(worst, g)
        if g < 0:
            collided = True
        path.append((car.x, car.y))
        if k % 10 == 0:
            foot.append(car.footprint())
        stuck = stuck + 1 if v == 0 else 0
        if pl.state == 'DONE' or stuck > 60:
            break

    ok = pl.state == 'DONE' and not collided
    res = dict(layout=layout if isinstance(layout, str) else 'custom', ok=ok, collided=collided,
               done=pl.state == 'DONE', worst_gap=worst, time=t, final_lane=pl.target,
               real_y=car.y, est_y=pl.pose.y, events=pl.events, path=path, foot=foot,
               rects=rects, planner=pl)
    if verbose:
        print(f"\n=== {res['layout']}  exit={exit_lane}  odom×{odom_scale}  imu드리프트{imu_drift_deg}° ===")
        for x, e in pl.events:
            print(f"  x={x:5.2f}m  {e}")
        print(f"  결과: {'성공' if ok else '실패'} | 충돌 {collided} | 최소간격 {worst*100:.1f}cm"
              f" | {t:.1f}s | 최종차선 {pl.target:+d} | 실제y {car.y:.3f} / 추정y {pl.pose.y:.3f}")
    return res


def plot(results, p, out_pdf):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from matplotlib.backends.backend_pdf import PdfPages
    plt.rcParams['font.family'] = ['NanumGothic', 'Noto Sans CJK KR', 'Malgun Gothic', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    with PdfPages(out_pdf) as pdf:
        for r in results:
            fig, ax = plt.subplots(figsize=(11, 4.2))
            lw, L = p.lane_width, p.section_length
            for k in (-1.5, 1.5):
                ax.axhline(k * lw, color='#c98a1a', lw=1.5)
            for k in (-0.5, 0.5):
                ax.plot([0, L], [k * lw, k * lw], '--', color='#999', lw=1)
            ax.add_patch(Polygon([(L, -0.3 * lw), (L + p.island_length, -0.3 * lw),
                                  (L + p.island_length, 0.3 * lw), (L, 0.3 * lw)],
                                 closed=True, fc='none', ec='#c98a1a', hatch='//', label='섬(가상)'))
            for rc in r['rects']:
                ax.add_patch(Polygon(sc.rect_corners(*rc), closed=True, fc='#555', ec='k'))
            for fp in r['foot']:
                ax.add_patch(Polygon(sc.rect_corners(*fp), closed=True, fc='none', ec='#3a7bd5', lw=0.6, alpha=0.6))
            xs, ys = zip(*r['path'])
            ax.plot(xs, ys, color='#d62728' if not r['ok'] else '#2ca02c', lw=2, label='실제 경로')
            for o in r['planner'].obs:
                if o.hits >= p.confirm_hits:
                    ax.add_patch(Polygon([(o.xmin, o.ymin), (o.x_end(p.obs_min_len), o.ymin),
                                          (o.x_end(p.obs_min_len), o.ymax), (o.xmin, o.ymax)],
                                         closed=True, fc='none', ec='orange', lw=1, ls=':'))
            ax.text(-0.3, 1.5 * lw, 'A쪽(+1)', va='bottom')
            ax.text(-0.3, -1.5 * lw, 'B쪽(-1)', va='top')
            ax.set_aspect('equal')
            ax.set_xlim(-0.5, L + p.island_length + 0.3)
            ax.set_ylim(-1.1, 1.1)
            ax.set_title(f"{r['layout']} | {'성공' if r['ok'] else '실패'} | 최소간격 {r['worst_gap']*100:.1f}cm"
                         f" | 최종차선 {r['final_lane']:+d}")
            ax.legend(loc='lower right', fontsize=8)
            pdf.savefig(fig, bbox_inches='tight')
            plt.close(fig)
    print(f"\nPDF 저장: {out_pdf}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--layout', default='mixed7', choices=list(sc.LAYOUTS))
    ap.add_argument('--exit', default='auto')
    ap.add_argument('--all', action='store_true', help='모든 배치 × 오차 조합')
    ap.add_argument('--trigger', type=float)
    ap.add_argument('--lookahead', type=float)
    ap.add_argument('--change_len', type=float)
    ap.add_argument('--v', type=float)
    ap.add_argument('--no_pdf', action='store_true')
    a = ap.parse_args()

    def make_p():
        p = AvoidParams()
        if a.trigger is not None: p.trigger_dist = a.trigger
        if a.lookahead is not None: p.lookahead = a.lookahead
        if a.change_len is not None: p.change_len = a.change_len
        if a.v is not None: p.v_cruise = a.v; p.v_change = min(p.v_change, a.v)
        return p

    results = []
    if a.all:
        cases = []
        for lay in sc.LAYOUTS:
            for ex in ('auto', 'A', 'B'):
                cases.append((lay, ex, 1.0, 0.0, 0.0, 0))
            cases.append((lay, 'auto', 1.04, 2.0, 0.05, 1))    # 오도메트리 +4%, IMU 2도 드리프트, 배치 흔들림
            cases.append((lay, 'auto', 0.96, -2.0, 0.05, 2))
        for lay, ex, sca, dr, jit, sd in cases:
            results.append(run(lay, make_p(), ex, sca, dr, jit, seed=sd))
        n_ok = sum(r['ok'] for r in results)
        print(f"\n총 {len(results)}건 중 성공 {n_ok}건")
        out = 'sim_all.pdf'
    else:
        results.append(run(a.layout, make_p(), a.exit))
        out = f'sim_{a.layout}.pdf'
    if not a.no_pdf:
        plot(results, make_p(), out)


if __name__ == '__main__':
    main()
