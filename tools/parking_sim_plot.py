"""주차 미션 시뮬레이션 그림 (ROS 없이). test/test_parking_node.py의 가상 트랙·가짜 라이다·차 운동 모형으로
ParkingV2(실제 주차 논리)를 돌리고, 차가 지나간 길과 단계(phase), 꺾기 시작 위치, 마지막 차 위치를 그린다.

  python3 tools/parking_sim_plot.py [출력.png]
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'autorace_mission', 'test'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'autorace_mission'))
import test_parking_node as T  # noqa: E402

PH_COLOR = {'WAIT_LINE': '#9aa0a6', 'FOLLOW': '#1a73e8', 'EXTRA': '#7baaf7', 'PAUSE': '#f9ab00',
            'TURN': '#d93025', 'BACK': '#a142f4', 'CHECK': '#12b5cb', 'PULLOUT': '#e37400',
            'ENTER': '#188038', 'DONE': '#000000'}


def run(parked, target, overrides=None, **kw):
    """drive()를 돌리면서 매 틱 라이다 위치(가짜 raycast 호출 인자)와 단계를 기록한다."""
    path = []
    orig = T.raycast

    def spy(px, py, th, boxes, *a, **k):
        path.append((px, py, th))
        return orig(px, py, th, boxes, *a, **k)
    T.raycast = spy
    log, out = [], {}
    try:
        n_in, hit, result = T.drive(parked, target, overrides, log=log, out=out, **kw)
    finally:
        T.raycast = orig
    # 라이다 위치 -> 뒷바퀴축
    rear = [(x - (T.WB + T.LIDAR_AHEAD) * math.cos(th), y - (T.WB + T.LIDAR_AHEAD) * math.sin(th), th)
            for x, y, th in path]
    phases = [i['phase'] for i in log]
    return dict(rear=rear, phases=phases, n_in=n_in, hit=hit, result=result, last=log[-1] if log else {},
                pose=out.get('pose'))


def draw(ax, title, parked, target, r, hide=()):
    for i in range(1, 7):
        x0, x1, yt, yb = T.cell(i)
        ax.add_patch(Rectangle((x0, yb), x1 - x0, yt - yb, fill=(i == target), fc='#e6f4ea', ec='#5f6368', lw=1))
        ax.text((x0 + x1) / 2, yb + 25, str(i), ha='center', fontsize=8, color='#5f6368')
    for k, i in enumerate(parked):
        bx0, bx1, by0, by1 = T.parked_box(i)
        ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, fc='#ffffff' if k in hide else '#80868b',
                               ec='#202124', hatch='//' if k in hide else None))
    ax.plot([0, 0], [0, 520], color='#202124', lw=3)                    # 주차 앞 흰 선
    ax.axhline(500, color='#fbbc04', lw=2)                              # 차로 바깥 선
    ax.axhline(0, color='#5f6368', lw=1, ls='--')                       # 주차 구역 위 테두리
    rear = r['rear']
    for k in range(1, len(rear)):
        ph = r['phases'][min(k, len(r['phases']) - 1)]
        fx0 = rear[k - 1][0] + T.WB * math.cos(rear[k - 1][2])
        fy0 = rear[k - 1][1] + T.WB * math.sin(rear[k - 1][2])
        fx1 = rear[k][0] + T.WB * math.cos(rear[k][2])
        fy1 = rear[k][1] + T.WB * math.sin(rear[k][2])
        ax.plot([fx0, fx1], [fy0, fy1], color=PH_COLOR.get(ph, 'k'), lw=2)
    # 꺾기 시작한 곳 (TURN 첫 틱)의 차
    for k, ph in enumerate(r['phases']):
        if ph == 'TURN':
            x, y, th = rear[k]
            cs = T.corners(x, y, th)
            ax.add_patch(Polygon([cs[0], cs[1], cs[3], cs[2]], fill=False, ec='#d93025', ls='--', lw=1.2))
            break
    x, y, th = r['pose']
    cs = T.corners(x, y, th)
    ok = r['n_in'] == 4 and not r['hit']
    ax.add_patch(Polygon([cs[0], cs[1], cs[3], cs[2]], fc='#34a853' if ok else '#ea4335', alpha=0.55, ec='k'))
    score = {4: '15점', 3: '7점'}.get(r['n_in'], '0점')
    ax.set_title(f"{title}\n모서리 {r['n_in']}/4 · {score}{' · 충돌' if r['hit'] else ''} · "
                 f"{r['last'].get('stop_how') or ''} · {r['result']}", fontsize=9)
    ax.set_xlim(-950, 2200)
    ax.set_ylim(-1300, 560)
    ax.set_aspect('equal')
    ax.tick_params(labelsize=7)


SCENARIOS = [
    ('대회 배치 (1·3·4·5·6 정차) → 2번', T.FIVE, 2, {}, dict(fov_deg=T.NARROW, camera=True), ()),
    ('1·3번만 → 2번', [1, 3], 2, {}, dict(fov_deg=T.NARROW, camera=True), ()),
    ('1번을 라이다가 못 봄', T.FIVE, 2, {}, dict(fov_deg=T.NARROW, camera=True, hide=(0,)), (0,)),
    ('꺾기 20cm 늦게 (turn_lead 120)', T.FIVE, 2, {'turn_lead_mm': 120.0}, dict(fov_deg=T.NARROW, camera=True), ()),
    ('꺾기 20cm 일찍 (turn_lead 520)', T.FIVE, 2, {'turn_lead_mm': 520.0}, dict(fov_deg=T.NARROW, camera=True), ()),
    ('차선에서 5cm 오른쪽으로 치우쳐 접근', T.FIVE, 2, {}, dict(fov_deg=T.NARROW, camera=True, y0=190.0), ()),
]

if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'parking_sim.png'
    plt.rcParams['font.family'] = ['Noto Sans CJK KR', 'Noto Serif CJK KR', 'NanumGothic', 'Malgun Gothic', 'DejaVu Sans']
    fig, axes = plt.subplots(3, 2, figsize=(12, 13))
    for ax, (title, parked, target, ov, kw, hide) in zip(axes.flat, SCENARIOS):
        r = run(parked, target, ov, **kw)
        draw(ax, title, parked, target, r, hide)
        print(f"{title}: {r['n_in']}/4 hit={r['hit']} {r['result']} stop_how={r['last'].get('stop_how')}")
    handles = [plt.Line2D([], [], color=c, lw=3, label=k) for k, c in PH_COLOR.items()]
    fig.legend(handles=handles, loc='lower center', ncol=10, fontsize=8)
    fig.suptitle('주차 미션 시뮬레이션 (앞바퀴 궤적, 단계별 색) — 빨간 점선: 꺾기 시작한 위치, 초록/빨강 칠: 마지막 위치', fontsize=11)
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(out, dpi=130)
    print('saved', out)
