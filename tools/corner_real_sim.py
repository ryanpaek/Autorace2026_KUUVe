"""11/19 좌하단 코너(라바콘 직전) 실제 도면 모양 + lane_core 그대로. 도면 좌표 X(동+), Y(북+) mm. 차는 서쪽으로 와서 북쪽으로 오른쪽 90°."""
import math, os, random, sys
import numpy as np, yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT + '/src/autorace_lane')
from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams
CFG = BevConfig.from_yaml(ROOT + '/src/autorace_bringup/config/bev_limo.yaml')
VALID = CFG.warp(np.full((480, 640), 255, np.uint8)) > 0
Y_ = yaml.safe_load(open(ROOT + '/src/autorace_bringup/config/params.yaml'))
FIELDS = set(LaneParams.__dataclass_fields__)
LP = {k: v for k, v in Y_['lane_node']['ros__parameters'].items() if k in FIELDS}

def arc(cx, cy, r, a0, a1, n=80):
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]
def dense(poly, step=4.0):
    out = []
    for (x0, y0), (x1, y1) in zip(poly[:-1], poly[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
    return np.array(out)
# 안쪽(북쪽 -> 동쪽) 선: Y1211 수평 -> R600 (1570,1800) -> X970 수직
INNER = dense([(3200, 1211)] + arc(1570, 1800, 589, -math.pi / 2, -math.pi, 60) + [(981, 1800), (981, 4200)])
# 바깥(남쪽 -> 서쪽) 선: Y685 수평 -> 살짝 내려 Y620 -> R670 (1070,1290) -> 살짝 안으로 X445 수직
OUTER = dense([(3200, 685), (1250, 685), (1150, 640), (1070, 620)] + arc(1070, 1290, 670, -math.pi / 2, -math.pi, 60)
              + [(400, 1290), (420, 1500), (445, 1630), (445, 4200)])
LINES = [INNER, OUTER]
LANE_X, LINE_Y = 713.0, 1820.0          # 라바콘 차로 중심 X (두 선 가운데), 시작선 Y (아래 가장자리)
RC = (1360.0, 1580.0, 640.0)            # 도면 경로 지도 코너 (미션 노드 미리보기 기준)

def render(X, Y, H, rnd, noise):
    img = np.zeros((CFG.bev_h, CFG.bev_w), np.uint8)
    c, s = math.cos(H), math.sin(H)
    for pts in LINES:
        dx, dy = pts[:, 0] - X, pts[:, 1] - Y
        f = dx * c + dy * s; lx = -dx * s + dy * c
        ok = (f > 0) & (f < 2500); f, lx = f[ok], lx[ok]
        lx = lx + rnd.gauss(0, noise)
        rows = np.round(CFG.y_mm_to_row(f)).astype(int)
        cols = np.round(CFG.vehicle_col + (-lx) / CFG.mm_per_px_x).astype(int)
        m = (rows >= 0) & (rows < CFG.bev_h) & (cols >= 2) & (cols < CFG.bev_w - 2)
        rr, cc = rows[m], cols[m]
        for d in range(-2, 3):
            img[rr, cc + d] = 255
    img[~VALID] = 0
    return img

FR, RR, HW = 200.0, 160.0, 115.0
def clearance(X, Y, H):
    c, s = math.cos(H), math.sin(H)
    pts = [(X + c * a - s * b, Y + s * a + c * b) for a in np.linspace(-RR, FR, 7) for b in (-HW, HW)]
    pts += [(X + c * FR - s * b, Y + s * FR + c * b) for b in np.linspace(-HW, HW, 5)]
    P = np.array(pts)
    out = []
    for L in LINES:
        d = np.sqrt(((P[:, None, :] - L[None, ::3, :]) ** 2).sum(-1)).min()
        out.append(d)
    return out   # [안쪽 선까지, 바깥 선까지] 차체 가장자리 최소 거리 (mm)

def inside_side(X, Y, L):
    return None

def run(kw=None, v=0.15, gain=1.0, lead=300.0, hz=13.0, delay=3, noise=8.0, plant=0.9, s_err=0.0, y0=0.0, seed=1,
        start_d=0.0, trace=False, h0=0.0, lag=0.0, mode='preview'):
    """start_d: 미리보기 코너 시작을 도면보다 이만큼 늦게(+)/일찍(-) (mm)"""
    kw = kw or {}
    rnd = random.Random(seed)
    p = LaneParams(**{**LP, **{k: v_ for k, v_ in kw.items() if k in FIELDS}})
    f = LaneFollower(CFG, p)
    X, Y, H = 2600.0, 948.0 + y0, math.pi + math.radians(h0)   # 서쪽
    yaw0 = math.pi
    kd = 0.0
    dt = 1 / hz; q = [0.0] * delay
    cin = cout = 9e9; at = None; rows = []
    done = False
    k0 = -1000.0 / RC[2] * gain
    dist = 0.0
    steps = 0
    stalled = False
    corner_start = (2600.0 - RC[0]) + start_d + s_err     # 출발에서 코너 시작까지 거리 (앞바퀴 기준)
    while True:
        Xf, Yf = X + 200 * math.cos(H), Y + 200 * math.sin(H)
        turned = -(H - yaw0)
        if turned >= math.radians(85): done = True
        f.route_step_mm = 100.0
        if mode == 'plain':
            f.route_corner = False
            f.route_k = [(-1000.0 / RC[2] if corner_start <= dist + s_err * 0 + j * 100 <= corner_start + RC[2] * math.pi / 2 else 0.0) for j in range(14)]
        elif not done:
            f.route_corner = True
            if turned < math.radians(5):
                st = corner_start - (dist + 200.0 - 200.0) + lead
                f.route_k = [round(k0, 4) if j * 100 >= st else 0.0 for j in range(14)]
            else:
                f.route_k = [round(k0, 4)] * 14
        else:
            f.route_corner = False; f.route_k = None
        img = render(X, Y, H, rnd, noise)
        out = f.process_bev(img, np.zeros_like(img), yaw=H, speed=v)
        q.append(out.kappa)
        kc = max(-1.96, min(1.96, plant * q.pop(0)))
        kd += (kc - kd) * (min(1.0, dt / lag) if lag > 0 else 1.0)
        k = kd
        vv = v * (out.speed_scale if out.speed_scale is not None else 1.0)
        H += vv * k * dt
        X += vv * 1000 * math.cos(H) * dt; Y += vv * 1000 * math.sin(H) * dt
        dist += vv * 1000 * dt
        ci, co = clearance(X, Y, H); cin = min(cin, ci); cout = min(cout, co)
        Yf = Y + 200 * math.sin(H); Xf = X + 200 * math.cos(H)
        if at is None and Yf >= LINE_Y:
            at = (Xf - LANE_X, math.degrees(H - math.pi / 2))      # + = 오른쪽(동쪽=안쪽), 방향 + = 왼쪽(서쪽)으로 틀어짐
        if trace: rows.append(f'X {X:6.0f} Y {Y:6.0f} H {math.degrees(H):6.1f} k {out.kappa:+.2f} {out.status:4s} {out.source:6s} 안쪽여유 {ci:4.0f} 바깥여유 {co:4.0f} look {out.lookahead_mm}')
        steps += 1
        if steps > 1500:
            stalled = True
            break
        if Y > LINE_Y + 900 or X < 0 or Y < 300 or dist > 6000: break
    return dict(stalled=stalled, cin=cin, cout=cout, line_dx=at[0] if at else 999, line_psi=at[1] if at else 99, rows=rows)

CONDS = [dict(), dict(noise=12.0, delay=4, hz=12.0), dict(s_err=200.0), dict(s_err=-200.0), dict(y0=-40.0, h0=-3.0), dict(y0=40.0, h0=3.0)]
def score(name, kw=None, plants=(0.8, 0.9), seeds=1, **common):
    res = []
    for pl in plants:
        for c in CONDS:
            for sd in range(seeds):
                res.append(run(kw, plant=pl, seed=sd, **{**common, **c}))
    ci = [r['cin'] for r in res]; co = [r['cout'] for r in res]
    dx = [r['line_dx'] for r in res]; ps = [r['line_psi'] for r in res]
    hit = sum(r['cin'] < 10 or r['cout'] < 10 or r['stalled'] or r['line_dx'] == 999 for r in res)
    print(f"{name:40s} 안쪽 선 여유 평균 {np.mean(ci):4.0f} 최소 {min(ci):4.0f} | 바깥 여유 평균 {np.mean(co):4.0f} 최소 {min(co):4.0f} | 선 닿음 {hit:2d}/{len(res)} | "
          f"시작선 옆 평균 {np.mean(dx):+4.0f} |최악| {max(abs(d) for d in dx):3.0f} 방향 평균 {np.mean(ps):+5.1f}° |최악| {max(abs(p) for p in ps):4.1f}°", flush=True)
    return res


if __name__ == '__main__':
    # python3 tools/corner_real_sim.py   -> params.yaml 값으로 라바콘 직전 코너(미리보기)·도면만 코너 점수
    MP = Y_['mission_node']['ros__parameters']
    la = float(MP.get('corner_preview_lookahead_mm', 0.0))
    kw = {'route_corner_zero_lane': True}
    if la > 0:
        kw['lookahead_mm'] = la
    score('params.yaml 미리보기 코너(라바콘 직전)', kw, gain=float(MP.get('m8_corner_k_gain', 1.0)), lag=0.15)
    score('params.yaml 도면만 코너(우상단)', {}, mode='plain', lag=0.15)
