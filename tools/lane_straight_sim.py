#!/usr/bin/env python3
"""차선 노드(lane_core) 그대로 넣은 직선 폐루프 시뮬레이션 (11/15). 일반 1차로(양쪽 노란 실선)와 M7 1차로(노란 실선 + 흰 점선).
카메라 12 Hz, 조향 지연 4프레임, 선 위치 잡음 12 mm, 매 프레임 방향 흔들림 0.6도. 노란 실선 지킴 예전/지금/끔 비교.

  python3 tools/lane_straight_sim.py                  # 둘 다 (약 15분)
  MODES=normal python3 tools/lane_straight_sim.py     # 일반 구간만
  VAR="{'이름': {'yellow_guard_gain': 1.2}}" python3 tools/lane_straight_sim.py   # 설정 바꿔 같이 비교
"""

import sys, os, math, random
import numpy as np
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'autorace_lane')
sys.path.insert(0, SRC)
from autorace_lane.lane_core import BevConfig, LaneFollower, LaneParams
CFG = BevConfig.from_yaml(os.path.join(SRC, '..', 'autorace_bringup', 'config', 'bev_limo.yaml'))
W = 515.0

def render(y0, psi, lines, noise_px=0, rnd=None):
    h, w = CFG.bev_h, CFG.bev_w
    img = np.zeros((h, w), np.uint8)
    hw = max(1, int(10 / CFG.mm_per_px_x))
    for row in range(h):
        f = CFG.row_to_y_mm(row)
        for Y in lines:
            dy = Y - y0
            s = (f - dy * math.sin(psi)) / math.cos(psi)
            l = -s * math.sin(psi) + dy * math.cos(psi)
            x = -l
            c = int(round(CFG.vehicle_col + x / CFG.mm_per_px_x + (rnd.gauss(0, noise_px) if noise_px else 0)))
            if 0 <= c - hw and c + hw < w:
                img[row, c - hw:c + hw + 1] = 255
    return img

def run(kw, y0=40.0, psi0_deg=3.0, T=15.0, v=0.3, hz=15.0, delay=2, noise_px=0.7, seed=1):
    rnd = random.Random(seed)
    p = LaneParams(lane_width_mm=W, lane_color='yellow', lookahead_mm=650.0, kappa_alpha=0.5, kappa_rate_max=0.6)
    for k, val in kw.items():
        setattr(p, k, val)
    f = LaneFollower(CFG, p)
    y, psi = y0, math.radians(psi0_deg)
    dt = 1.0 / hz
    q = [0.0] * delay
    ys, ks, guards = [], [], 0
    for i in range(int(T * hz)):
        bev = render(y, psi, (W / 2, -W / 2), noise_px, rnd)
        out = f.process_bev(bev, np.zeros_like(bev), yaw=psi, speed=v)
        q.append(out.kappa)
        k = q.pop(0)
        psi += v * k * dt
        y += v * 1000.0 * math.sin(psi) * dt
        ys.append(y); ks.append(out.kappa)
        guards += 1 if getattr(out, 'yellow_guard', None) else 0
    ys = np.array(ys); ks = np.array(ks)
    half = len(ys) // 3
    flips = int(np.sum(np.diff(np.sign(ks[np.abs(ks) > 0.3])) != 0))
    return dict(max_y=float(np.max(np.abs(ys))), late_rms=float(np.sqrt(np.mean(ys[half:] ** 2))),
                late_amp=float(np.ptp(ys[half:])), kmax=float(np.max(np.abs(ks))), flips=flips,
                guard_frac=guards / len(ys))


def render3(y0, psi, rnd, noise_px):
    """3차선 1차로: 왼쪽 노란 실선 +260, 오른쪽 흰 점선 -260 (lane, white 마스크)"""
    h,w=CFG.bev_h,CFG.bev_w
    lane=np.zeros((h,w),np.uint8); white=np.zeros((h,w),np.uint8)
    hw=max(1,int(10/CFG.mm_per_px_x)); period=int(200/CFG.mm_per_px_y)
    for row in range(h):
        f=CFG.row_to_y_mm(row)
        for Y,dashed in ((260.0,False),(-260.0,True)):
            dy=Y-y0; s=(f-dy*math.sin(psi))/math.cos(psi)
            if dashed and (int(s/100.0))%2: continue
            l=-s*math.sin(psi)+dy*math.cos(psi)
            c=int(round(CFG.vehicle_col-l/CFG.mm_per_px_x+rnd.gauss(0,noise_px)))
            if 0<=c-hw and c+hw<w:
                lane[row,c-hw:c+hw+1]=255
                if dashed: white[row,c-hw:c+hw+1]=255
    return lane,white
def run(kw, mode, y0, psi0, delay=4, hz=12, noise=3.0, dist=0.6, seed=1, T=12.0, v=0.3):
    rnd=random.Random(seed)
    if mode=='normal':
        p=LaneParams(lane_width_mm=W, lane_color='yellow', lookahead_mm=650.0, kappa_alpha=0.5, kappa_rate_max=0.6)
    else:
        p=LaneParams(lane_width_mm=520.0, lane_color='both', lookahead_mm=650.0, kappa_alpha=0.5, kappa_rate_max=0.6)
    for k,val in kw.items(): setattr(p,k,val)
    f=LaneFollower(CFG,p)
    y,psi=y0,math.radians(psi0); dt=1/hz; q=[0.0]*delay
    ys=[];ks=[];g=0
    for i in range(int(T*hz)):
        if mode=='normal':
            lane=render(y,psi,(W/2,-W/2),noise,rnd); white=np.zeros_like(lane)
        else:
            lane,white=render3(y,psi,rnd,noise)
        out=f.process_bev(lane,white,yaw=psi,speed=v)
        q.append(out.kappa); k=q.pop(0)
        psi+=v*k*dt+math.radians(rnd.gauss(0,dist)); y+=v*1000*math.sin(psi)*dt
        ys.append(y); ks.append(out.kappa); g+=1 if out.yellow_guard else 0
    ys=np.array(ys); ks=np.array(ks); h=len(ys)//3
    flips=int(np.sum(np.diff(np.sign(ks[np.abs(ks)>0.3]))!=0))
    # 노란 실선(+260)까지 차체 왼쪽 면(차 반폭 120) 여유
    margin=260.0-(np.max(ys)+120.0)
    return np.max(np.abs(ys)), np.ptp(ys[h:]), np.max(np.abs(ks)), flips, g/len(ys), margin
NEW={}; OLD={"yellow_guard_one_side_only":False}; OFF={"yellow_solid_guard":False}
import os
MODES=os.environ.get("MODES","normal,3lane").split(",")
VAR=eval(os.environ.get("VAR","{}"))
for mode,cases in [(m,c) for m,c in (("normal",((60,4),(100,-6))),("3lane",((40,6),(80,10)))) if m in MODES]:
    print(f'== {"일반 1차로(양쪽 노란 실선)" if mode=="normal" else "M7 1차로(왼쪽 노란 실선 + 오른쪽 흰 점선)"}  지연 4프레임 12Hz 잡음 12mm')
    for y0,ps in cases:
        for name,kw in [(n,k) for n,k in (("예전 main",OLD),("고친 것",NEW),("지킴 끔",OFF))]+[(n,k) for n,k in VAR.items()]:
            m=np.mean([run(kw,mode,y0,ps,seed=s) for s in range(4)],axis=0)
            print(f'  출발 {y0:+d}mm {ps:+d}° {name:8s} 흔들림폭 {m[1]:4.0f}mm 최대조향 {m[2]:.2f} 뒤집힘 {m[3]:4.1f} 지킴작동 {100*m[4]:3.0f}% 노란선까지 최소여유 {m[5]:4.0f}mm', flush=True)
