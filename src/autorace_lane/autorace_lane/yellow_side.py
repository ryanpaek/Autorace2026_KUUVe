"""차 바로 앞 노란 선까지 옆 거리 (yellow_side). 11/19 추가.

목적: 코너 전 직선에서 '차가 차로 안에서 바깥/안쪽으로 몇 mm 밀려 있나'를 따로 잰다.
  lane_node(lane_core.py)는 건드리지 않는다. BevConfig(BEV 변환)만 같이 쓴다.
  -> 결과는 /lane/yellow_side 토픽으로만 나간다. 아직 아무 노드도 이 값으로 조향하지 않는다.

방법 (원리):
  ① BEV에서 '차 앞 띠'(기본 800~1000 mm 앞, 2행마다 = 25줄)만 뽑는다.
     (550~750은 카메라 화각 밖: 550 mm 앞에서 보이는 옆 폭이 -188~+220 mm라 반폭 257 mm 선이 안 잡힘. 11/19 실측 bev_limo.yaml)
     전체 400x400 BEV를 만들지 않고, 그 줄들의 원본 화소 위치를 미리 계산해 둔 remap 표로 바로 가져온다.
     (lane_core는 마스크를 만든 뒤 INTER_NEAREST로 warp -> 화소 하나씩 옮기는 것이라 결과 화소가 같다)
  ② 그 띠에서만 HSV 노란색 마스크 (임계값은 lane_node와 같은 값)
  ③ 줄마다 노란 덩어리(run)를 찾고, 차 중심선 왼쪽에서 가장 가까운 것 = 왼쪽 선, 오른쪽에서 가장 가까운 것 = 오른쪽 선
     너무 얇거나(점 잡음) 너무 넓은(정지선·얼룩) 덩어리는 버린다
  ④ 줄별 거리의 중앙값 = 이 프레임의 l_mm / r_mm. 잡힌 줄이 min_rows 미만이면 ok=False
     옅은 노랑 다리: 그 줄에서 진한 노랑을 못 찾으면, 직전(필터) 선 위치 +-bridge_mm 안의 옅은 노랑(S>=20, V>=200)도 인정.
       조명이 세면 노란 실선이 하얗게 날아감 (lane_core white_bridge와 같은 생각, 11/19). 새 선은 만들지 않음
  ⑤ SideFilter: 0.5초 중앙값 + 한 번에 50 mm 넘게 튀면 버림 (3번 연속 튀면 새 값 인정)
  오프셋 = (l - r) / 2  (오른쪽 +, lane offset_mm과 같은 방향). 양쪽 다 보일 때만 offset_mm.
  한쪽만 보이면 반폭(257.5)으로 추정한 값을 offset_one_mm에 따로 (정확도 낮음, 참고용)
"""

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

from autorace_lane.lane_core import BevConfig


class BandSampler:
    """BEV의 몇 줄만 원본 영상에서 바로 뽑는 remap 표."""

    def __init__(self, cfg: BevConfig, near_from_mm: float = 800.0,
                 near_to_mm: float = 1000.0, row_step: int = 2):
        self.cfg = cfg
        r0 = cfg.y_mm_to_row(near_to_mm)       # 먼 쪽 (작은 행 번호)
        r1 = cfg.y_mm_to_row(near_from_mm)     # 가까운 쪽
        lo = int(max(0, math.ceil(min(r0, r1))))
        hi = int(min(cfg.bev_h - 1, math.floor(max(r0, r1))))
        step = max(1, int(row_step))
        self.rows = np.arange(lo, hi + 1, step, dtype=np.int32)
        self.ys_mm = np.array([cfg.row_to_y_mm(r) for r in self.rows], np.float64)
        cols = np.arange(cfg.bev_w, dtype=np.float64)
        self.x_mm = (cols - cfg.vehicle_col) * cfg.mm_per_px_x
        cc, rr = np.meshgrid(cols, self.rows.astype(np.float64))
        Hinv = np.linalg.inv(np.asarray(cfg.H, np.float64))
        den = Hinv[2, 0] * cc + Hinv[2, 1] * rr + Hinv[2, 2]
        u = (Hinv[0, 0] * cc + Hinv[0, 1] * rr + Hinv[0, 2]) / den
        v = (Hinv[1, 0] * cc + Hinv[1, 1] * rr + Hinv[1, 2]) / den
        # warpPerspective(INTER_NEAREST)와 같게: 가장 가까운 화소, 영상 밖은 -1 (검은색)
        self.map_x = u.astype(np.float32)
        self.map_y = v.astype(np.float32)

    def sample(self, bgr: np.ndarray) -> np.ndarray:
        return cv2.remap(bgr, self.map_x, self.map_y, cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def yellow_mask(bgr: np.ndarray, h_min: int, h_max: int, s_min: int, v_min: int) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array([h_min, s_min, v_min], np.uint8),
                       np.array([h_max, 255, 255], np.uint8))


def row_runs(row: np.ndarray) -> List[Tuple[int, int]]:
    """0/255 한 줄에서 켜진 구간 [(시작, 끝+1), ...]."""
    on = (row > 0).astype(np.int8)
    d = np.diff(np.concatenate(([0], on, [0])))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)
    return list(zip(starts.tolist(), ends.tolist()))


def _pick(runs, x_mm, lmin, lmax, side, near=None, bridge_mm=0.0):
    """한 줄의 run들 중 side('L'|'R')에서 차에 가장 가까운 선의 x(mm). near가 있으면 그 둘레만."""
    best = None
    for a, b in runs:
        w = b - a
        if w < lmin or w > lmax:
            continue
        x = float(x_mm[a:b].mean())
        if (side == 'L') != (x < 0):
            continue
        if near is not None and abs(x - near) > bridge_mm:
            continue
        if best is None or abs(x) < abs(best):
            best = x
    return best


def measure_band(mask: np.ndarray, x_mm: np.ndarray, mm_per_px: float,
                 min_run_mm: float = 8.0, max_run_mm: float = 120.0,
                 min_rows: int = 5, pale: Optional[np.ndarray] = None,
                 prev_l: Optional[float] = None, prev_r: Optional[float] = None,
                 bridge_mm: float = 60.0) -> Dict:
    """띠 마스크(줄 x 열)에서 왼쪽/오른쪽 가장 가까운 노란 선까지 거리 (mm, 양수).

    pale: 옅은 노랑 마스크 (같은 크기). prev_l/prev_r: 직전 거리 (양수 mm). 둘 다 있을 때만 다리 사용.
    """
    lmin = max(1, int(round(min_run_mm / mm_per_px)))
    lmax = max(lmin, int(round(max_run_mm / mm_per_px)))
    ls: List[float] = []
    rs: List[float] = []
    nb = 0
    for i, row in enumerate(mask):
        runs = row_runs(row)
        best_l = _pick(runs, x_mm, lmin, lmax, 'L')
        best_r = _pick(runs, x_mm, lmin, lmax, 'R')
        if pale is not None and ((best_l is None and prev_l is not None)
                                 or (best_r is None and prev_r is not None)):
            pruns = row_runs(pale[i])
            if best_l is None and prev_l is not None:
                best_l = _pick(pruns, x_mm, lmin, lmax, 'L', -prev_l, bridge_mm)
                nb += best_l is not None
            if best_r is None and prev_r is not None:
                best_r = _pick(pruns, x_mm, lmin, lmax, 'R', prev_r, bridge_mm)
                nb += best_r is not None
        if best_l is not None:
            ls.append(-best_l)
        if best_r is not None:
            rs.append(best_r)
    out = {'n_l': len(ls), 'n_r': len(rs), 'n_bridge': nb, 'rows': int(mask.shape[0])}
    out['raw_l'] = float(np.median(ls)) if ls else None
    out['raw_r'] = float(np.median(rs)) if rs else None
    out['l_ok'] = len(ls) >= min_rows
    out['r_ok'] = len(rs) >= min_rows
    # 줄 사이 흩어짐 (선이 비스듬하면 큼). 판단 보조용
    out['sd_l'] = float(np.std(ls)) if len(ls) >= 2 else None
    out['sd_r'] = float(np.std(rs)) if len(rs) >= 2 else None
    return out


class SideFilter:
    """한쪽 거리의 시간 필터: window_s 중앙값 + 튐 거르기."""

    def __init__(self, window_s: float = 0.5, max_jump_mm: float = 50.0,
                 lo_mm: float = 120.0, hi_mm: float = 450.0, reset_after: int = 3):
        self.window_s = window_s
        self.max_jump_mm = max_jump_mm
        self.lo_mm = lo_mm
        self.hi_mm = hi_mm
        self.reset_after = reset_after
        self.buf: Deque[Tuple[float, float]] = deque()
        self.n_jump = 0
        self.rejected = 0

    def _trim(self, t: float):
        while self.buf and t - self.buf[0][0] > self.window_s:
            self.buf.popleft()

    def value(self) -> Optional[float]:
        if len(self.buf) < 2:
            return None
        return float(np.median([v for _, v in self.buf]))

    def update(self, t: float, v: Optional[float], ok: bool) -> Optional[float]:
        self._trim(t)
        if ok and v is not None and self.lo_mm <= v <= self.hi_mm:
            cur = self.value()
            if cur is not None and abs(v - cur) > self.max_jump_mm:
                self.n_jump += 1
                self.rejected += 1
                if self.n_jump >= self.reset_after:     # 계속 튀면 진짜로 바뀐 것
                    self.buf.clear()
                    self.buf.append((t, v))
                    self.n_jump = 0
            else:
                self.buf.append((t, v))
                self.n_jump = 0
        return self.value()


def offset_from_sides(l_mm: Optional[float], r_mm: Optional[float],
                      half_width_mm: float = 257.5) -> Tuple[Optional[float], str]:
    """차로 중심에서 차가 얼마나 밀렸나 (mm, 오른쪽 +)와 근거 ('both' | 'L' | 'R' | 'none')."""
    if l_mm is not None and r_mm is not None:
        return (l_mm - r_mm) / 2.0, 'both'
    if l_mm is not None:
        return l_mm - half_width_mm, 'L'
    if r_mm is not None:
        return half_width_mm - r_mm, 'R'
    return None, 'none'


class YellowSide:
    """한 프레임 처리 + 필터. 노드와 오프라인 검증이 같이 쓴다."""

    def __init__(self, cfg: BevConfig, hsv=(15, 40, 80, 80), near_from_mm=800.0,
                 near_to_mm=1000.0, row_step=2, min_run_mm=8.0, max_run_mm=120.0,
                 min_rows=5, half_width_mm=257.5, window_s=0.5, max_jump_mm=50.0,
                 lo_mm=120.0, hi_mm=450.0, bridge_mm=60.0, pale_s_min=20, pale_v_min=200):
        self.sampler = BandSampler(cfg, near_from_mm, near_to_mm, row_step)
        self.mm_per_px = cfg.mm_per_px_x
        self.hsv = tuple(int(v) for v in hsv)
        self.min_run_mm = min_run_mm
        self.max_run_mm = max_run_mm
        self.min_rows = int(min_rows)
        self.half_width_mm = half_width_mm
        self.bridge_mm = float(bridge_mm)          # 0이면 옅은 노랑 다리 끔
        self.pale_s_min = int(pale_s_min)
        self.pale_v_min = int(pale_v_min)
        self.fl = SideFilter(window_s, max_jump_mm, lo_mm, hi_mm)
        self.fr = SideFilter(window_s, max_jump_mm, lo_mm, hi_mm)

    def measure(self, bgr: np.ndarray) -> Dict:
        band = self.sampler.sample(bgr)
        h0, h1, s0, v0 = self.hsv
        hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, np.array([h0, s0, v0], np.uint8), np.array([h1, 255, 255], np.uint8))
        pl, pr = self.fl.value(), self.fr.value()
        pale = None
        if self.bridge_mm > 0 and (pl is not None or pr is not None):
            pale = cv2.inRange(hsv, np.array([h0, self.pale_s_min, self.pale_v_min], np.uint8),
                               np.array([h1, 255, 255], np.uint8))
        return measure_band(m, self.sampler.x_mm, self.mm_per_px,
                            self.min_run_mm, self.max_run_mm, self.min_rows,
                            pale, pl, pr, self.bridge_mm)

    def step(self, t: float, bgr: np.ndarray) -> Dict:
        d = self.measure(bgr)
        lf = self.fl.update(t, d['raw_l'], d['l_ok'])
        rf = self.fr.update(t, d['raw_r'], d['r_ok'])
        off, src = offset_from_sides(lf, rf, self.half_width_mm)
        # 한쪽만으로 낸 값은 따로 (05·06·08 직선에서 lane offset_mm과 평균 차이 40~150 mm -> 믿기 어려움)
        one = off if src in ('L', 'R') else None
        if one is not None:
            off = None
        d.update({'l_mm': lf, 'r_mm': rf, 'offset_mm': off, 'offset_one_mm': one, 'src': src,
                  'y_mm': float(self.sampler.ys_mm.mean())})
        return d
