"""차선 인식 + Pure Pursuit 핵심 로직 (ROS 의존성 없음).

D-Racer(ryanpaek/D-Racer-Kit, release/v1.0.0)의 lane_following_node.py BEV 경로에서
검증된 아이디어를 LIMO Pro + Autorace 2026 트랙에 맞게 간추려 옮긴 초안이다.
실차 검증 전이므로 파라미터 기본값은 출발점일 뿐이다.

D-Racer에서 옮긴 것
  - 미터 단위 BEV 그리드 + 차량 중심 열(vehicle_col) 기준 절대 좌/우 판정
  - 하단 밴드 열 히스토그램 -> 측별 후보 최대 3개 -> 폭 게이트 + 연속성 점수로 쌍 선택
  - 한쪽만 보일 때 국소 기울기에 수직으로 반폭 외삽
  - 행별 중점 중앙선 -> 잔차 기반 2-pass 로버스트 피팅 -> 곡률
  - Pure Pursuit, 중앙 점프 게이트(코스팅), 차선을 놓치면 잠시 버티다 정지

바꾼 것
  - 고정 열 창 대신 슬라이딩 윈도우로 차선을 위로 추적한다 (트랙 곡률 R600이 좁아서)
  - 차로 폭을 구간별로 바꿀 수 있다 (일반 ~530mm, 지그재그 ~570mm, 3차선 500mm)
  - 가로로 긴 흰 선(정지선·시작선)을 차선 마스크에서 빼고, 정지선 거리로 따로 보고한다
  - 출력은 조향값이 아니라 곡률 kappa(1/m)다. ROS Twist로는 angular.z = v * kappa
  - 차선을 놓치면 IMU yaw로 헤딩을 붙잡는다 (D-Racer는 IMU가 없어 마지막 조향을 재생했음)

좌표 규약
  - BEV 그리드: 행 0 = 가장 먼 곳, 행 bev_h-1 = 가장 가까운 곳, 열은 오른쪽으로 증가
  - 내부 차량 좌표: x_mm = 오른쪽 +, y_mm = 전방 +, 원점 = 제어 기준점
    (애커만 모드면 뒤차축 중심, 사륜 차동 모드면 차체 중심)
  - 밖으로 내보내는 값(kappa, offset, heading)은 ROS 규약: 왼쪽이 + (angular.z와 같은 부호)
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field, fields
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml


def wrap_angle(a: float) -> float:
    """각도를 [-pi, pi)로 감는다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# BEV 캘리브레이션
# ---------------------------------------------------------------------------
@dataclass
class BevConfig:
    """카메라 이미지 -> 미터 단위 탑뷰(BEV) 그리드 변환 정보."""

    H: np.ndarray                       # 카메라 이미지 px -> BEV 그리드 px (3x3)
    image_width: int = 640              # 이 크기로 리사이즈한 뒤 변환한다
    image_height: int = 480
    bev_w: int = 400
    bev_h: int = 400
    mm_per_px_x: float = 4.0
    mm_per_px_y: float = 4.0
    vehicle_col: float = 200.0          # 차량 중심선이 지나는 BEV 열
    ref_to_bev_bottom_mm: float = 300.0  # 제어 기준점 -> BEV 맨 아래 행까지 전방 거리
    ref_to_front_axle_mm: float = 200.0  # 제어 기준점 -> 앞차축 (뒤차축 기준이면 휠베이스)
    calibrated: bool = False

    _KEYS = ('image_width', 'image_height', 'bev_w', 'bev_h', 'mm_per_px_x',
             'mm_per_px_y', 'vehicle_col', 'ref_to_bev_bottom_mm',
             'ref_to_front_axle_mm')

    # --- 생성 ---------------------------------------------------------------
    @classmethod
    def from_rectangle(cls, img_pts, rect_w_mm: float, rect_l_mm: float,
                       near_edge_mm: float, image_size: Tuple[int, int],
                       lateral_offset_mm: float = 0.0, bev_w: int = 400,
                       bev_h: int = 400, mm_per_px: float = 4.0,
                       ref_to_bev_bottom_mm: Optional[float] = None,
                       ref_to_front_axle_mm: float = 200.0) -> 'BevConfig':
        """바닥에 놓은 직사각형의 이미지 꼭짓점 4개로 호모그래피를 만든다.

        img_pts 순서: 가까운-왼쪽, 가까운-오른쪽, 먼-오른쪽, 먼-왼쪽 (이미지 px)
        rect_w_mm: 좌우 폭, rect_l_mm: 전후 길이
        near_edge_mm: 제어 기준점 -> 직사각형의 가까운 변까지 전방 거리
        lateral_offset_mm: 직사각형 중심이 차량 중심선보다 오른쪽이면 +
        ref_to_bev_bottom_mm: BEV 맨 아래 행의 전방 거리. 생략하면 가까운 변 - 150mm
        """
        if ref_to_bev_bottom_mm is None:
            ref_to_bev_bottom_mm = max(0.0, near_edge_mm - 150.0)
        vc = bev_w / 2.0

        def to_grid(x_mm, y_mm):
            return [vc + x_mm / mm_per_px,
                    (bev_h - 1) - (y_mm - ref_to_bev_bottom_mm) / mm_per_px]

        hw, cx = rect_w_mm / 2.0, lateral_offset_mm
        dst = np.float32([
            to_grid(cx - hw, near_edge_mm),
            to_grid(cx + hw, near_edge_mm),
            to_grid(cx + hw, near_edge_mm + rect_l_mm),
            to_grid(cx - hw, near_edge_mm + rect_l_mm),
        ])
        H = cv2.getPerspectiveTransform(np.float32(img_pts), dst)
        return cls(H=H.astype(np.float64), image_width=int(image_size[0]),
                   image_height=int(image_size[1]), bev_w=int(bev_w),
                   bev_h=int(bev_h), mm_per_px_x=float(mm_per_px),
                   mm_per_px_y=float(mm_per_px), vehicle_col=float(vc),
                   ref_to_bev_bottom_mm=float(ref_to_bev_bottom_mm),
                   ref_to_front_axle_mm=float(ref_to_front_axle_mm),
                   calibrated=True)

    @classmethod
    def from_yaml(cls, path: str) -> 'BevConfig':
        with open(path, 'r', encoding='utf-8') as f:
            d = yaml.safe_load(f)['bev']
        H = np.array(d['homography'], dtype=np.float64).reshape(3, 3)
        kw = {k: d[k] for k in cls._KEYS if k in d}
        for k in ('image_width', 'image_height', 'bev_w', 'bev_h'):
            if k in kw:
                kw[k] = int(kw[k])
        return cls(H=H, calibrated=bool(d.get('calibrated', False)), **kw)

    def to_dict(self) -> dict:
        d = {k: getattr(self, k) for k in self._KEYS}
        d['homography'] = [float(v) for v in np.asarray(self.H).ravel()]
        d['calibrated'] = bool(self.calibrated)
        return d

    def to_yaml(self, path: str, note: str = '') -> None:
        out = {'bev': self.to_dict()}
        if note:
            out['bev']['note'] = note
        with open(path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)

    # --- 변환 ---------------------------------------------------------------
    def warp(self, img: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(img, self.H, (self.bev_w, self.bev_h),
                                   flags=cv2.INTER_NEAREST)

    def row_to_y_mm(self, row):
        """BEV 행 -> 제어 기준점에서의 전방 거리(mm)."""
        return self.ref_to_bev_bottom_mm + ((self.bev_h - 1) - row) * self.mm_per_px_y

    def y_mm_to_row(self, y_mm):
        return (self.bev_h - 1) - (y_mm - self.ref_to_bev_bottom_mm) / self.mm_per_px_y

    def col_to_x_mm(self, col):
        """BEV 열 -> 차량 중심선에서의 횡거리(mm, 오른쪽 +)."""
        return (col - self.vehicle_col) * self.mm_per_px_x


# ---------------------------------------------------------------------------
# 파라미터
# ---------------------------------------------------------------------------
@dataclass
class LaneParams:
    """차선 노드 파라미터. 전부 ROS 파라미터로 노출된다 (lane_node.py)."""

    # 색 마스크 (OpenCV HSV: H 0~179). 트랙 실물 색을 보고 lane_color를 정할 것
    lane_color: str = 'both'          # 'white' | 'yellow' | 'both'
    # 선 종류 판단 (M7 3차선 차로 번호). 점선 마디·간격은 실측 필요 — cov 값을 보고 line_solid_cov 조정
    line_types: bool = True
    line_type_range_mm: float = 900.0    # 가까운 이 거리 안만 봄
    line_type_min_mm: float = 300.0      # 이만큼 이상 따라간 선만 판단
    line_type_half_mm: float = 30.0      # 선 좌우 띠 반폭
    line_solid_cov: float = 0.8          # 띠에 선이 있는 행 비율이 이보다 낮으면 점선
    white_v_min: int = 180
    white_s_max: int = 60
    yellow_h_min: int = 15
    yellow_h_max: int = 40
    yellow_s_min: int = 80
    yellow_v_min: int = 80
    # 11/19 대회장 1~4차: 조명 센 곳에서 노란 선이 노출 과다로 흰색이 됨 (1119_09 첫 코너 앞: 선 V 253~254, S 150 -> 30, 37 s부터 RGB 모두 250+).
    #   일반 모드(lane_color yellow)에서 직전 프레임에 따라가던 선 둘레 ±white_bridge_mm 안의 흰색·옅은 노랑 화소만 차선으로 이어 붙인다.
    #   새 선은 만들지 않음. 가로로 white_bridge_max_w_mm 넘게 이어진 밝은 덩어리(바닥 반사)는 버림. 0 = 끔
    white_bridge_mm: float = 0.0
    white_bridge_max_w_mm: float = 60.0
    white_bridge_keep_frames: int = 3       # 선을 한 프레임 놓쳐도 이만큼은 직전 선 둘레를 계속 봄
    white_bridge_pale_s_min: int = 20       # 옅은 노랑: 노란 색상 범위 + 이 채도 이상 + 아래 밝기 이상
    white_bridge_pale_v_min: int = 200
    # 11/19: 선을 놓쳐 hold/route/search로 갈 때 속도 상한 (m/s, 0 = 끔). 1~4차 첫 코너 앞에서 선 없이 0.4 m/s로 3 m 달림
    lost_speed: float = 0.0
    # 마스크 전처리
    suppress_transverse: bool = True  # 가로로 긴 선(정지선·시작선)을 차선 마스크에서 제거
    max_run_mm: float = 150.0         # 이보다 긴 가로 연속 화소는 차선이 아니라고 본다
    dilate_px: int = 1
    # 차선 쌍 찾기
    lane_width_mm: float = 530.0      # 좌우 차선 중심 사이 거리. 미션이 구간별로 바꾼다
    hist_band_mm: float = 600.0       # 히스토그램에 쓰는 하단 밴드 높이
    min_peak: int = 20                # 후보 열이 밴드 안에서 가져야 할 최소 화소 수
    pair_min_ratio: float = 0.6       # 쌍 폭 하한 (예상 폭 대비). 노면 마름모와 짝을 막음
    pair_max_ratio: float = 1.35
    # 11/19: 갈림길 섬 기준 (left_only/right_only에서만). A(왼쪽 갈래)면 섬 왼쪽 변 = 내 차로 오른쪽 선(right_only),
    #   B면 섬 오른쪽 변 = 왼쪽 선(left_only). 차가 섬 변 위에 걸쳐 있으면 가장 가까운 선이 섬 반대쪽 변이 됨(1119_09 133~136 s) ->
    #   그 갈래 바깥선(B면 맨 오른쪽)이 보이면 거기서 차로 폭(0.8~1.25배) 안쪽에 있는 선을 섬 변으로 고른다. false = 예전
    island_ref: bool = False
    island_letter_frames: int = 8          # 그 갈래 바닥 글자를 본 뒤 이 프레임 동안 글자 위치로 섬 변을 고름 (글자 = 차로 가운데)
    continuity_lambda: float = 3.0    # 쌍 점수 = |폭-예상| + lambda*|중점-직전 중점|
    side_deadband_mm: float = 40.0    # 한쪽 선이 중심 근처일 때 좌/우 판정 유지 폭
    side_track_mm: float = 300.0      # 한쪽 선만 보일 때, 직전 시작점에서 이 안이면 같은 선으로 본다 (차로 폭의 절반 남짓)
    min_component_px: int = 120       # 히스토그램이 비면(급곡선의 비스듬한 선) 덩어리로 찾는 최소 화소 수
    # 슬라이딩 윈도우
    n_windows: int = 16
    window_margin_mm: float = 100.0
    window_min_pix: int = 8
    window_max_miss: int = 4
    # 중앙선 / Pure Pursuit
    fit_order: int = 2                # 지그재그 구간은 1
    min_points: int = 4               # 중앙선 최소 점 수 (윈도우 단위)
    lookahead_mm: float = 650.0       # 제어 기준점에서 목표점까지 전방 거리
    target_offset_mm: float = 0.0     # 차로 중심에서 옆으로 비켜 갈 거리 (왼쪽 +). 3차선 변경용
    kappa_max: float = 2.1            # 1/m. 최소 회전반경 약 0.46m(안쪽 바퀴 28도, 가상 바퀴 0.41rad)
    # 원호 모델: 급곡선(R~0.6m)에서는 보이는 선이 짧고 옆으로 누워 x(y) 다항식이 틀린다.
    # 보이는 선 조각에 원을 맞춰 중앙선 원을 만들고, 그 원 위에서 목표점을 잡는다.
    circle_fit: bool = True
    circle_max_r_mm: float = 2500.0   # 이보다 큰 원(완만한 곡선·직선)은 다항식을 쓴다
    circle_min_r_mm: float = 250.0
    circle_max_rms_mm: float = 30.0   # 원 맞춤 잔차 상한 (실제 테이프는 BEV에서 40~60mm 두께)
    circle_min_gain_mm: float = 8.0   # 직선 대비 원이 줄여 주는 잔차가 이 이상일 때만 원을 쓴다 (직선 구간 보호)
    direct_target: bool = True        # 보이는 선을 따라 걸어 목표점을 바로 잡는다 (코너 모양 가정 없음)
    # 지연 보상: 사진을 찍은 뒤 바퀴가 실제로 꺾이기까지 걸리는 시간(초). 그만큼 차가 이미
    # 움직였다고 보고, 예측 위치에서 목표점을 본다. 10/30 녹화로 잰 명령->실제 회전 지연은
    # 0.28~0.40초 (카메라·처리 지연 별도). 0이면 끔.
    latency_comp_sec: float = 0.35
    # 코너 바깥 선만 보일 때 반폭에 더 얹는 거리(mm). 코너 가운데서 차로가 약 730mm(반폭 365)로
    # 넓어지므로 최대 약 100. 0이면 끔
    corner_outer_extra_mm: float = 75.0
    circle_min_points: int = 4
    # 선 따라 걷기 보강 (11/06, 두 번째 코너)
    walk_smooth_pts: int = 7          # 걸은 점(40mm 간격)을 이 개수로 이동 평균 -> 매트 주름 흔들림 줄임. 1이면 끔
    walk_kink_deg: float = 0.0        # >0이면 120mm 사이 이만큼 넘게 꺾일 때 끊음 (3차선 입구용 35). 매트 이음매 어긋남도 끊어 코너가 늦어져서 기본 끔
    pair_agree_ratio: float = 0.5     # 두 선 목표점 거리 < 이 비율 x 차로 폭이면 평균. 넘으면 긴 선 하나만
    far_start_mm: float = 250.0       # 선 조각이 lookahead+이 거리보다 멀리서 시작하면 안 씀. 0이면 끔
    # 시간 필터 / 게이트
    kappa_alpha: float = 0.5          # 곡률 명령 EMA (1.0 = 필터 없음)
    kappa_rate_max: float = 0.6       # 프레임당 곡률 변화 한계 (1/m)
    jump_max_mm: float = 120.0        # 목표점 횡위치가 한 프레임에 이보다 튀면 버린다
    jump_max_frames: int = 6          # 이만큼 연속으로 튀면 새 값을 받아들인다
    lost_hold_frames: int = 30        # 차선을 놓친 뒤 버티는 프레임 수 (30fps면 1초)
    lost_corner_window: bool = True   # 11/18: 그때 바로 앞 도면 곡률이 0이어도(위치 s 뒤처짐) 앞 창 안의 코너 곡률로 돎
    lost_corner_frames: int = 120     # 11/12 밤: M7 직전 코너 미리보기가 있을 때 선을 놓쳐도 코너 곡률로 버티는 프레임 수
    # 11/18: 코너 미리보기가 켜져 있어도 아직 코너 앞(미리보기 곡률 0)이면 섞지 않고 차선 그대로.
    #   예전엔 0과 80% 섞여 차선 조향이 20%로 줄었다 -> M2 합류(B 왼쪽 비스듬)를 못 따라 직진, A는 비스듬한 채 코너 안쪽으로
    route_corner_zero_lane: bool = False   # 미션 노드가 M2_EXIT에서만 켬 (/lane/config). M7·M1 코너는 예전 그대로
    route_corner_weight: float = 0.8  # 11/12 밤: M7 직전 코너에서 선이 보여도 코너 곡률을 섞는 비율 (실차 60~70도 뒤 직진 -> 0.6에서 0.8)
    # 11/12 밤 실차 요청: 미리 아는 코너(M7 직전, M2 뒤 = 미션이 코너 미리보기를 줄 때)에서는 안쪽 선 기준.
    #   코너에서 바깥 선은 화면(BEV) 밖으로 나가거나 다른 차로 선과 헷갈리고, 안쪽 선은 차 가까이 계속 보인다.
    #   미리보기 앞 corner_inner_ahead_mm 안에 코너 곡률이 있으면 양쪽이 보여도 안쪽 선 하나 + 반폭으로 중앙선을 만든다
    # 11/12 밤 실차 요청: 선을 오래 놓치면 서서 끝내지 말고 찾아본다 (미션이 M7 차선 구간·M2·M2 뒤에서만 켬).
    #   놓친 직후(lost_hold_frames까지): hold_axis_deg(미션이 준 직선 방향)가 있으면 그 방향으로 조향을 서서히 풂
    #     (합류 사선에서 놓치면 예전엔 사선 방향을 유지해 반대쪽 선을 뚫고 나감)
    #   그 뒤: 기준 방향에서 '직전 조향의 반대쪽'으로 lost_search_deg 틀어 lost_search_mm, 다시 반대쪽으로 2배 거리, 그래도 없으면 정지.
    #   선이 다시 보이면 언제든 바로 차선 주행 (정지 중에도 계속 인식)
    lost_search: bool = False
    lost_search_deg: float = 30.0
    lost_search_mm: float = 350.0
    lost_search_speed: float = 0.6     # 찾는 동안 속도 비율
    hold_axis_deg: float = 999.0       # 놓쳤을 때 붙잡을 직선 방향 (IMU yaw, deg). 999 = 없음 (마지막 차선 방향)
    corner_inner_line: bool = True
    corner_inner_ahead_mm: float = 600.0
    lost_decay: float = 0.95          # IMU가 없을 때 마지막 곡률의 프레임당 감쇠
    # 헤딩 유지 (IMU)
    yaw_kp: float = 1.5               # 헤딩 오차 1 rad당 요레이트 명령 (rad/s)
    # 정지선
    # 11/15 바닥 갈림길 글자 A/B (흰 페인트): 차로 안 글자 크기 흰 덩어리의 구멍 수로 구분 (A = 구멍 1, B = 구멍 2).
    #   표지판(YOLO)을 놓쳐도 갈림길에 들어간 뒤 어느 쪽(A/B)인지 안다. 크기는 실측 전 넉넉히
    floor_letter: bool = True
    floor_letter_min_h_mm: float = 120.0    # 진행 방향 글자 높이 범위
    floor_letter_max_h_mm: float = 700.0
    floor_letter_min_w_mm: float = 60.0     # 옆 폭 범위
    floor_letter_max_w_mm: float = 450.0
    floor_letter_max_off_mm: float = 260.0  # 글자 가운데가 차 중심에서 옆으로 이 안 (내 차로)
    floor_letter_hole_mm2: float = 600.0    # 구멍으로 셀 최소 넓이 (점선·잡음 구멍 제외)
    floor_letter_max_fill: float = 0.85     # 덩어리 상자 중 흰 비율이 이보다 크면 글자 아님 (가로선·막대)
    stopline_fill: float = 0.6        # 차로 안쪽 폭 중 흰 화소 비율이 이 이상이면 가로선
    # 11/16: 비스듬한 가로선 (곡선 돌자마자 나오는 라바콘 시작선). 0°로 못 찾으면 띠를 ±이 각도까지 기울여(전단) 다시 찾는다.
    #   0°에서 선이 잡히면 예전과 똑같다. 거리는 차로 가운데 열에서 잰다. 0 = 끔 (예전: 40 mm 선 약 ±11°까지)
    stopline_max_angle_deg: float = 15.0
    stopline_angle_step_deg: float = 2.5
    stopline_min_mm: float = 8.0
    stopline_max_mm: float = 80.0
    crosswalk_look_mm: float = 150.0  # 가로선 바로 뒤 이 거리 안에서 횡단보도 막대를 찾는다
    bar_fill_min: float = 0.15
    bar_fill_max: float = 0.75
    bar_min_segments: int = 2
    # 11/09 bag: 정지선 뒤 150mm 안 막대 행 비율 0.76~1.0, '천천히' 글자 행은 0.47 이하.
    # 0.3이면 글자도 횡단보도 선이 됐다 (M3·M1 정지 위치와 도면 위치가 글자에 끌려감)
    bar_row_ratio: float = 0.6
    # 가로띠 안 행마다 흰 구간 수의 평균. 실제 선 1~3.8, 글자 5~9, 횡단보도 블록 가장자리 4~5
    stopline_max_segments: float = 3.9
    # BEV 맨 아래·맨 위 행에 닿은 띠는 잘린 것이라 가까운 가장자리 거리를 모른다.
    # (11/09: 정지선이 사각지대로 들어간 뒤 횡단보도 블록이 "348mm 앞 선"으로 계속 잡혀 끝선에서 섬)
    stopline_skip_edges: bool = True
    # 속도
    curve_slowdown: float = 0.3       # |kappa| = kappa_max일 때 속도를 30% 줄인다
    # --- 11/06 고도화 (전부 기본 꺼짐: 0이면 예전과 똑같이 달린다) ---
    # 차선 기억: 최근에 본 중앙선 점을 odom+IMU로 옮겨 쌓아 두고, 선을 놓치거나(lost)
    # 중앙선이 lookahead까지 안 닿을 때(low_conf) 그 점들로 목표점을 잡는다.
    # 값 = 기억할 주행 거리(mm). 권장 600. 위치(pose)가 안 들어오면 자동으로 안 쓴다.
    memory_mm: float = 0.0
    memory_ring_mm: float = 60.0
    memory_on_low_conf: bool = True   # False면 완전히 놓쳤을 때만 기억을 쓴다      # 목표점: lookahead ± 이 거리 안의 기억 점 중앙값
    # 도면 곡률 미리 넣기 (미션 노드가 /route/preview로 보냄). 0이면 끔.
    route_ff_weight: float = 0.0      # 차선이 보일 때 도면 곡률 섞는 비율 (권장 0.3)
    route_ff_weight_lost: float = 0.0  # 놓쳤을 때(hold) 비율 (권장 0.8)
    route_ff_preview_mm: float = 300.0  # 앞바퀴에서 이만큼 앞의 도면 곡률을 쓴다
    route_ff_gate: float = 0.8        # 두 선이 다 잘 보이는데 도면과 이만큼(1/m) 넘게 다르면 도면 무시
    route_ff_weight_both: float = 0.0  # 두 선이 다 잘 보일 때 비율. 모의 시험에서 섞으면 오히려 나빠져 기본 0
    # 도면 복귀: 차선을 놓쳤을 때(기억도 없을 때) 미션이 보낸 도면 중심선 점(차 기준)으로 목표점
    route_recover: bool = False
    # 코너 들어가기 전 감속: 앞에 보이는(또는 도면상) 곡률이 preview_kappa_ref면 이 비율만큼 감속
    preview_slowdown: float = 0.0     # 권장 0.25
    preview_kappa_ref: float = 1.5
    # 속도에 따라 lookahead 늘리기: lookahead = lookahead_mm + 이 값 * max(0, 속도 - 0.3). 상한 lookahead_max_mm
    #   11/19: 0.3 m/s 넘을 때만 늘림 (그 밑은 미션이 정한 lookahead 그대로 -> 커브 0.15·M7·지그재그·코너 미리보기 동작 안 바뀜).
    #   1118_10 직선 0.47 m/s S자(판단~바퀴 지연 약 0.6 s) 시뮬레이션: 1000이면 ±53 -> ±5 mm
    lookahead_per_mps: float = 0.0
    lookahead_max_mm: float = 750.0
    # 한쪽 선만 보일 때 lookahead (0이면 같은 값). 모의 맵 코너는 바깥 선이 안쪽 선보다 60~70cm 늦게
    # 휘어서, 바깥 선만 보일 때 650 앞은 아직 직선 -> 늦게 꺾는다 (11/06 두 번째 코너 이탈)
    single_lookahead_mm: float = 0.0
    # 사라진 안쪽 선 원 이어 쓰기: 두 선이 보일 때 한쪽 선이 자기 쪽으로 휘는 원(R350~1500)이면 기억했다가,
    # 그 선이 시야 밖으로 나가 반대쪽(바깥) 선만 보이면 기억한 원을 안쪽 선으로 써서 가운데를 잡는다.
    # 값 = 기억을 쓰는 최대 주행 거리(mm, 권장 1000). 위치(pose) 필요. 0이면 끔
    lost_arc_mm: float = 0.0
    lost_arc_max_turn_deg: float = 75.0   # 기억한 뒤 차가 이만큼 돌면(코너 끝) 그만 쓴다
    # 11/14: 노란 실선은 트랙 바깥 경계선이므로 절대 이탈하면 안 됨 -> 가까워지면 안쪽으로 강하게 조향
    yellow_solid_guard: bool = True
    yellow_guard_margin_mm: float = 200.0  # 차 중심에서 노란 실선까지 옆 거리가 이 안이면 강하게 안쪽 조향 시작
    yellow_guard_hard_mm: float = 140.0    # 이 안이면 변화율 제한을 무시하고 최대 곡률(kappa_max)로 즉시 조향
    yellow_guard_gain: float = 2.0
    # 11/15 실차: 직선에서 좌우로 왔다갔다 하다 첫 코너에서 이탈. 일반 구간은 양쪽 선이 모두 노란 실선이라 차가 6 cm만
    #   비켜도 지킴이 걸리고 14 cm면 순간 최대 조향 -> 반대쪽에서 또 걸림 (차선 시뮬레이션: 지연 4프레임·잡음 12 mm에서
    #   최대 조향 2.1을 15초에 14번 좌우로 뒤집음, 지킴 끄면 0번). 반대쪽 선도 노란 실선이면(= 일반 1차로) 지킴 안 함:
    #   그때는 양쪽 선 가운데 맞추기가 이미 양쪽 경계를 본다. M7 바깥 차로(노란 실선 + 흰 점선)·한쪽 선만 보일 때만 지킴
    yellow_guard_one_side_only: bool = True
    # 11/15 실차: 모든 코너에서 '초반에 과하게 꺾고 그 뒤 직진해 뚫고 나감', M2에서 섬 쪽을 밟음.
    #   카메라는 차 앞 0.55 m부터만 보이는데 지킴은 그 앞 노란선까지 거리를 잰다 -> 코너 앞에서 바깥 선이 앞쪽에서 휘어 들어오면
    #   '노란선에 붙었다'로 보고 코너 0.6 m 전부터 끝까지 꺾음 (코너 시뮬레이션: 안쪽 35 cm 밀려 선 밟음, 끄면 11 cm).
    #   M2 한쪽 선 모드도 휘어 나가는 바깥선을 '붙었다'로 보고 섬 쪽으로 밀어냄. 3차선(M7, lane_color both) 직선 구간에서만 지킴
    yellow_guard_three_lane_only: bool = True
    # 11/15 실차 '커브마다 아슬아슬': 차선이 커브라고 할 때(|곡률| > corner_gain_from) 조향을 corner_kappa_gain 배.
    #   직선·작은 보정에는 안 걸림. 1.0 = 끔
    # 11/15 저녁 실차: 차가 차선과 크게 틀어지면(직각에 가까우면) 노란 차선이 가로 선으로 보여 정지선처럼 지워지거나 놓침.
    #   일반 구간(lane_color yellow)에서 선을 놓쳤는데 크게 기운(realign_deg 이상) 노란 선 덩어리가 보이면 '내 방향이 틀렸다'로 보고
    #   그 선과 나란해지는 쪽으로 돈다 (정지선은 흰색이라 노란 마스크에 없음). 0 = 끔
    realign_deg: float = 0.0           # 꺼 둠 (11/15 시험): 45°로 켜면 코너의 휘는 선을 가로선으로 착각해 코너 시뮬레이션 선 밟음 0 -> 15/15, 크게 틀어진 직선에선 효과 없음(카메라 사각 0.55 m)
    realign_min_mm: float = 200.0      # 이보다 짧은 덩어리는 안 씀 (노이즈)
    realign_speed: float = 0.6         # 그동안 속도 비율
    corner_kappa_gain: float = 1.1     # 코너 시뮬레이션: 바깥 36 -> 26 mm, 안쪽 112 -> 120 mm (안쪽 선까지 17 mm). 1.2면 안쪽 여유 10 mm
    corner_gain_from: float = 0.6
    # 11/18: 평상시 0.5 m/s로 올리며 커브에서만 lookahead를 늘림 (커브를 일찍 보고 일찍·안쪽으로 꺾게).
    #   커브 판단 = 직전 조향 곡률 또는 도면 앞 곡률 |k| > corner_ld_on (꺼질 땐 corner_ld_off 밑).
    #   기본 lookahead가 corner_ld_base_min 이상인 구간(일반 주행)에서만. 지그재그(260)·M7(400) 등 미션이 줄인 구간은 그대로. 0 = 끔
    corner_lookahead_mm: float = 700.0   # 코너 시뮬레이션(R640, 0.3·0.5 m/s): 650 -> 700 안쪽 최악 112 -> 120 mm, 선 밟음 0. 750까지 0, 800부터 안쪽 선 밟음
    corner_ld_on: float = 0.6
    corner_ld_off: float = 0.3
    corner_ld_base_min: float = 600.0
    # 11/19: 모든 커브 속도 상한 (m/s, 0 = 끔). 커브 판단 = 직전 조향 곡률 또는 도면 앞 corner_speed_look_mm 안 곡률 |k| > corner_ld_on
    #   (꺼질 땐 corner_ld_off 밑). 일반 주행(lookahead >= corner_ld_base_min) 또는 미션 노드 코너 미리보기 중일 때만
    #   (지그재그 260·M7 3차선 400처럼 미션이 lookahead를 줄인 구간의 큰 조향에는 안 걸림)
    corner_speed: float = 0.0
    corner_speed_look_mm: float = 600.0
    # 11/19 실차 '코너를 아직 크게 돎': 위 커브 판단에 걸릴 때만 조향 배율 (0 = corner_kappa_gain 그대로).
    #   실제 도면 코너 모양 시뮬레이션(0.15 m/s, 실제 회전 80~90%): 1.1 -> 1.2 바깥 여유 최소 23 -> 54 mm, 선 닿음 0
    curve_kappa_gain: float = 0.0
    yellow_guard_look_mm: float = 600.0   # 노란 실선까지 거리를 잴 때 이 앞까지만 (400으로 줄이면 M7 1차로 시뮬레이션 흔들림 37 -> 89 mm)
    yellow_guard_min_urg: float = 0.35    # 지킴이 걸리는 순간 최소 세기 (0이면 margin에서 0부터 hard까지 서서히)
    # 11/19 corner_fix (실험, 기본 꺼짐 = 기존 동작). 05·06·08판 분석: 코너 중 조향이 풀리거나 뒤집혀 실패
    keep_kappa_on_heading_switch: bool = False  # normal<->heading 전환 때 조향 명령·코너 판단을 0으로 지우지 않음 (4번 강제 원호 깜빡임)
    corner_lock: bool = False          # 코너 미리보기 중 |도면 곡률| >= corner_ld_on 이면 기존 섞기 결과에 '덜 꺾기 하한'만 건다
    corner_lock_corr_max: float = 0.3  #   코너 방향으로 최소 |도면 곡률| - 이 값 (1/m). 더 꺾는 쪽(바깥 밀림 보정)은 기존 그대로

    @classmethod
    def from_dict(cls, d: dict) -> 'LaneParams':
        names = {f.name for f in fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})

    def as_dict(self) -> dict:
        return asdict(self)


MODES = ('normal', 'zigzag', 'left_only', 'right_only', 'heading')


# ---------------------------------------------------------------------------
# 출력 / 중간 결과
# ---------------------------------------------------------------------------
@dataclass
class Detection:
    status: str                         # 'BOTH' | 'L' | 'R' | 'NONE'
    left_x0: Optional[float] = None     # 하단 밴드에서 찾은 시작 열 (BEV px)
    right_x0: Optional[float] = None
    expected_w_px: float = 0.0


@dataclass
class Centerline:
    pts_mm: np.ndarray                  # (n, 2) [x 오른쪽+, y 전방+], 가까운 -> 먼 순
    coeffs: np.ndarray                  # x(y) 다항식 계수 (np.polyval 순서)
    y_near: float
    y_far: float
    # 원호 모델 (cx, cy, rc, s, turn): 중심(mm), 중앙선 반경(mm),
    # 진행 방향 s(+1 반시계, -1 시계, x 오른쪽+ y 전방+ 좌표), turn(+1 오른쪽 곡선, -1 왼쪽)
    circle: Optional[Tuple[float, float, float, int, int]] = None


@dataclass
class LaneOutput:
    kappa: float = 0.0                  # 최종 명령 곡률 (1/m, 왼쪽 +). angular.z = v*kappa
    speed_scale: float = 1.0            # 곡선 감속 배율. 0이면 정지
    status: str = 'NONE'                # 'BOTH' | 'L' | 'R' | 'NONE'
    source: str = 'lane'                # 'lane' | 'coast' | 'hold' | 'heading' | 'stop'
    offset_mm: Optional[float] = None   # 가장 가까운 행에서 차로 중심의 횡위치 (왼쪽 +)
    heading_rad: Optional[float] = None  # 차로 방향 (왼쪽 +)
    path_kappa: Optional[float] = None  # 중앙선 피팅 곡률 (왼쪽 +, 1/m)
    target_mm: Optional[Tuple[float, float]] = None  # PP 목표점 (x 오른쪽+, y 전방+)
    low_conf: bool = False              # 중앙선이 lookahead까지 안 닿음
    stoplines_mm: List[float] = field(default_factory=list)  # 앞바퀴 -> 모든 흰 가로선 (가까운 순)
    crosswalk_lines_mm: List[float] = field(default_factory=list)  # 그중 횡단보도 막대가 뒤따르는 선
    # 흰·노란 가로선 모두 (차 정면 기준). 주차 칸 끝 선(줄 사이 흰 선, 바깥 노란 테두리)용
    end_lines_mm: List[float] = field(default_factory=list)
    # 양쪽 선 종류 (M7 3차선에서 몇 번 차로인지): {'color': 'yellow'|'white', 'dashed': bool, 'cov': 0~1}
    line_left: Optional[dict] = None
    line_right: Optional[dict] = None
    # 3차선 차로 번호 (진행 방향 기준 +1 왼쪽/1차선, 0 가운데/2차선, -1 오른쪽/3차선). 모르면 None
    #   왼쪽 노란 실선 + 오른쪽 흰 점선 = +1, 양쪽 흰 점선 = 0, 왼쪽 흰 점선 + 오른쪽 노란 실선 = -1
    lane_id: Optional[int] = None
    yellow_guard: Optional[str] = None  # 노란 실선 이탈 방지 강제 조향 작동 여부 ('L' | 'R' | None)
    # 11/18 시험: 노란 실선까지 옆 거리 (차 중심선 기준 mm, 양수) + 잰 곳의 앞 거리 (제어 기준점 기준 mm).
    #   M7 재생 노드가 '차로 변경 중 노란선 밟기 직전'을 판단하는 데 씀. 그쪽 선이 노란 실선이 아니면 None
    yellow_l_mm: Optional[float] = None
    yellow_r_mm: Optional[float] = None
    yellow_f_mm: Optional[float] = None
    floor_letter: Optional[str] = None  # 11/15 바닥 갈림길 글자 'A' | 'B' (내 차로 안, 구멍 수로 구분)
    floor_letter_mm: Optional[float] = None    # 앞바퀴 -> 글자 가까운 끝
    floor_letter_off_mm: Optional[float] = None  # 글자 가운데 옆 위치 (오른쪽 +)
    mem_pts: int = 0                    # 기억 점 수 (차선 기억이 켜졌을 때)
    ff_kappa: Optional[float] = None    # 도면 곡률 (왼쪽 +, 1/m). 미리 넣기가 켜졌을 때
    lookahead_mm: Optional[float] = None
    debug: dict = field(default_factory=dict)

    def status_dict(self) -> dict:
        d = {k: getattr(self, k) for k in (
            'mem_pts', 'ff_kappa', 'lookahead_mm',
            'kappa', 'speed_scale', 'status', 'source', 'offset_mm',
            'heading_rad', 'path_kappa', 'low_conf', 'stoplines_mm',
            'crosswalk_lines_mm', 'end_lines_mm', 'line_left', 'line_right', 'lane_id',
            'yellow_guard', 'yellow_l_mm', 'yellow_r_mm', 'yellow_f_mm',
            'floor_letter', 'floor_letter_mm', 'floor_letter_off_mm')}
        d['target_mm'] = list(self.target_mm) if self.target_mm else None
        cl = self.debug.get('centerline')
        d['model'] = ('circle' if cl.circle is not None else 'poly') if cl is not None else None
        d['circle_r_mm'] = round(cl.circle[2]) if cl is not None and cl.circle is not None else None
        d['inner'] = self.debug.get('inner')      # 코너 안쪽 선 기준이면 'L'/'R'
        d['white_bridge'] = self.debug.get('white_bridge')   # 11/19: 흰색 이어 붙이기로 더한 화소 수
        d['realign_deg'] = self.debug.get('realign_deg')   # 크게 기운 노란 선으로 방향 바로잡는 중이면 그 선 각도
        d['stopline_angle_deg'] = self.debug.get('stopline_angle_deg')  # 가로선을 찾은 기울기 (0 = 예전처럼 수평)
        return d


# ---------------------------------------------------------------------------
# 마스크
# ---------------------------------------------------------------------------
def color_masks(bgr: np.ndarray, p: LaneParams) -> Tuple[np.ndarray, np.ndarray]:
    """(차선 마스크, 흰색 마스크)를 돌려준다. 흰색 마스크는 정지선 검출용."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, p.white_v_min], np.uint8),
                        np.array([179, p.white_s_max, 255], np.uint8))
    yellow = cv2.inRange(hsv, np.array([p.yellow_h_min, p.yellow_s_min,
                                        p.yellow_v_min], np.uint8),
                         np.array([p.yellow_h_max, 255, 255], np.uint8))
    if p.lane_color == 'white':
        lane = white
    elif p.lane_color == 'yellow':
        lane = yellow
    else:
        lane = cv2.bitwise_or(white, yellow)
    return lane, white


def remove_transverse(bev_mask: np.ndarray, max_run_px: float) -> np.ndarray:
    """가로로 max_run_px 이상 이어진 화소(정지선 등)를 지운다."""
    k = max(3, int(round(max_run_px)))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, 1))
    long_runs = cv2.morphologyEx(bev_mask, cv2.MORPH_OPEN, kernel)
    return cv2.subtract(bev_mask, long_runs)


# ---------------------------------------------------------------------------
# 차선 쌍 찾기 (하단 밴드 히스토그램)
# ---------------------------------------------------------------------------
def _side_candidates(col_sum: np.ndarray, lo: int, hi: int, min_peak: int,
                     suppress_px: int, k: int = 3) -> List[Tuple[float, int]]:
    """[lo, hi) 열에서 강한 순으로 후보를 최대 k개 고른다 (주변 suppress_px는 억제)."""
    seg = col_sum[lo:hi].astype(np.int64, copy=True)
    out = []
    for _ in range(k):
        if seg.size == 0:
            break
        i = int(np.argmax(seg))
        pk = int(seg[i])
        if pk < min_peak:
            break
        out.append((float(lo + i), pk))
        seg[max(0, i - suppress_px): i + suppress_px + 1] = 0
    return out


def detect_lanes(bev: np.ndarray, cfg: BevConfig, p: LaneParams, mode: str,
                 mid_ema: Optional[float], last_side: Optional[str],
                 last_lr: Tuple[Optional[float], Optional[float]] = (None, None),
                 hint_center: Optional[float] = None) -> Detection:
    h, w = bev.shape[:2]
    exp_w = p.lane_width_mm / cfg.mm_per_px_x
    band_rows = int(min(h, max(10, round(p.hist_band_mm / cfg.mm_per_px_y))))
    col_sum = (bev[h - band_rows:] > 0).sum(axis=0)
    vc = cfg.vehicle_col
    vci = int(round(min(max(vc, 1.0), w - 1.0)))
    suppress = max(2, int(round(p.window_margin_mm / cfg.mm_per_px_x)))
    det = Detection(status='NONE', expected_w_px=exp_w)

    if mode in ('left_only', 'right_only') and p.island_ref:
        # 11/19 섬 기준 (island_ref 설명). left_only = B: 섬 오른쪽 변이 차 왼쪽, right_only = A: 섬 왼쪽 변이 차 오른쪽
        cands = _side_candidates(col_sum, 0, w, p.min_peak, suppress, k=6)
        if cands:
            xs = sorted(c[0] for c in cands)
            pick = None
            if hint_center is not None:
                # 바닥 글자(A/B)는 그 갈래 차로 한가운데 -> 섬 변은 글자에서 반폭 섬 쪽
                exp_x = hint_center - exp_w / 2.0 if mode == 'left_only' else hint_center + exp_w / 2.0
                near = min(xs, key=lambda x: abs(x - exp_x))
                if abs(near - exp_x) <= 0.35 * exp_w:
                    pick = near
            if pick is None and mode == 'left_only' and len(xs) >= 2:
                outer = xs[-1]                         # B 바깥선 (맨 오른쪽)
                inner = [x for x in xs[:-1] if 0.8 * exp_w <= outer - x <= 1.25 * exp_w]
                if inner and outer > vc:
                    pick = max(inner)
            elif pick is None and mode == 'right_only' and len(xs) >= 2:
                outer = xs[0]                          # A 바깥선 (맨 왼쪽)
                inner = [x for x in xs[1:] if 0.8 * exp_w <= x - outer <= 1.25 * exp_w]
                if inner and outer < vc:
                    pick = min(inner)
            if pick is None:
                expect = vc - exp_w / 2.0 if mode == 'left_only' else vc + exp_w / 2.0
                pick = min(xs, key=lambda x: abs(x - expect))
            if mode == 'left_only':
                det.left_x0, det.status = pick, 'L'
            else:
                det.right_x0, det.status = pick, 'R'
        return det

    if mode in ('left_only', 'right_only'):
        # 갈림길: 한쪽 경계만 따라간다. 중심선을 조금 넘어온 선까지 허용.
        if mode == 'left_only':
            cands = _side_candidates(col_sum, 0, min(w, vci + suppress),
                                     p.min_peak, suppress)
            expect = vc - exp_w / 2.0
        else:
            cands = _side_candidates(col_sum, max(0, vci - suppress), w,
                                     p.min_peak, suppress)
            expect = vc + exp_w / 2.0
        if cands:
            x = min(cands, key=lambda c: abs(c[0] - expect))[0]
            if mode == 'left_only':
                det.left_x0, det.status = x, 'L'
            else:
                det.right_x0, det.status = x, 'R'
        return det

    left_c = _side_candidates(col_sum, 0, vci, p.min_peak, suppress)
    right_c = _side_candidates(col_sum, vci, w, p.min_peak, suppress)

    # 하단 밴드 덩어리 번호. 두껍고 비스듬한 선 하나가 히스토그램에 봉우리 두 개를 만들면
    # 같은 선을 좌우 쌍으로 착각한다 -> 같은 덩어리면 쌍이 아니다.
    band = (bev[h - band_rows:] > 0).astype(np.uint8)
    n_lab, lab, stats, _ = cv2.connectedComponentsWithStats(band, connectivity=8)

    def labels_at(x):
        c0, c1 = int(max(0, x - 2)), int(min(w, x + 3))
        v = np.unique(lab[:, c0:c1])
        return set(int(t) for t in v if t > 0)

    best, best_score = None, None
    for lx, _ in left_c:
        for rx, _ in right_c:
            pw = rx - lx
            if pw < p.pair_min_ratio * exp_w or pw > p.pair_max_ratio * exp_w:
                continue
            if labels_at(lx) & labels_at(rx):
                continue
            score = abs(pw - exp_w)
            if mid_ema is not None and p.continuity_lambda > 0.0:
                score += p.continuity_lambda * abs((lx + rx) / 2.0 - mid_ema)
            if best is None or score < best_score:
                best, best_score = (lx, rx), score
    if best is not None:
        det.left_x0, det.right_x0 = best
        det.status = 'BOTH'
        return det

    # 쌍이 없으면 가장 강한 선 하나. 좌/우는 차량 중심선 기준 절대 판정.
    singles = left_c[:1] + right_c[:1]
    if not singles:
        # 급곡선에서는 선이 비스듬히 누워 열별 화소 수가 min_peak에 못 미친다.
        # 하단 밴드에서 가장 큰 덩어리를 찾아 그 아래쪽 끝을 시작점으로 쓴다.
        if n_lab <= 1:
            return det
        k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        if stats[k, cv2.CC_STAT_AREA] < p.min_component_px:
            return det
        ys, xs = np.nonzero(lab == k)
        low = ys >= ys.max() - 8
        singles = [(float(xs[low].mean()), int(stats[k, cv2.CC_STAT_AREA]))]
    x, _ = max(singles, key=lambda c: c[1])
    side = 'L' if x < vc else 'R'
    dead = p.side_deadband_mm / cfg.mm_per_px_x
    if abs(x - vc) <= dead and last_side in ('L', 'R'):
        side = last_side          # 선 위에 걸쳐 있을 때는 직전 판정을 유지
    # 급곡선에서는 바깥 선이 차 앞을 가로질러 중심선 반대편으로 넘어간다.
    # 직전에 본 좌/우 선 위치 중 가까운 쪽이 일정 거리 안이면 그 선으로 본다.
    lim = p.side_track_mm / cfg.mm_per_px_x
    near = [(abs(x - lx_), sd) for lx_, sd in zip(last_lr, ('L', 'R')) if lx_ is not None]
    if near:
        d, sd = min(near)
        if d <= lim:
            side = sd
    if side == 'L':
        det.left_x0, det.status = x, 'L'
    else:
        det.right_x0, det.status = x, 'R'
    return det


# ---------------------------------------------------------------------------
# 슬라이딩 윈도우 추적
# ---------------------------------------------------------------------------
def track_line(bev: np.ndarray, x0: float, cfg: BevConfig,
               p: LaneParams) -> np.ndarray:
    """x0에서 시작해 아래 -> 위로 한 선을 따라간다. (n, 2) [col, row] 배열 (윈도우별 무게중심).

    놓친 윈도우는 직전 이동량(관성)으로 건너뛰고, window_max_miss번 넘게 놓치면 멈춘다.
    출력 배열의 i번째 행은 윈도우 i(아래부터)에 대응하며, 못 찾은 윈도우는 NaN이다.
    """
    h, w = bev.shape[:2]
    n = max(2, int(p.n_windows))
    win_h = h / n
    margin = max(2, int(round(p.window_margin_mm / cfg.mm_per_px_x)))
    out = np.full((n, 2), np.nan)
    x, shift, miss = float(x0), 0.0, 0
    last_x = None
    for i in range(n):
        y_hi = int(round(h - i * win_h))
        y_lo = int(round(h - (i + 1) * win_h))
        lo, hi = int(max(0, x - margin)), int(min(w, x + margin + 1))
        if hi - lo < 2:
            break
        ys, xs = np.nonzero(bev[y_lo:y_hi, lo:hi])
        if xs.size >= p.window_min_pix:
            x_new = lo + float(xs.mean())
            out[i] = (x_new, y_lo + float(ys.mean()))
            if last_x is not None:
                shift = float(np.clip(x_new - last_x, -margin / 2.0, margin / 2.0))
            last_x = x_new
            x = x_new + shift
            miss = 0
        else:
            miss += 1
            if miss > p.window_max_miss:
                break
            x += shift
    return out


def classify_line(track: Optional[np.ndarray], lane_mask: np.ndarray, white_mask: Optional[np.ndarray],
                  cfg: BevConfig, p: 'LaneParams') -> Optional[dict]:
    """따라간 선(track)의 색(노랑/흰)과 실선/점선. 선 열을 행마다 보간해 좌우 ±line_type_half_mm 띠를 본다.

    점선: 띠 안에 선 화소가 있는 행 비율(cov)이 line_solid_cov보다 낮음. 색: 선 화소 중 흰 마스크가 아닌 비율.
    가까운 line_type_range_mm 안에서 line_type_min_mm 이상 따라갔을 때만 판단 (모르면 None)."""
    if track is None or white_mask is None:
        return None
    ok = ~np.isnan(track[:, 0])
    if ok.sum() < 2:
        return None
    h, w = lane_mask.shape[:2]
    rows_t, cols_t = track[ok, 1], track[ok, 0]
    r_lo = max(int(rows_t.min()), h - int(p.line_type_range_mm / cfg.mm_per_px_y))
    r_hi = min(int(rows_t.max()), h - 1)
    if (r_hi - r_lo) * cfg.mm_per_px_y < p.line_type_min_mm:
        return None
    order = np.argsort(rows_t)
    rows = np.arange(r_lo, r_hi + 1)
    cols = np.interp(rows, rows_t[order], cols_t[order])
    hw = max(2, int(round(p.line_type_half_mm / cfg.mm_per_px_x)))
    hit, n_lane, n_white = 0, 0, 0
    for r, c in zip(rows, cols):
        lo, hi = max(0, int(c) - hw), min(w, int(c) + hw + 1)
        band = lane_mask[r, lo:hi] > 0
        k = int(band.sum())
        if k:
            hit += 1
            n_lane += k
            n_white += int((white_mask[r, lo:hi][band] > 0).sum())
    if n_lane == 0:
        return None
    cov = hit / float(len(rows))
    yellow = 1.0 - n_white / float(n_lane)
    return {'color': 'yellow' if yellow >= 0.5 else 'white', 'dashed': cov < p.line_solid_cov,
            'cov': round(cov, 2), 'yellow': round(yellow, 2)}


def lane_id_from_types(left: Optional[dict], right: Optional[dict]) -> Optional[int]:
    """3차선에서 차로 번호 (+1 왼쪽 / 0 가운데 / -1 오른쪽). 규칙이 안 맞으면 None."""
    if not left or not right:
        return None
    ly = left['color'] == 'yellow' and not left['dashed']
    ry = right['color'] == 'yellow' and not right['dashed']
    lw = left['color'] == 'white' and left['dashed']
    rw = right['color'] == 'white' and right['dashed']
    if ly and rw:
        return 1
    if lw and rw:
        return 0
    if lw and ry:
        return -1
    return None


# ---------------------------------------------------------------------------
# 중앙선
# ---------------------------------------------------------------------------
def _local_slope_mm(track: np.ndarray, cfg: BevConfig) -> np.ndarray:
    """윈도우별 차선 기울기 dx_mm/dy_mm (전방으로 갈수록 오른쪽이면 +)."""
    out = np.zeros(len(track))
    ok = ~np.isnan(track[:, 0])
    idx = np.nonzero(ok)[0]
    if idx.size >= 2:
        x = track[idx, 0] * cfg.mm_per_px_x
        y = cfg.row_to_y_mm(track[idx, 1])
        out[idx] = np.clip(np.gradient(x, y), -2.0, 2.0)
    return out


def _robust_polyfit(y: np.ndarray, x: np.ndarray, order: int) -> Optional[np.ndarray]:
    """잔차가 큰 점을 한 번 걸러내고 다시 맞춘다 (D-Racer 2-pass 방식)."""
    order = int(max(1, min(order, len(y) - 1)))
    coeffs = None
    fy, fx = y, x
    for _ in range(2):
        try:
            cand = np.polyfit(fy, fx, order)
        except (np.linalg.LinAlgError, ValueError):
            break
        if not np.all(np.isfinite(cand)):
            break
        coeffs = cand
        resid = np.abs(fx - np.polyval(coeffs, fy))
        cut = max(4.0 * float(np.median(resid)), 15.0)
        keep = resid <= cut
        if keep.all() or keep.sum() < order + 2:
            break
        fy, fx = fy[keep], fx[keep]
    return coeffs


def fit_circle(x: np.ndarray, y: np.ndarray) -> Optional[Tuple[float, float, float, float]]:
    """대수적 원 맞춤 (Kasa) + 잔차 큰 점 한 번 제거. (cx, cy, r, rms) mm."""
    def _fit(x, y):
        A = np.stack([x, y, np.ones_like(x)], axis=1)
        b = x * x + y * y
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return None
        cx, cy = sol[0] / 2.0, sol[1] / 2.0
        r2 = sol[2] + cx * cx + cy * cy
        if not np.isfinite(r2) or r2 <= 0:
            return None
        r = math.sqrt(r2)
        # 대수적 맞춤은 짧고 두꺼운 호에서 반경을 작게 잡는다 -> 기하학적 거리로 다듬는다
        for _ in range(20):
            dx, dy = x - cx, y - cy
            d = np.hypot(dx, dy)
            if np.any(d < 1e-6):
                break
            J = np.stack([-dx / d, -dy / d, -np.ones_like(d)], axis=1)
            res = d - r
            try:
                step, *_ = np.linalg.lstsq(J, -res, rcond=None)
            except np.linalg.LinAlgError:
                break
            cx, cy, r = cx + step[0], cy + step[1], r + step[2]
            if not np.all(np.isfinite([cx, cy, r])) or r <= 0:
                return None
            if abs(step).max() < 0.05:
                break
        return cx, cy, r
    if len(x) < 3:
        return None
    res = _fit(x, y)
    if res is None:
        return None
    cx, cy, r = res
    d = np.abs(np.hypot(x - cx, y - cy) - r)
    keep = d <= max(3.0 * float(np.median(d)), 8.0)
    if 3 <= keep.sum() < len(x):
        res2 = _fit(x[keep], y[keep])
        if res2 is not None:
            cx, cy, r = res2
            x, y = x[keep], y[keep]
            d = np.abs(np.hypot(x - cx, y - cy) - r)
    return cx, cy, r, float(np.sqrt(np.mean(d * d)))


def _track_mm(track: Optional[np.ndarray], cfg: BevConfig) -> Optional[np.ndarray]:
    """추적 결과(윈도우 순서 = 진행 순서)를 mm 점 배열로."""
    if track is None:
        return None
    ok = ~np.isnan(track[:, 0])
    if ok.sum() == 0:
        return None
    return np.stack([cfg.col_to_x_mm(track[ok, 0]), cfg.row_to_y_mm(track[ok, 1])], axis=1)


def circle_centerline(pts: np.ndarray, side: str, half_mm: float,
                      p: 'LaneParams', fit_pts: Optional[np.ndarray] = None
                      ) -> Optional[Tuple[float, float, float, int, int]]:
    """진행 순서 점들(선 하나 또는 중앙점)로 중앙선 원을 만든다.

    side: 'L' 왼쪽 선, 'R' 오른쪽 선, 'C' 이미 중앙점.
    fit_pts: 원 맞춤에 쓸 촘촘한 점(선 화소). 없으면 pts로 맞춘다. pts는 진행 방향 판단용.
    """
    if pts is None or len(pts) < 2:
        return None
    if fit_pts is None:
        if len(pts) < p.circle_min_points:
            return None
        fit_pts = pts
    elif len(fit_pts) < 20:
        return None
    fit = fit_circle(fit_pts[:, 0], fit_pts[:, 1])
    if fit is None:
        return None
    cx, cy, r, rms = fit
    if not (p.circle_min_r_mm <= r <= p.circle_max_r_mm) or rms > p.circle_max_rms_mm:
        return None
    # 직선으로도 충분히 설명되면(직선 구간·두꺼운 선의 잡음) 원을 쓰지 않는다
    fp = fit_pts - fit_pts.mean(axis=0)
    _, sv, _ = np.linalg.svd(fp, full_matrices=False)
    rms_line = float(sv[-1] / math.sqrt(len(fp)))
    # 선 두께 때문에 두 잔차 모두 커지므로 비율 대신 '원이 줄여 준 잔차'로 본다
    gain = math.sqrt(max(0.0, rms_line * rms_line - rms * rms))
    if gain < p.circle_min_gain_mm:
        return None
    # 진행 방향: 가까운 점 -> 몇 칸 앞 점
    j = min(len(pts) - 1, 2)
    # 가까운 점은 원 위로 옮겨 쓴다 (윈도우 무게중심 오차 제거)
    t = pts[j] - pts[0]
    nt = float(np.hypot(t[0], t[1]))
    if nt < 1e-6:
        return None
    t = t / nt
    p0 = pts[0]
    right_n = np.array([t[1], -t[0]])                 # 진행 방향의 오른쪽
    turn = 1 if float(np.dot(np.array([cx, cy]) - p0, right_n)) > 0 else -1
    if side in ('L', 'R'):
        # 이 트랙 코너는 동심원이 아니다: 안쪽 선 R600, 바깥 선 R650로 각자 모서리를 깎은 모양이라
        # 코너 가운데서 차로가 약 730mm로 넓어진다. 그래서 선 원의 반경을 반폭만큼 늘이고 줄이면
        # (동심원 가정) 바깥 선 기준 중앙선이 R~350으로 너무 급해진다(10/30 curve_v3).
        # 대신 반경은 그대로 두고 원을 차로 쪽으로 반폭만큼 평행 이동한다.
        n_r = np.array([t[1], -t[0]])
        shift = n_r * half_mm if side == 'L' else -n_r * half_mm
        cx, cy = cx + float(shift[0]), cy + float(shift[1])
        p0 = p0 + shift
        rc = r
        turn = 1 if float(np.dot(np.array([cx, cy]) - p0, n_r)) > 0 else -1
    else:
        rc = r
    if rc < 150.0:
        return None
    cz = (p0[0] - cx) * t[1] - (p0[1] - cy) * t[0]
    s = 1 if cz > 0 else -1
    return float(cx), float(cy), float(rc), s, turn


def _nearest_midpoints(a: np.ndarray, b: np.ndarray, lane_w_mm: float,
                       n: int = 120) -> Optional[np.ndarray]:
    """a의 각 점과 b에서 가장 가까운 점의 중점 (가까운 -> 먼 순). 거리가 차로 폭과 너무 다르면 버린다."""
    if a is None or b is None or len(a) < 10 or len(b) < 10:
        return None
    ia = np.linspace(0, len(a) - 1, min(n, len(a))).astype(int)
    aa = a[ia]
    d = np.hypot(aa[:, None, 0] - b[None, :, 0], aa[:, None, 1] - b[None, :, 1])
    j = d.argmin(axis=1)
    dist = d[np.arange(len(aa)), j]
    ok = (dist > 0.6 * lane_w_mm) & (dist < 1.6 * lane_w_mm)
    if ok.sum() < 20:
        return None
    m = 0.5 * (aa[ok] + b[j[ok]])
    return m[np.argsort(np.hypot(m[:, 0], m[:, 1]))]


def straight_centerline(pix: np.ndarray, side: str, half_mm: float,
                        big_mm: float = 1.0e5) -> Optional[Tuple[float, float, float, int, int]]:
    """선 화소를 직선으로 보고 차로 쪽으로 반폭 옮긴 중앙선을 아주 큰 원으로 표현한다.

    코너를 빠져나올 때처럼 한쪽 선이 옆으로 누워 슬라이딩 윈도우가 점을 몇 개 못 찾을 때 쓴다.
    """
    if pix is None or len(pix) < 20:
        return None
    m = pix.mean(axis=0)
    _, _, vt = np.linalg.svd(pix - m, full_matrices=False)
    d = vt[0]
    near = pix[np.argmin(np.hypot(pix[:, 0], pix[:, 1]))]
    if float(np.dot(m - near, d)) < 0:
        d = -d                               # 가까운 끝 -> 먼 쪽이 진행 방향
    n_r = np.array([d[1], -d[0]])
    p0 = near + (n_r * half_mm if side == 'L' else -n_r * half_mm)
    c = p0 + n_r * big_mm                    # 진행 방향 오른쪽으로 아주 먼 중심 = 거의 직선
    return float(c[0]), float(c[1]), float(big_mm), -1, 1


def walk_line(pix: np.ndarray, step_mm: float = 40.0, max_steps: int = 80,
              kink_deg: float = 0.0) -> Optional[np.ndarray]:
    """선 화소를 차에 가까운 끝부터 따라 걸어 일정 간격의 점열로 만든다 (진행 순서).

    슬라이딩 윈도우는 아래->위로만 움직여 옆으로 누운 선(코너 출구)을 못 따라간다.
    여기서는 방향과 상관없이 이웃 화소 덩어리를 따라간다.
    kink_deg > 0: 3걸음(120mm) 사이 방향이 이만큼 넘게 바뀌면 거기서 멈춘다.
      트랙 코너는 R580 이상이라 120mm에 12도 정도만 돈다. 3차선 입구 테이퍼(45도 꺾임)·
      갈림길 섬 끝처럼 급하게 꺾인 선을 차로로 따라가지 않게 한다.
    """
    if pix is None or len(pix) < 20:
        return None
    dist0 = np.hypot(pix[:, 0], pix[:, 1])
    start = pix[dist0 <= dist0.min() + step_mm].mean(axis=0)
    d0 = pix.mean(axis=0) - start
    n0 = float(np.hypot(d0[0], d0[1]))
    direc = d0 / n0 if n0 > 1e-6 else np.array([0.0, 1.0])
    pts = [start]
    cur = start
    for _ in range(max_steps):
        d = pix - cur
        dist = np.hypot(d[:, 0], d[:, 1])
        ring = (dist > 0.5 * step_mm) & (dist < 1.5 * step_mm)
        ahead = ring & ((d @ direc) > 0.5 * dist)
        if ahead.sum() < 3:
            break
        nxt = pix[ahead].mean(axis=0)
        v = nxt - cur
        nv = float(np.hypot(v[0], v[1]))
        if nv < 1e-6:
            break
        if kink_deg > 0.0 and len(pts) >= 4:
            a_new = math.atan2(v[1], v[0])
            back = pts[-1] - pts[-4]
            a_old = math.atan2(back[1], back[0])
            if abs(wrap_angle(a_new - a_old)) > math.radians(kink_deg):
                break
        direc = 0.6 * direc + 0.4 * (v / nv)
        direc = direc / float(np.hypot(direc[0], direc[1]))
        pts.append(nxt)
        cur = nxt
    return np.array(pts) if len(pts) >= 4 else None


def smooth_polyline(pts: np.ndarray, win: int) -> np.ndarray:
    """점열 이동 평균 (양 끝은 남은 점만으로). 매트 주름으로 구불거리는 선을 편다."""
    if win <= 1 or len(pts) < 3:
        return pts
    h = win // 2
    out = np.empty_like(pts)
    for i in range(len(pts)):
        a, b = max(0, i - h), min(len(pts), i + h + 1)
        k = min(i - a, b - 1 - i)                    # 대칭 창 (끝점은 덜 평균)
        out[i] = pts[i - k:i + k + 1].mean(axis=0)
    return out


def offset_polyline(pts: np.ndarray, side: str, half_mm: float,
                    outer_extra_mm: float = 0.0, extra_r_mm: float = 700.0) -> np.ndarray:
    """선 점열을 차로 쪽(왼쪽 선이면 오른쪽, 오른쪽 선이면 왼쪽)으로 반폭 옮긴 중앙선 점열.

    outer_extra_mm: 이 선이 코너의 바깥 선이면(왼쪽 선이 오른쪽으로 휠 때, 오른쪽 선이 왼쪽으로 휠 때)
      그 점의 굽은 정도에 비례해 더 안쪽으로 옮긴다. 모의 맵 코너는 안쪽 R600 / 바깥 R650이 동심원이
      아니라 코너 가운데서 차로가 약 730mm로 넓어진다. 바깥 선에서 반폭(257)만 옮기면 중앙선이 바깥으로
      치우쳐 코너를 늦게 꺾는다 (11/02 실차: 우코너 진입 중 왼쪽으로 조향).
      반경이 extra_r_mm 이하로 굽은 곳에서 최대.
    """
    t = np.gradient(pts, axis=0)
    t = t / np.maximum(np.hypot(t[:, 0], t[:, 1]), 1e-6)[:, None]
    n_r = np.stack([t[:, 1], -t[:, 0]], axis=1)
    off = np.full(len(pts), half_mm)
    if outer_extra_mm > 0.0 and len(pts) >= 5:
        # 점마다 곡률 (오른쪽으로 휘면 +): 방향각 변화 / 거리, 이웃 몇 점으로 평균
        ang = np.unwrap(np.arctan2(t[:, 0], t[:, 1]))
        ds = np.maximum(np.hypot(*np.gradient(pts, axis=0).T), 1e-6)
        k = np.gradient(ang) / ds
        k = np.convolve(k, np.ones(5) / 5.0, mode='same')
        outward = k if side == 'L' else -k                # 바깥 선일 때 +
        frac = np.clip(outward * extra_r_mm, 0.0, 1.0)
        off = off + outer_extra_mm * frac
    sgn = 1.0 if side == 'L' else -1.0
    return pts + sgn * n_r * off[:, None]


def polyline_target(cpts: np.ndarray, lookahead_mm: float) -> Tuple[Tuple[float, float], bool]:
    """진행 순서 중앙선 점열에서 기준점으로부터 lookahead_mm 거리를 처음 넘는 점 (보간)."""
    r = np.hypot(cpts[:, 0], cpts[:, 1])
    idx = np.nonzero(r >= lookahead_mm)[0]
    if idx.size == 0:
        return (float(cpts[-1, 0]), float(cpts[-1, 1])), True
    i = int(idx[0])
    if i == 0:
        return (float(cpts[0, 0]), float(cpts[0, 1])), True
    a = (lookahead_mm - r[i - 1]) / max(r[i] - r[i - 1], 1e-6)
    pt = cpts[i - 1] + a * (cpts[i] - cpts[i - 1])
    return (float(pt[0]), float(pt[1])), False


def direct_target(left_pix, right_pix, status: str, half_mm: float, lookahead_mm: float,
                  lane_w_mm: float, outer_extra_mm: float = 0.0, smooth_win: int = 1,
                  kink_deg: float = 0.0, far_start_mm: float = 0.0, agree_ratio: float = 0.5):
    """보이는 선을 따라 걸어 반폭 옮긴 점열 위에서 바로 목표점을 잡는다 (원·다항식 가정 없음).

    반환: (target(x, y), low_conf, centerline 점열) 또는 None.
    두 선이 다 보이면 두 목표점의 평균을 쓴다(폭이 이상하면 더 긴 선만).
    """
    cands = []
    for side, pix in (('L', left_pix), ('R', right_pix)):
        if status not in (side, 'BOTH') or pix is None:
            continue
        w = walk_line(pix, kink_deg=kink_deg)
        if w is None:
            continue
        if far_start_mm > 0.0 and math.hypot(w[0, 0], w[0, 1]) > lookahead_mm + far_start_mm:
            continue                     # 차 앞 가까운 부분 없이 멀리 떨어진 조각만 있는 선
        w = smooth_polyline(w, smooth_win)
        # 두 선이 다 보이면 평균이 넓어진 차로를 알아서 반영하므로 보정은 한쪽 선만 보일 때만
        c = offset_polyline(w, side, half_mm, outer_extra_mm if status != 'BOTH' else 0.0)
        tgt, low = polyline_target(c, lookahead_mm)
        cands.append((tgt, low, c, len(w)))
    if not cands:
        return None
    if len(cands) == 2:
        (ta, la, ca, na), (tb, lb, cb, nb) = cands
        if math.hypot(ta[0] - tb[0], ta[1] - tb[1]) < agree_ratio * lane_w_mm and not (la or lb):
            tgt = (0.5 * (ta[0] + tb[0]), 0.5 * (ta[1] + tb[1]))
            return tgt, False, ca if na >= nb else cb
        cands.sort(key=lambda c: (c[1], -c[3]))      # 끝까지 닿는 쪽, 더 긴 쪽 우선
    tgt, low, c, _ = cands[0]
    return tgt, low, c


def corner_lock_kappa(k: float, k_ff: float, corr_max: float) -> float:
    """corner_fix 코너 고정: 코너 방향으로 최소 |k_ff| - corr_max 는 꺾게 한다.
    덜 꺾거나 반대로 꺾는 것(L/R 헷갈림)만 막고, 더 꺾는 쪽은 k 그대로 (kappa_max 클램프는 뒤에서)."""
    floor = abs(k_ff) - corr_max
    return max(k, floor) if k_ff > 0 else min(k, -floor)


def pp_kappa_with_latency(tgt, kappa_cmd_left: float, speed: float, latency_sec: float) -> float:
    """목표점(x 오른쪽+, y 전방+, mm)으로 가는 Pure Pursuit 곡률 (1/m, 오른쪽 +).

    latency_sec 동안 차가 지금 명령(kappa_cmd_left, 왼쪽 +)대로 speed(m/s)로 움직였다고 보고,
    그 예측 위치·방향에서 목표점을 다시 본다. 늦게 꺾는 것(명령->바퀴 지연)을 줄인다.
    """
    x, y = float(tgt[0]), float(tgt[1])
    if latency_sec > 0.0 and speed > 0.0:
        s = speed * latency_sec * 1000.0                 # mm
        dpsi = kappa_cmd_left * s / 1000.0               # rad, 왼쪽 +
        if abs(kappa_cmd_left) < 1e-6:
            px, py = 0.0, s
        else:
            r = 1000.0 / kappa_cmd_left                  # mm, 왼쪽 +
            px, py = -r * (1.0 - math.cos(dpsi)), r * math.sin(dpsi)
        qx, qy = x - px, y - py
        c, sn = math.cos(dpsi), math.sin(dpsi)
        x, y = c * qx + sn * qy, -sn * qx + c * qy
    d2 = x * x + y * y
    if d2 < 1.0 or y <= 0.0:
        # 예측 위치에서 목표가 옆·뒤에 있으면 그쪽으로 최대한
        return math.copysign(2.1, x) if abs(x) > 1.0 else 0.0
    return 2000.0 * x / d2


def _circle_point(circ, phi: float) -> Tuple[float, float]:
    cx, cy, rc, _, _ = circ
    return cx + rc * math.cos(phi), cy + rc * math.sin(phi)


def circle_nearest(circ) -> Tuple[float, float, float]:
    """원 위에서 기준점(원점)에 가장 가까운 점 (x, y, phi)."""
    cx, cy = circ[0], circ[1]
    phi = math.atan2(-cy, -cx)
    x, y = _circle_point(circ, phi)
    return x, y, phi


def circle_sample(circ, arc_mm: float, n: int = 30) -> np.ndarray:
    """가장 가까운 점부터 진행 방향으로 arc_mm만큼의 점들 (n, 2)."""
    _, _, phi0 = circle_nearest(circ)
    rc, s = circ[2], circ[3]
    phis = phi0 + s * np.linspace(0.0, arc_mm / rc, n)
    return np.stack([circ[0] + rc * np.cos(phis), circ[1] + rc * np.sin(phis)], axis=1)


def circle_target(circ, lookahead_mm: float) -> Tuple[Tuple[float, float], bool]:
    """원 위에서 기준점으로부터 직선거리 lookahead_mm인 점 중 진행 방향 첫 점.
    교점이 없으면(차가 경로에서 멀다) 가장 가까운 점에서 호 길이 lookahead만큼 앞 점."""
    cx, cy, rc, s, _ = circ
    _, _, phi0 = circle_nearest(circ)
    D = math.hypot(cx, cy)
    if D > 1e-6:
        c = (lookahead_mm ** 2 - D * D - rc * rc) / (2.0 * rc * D)
        if -1.0 <= c <= 1.0:
            psi = math.atan2(cy, cx)
            best = None
            for phi in (psi + math.acos(c), psi - math.acos(c)):
                u = (s * (phi - phi0)) % (2.0 * math.pi)
                if best is None or u < best[0]:
                    best = (u, phi)
            if best[0] <= math.pi:
                return _circle_point(circ, best[1]), False
    return _circle_point(circ, phi0 + s * lookahead_mm / rc), True


def circle_geometry(circ) -> Tuple[float, float, float]:
    """가장 가까운 점에서 (곡률 [1/m, 오른쪽 +], 헤딩 [rad, 오른쪽 +], 횡위치 x [mm, 오른쪽 +])."""
    cx, cy, rc, s, turn = circ
    x, y, phi = circle_nearest(circ)
    ux, uy = (x - cx) / rc, (y - cy) / rc
    tx, ty = -s * uy, s * ux                          # 진행 방향 접선
    return turn * 1000.0 / rc, math.atan2(tx, ty), x



def _line_pixels_mm(mask: Optional[np.ndarray], track: Optional[np.ndarray], cfg: BevConfig,
                    max_pts: int = 600) -> Optional[np.ndarray]:
    """추적한 선이 지나는 덩어리(연결 성분)의 화소를 mm 점으로. 원 맞춤용."""
    if mask is None or track is None:
        return None
    ok = ~np.isnan(track[:, 0])
    if ok.sum() == 0:
        return None
    n, lab = cv2.connectedComponents((mask > 0).astype(np.uint8), connectivity=8)
    h, w = mask.shape[:2]
    keep = set()
    for c, r in track[ok]:
        ci, ri = int(round(c)), int(round(r))
        win = lab[max(0, ri - 3):min(h, ri + 4), max(0, ci - 3):min(w, ci + 4)]
        keep.update(int(v) for v in np.unique(win) if v > 0)
    if not keep:
        return None
    ys, xs = np.nonzero(np.isin(lab, list(keep)))
    if xs.size > max_pts:
        idx = np.linspace(0, xs.size - 1, max_pts).astype(int)
        xs, ys = xs[idx], ys[idx]
    return np.stack([cfg.col_to_x_mm(xs.astype(float)), cfg.row_to_y_mm(ys.astype(float))], axis=1)


def build_centerline(left: Optional[np.ndarray], right: Optional[np.ndarray],
                     det: Detection, cfg: BevConfig, p: LaneParams,
                     fit_order: int, use_circle: bool = True,
                     mask: Optional[np.ndarray] = None) -> Optional[Centerline]:
    """좌/우 추적 결과로 중앙선을 만든다 (윈도우 단위).

    급곡선이면 원호 모델(circle)을 함께 붙인다. 목표점·곡률은 원호가 있으면 원호로 계산한다.
    """
    exp_w = det.expected_w_px
    half_mm = p.lane_width_mm / 2.0
    rows, xs = [], []
    n = len(left) if left is not None else len(right)
    sl = _local_slope_mm(left, cfg) if left is not None else None
    sr = _local_slope_mm(right, cfg) if right is not None else None
    for i in range(n):
        okl = left is not None and not np.isnan(left[i, 0])
        okr = right is not None and not np.isnan(right[i, 0])
        if okl and okr:
            pw = right[i, 0] - left[i, 0]
            if p.pair_min_ratio * exp_w <= pw <= p.pair_max_ratio * exp_w:
                xs.append((left[i, 0] + right[i, 0]) / 2.0)
                rows.append((left[i, 1] + right[i, 1]) / 2.0)
                continue
            # 폭이 이상하면 이 행은 한쪽만 믿는다 (시작 쪽 = 상태가 정한 쪽)
            okl, okr = (det.status != 'R'), (det.status == 'R')
        if okl:
            off = half_mm * math.sqrt(1.0 + sl[i] ** 2) / cfg.mm_per_px_x
            xs.append(left[i, 0] + off)
            rows.append(left[i, 1])
        elif okr:
            off = half_mm * math.sqrt(1.0 + sr[i] ** 2) / cfg.mm_per_px_x
            xs.append(right[i, 0] - off)
            rows.append(right[i, 1])
    circ = None
    if use_circle and p.circle_fit:
        lpts, rpts = _track_mm(left, cfg), _track_mm(right, cfg)
        lpix = _line_pixels_mm(mask, left, cfg)
        rpix = _line_pixels_mm(mask, right, cfg)
        if det.status == 'BOTH' and lpix is not None and rpix is not None:
            # 행별 중점은 곡선에서 중앙선 위에 있지 않다. 왼쪽 선 각 점에서 오른쪽 선의
            # 가장 가까운 점까지의 중점으로 중앙선을 만든다.
            mids = _nearest_midpoints(lpix, rpix, p.lane_width_mm)
            if mids is not None and lpts is not None and len(lpts) >= 2:
                circ = circle_centerline(lpts + (mids[0] - lpts[0]), 'C', half_mm, p,
                                         fit_pts=mids)
        if circ is None:
            # 한쪽 선만 쓴다. 둘 다 있으면 원에 잘 맞는(잔차 작은) 선을 고른다.
            cands = []
            for side, tr, tp, pix in (('L', left, lpts, lpix), ('R', right, rpts, rpix)):
                if det.status in (side, 'BOTH') and pix is not None:
                    c = circle_centerline(tp, side, half_mm, p, fit_pts=pix)
                    if c is not None:
                        f = fit_circle(pix[:, 0], pix[:, 1])
                        cands.append((f[3] if f else 1e9, -len(pix), c))
            if cands:
                circ = min(cands, key=lambda t: (t[0], t[1]))[2]
        if circ is None and len(xs) < max(2, p.min_points) and det.status in ('L', 'R'):
            # 원도 다항식도 안 되면(선이 옆으로 누워 윈도우 점이 적다) 직선으로
            pix = lpix if det.status == 'L' else rpix
            circ = straight_centerline(pix, det.status, half_mm)
    if len(xs) < max(2, p.min_points) and circ is None:
        return None
    coeffs = None
    if len(xs) >= 2:
        x_mm = cfg.col_to_x_mm(np.asarray(xs))
        y_mm = cfg.row_to_y_mm(np.asarray(rows))
        order = np.argsort(y_mm)
        x_mm, y_mm = x_mm[order], y_mm[order]
        coeffs = _robust_polyfit(y_mm, x_mm, fit_order)
    if circ is not None:
        # 다항식은 정지선 중심·디버그용 근사. 원호 위 점으로 다시 맞춘다.
        pts = circle_sample(circ, 900.0)
        fwd = pts[:, 1] >= cfg.ref_to_bev_bottom_mm - 1.0
        sp = pts[fwd] if fwd.sum() >= 3 else pts
        c2 = _robust_polyfit(sp[:, 1], sp[:, 0], 2) if np.ptp(sp[:, 1]) > 1.0 else None
        if c2 is not None:
            coeffs = c2
        if coeffs is None:
            coeffs = np.array([float(sp[0, 0])])
        return Centerline(pts_mm=pts, coeffs=coeffs, y_near=float(sp[:, 1].min()),
                          y_far=float(sp[:, 1].max()), circle=circ)
    if coeffs is None:
        return None
    pts = np.stack([np.polyval(coeffs, y_mm), y_mm], axis=1)
    return Centerline(pts_mm=pts, coeffs=coeffs, y_near=float(y_mm[0]),
                      y_far=float(y_mm[-1]))


def pure_pursuit(cl: Centerline, lookahead_mm: float,
                 offset_right_mm: float = 0.0) -> Tuple[float, Tuple[float, float], bool]:
    """목표점 = 중앙선 다항식을 전방 lookahead_mm에서 평가한 점 (+ 옆으로 offset).

    반환: (kappa_right [1/m, 오른쪽 +], 목표점(x, y) mm, low_conf)
    기준점(원점)에서 목표점을 지나는 원의 곡률 = 2x / (x^2 + y^2).
    """
    if cl.circle is not None:
        (x_t, y_t), low_conf = circle_target(cl.circle, float(lookahead_mm))
        if offset_right_mm:
            # 진행 방향의 오른쪽 법선으로 비켜 간다
            cx, cy, rc, s = cl.circle[0], cl.circle[1], cl.circle[2], cl.circle[3]
            ux, uy = (x_t - cx) / rc, (y_t - cy) / rc
            tx, ty = -s * uy, s * ux
            x_t, y_t = x_t + offset_right_mm * ty, y_t - offset_right_mm * tx
        d2 = x_t * x_t + y_t * y_t
        if d2 < 1.0:
            return 0.0, (x_t, y_t), True
        return 2000.0 * x_t / d2, (x_t, y_t), low_conf
    y_t = float(lookahead_mm)
    low_conf = y_t > cl.y_far + 1e-6
    if low_conf:
        y_t = cl.y_far          # 중앙선이 짧으면 가장 먼 점까지만
    if y_t < cl.y_near:
        # 보이는 구간보다 가까운 곳은 다항식을 뒤로 늘리지 않는다 (곡선에서 반대로 꺾던 원인)
        y_t, low_conf = cl.y_near, True
    x_t = float(np.polyval(cl.coeffs, y_t)) + offset_right_mm
    d2 = x_t * x_t + y_t * y_t
    if d2 < 1.0:
        return 0.0, (x_t, y_t), True
    return 2000.0 * x_t / d2, (x_t, y_t), low_conf


def path_geometry(cl: Centerline, y_eval: float) -> Tuple[float, float]:
    """(곡률 [1/m, 오른쪽 +], 헤딩 [rad, 오른쪽 +]) at y_eval."""
    d1 = np.polyder(cl.coeffs, 1)
    d2 = np.polyder(cl.coeffs, 2) if len(cl.coeffs) > 2 else np.array([0.0])
    xp = float(np.polyval(d1, y_eval))
    xpp = float(np.polyval(d2, y_eval))
    kappa = 1000.0 * xpp / (1.0 + xp * xp) ** 1.5
    return kappa, math.atan(xp)


# ---------------------------------------------------------------------------
# 정지선
# ---------------------------------------------------------------------------
def detect_floor_letter(bev_white: np.ndarray, cfg: 'BevConfig', p: 'LaneParams'):
    """바닥 흰 글자 A/B. 글자 크기·내 차로 안·화면 가장자리에 안 잘린 흰 덩어리의 구멍 수: 1 = 'A', 2 = 'B'.
    반환 (글자, 앞바퀴 -> 가까운 끝 mm, 옆 위치 mm 오른쪽 +) 또는 None (가장 가까운 것 하나)."""
    img = (bev_white > 0).astype(np.uint8)
    h, w = img.shape[:2]
    cnts, hier = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return None
    hier = hier[0]
    px_area = cfg.mm_per_px_x * cfg.mm_per_px_y
    best = None
    for i, c in enumerate(cnts):
        if hier[i][3] != -1:
            continue                                   # 바깥 윤곽만
        x, y, bw, bh = cv2.boundingRect(c)
        if x <= 0 or y <= 0 or x + bw >= w or y + bh >= h:
            continue                                   # 화면 가장자리에 잘림 (글자 전체가 안 보임)
        hh, ww = bh * cfg.mm_per_px_y, bw * cfg.mm_per_px_x
        if not (p.floor_letter_min_h_mm <= hh <= p.floor_letter_max_h_mm
                and p.floor_letter_min_w_mm <= ww <= p.floor_letter_max_w_mm):
            continue
        off = cfg.col_to_x_mm(x + bw / 2.0)
        if abs(off) > p.floor_letter_max_off_mm:
            continue
        fill = float(img[y:y + bh, x:x + bw].sum()) / max(1.0, bw * bh)   # 흰 화소 비율 (구멍 빼고)
        holes = 0
        j = hier[i][2]
        while j != -1:
            if cv2.contourArea(cnts[j]) * px_area >= p.floor_letter_hole_mm2:
                holes += 1
            j = hier[j][0]
        if holes not in (1, 2) or fill > p.floor_letter_max_fill:
            continue
        near = float(cfg.row_to_y_mm(y + bh) - cfg.ref_to_front_axle_mm)
        if best is None or near < best[1]:
            best = ('A' if holes == 1 else 'B', near, float(off))
    return best


def _shear_band(band: np.ndarray, ccol: float, angle_deg: float, cfg: BevConfig) -> np.ndarray:
    """띠를 세로로 전단: angle_deg만큼 기운 가로선이 수평이 되게 (가운데 열 ccol은 그대로)."""
    if angle_deg == 0.0:
        return band
    a = math.tan(math.radians(angle_deg)) * cfg.mm_per_px_x / cfg.mm_per_px_y
    M = np.array([[1.0, 0.0, 0.0], [a, 1.0, -a * ccol]], np.float32)
    h, w = band.shape[:2]
    out = cv2.warpAffine(band.astype(np.uint8), M, (w, h), flags=cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    return out > 0


def detect_stoplines(bev_white: np.ndarray, center_col: float, cfg: BevConfig,
                     p: LaneParams, max_angle_deg: Optional[float] = None,
                     info: Optional[dict] = None) -> List[Tuple[float, bool]]:
    """차로 안쪽을 가로지르는 흰 띠를 찾는다.

    반환: [(앞바퀴 -> 띠의 가까운 가장자리 거리 mm, 바로 뒤에 횡단보도 막대가 있는지), ...]
    가까운 순. 미션 시작선과 정지선은 둘 다 흰 가로선이라, 횡단보도 막대가 뒤따르는지로
    정지선을 가린다. 카메라가 못 보는 가까운 구간(BEV 맨 아래 행보다 가까운 곳)은
    미션 쪽에서 오도메트리로 이어 받아야 한다.

    max_angle_deg (None이면 p.stopline_max_angle_deg): 0°에서 가로선 행이 하나도 없으면 띠를 ±이 각도까지
    기울여 가장 잘 맞는 각도로 다시 찾는다 (가로선은 서로 나란하므로 한 프레임에 각도 하나).
    거리는 차로 가운데 열 기준. info에 'angle_deg'(쓴 각도)를 넣는다.
    """
    h, w = bev_white.shape[:2]
    half = 0.35 * p.lane_width_mm / cfg.mm_per_px_x
    c0 = int(max(0, round(center_col - half)))
    c1 = int(min(w, round(center_col + half)))
    if c1 - c0 < 5:
        return []
    band = bev_white[:, c0:c1] > 0
    fill = band.mean(axis=1)
    max_a = p.stopline_max_angle_deg if max_angle_deg is None else max_angle_deg
    used = 0.0
    if max_a > 0.0 and not (fill >= p.stopline_fill).any() and band.any():
        step = max(0.5, float(p.stopline_angle_step_deg))
        # 각도 추정은 max_a + 10°까지 해 보고, 가장 잘 맞는 각도가 max_a 안일 때만 쓴다
        # (0°도 원래 ±10° 남짓은 잡으므로 max_a까지만 기울이면 max_a + 10°짜리 선도 잡혀 버림)
        n = int(math.floor((max_a + 10.0) / step + 1e-6))
        best = (float(fill.max()), 0.0, band, fill)
        for k in range(1, n + 1):
            for sgn in (1.0, -1.0):
                a = sgn * k * step
                b = _shear_band(band, center_col - c0, a, cfg)
                f = b.mean(axis=1)
                if float(f.max()) > best[0] + 0.02:          # 조금이라도 나아야 (같으면 작은 각도)
                    best = (float(f.max()), a, b, f)
        if best[0] >= p.stopline_fill and abs(best[1]) <= max_a + 0.5 * step:
            _, used, band, fill = best
    if info is not None:
        info['angle_deg'] = used
    # 행마다 흰 구간(막대) 개수
    edges = np.diff(np.pad(band.astype(np.int8), ((0, 0), (1, 1))), axis=1)
    segments = (edges == 1).sum(axis=1)
    on = fill >= p.stopline_fill
    look = max(3, int(round(p.crosswalk_look_mm / cfg.mm_per_px_y)))
    out = []
    r = h - 1
    while r >= 0:
        if on[r]:
            r_end = r
            while r_end - 1 >= 0 and on[r_end - 1]:
                r_end -= 1
            thick = (r - r_end + 1) * cfg.mm_per_px_y
            clipped = p.stopline_skip_edges and (r == h - 1 or r_end == 0)
            seg_ok = float(segments[r_end:r + 1].mean()) <= p.stopline_max_segments
            if p.stopline_min_mm <= thick <= p.stopline_max_mm and not clipped and seg_ok:
                y_near_edge = cfg.row_to_y_mm(r + 0.5)
                r0 = max(0, r_end - look)
                f, s = fill[r0:r_end], segments[r0:r_end]
                bar_rows = int(np.sum((f >= p.bar_fill_min) & (f <= p.bar_fill_max)
                                      & (s >= p.bar_min_segments)))
                bars = bar_rows >= max(3, p.bar_row_ratio * max(1, r_end - r0))
                out.append((float(y_near_edge - cfg.ref_to_front_axle_mm), bool(bars)))
            r = r_end - 1
        else:
            r -= 1
    return sorted(out)


# ---------------------------------------------------------------------------
# 차량 위치 적분 / 차선 기억
# ---------------------------------------------------------------------------
class PoseIntegrator:
    """odom 이동 거리 + IMU 방향으로 평면 위치를 쌓는다 (mm, rad).

    11/02 캘리브레이션: odom 거리는 정확(1.006), odom 방향은 조향 중 틀릴 수 있어 방향은 IMU.
    IMU가 없으면 odom 방향을 쓴다.
    """

    def __init__(self):
        self.x = self.y = 0.0
        self.yaw: Optional[float] = None
        self.imu_yaw: Optional[float] = None
        self._last_xy = None
        self.dist_mm = 0.0

    def on_imu_yaw(self, yaw: float) -> None:
        self.imu_yaw = float(yaw)

    def on_odom(self, x_m: float, y_m: float, odom_yaw: float, v: float = 1.0) -> None:
        yaw = self.imu_yaw if self.imu_yaw is not None else odom_yaw
        if self._last_xy is not None:
            ds = 1000.0 * math.hypot(x_m - self._last_xy[0], y_m - self._last_xy[1])
            if v < 0.0:
                ds = -ds
            self.x += ds * math.cos(yaw)
            self.y += ds * math.sin(yaw)
            self.dist_mm += abs(ds)
        self._last_xy = (x_m, y_m)
        self.yaw = yaw

    @property
    def pose(self) -> Optional[Tuple[float, float, float]]:
        if self.yaw is None:
            return None
        return self.x, self.y, self.yaw


def vehicle_to_world(pts: np.ndarray, pose) -> np.ndarray:
    """차량 좌표(x 오른쪽+, y 전방+, mm) -> 세계 좌표. pose = (X, Y, yaw 반시계+)."""
    X, Y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    # 전방 = (c, s), 오른쪽 = (s, -c)
    return np.stack([X + pts[:, 1] * c + pts[:, 0] * s,
                     Y + pts[:, 1] * s - pts[:, 0] * c], axis=1)


def world_to_vehicle(pts: np.ndarray, pose) -> np.ndarray:
    X, Y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = pts[:, 0] - X, pts[:, 1] - Y
    return np.stack([dx * s - dy * c, dx * c + dy * s], axis=1)


class LaneMemory:
    """최근 중앙선 점(세계 좌표)을 주행 거리 기준으로 보관한다."""

    def __init__(self):
        self.items = []          # [(dist_mm, pts_world)]

    def clear(self) -> None:
        self.items = []

    def add(self, pts_vehicle: np.ndarray, pose, dist_mm: float, step_mm: float = 30.0) -> None:
        if pts_vehicle is None or len(pts_vehicle) < 2:
            return
        p = pts_vehicle[np.hypot(pts_vehicle[:, 0], pts_vehicle[:, 1]) < 1300.0]
        if len(p) < 2:
            return
        # 점 간격을 일정하게 (원호 샘플·보행 점열 모두)
        seg = np.hypot(*np.diff(p, axis=0).T)
        s = np.r_[0.0, np.cumsum(seg)]
        if s[-1] < step_mm:
            return
        q = np.arange(0.0, s[-1], step_mm)
        p = np.stack([np.interp(q, s, p[:, 0]), np.interp(q, s, p[:, 1])], axis=1)
        self.items.append((dist_mm, vehicle_to_world(p, pose)))

    def prune(self, dist_mm: float, keep_mm: float) -> None:
        self.items = [it for it in self.items if dist_mm - it[0] <= keep_mm]

    def points(self, pose) -> Optional[np.ndarray]:
        if not self.items:
            return None
        return world_to_vehicle(np.vstack([it[1] for it in self.items]), pose)


def memory_target(pts: np.ndarray, lookahead_mm: float, ring_mm: float):
    """기억 점(차량 좌표) 중 기준점에서 lookahead 거리 근처, 앞쪽 점들의 중앙값.

    반환 ((x, y), low_conf) 또는 None. 고리에 점이 없으면 앞쪽 가장 먼 점(0.6L 이상)."""
    if pts is None or len(pts) == 0:
        return None
    fwd = pts[pts[:, 1] > 50.0]
    if len(fwd) == 0:
        return None
    r = np.hypot(fwd[:, 0], fwd[:, 1])
    ring = fwd[np.abs(r - lookahead_mm) <= ring_mm]
    if len(ring) >= 3:
        m = np.median(ring, axis=0)
        return (float(m[0]), float(m[1])), False
    i = int(np.argmax(r))
    if r[i] >= 0.6 * lookahead_mm:
        return (float(fwd[i, 0]), float(fwd[i, 1])), True
    return None


# ---------------------------------------------------------------------------
# 상태를 가진 추종기
# ---------------------------------------------------------------------------
class LaneFollower:
    """프레임마다 process()를 부르면 곡률 명령(LaneOutput)을 돌려준다."""

    def __init__(self, cfg: BevConfig, params: Optional[LaneParams] = None,
                 mask_fn=None):
        """mask_fn(bgr, params) -> (차선 마스크, 흰색 마스크). None이면 HSV(color_masks).

        딥러닝 분할 모델을 쓸 때는 seg_infer.SegMasker를 넘긴다. 이후 단계(BEV, 쌍 선택,
        중앙선, 정지선, Pure Pursuit)는 마스크 출처와 상관없이 똑같다.
        """
        self.cfg = cfg
        self.p = params or LaneParams()
        self.mask_fn = mask_fn
        self.mode = 'normal'
        self.fit_order_override: Optional[int] = None
        self.yaw_ref: Optional[float] = None
        self.memory = LaneMemory()
        self.arc_cand = {'L': None, 'R': None}   # 측면별 (중심 세계좌표, 반경, 거리, yaw)
        # 도면 곡률 미리보기: 앞바퀴에서 0, step, 2*step ... mm 앞의 곡률 (왼쪽 +, 1/m).
        # 노드가 매 프레임 넣는다 (오래되면 None).
        self.route_k: Optional[List[float]] = None
        self.route_corner: bool = False      # 미리보기가 M7 직전 코너(IMU)에서 온 것인가
        self.route_step_mm: float = 100.0
        self.route_pts: Optional[List[Tuple[float, float]]] = None   # 도면 중심선 (차 기준, 복귀용)
        self.reset()

    # --- 상태 ---------------------------------------------------------------
    def reset(self) -> None:
        self.mid_ema = None
        self.last_side = None
        self.last_lr = (None, None)          # 직전 프레임 좌/우 선 시작점 (BEV 열)
        self.prev_line_masks = (None, None)  # 직전 프레임 좌/우 선 화소 (BEV, 조금 부풀림)
        self.prev_tracks = [None, None]      # 11/19 흰색 이어 붙이기: 직전 좌/우 선 윈도우 중심 (BEV [col,row])
        self.prev_tracks_age = [0, 0]
        self._bev_bridge_src = None
        self.prev_target_x = None
        self.reject_count = 0
        self.lost_frames = 0
        self.kappa_cmd = 0.0
        self._letter_seen = None             # 11/19 섬 기준: (글자, 차 기준 옆 mm, 본 프레임 번호)
        self._frame_n = 0
        self.in_corner_ld = False
        self.in_corner_spd = False
        self.last_good_kappa = 0.0
        self.hold_yaw_ref = None
        self._last_offset = 0.0
        if hasattr(self, 'memory'):
            self.memory.clear()
        self.arc_cand = {'L': None, 'R': None}

    def set_mode(self, mode: str, yaw: Optional[float] = None,
                 yaw_ref: Optional[float] = None,
                 fit_order: Optional[int] = None) -> None:
        """미션이 구간에 맞춰 모드를 바꾼다. 모드가 바뀌면 시간 상태를 초기화한다."""
        if mode not in MODES:
            raise ValueError(f'unknown lane mode: {mode}')
        if mode != self.mode:
            # 코너 도중(미리보기 앞 창에 코너 곡률)일 때만. 코너 끝·합류 S자 등 다른 전환은 기존처럼 0부터
            keep = (self.p.keep_kappa_on_heading_switch and {mode, self.mode} == {'normal', 'heading'}
                    and bool(self.route_corner and self.route_k
                             and max(abs(k) for k in self.route_k) >= self.p.corner_ld_on))
            saved = (self.kappa_cmd, self.in_corner_spd, self.in_corner_ld)
            self.mode = mode
            self.reset()
            if keep:
                # 강제 원호(heading)와 차선(normal)이 코너 중에 오갈 때 조향이 0부터 다시 오르며 풀리던 것
                self.kappa_cmd, self.in_corner_spd, self.in_corner_ld = saved
        self.fit_order_override = fit_order
        if mode == 'heading':
            self.yaw_ref = yaw_ref if yaw_ref is not None else yaw

    # --- 메인 ---------------------------------------------------------------
    def process(self, bgr: np.ndarray, yaw: Optional[float] = None,
                speed: float = 0.3, pose=None, dist_mm: Optional[float] = None) -> LaneOutput:
        """pose = (X mm, Y mm, yaw rad 반시계+), dist_mm = 누적 주행 거리. 차선 기억에만 쓴다."""
        cfg, p = self.cfg, self.p
        if bgr.shape[1] != cfg.image_width or bgr.shape[0] != cfg.image_height:
            bgr = cv2.resize(bgr, (cfg.image_width, cfg.image_height),
                             interpolation=cv2.INTER_AREA)
        if self.mask_fn is None:
            lane, white = color_masks(bgr, p)
        else:
            try:
                lane, white = self.mask_fn(bgr, p)
            except TypeError:
                lane, white = self.mask_fn(bgr)
        bev_lane = cfg.warp(lane)
        bev_white = cfg.warp(white)
        self._bev_bridge_src = None
        if p.white_bridge_mm > 0.0 and p.lane_color == 'yellow' and self.mask_fn is None:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            pale = cv2.inRange(hsv, np.array([p.yellow_h_min, p.white_bridge_pale_s_min,
                                              p.white_bridge_pale_v_min], np.uint8),
                               np.array([p.yellow_h_max, 255, 255], np.uint8))
            self._bev_bridge_src = cv2.bitwise_or(bev_white, cfg.warp(pale))
        return self.process_bev(bev_lane, bev_white, yaw=yaw, speed=speed, pose=pose,
                                dist_mm=dist_mm)

    def process_bev(self, bev_lane: np.ndarray, bev_white: Optional[np.ndarray] = None,
                    yaw: Optional[float] = None, speed: float = 0.3,
                    pose=None, dist_mm: Optional[float] = None) -> LaneOutput:
        cfg, p = self.cfg, self.p
        raw_lane = bev_lane                 # 가로선 찾기용 (아래 팽창·가로 성분 제거 전)
        bridge_px = self._white_bridge(bev_lane, bev_white)
        if bridge_px:
            bev_lane = cv2.bitwise_or(bev_lane, self._bridge_add)
        look = p.lookahead_mm
        v_look = speed
        if self.in_corner_spd and p.corner_speed > 0.0:
            v_look = min(speed, p.corner_speed)     # 11/19: 커브(속도 상한 걸림)에서는 줄인 속도 기준 (명령 0.4로 750이면 우상단 코너 바깥 여유 43 -> 28 mm)
        if p.lookahead_per_mps and v_look > 0.3:
            look = float(min(p.lookahead_mm + p.lookahead_per_mps * (v_look - 0.3),
                             max(p.lookahead_max_mm, p.lookahead_mm)))
        if p.corner_lookahead_mm > look and p.lookahead_mm >= p.corner_ld_base_min and self.mode == 'normal':
            k_ahead = abs(self.kappa_cmd)
            if self.route_k:
                k_r = self._route_kappa(p.route_ff_preview_mm + speed * p.latency_comp_sec * 1000.0)
                if k_r is not None:
                    k_ahead = max(k_ahead, abs(k_r))
            self.in_corner_ld = k_ahead > (p.corner_ld_off if self.in_corner_ld else p.corner_ld_on)
            if self.in_corner_ld:
                look = float(p.corner_lookahead_mm)
        else:
            self.in_corner_ld = False
        self._update_corner_flag()                # 11/19: 커브 속도 상한·커브 조향 배율용 (이전 프레임 조향 + 도면 앞 곡률)
        if p.dilate_px > 0:
            k = 2 * int(p.dilate_px) + 1
            bev_lane = cv2.dilate(bev_lane, np.ones((k, k), np.uint8))
        if p.suppress_transverse:
            bev_lane = remove_transverse(bev_lane, p.max_run_mm / cfg.mm_per_px_x)

        out = LaneOutput()
        out.debug['bev'] = bev_lane
        self._frame_n = getattr(self, '_frame_n', 0) + 1
        hint = None
        ls = getattr(self, '_letter_seen', None)
        if (p.island_ref and self.mode in ('left_only', 'right_only') and ls is not None
                and self._frame_n - ls[2] <= p.island_letter_frames
                and ls[0] == ('B' if self.mode == 'left_only' else 'A')):
            hint = cfg.vehicle_col + ls[1] / cfg.mm_per_px_x
        det = detect_lanes(bev_lane, cfg, p, self.mode, self.mid_ema, self.last_side,
                           self.last_lr, hint_center=hint)
        n_lab, lab_full = cv2.connectedComponents((bev_lane > 0).astype(np.uint8), connectivity=8)
        if det.status in ('L', 'R') and self.mode not in ('left_only', 'right_only'):
            self._keep_side_by_overlap(det, lab_full)
        inner = self.corner_inner_side()
        out.debug['inner'] = inner
        if inner is not None and det.status == 'BOTH':
            if inner == 'R':
                det.left_x0, det.status = None, 'R'
            else:
                det.right_x0, det.status = None, 'L'
        out.status = det.status

        left = track_line(bev_lane, det.left_x0, cfg, p) if det.left_x0 is not None else None
        right = track_line(bev_lane, det.right_x0, cfg, p) if det.right_x0 is not None else None
        out.debug['left'], out.debug['right'] = left, right
        out.debug['white_bridge'] = bridge_px or None
        for i, tr in enumerate((left, right)):
            if tr is not None and int((~np.isnan(tr[:, 0])).sum()) >= 2:
                self.prev_tracks[i], self.prev_tracks_age[i] = tr, 0
            else:
                self.prev_tracks_age[i] += 1
                if self.prev_tracks_age[i] > p.white_bridge_keep_frames:
                    self.prev_tracks[i] = None
        if p.line_types:
            out.line_left = classify_line(left, raw_lane, bev_white, cfg, p)
            out.line_right = classify_line(right, raw_lane, bev_white, cfg, p)
            if det.status == 'BOTH':
                out.lane_id = lane_id_from_types(out.line_left, out.line_right)
            elif p.lane_color == 'both':
                # 11/14: 3차선(M7)에서 한쪽 선만 보여도 그것이 노란 실선이면 바깥 차로(왼쪽=1차선, 오른쪽=3차선)
                if (det.status == 'L' and out.line_left
                        and out.line_left.get('color') == 'yellow' and not out.line_left.get('dashed', False)):
                    out.lane_id = 1
                elif (det.status == 'R' and out.line_right
                      and out.line_right.get('color') == 'yellow' and not out.line_right.get('dashed', False)):
                    out.lane_id = -1
                else:
                    out.lane_id = None
            else:
                out.lane_id = None
        self.prev_line_masks = (self._line_mask(lab_full, left), self._line_mask(lab_full, right))

        fit_order = self.fit_order_override or (1 if self.mode == 'zigzag' else p.fit_order)
        cl = None
        if det.status != 'NONE':
            cl = build_centerline(left, right, det, cfg, p, fit_order,
                                  use_circle=(self.mode != 'zigzag'), mask=bev_lane)
        out.debug['centerline'] = cl

        # 연속성 상태 갱신
        if det.status == 'BOTH':
            mid = (det.left_x0 + det.right_x0) / 2.0
            self.mid_ema = mid if self.mid_ema is None else 0.3 * mid + 0.7 * self.mid_ema
            self.last_side = None
            self.last_lr = (det.left_x0, det.right_x0)
        elif det.status in ('L', 'R'):
            self.last_side = det.status
            self.last_lr = (det.left_x0, None) if det.status == 'L' else (None, det.right_x0)
        else:
            self.mid_ema, self.last_side = None, None
            self.last_lr = (None, None)

        kappa_raw, source = None, 'lane'
        direct = None
        if det.status in ('L', 'R') and p.single_lookahead_mm > look:
            look = float(p.single_lookahead_mm)
        lpix_mm = _line_pixels_mm(bev_lane, left, cfg)
        rpix_mm = _line_pixels_mm(bev_lane, right, cfg)
        arc_on = (p.lost_arc_mm > 0.0 and pose is not None and dist_mm is not None
                  and self.mode not in ('zigzag', 'heading', 'left_only', 'right_only'))
        if arc_on and det.status == 'BOTH':
            for side, pix in (('L', lpix_mm), ('R', rpix_mm)):
                self.arc_cand[side] = self._inner_arc(side, pix, pose, dist_mm)
        if (p.direct_target and det.status != 'NONE' and p.target_offset_mm == 0.0
                and self.mode not in ('zigzag', 'heading')):
            direct = direct_target(lpix_mm, rpix_mm, det.status,
                                   p.lane_width_mm / 2.0, look, p.lane_width_mm,
                                   p.corner_outer_extra_mm, p.walk_smooth_pts,
                                   p.walk_kink_deg, p.far_start_mm, p.pair_agree_ratio)
            out.debug['direct'] = direct[2] if direct is not None else None
            if arc_on and det.status in ('L', 'R'):
                at = self._lost_arc_target(det.status, pose, dist_mm, look)
                if at is not None:
                    out.debug['arc_tgt'] = at
                    if direct is not None:
                        t0 = direct[0]
                        if math.hypot(t0[0] - at[0], t0[1] - at[1]) < p.lane_width_mm:
                            # 두 선이 다 보일 때처럼 두 목표점의 가운데
                            direct = ((0.5 * (t0[0] + at[0]), 0.5 * (t0[1] + at[1])), False,
                                      direct[2])
                    else:
                        direct = (at, False, np.array([[0.0, cfg.ref_to_bev_bottom_mm], at]))
        if cl is None and direct is not None:
            # 원·다항식이 안 돼도 목표점은 있다 (코너 출구처럼 선이 옆으로 누운 경우)
            c = direct[2]
            cl = Centerline(pts_mm=c, coeffs=np.array([float(c[0, 0])]), y_near=float(c[0, 1]),
                            y_far=float(c[:, 1].max()))
        if cl is not None:
            if cl.circle is not None:
                k_path, psi, x_near = circle_geometry(cl.circle)
            else:
                k_path, psi = path_geometry(cl, cl.y_near)
                x_near = float(np.polyval(cl.coeffs, cl.y_near))
            out.path_kappa = -k_path
            out.heading_rad = -psi
            out.offset_mm = -x_near
            if p.target_offset_mm != self._last_offset:
                self.prev_target_x = None      # 의도한 비켜 가기는 점프 게이트에 안 걸리게
                self._last_offset = p.target_offset_mm
            if direct is not None:
                tgt, low = direct[0], direct[1]
                k_r = pp_kappa_with_latency(tgt, self.kappa_cmd, speed, p.latency_comp_sec)
            else:
                k_r, tgt, low = pure_pursuit(cl, look, -p.target_offset_mm)
                k_r = pp_kappa_with_latency(tgt, self.kappa_cmd, speed, p.latency_comp_sec)
            out.target_mm, out.low_conf = tgt, low
            # 중앙 점프 게이트: 목표점 횡위치가 갑자기 튀면 그 프레임은 버린다
            if (self.prev_target_x is not None
                    and abs(tgt[0] - self.prev_target_x) > p.jump_max_mm
                    and self.reject_count < p.jump_max_frames):
                self.reject_count += 1
                source = 'coast'
            else:
                self.reject_count = 0
                self.prev_target_x = tgt[0]
                kappa_raw = -k_r
                self.lost_frames = 0
                self.last_good_kappa = kappa_raw
                if yaw is not None:
                    # 차로 방향(월드 yaw)을 기억해 두었다가 놓치면 그 방향을 붙잡는다
                    self.hold_yaw_ref = wrap_angle(yaw + out.heading_rad)

        # 차선 기억: 좋은 프레임의 중앙선을 쌓고, 놓쳤거나 짧을 때 꺼내 쓴다
        use_mem = p.memory_mm > 0.0 and pose is not None and dist_mm is not None
        if use_mem:
            if kappa_raw is not None and source == 'lane' and cl is not None:
                self.memory.add(direct[2] if direct is not None else cl.pts_mm, pose, dist_mm)
            self.memory.prune(dist_mm, p.memory_mm)
            need = (self.mode not in ('heading', 'zigzag')
                    and (kappa_raw is None and source != 'coast'
                         or (p.memory_on_low_conf and source == 'lane' and out.low_conf)))
            if need:
                mt = memory_target(self.memory.points(pose), look, p.memory_ring_mm)
                if mt is not None and (kappa_raw is None or not mt[1]):
                    tgt, low = mt
                    k_r = pp_kappa_with_latency(tgt, self.kappa_cmd, speed, p.latency_comp_sec)
                    kappa_raw, source = -k_r, 'memory'
                    out.target_mm, out.low_conf = tgt, low
                    self.prev_target_x = tgt[0]
                    if yaw is not None and self.lost_frames == 0 and out.heading_rad is not None:
                        self.hold_yaw_ref = wrap_angle(yaw + out.heading_rad)
            out.mem_pts = int(sum(len(it[1]) for it in self.memory.items))
        out.lookahead_mm = look
        if (p.route_recover and self.route_pts and kappa_raw is None and source != 'coast'
                and self.mode not in ('heading', 'zigzag')):
            mt = memory_target(np.asarray(self.route_pts, float), look, 80.0)
            if mt is not None:
                tgt = mt[0]
                k_r = pp_kappa_with_latency(tgt, self.kappa_cmd, speed, p.latency_comp_sec)
                kappa_raw, source = -k_r, 'route'
                out.target_mm, out.low_conf = tgt, True
                self.prev_target_x = tgt[0]

        if self.mode == 'heading' and yaw is not None and self.yaw_ref is not None:
            kappa_raw, source = self._heading_kappa(yaw, self.yaw_ref, speed), 'heading'
        elif source == 'coast':
            kappa_raw = self.kappa_cmd
        elif ((kappa_raw is None or (out.status in ('L', 'R') and source == 'lane'))
              and self._realign(raw_lane, yaw, speed, out) is not None):
            # 놓쳤거나, 한쪽 선만 보이는데 크게 기운 노란 선이 있으면 그 선은 '가로지르는 차선' (따라가면 뚫고 나감)
            kappa_raw, source = self._realign_k, 'realign'
            out.speed_scale = p.realign_speed
        elif kappa_raw is None:
            # 차선을 놓침
            self.lost_frames += 1
            k_corner = (self._route_kappa(p.route_ff_preview_mm + speed * p.latency_comp_sec * 1000.0)
                        if self.route_corner and self.route_k else None)
            if k_corner is not None and p.lost_corner_window and abs(k_corner) < p.corner_ld_on:
                # 11/18 1118_03 M2 출구 코너: 도면 위치 s가 약 1 m 뒤처져 '아직 코너 전'이라 선을 놓친 1.5초 동안 직진 -> 트랙 밖 -> 정지.
                #   코너 미리보기가 켜져 있는 동안(미션 노드가 IMU로 '아직 코너 중')은 앞 창(route_k, 약 1.4 m) 안의 코너 곡률을 씀
                kmax = max(self.route_k, key=abs)
                if abs(kmax) >= p.corner_ld_on:
                    k_corner = float(kmax)
            if k_corner is not None and self.lost_frames <= p.lost_corner_frames:
                # 11/12 밤: M7 직전 코너(미션 노드가 IMU로 '아직 코너 중'이라고 보냄)에서 선을 놓치면
                #   방향 유지(직진) 대신 코너 곡률로 계속 돈다
                source, kappa_raw = 'route', k_corner
            elif self.lost_frames > p.lost_hold_frames:
                sk = self._lost_search(yaw, dist_mm, speed) if p.lost_search else None
                if sk is not None:
                    source, kappa_raw = 'search', sk
                    out.speed_scale = p.lost_search_speed
                else:
                    source, kappa_raw = 'stop', 0.0
                    out.speed_scale = 0.0
            elif yaw is not None and (self.hold_yaw_ref is not None or abs(p.hold_axis_deg) <= 360.0):
                source = 'hold'
                ref = math.radians(p.hold_axis_deg) if abs(p.hold_axis_deg) <= 360.0 else self.hold_yaw_ref
                kappa_raw = self._heading_kappa(yaw, ref, speed)
            else:
                source = 'hold'
                kappa_raw = self.last_good_kappa * (p.lost_decay ** self.lost_frames)
        # 도면 곡률 미리 넣기
        k_ff = self._route_kappa(p.route_ff_preview_mm + speed * p.latency_comp_sec * 1000.0)
        corner_locked = False
        if k_ff is not None and self.route_corner and source in ('lane', 'memory') and (abs(k_ff) > 1e-6 or not p.route_corner_zero_lane):
            # 11/12 밤 실차: M7 직전 코너(미션 노드가 IMU로 '아직 코너 중')에서 차선 노드가 선을 '보고' 있는데도 직진했다.
            #   이 코너에서는 선이 잘 보여도 코너 곡률을 route_corner_weight 만큼 섞고, 도면과 다르다고 버리지 않는다
            out.ff_kappa = k_ff
            k_lane = kappa_raw
            kappa_raw = kappa_raw + p.route_corner_weight * (k_ff - kappa_raw)
            # 11/12 밤: 코너에서 바깥으로 밀려 선을 밟고 놓침 -> 차선이 코너 방향으로 더 크게 꺾으라고 하면 그 값을 그대로 (적어도 코너 곡률, 필요하면 더)
            if k_lane * k_ff > 0 and abs(k_lane) > abs(kappa_raw):
                kappa_raw = k_lane
            if p.corner_lock and abs(k_ff) >= p.corner_ld_on:
                # corner_fix: L/R 헷갈림으로 코너 중 조향이 풀리거나 뒤집히지 않게 '덜 꺾기 하한'만. 아래 코너 배율도 안 곱함
                #   (선 보일 때 1.2배·놓쳤을 때(route) 1배로 조향이 출렁이던 것)
                kappa_raw = corner_lock_kappa(kappa_raw, k_ff, p.corner_lock_corr_max)
                corner_locked = True
        elif k_ff is not None and (p.route_ff_weight > 0.0 or p.route_ff_weight_lost > 0.0):
            out.ff_kappa = k_ff
            w = 0.0
            strong = out.status == 'BOTH' and not out.low_conf
            if source in ('lane', 'memory'):
                # 선이 약할 때(한쪽만·짧음·기억)만 도면을 섞는다
                w = p.route_ff_weight_both if strong else p.route_ff_weight
                if strong and abs(k_ff - kappa_raw) > p.route_ff_gate:
                    w = 0.0                   # 차선이 확실한데 도면과 크게 다르면 위치 추정이 틀린 것
            elif source in ('hold', 'route'):
                w = p.route_ff_weight_lost
            kappa_raw = kappa_raw + w * (k_ff - kappa_raw)
        out.source = source

        gain = p.corner_kappa_gain
        if self.in_corner_spd and p.curve_kappa_gain > 0.0:
            gain = p.curve_kappa_gain                              # 11/19: 커브(일반 주행·코너 미리보기)에서만 더 크게. 지그재그 등은 그대로
        if (gain != 1.0 and not corner_locked and source in ('lane', 'memory') and kappa_raw is not None
                and abs(kappa_raw) > p.corner_gain_from):
            kappa_raw = kappa_raw * gain                           # 커브: 조금 더 크게 (위 corner_kappa_gain 설명)

        # 11/14: 노란 실선 이탈 방지 강제 조향 (M7 3차선 바깥 노란 실선 및 일반 구간 노란 실선 경계 보호)
        hard_guard = False
        if (p.yellow_solid_guard and source not in ('stop', 'heading') and self.mode == 'normal'
                and (p.lane_color == 'both' or not p.yellow_guard_three_lane_only)):
            l_yellow_solid = (out.line_left is not None
                              and out.line_left.get('color') == 'yellow'
                              and not out.line_left.get('dashed', False))
            r_yellow_solid = (out.line_right is not None
                              and out.line_right.get('color') == 'yellow'
                              and not out.line_right.get('dashed', False))
            if p.yellow_guard_one_side_only and l_yellow_solid and r_yellow_solid:
                l_yellow_solid = r_yellow_solid = False          # 일반 1차로 (양쪽 노란 실선): 가운데 맞추기에 맡김
            y_lim = min(look, p.yellow_guard_look_mm)
            dist_l, dist_r = None, None
            if l_yellow_solid and lpix_mm is not None and len(lpix_mm) > 0:
                near_l = lpix_mm[lpix_mm[:, 1] <= y_lim]
                pts_l = near_l if len(near_l) >= 3 else lpix_mm
                dist_l = -float(np.percentile(pts_l[:, 0], 85))
                out.yellow_l_mm = dist_l
                out.yellow_f_mm = float(np.median(pts_l[:, 1]))
            if r_yellow_solid and rpix_mm is not None and len(rpix_mm) > 0:
                near_r = rpix_mm[rpix_mm[:, 1] <= y_lim]
                pts_r = near_r if len(near_r) >= 3 else rpix_mm
                dist_r = float(np.percentile(pts_r[:, 0], 15))
                out.yellow_r_mm = dist_r
                if out.yellow_f_mm is None:
                    out.yellow_f_mm = float(np.median(pts_r[:, 1]))
            span = max(p.yellow_guard_margin_mm - p.yellow_guard_hard_mm, 1.0)
            if dist_l is not None and dist_l < p.yellow_guard_margin_mm and (dist_r is None or dist_l <= dist_r):
                out.yellow_guard = 'L'
                urg = max(p.yellow_guard_min_urg, min(1.0, (p.yellow_guard_margin_mm - dist_l) / span))
                k_guard = -min(p.kappa_max, urg * p.kappa_max * p.yellow_guard_gain)
                kappa_raw = min(kappa_raw, k_guard)
                hard_guard = dist_l <= p.yellow_guard_hard_mm
            elif dist_r is not None and dist_r < p.yellow_guard_margin_mm:
                out.yellow_guard = 'R'
                urg = max(p.yellow_guard_min_urg, min(1.0, (p.yellow_guard_margin_mm - dist_r) / span))
                k_guard = min(p.kappa_max, urg * p.kappa_max * p.yellow_guard_gain)
                kappa_raw = max(kappa_raw, k_guard)
                hard_guard = dist_r <= p.yellow_guard_hard_mm

        # 곡률 명령 필터: EMA -> 변화율 제한 -> 클램프
        if hard_guard:
            k_f = kappa_raw
        else:
            k_f = p.kappa_alpha * kappa_raw + (1.0 - p.kappa_alpha) * self.kappa_cmd
            k_f = self.kappa_cmd + float(np.clip(k_f - self.kappa_cmd,
                                                 -p.kappa_rate_max, p.kappa_rate_max))
        self.kappa_cmd = float(np.clip(k_f, -p.kappa_max, p.kappa_max))
        out.kappa = self.kappa_cmd
        if source != 'stop':
            out.speed_scale = 1.0 - p.curve_slowdown * min(1.0, abs(out.kappa) / p.kappa_max)
            if p.preview_slowdown > 0.0:
                # 꺾기 전에 감속: 앞에 보이는 원호 곡률, 도면상 1 m 안 최대 곡률
                kp = 0.0
                if cl is not None and cl.circle is not None:
                    kp = 1000.0 / max(cl.circle[2], 1.0)
                if self.route_k:
                    n = int(1000.0 / self.route_step_mm) + 1
                    kp = max(kp, max(abs(k) for k in self.route_k[:n]))
                out.speed_scale = min(out.speed_scale, 1.0 - p.preview_slowdown
                                      * min(1.0, kp / max(p.preview_kappa_ref, 1e-3)))
            if p.corner_speed > 0.0 and speed > p.corner_speed and self.in_corner_spd:
                out.speed_scale = min(out.speed_scale, p.corner_speed / speed)
            if p.lost_speed > 0.0 and speed > p.lost_speed and source in ('hold', 'route', 'search'):
                out.speed_scale = min(out.speed_scale, p.lost_speed / speed)   # 11/19: 선 없이 빨리 달리지 않게

        # 정지선 (흰색 마스크 기준, 차로 안쪽만)
        if bev_white is not None:
            center_col = cfg.vehicle_col
            if cl is not None:
                center_col = cfg.vehicle_col + float(np.polyval(cl.coeffs, cl.y_near)) / cfg.mm_per_px_x
            sl_info = {}
            lines = detect_stoplines(bev_white, center_col, cfg, p, info=sl_info)
            out.stoplines_mm = [d for d, _ in lines]
            out.debug['stopline_angle_deg'] = sl_info.get('angle_deg', 0.0) if lines else None
            out.crosswalk_lines_mm = [d for d, bars in lines if bars]
            if p.floor_letter:
                fl = detect_floor_letter(bev_white, cfg, p)
                if fl is not None:
                    out.floor_letter, out.floor_letter_mm, out.floor_letter_off_mm = fl
                    self._letter_seen = (fl[0], float(fl[2]), self._frame_n)
            # 주차 칸 끝 선: 흰+노란 마스크, 차 정면 기준 (칸 안에서는 차선 중심 추정이 엉뚱할 수 있음)
            both = bev_white if raw_lane is None else cv2.bitwise_or(bev_white, raw_lane)
            out.end_lines_mm = [d for d, _ in detect_stoplines(both, cfg.vehicle_col, cfg, p, max_angle_deg=0.0)]
        return out

    def _update_corner_flag(self) -> bool:
        """11/19: 지금 커브인가 (corner_speed 속도 상한·curve_kappa_gain). 히스테리시스"""
        p = self.p
        if self.mode != 'normal' or not (p.lookahead_mm >= p.corner_ld_base_min or self.route_corner):
            self.in_corner_spd = False
            return False
        k_ahead = abs(self.kappa_cmd)
        if self.route_k:
            n = max(1, int(p.corner_speed_look_mm / max(self.route_step_mm, 1.0)) + 1)
            k_ahead = max(k_ahead, max(abs(k) for k in self.route_k[:n]))
        self.in_corner_spd = k_ahead > (p.corner_ld_off if self.in_corner_spd else p.corner_ld_on)
        return self.in_corner_spd

    # --- 선 동일성 (프레임 간) --------------------------------------------------
    @staticmethod
    def _labels_near(lab: np.ndarray, pts) -> set:
        h, w = lab.shape[:2]
        out = set()
        for c, r in pts:
            ci, ri = int(round(c)), int(round(r))
            win = lab[max(0, ri - 3):min(h, ri + 4), max(0, ci - 3):min(w, ci + 4)]
            out.update(int(v) for v in np.unique(win) if v > 0)
        return out

    def _line_mask(self, lab: np.ndarray, track: Optional[np.ndarray]) -> Optional[np.ndarray]:
        """추적한 선이 지나는 덩어리 화소. 다음 프레임에서 같은 선인지 가리는 데 쓴다."""
        if track is None:
            return None
        ok = ~np.isnan(track[:, 0])
        if ok.sum() == 0:
            return None
        labels = self._labels_near(lab, track[ok])
        if not labels:
            return None
        m = np.isin(lab, list(labels)).astype(np.uint8)
        return cv2.dilate(m, np.ones((19, 19), np.uint8)) > 0   # 한 프레임 이동·회전 여유

    def _white_bridge(self, bev_lane: np.ndarray, bev_white: Optional[np.ndarray]) -> int:
        """11/19: 직전에 따라가던 선 둘레의 흰색·옅은 노랑 화소 (white_bridge_mm 설명). 붙일 화소 수를 돌려주고
        self._bridge_add에 그 마스크를 둔다."""
        p, cfg = self.p, self.cfg
        self._bridge_add = None
        src = self._bev_bridge_src if self._bev_bridge_src is not None else bev_white
        if (p.white_bridge_mm <= 0.0 or p.lane_color != 'yellow' or self.mode != 'normal'
                or src is None or src.shape != bev_lane.shape):
            return 0
        corr = np.zeros_like(bev_lane)
        th = max(1, int(round(2.0 * p.white_bridge_mm / cfg.mm_per_px_x)))
        for tr in self.prev_tracks:
            if tr is None:
                continue
            pts = tr[~np.isnan(tr[:, 0])]
            if len(pts) < 2:
                continue
            pts = pts[np.argsort(-pts[:, 1])]            # 아래 -> 위
            ext = np.vstack([[pts[0, 0], bev_lane.shape[0] - 1], pts])   # BEV 아래 끝까지 이어 그림
            cv2.polylines(corr, [np.round(ext).astype(np.int32).reshape(-1, 1, 2)], False, 255, th)
        if not corr.any():
            return 0
        add = cv2.bitwise_and(src, corr)
        add = cv2.bitwise_and(add, cv2.bitwise_not(bev_lane))
        add = remove_transverse(add, p.white_bridge_max_w_mm / cfg.mm_per_px_x)
        n = int(cv2.countNonZero(add))
        if n == 0:
            return 0
        self._bridge_add = add
        return n

    def _keep_side_by_overlap(self, det: 'Detection', lab: np.ndarray) -> None:
        """한쪽 선만 보일 때, 직전 프레임의 좌/우 선 화소와 더 많이 겹치는 쪽으로 판정한다.

        급곡선에서 바깥 선은 차 앞을 가로질러 반대편으로 넘어가고, 두꺼운 선은 시작점이
        크게 튈 수 있어서 시작점 거리만으로는 같은 선인지 모른다.
        """
        pl, pr = self.prev_line_masks
        if pl is None and pr is None:
            return
        x0 = det.left_x0 if det.status == 'L' else det.right_x0
        h = lab.shape[0]
        band = int(min(h, max(10, round(self.p.hist_band_mm / self.cfg.mm_per_px_y))))
        rows = np.arange(h - 1, h - band - 1, -3)
        labels = self._labels_near(lab, [(x0, r) for r in rows])
        if not labels:
            return
        comp = np.isin(lab, list(labels))
        ol = int((comp & pl).sum()) if pl is not None else 0
        orr = int((comp & pr).sum()) if pr is not None else 0
        if max(ol, orr) < 30 or ol == orr:
            return
        side = 'L' if ol > orr else 'R'
        if side != det.status:
            det.left_x0, det.right_x0 = (x0, None) if side == 'L' else (None, x0)
            det.status = side

    def corner_inner_side(self) -> Optional[str]:
        """미리 아는 코너 안(또는 바로 앞)이면 안쪽 선 쪽 'L'/'R', 아니면 None (corner_inner_line 설명)"""
        p = self.p
        if not (p.corner_inner_line and self.route_corner and self.route_k and self.mode == 'normal'):
            return None
        n = int(p.corner_inner_ahead_mm / max(self.route_step_mm, 1.0)) + 1
        for k in self.route_k[:n]:
            if abs(k) > 0.1:
                return 'L' if k > 0 else 'R'
        return None

    def _inner_arc(self, side: str, pix, pose, dist_mm: float):
        """이 선이 자기 쪽으로 휘는 원이면 (중심 세계좌표, 반경, 거리, yaw). 아니면 None."""
        if pix is None or len(pix) < 40:
            return None
        if np.ptp(pix[:, 1]) < 150.0:
            return None
        f = fit_circle(pix[:, 0], pix[:, 1])
        if f is None:
            return None
        cx, cy, r, rms = f
        if not (350.0 <= r <= 1500.0) or rms > 25.0:
            return None
        mx = float(np.mean(pix[:, 0]))
        if (side == 'R' and cx <= mx) or (side == 'L' and cx >= mx):
            return None                        # 반대쪽으로 휘면 안쪽 선이 아니다
        cw = vehicle_to_world(np.array([[cx, cy]]), pose)[0]
        return (cw, float(r), float(dist_mm), float(pose[2]))

    def _lost_arc_target(self, seen: str, pose, dist_mm: float, look: float):
        """안 보이는 쪽(seen 반대)의 기억한 원을 반폭만큼 바깥으로 넓힌 중앙선 원 위, lookahead 거리 점."""
        p = self.p
        lost = 'R' if seen == 'L' else 'L'
        cand = self.arc_cand.get(lost)
        if cand is None:
            return None
        cw, r, d0, yaw0 = cand
        if dist_mm - d0 > p.lost_arc_mm or \
                abs(wrap_angle(pose[2] - yaw0)) > math.radians(p.lost_arc_max_turn_deg):
            self.arc_cand[lost] = None
            return None
        c = world_to_vehicle(np.array([cw]), pose)[0]
        rc = r + p.lane_width_mm / 2.0
        d = float(math.hypot(c[0], c[1]))
        if d < 1e-3 or not (abs(rc - look) <= d <= rc + look):
            return None
        a = (look * look - rc * rc + d * d) / (2.0 * d)
        h = math.sqrt(max(look * look - a * a, 0.0))
        base = c * (a / d)
        perp = np.array([-c[1], c[0]]) / d
        sgn = -1.0 if lost == 'R' else 1.0     # 오른쪽 원: 시계 방향으로 진행
        phi0 = math.atan2(-c[1], -c[0])        # 차에서 가장 가까운 원 위 점의 각
        best = None
        for q in (base + h * perp, base - h * perp):
            if q[1] <= 0.0:
                continue
            th = math.atan2(q[1] - c[1], q[0] - c[0])
            dth = (sgn * (th - phi0)) % (2.0 * math.pi)
            if best is None or dth < best[0]:
                best = (dth, (float(q[0]), float(q[1])))
        return best[1] if best is not None else None

    def _route_kappa(self, d_mm: float, half_win_mm: float = 150.0) -> Optional[float]:
        """앞바퀴에서 d_mm 앞 도면 곡률 (앞뒤 half_win 평균 -> 코너 시작에서 계단 대신 경사)."""
        if not self.route_k:
            return None
        ks = np.asarray(self.route_k, float)
        ds = np.arange(len(ks)) * self.route_step_mm
        q = np.linspace(d_mm - half_win_mm, d_mm + half_win_mm, 7)
        return float(np.mean(np.interp(q, ds, ks)))

    def _realign(self, raw_lane, yaw, speed, out) -> Optional[float]:
        """선을 놓쳤을 때 크게 기운 노란 선이 보이면 그 선과 나란해지는 곡률 (realign_deg 설명). 없으면 None."""
        p, cfg = self.p, self.cfg
        if p.realign_deg <= 0.0 or p.lane_color != 'yellow' or self.mode != 'normal' or raw_lane is None:
            return None
        m = (raw_lane > 0).astype(np.uint8)
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        best = None
        for i in range(1, n):
            ys, xs = np.nonzero(lab == i)
            if len(xs) < 20:
                continue
            xmm = (xs - cfg.vehicle_col) * cfg.mm_per_px_x                  # 오른쪽 +
            ymm = cfg.row_to_y_mm(ys)                                       # 앞 +
            if math.hypot(np.ptp(xmm), np.ptp(ymm)) < p.realign_min_mm:
                continue
            pts = np.stack([xmm, ymm], 1).astype(np.float32)
            vx, vy, _, _ = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            if vy < 0:
                vx, vy = -vx, -vy
            a = math.atan2(float(vx), float(vy))                            # 선 방향, 오른쪽 + (차 진행 방향 0)
            d = float(np.min(np.hypot(xmm, ymm)))
            if abs(a) >= math.radians(p.realign_deg) and (best is None or d < best[1]):
                best = (a, d)
        if best is None:
            return None
        a = best[0]
        out.debug['realign_deg'] = round(math.degrees(a), 1)
        if yaw is not None:
            k = self._heading_kappa(yaw, yaw - a, speed)                    # 오른쪽으로 a 돌면 선과 나란함
        else:
            k = -math.copysign(min(p.kappa_max, abs(a) * 2.0), a)
        self._realign_k = float(max(-p.kappa_max, min(p.kappa_max, k)))
        self.lost_frames = 0
        return self._realign_k

    def _lost_search(self, yaw, dist_mm, speed) -> Optional[float]:
        """선을 오래 놓쳤을 때 찾기 조향 (lost_search 설명). 다 찾아봤으면 None (-> 정지)."""
        p = self.p
        if yaw is None:
            return None
        sr = getattr(self, 'search', None)
        if sr is None or sr['frame'] != self.lost_frames - 1:
            ref = (math.radians(p.hold_axis_deg) if abs(p.hold_axis_deg) <= 360.0
                   else self.hold_yaw_ref if self.hold_yaw_ref is not None else yaw)
            if abs(self.last_good_kappa) > 0.15:
                first = -1.0 if self.last_good_kappa > 0 else 1.0      # 직전 조향(왼쪽 +)의 반대쪽
            else:
                first = 1.0 if self.mode == 'left_only' else -1.0       # 따라가던 선 쪽
            sr = self.search = {'ref': ref, 'first': first, 'd0': dist_mm, 'n': 0, 'frame': self.lost_frames}
        sr['frame'] = self.lost_frames
        sr['n'] += 1
        if dist_mm is not None and sr['d0'] is not None:
            d = dist_mm - sr['d0']
        else:
            d = sr['n'] * max(abs(speed), 0.05) * p.lost_search_speed * 70.0   # 프레임당 약 70 ms
        if d < p.lost_search_mm:
            side = sr['first']
        elif d < 3.0 * p.lost_search_mm:
            side = -sr['first']
        else:
            return None
        return self._heading_kappa(yaw, sr['ref'] + side * math.radians(p.lost_search_deg), speed)

    def _heading_kappa(self, yaw: float, yaw_ref: float, speed: float) -> float:
        """yaw_ref를 향하도록 하는 곡률 (왼쪽 +). omega = -kp*err, kappa = omega/v."""
        err = wrap_angle(yaw - yaw_ref)
        omega = -self.p.yaw_kp * err
        return omega / max(abs(speed), 0.05)


# ---------------------------------------------------------------------------
# 디버그 그림
# ---------------------------------------------------------------------------
def draw_debug(out: LaneOutput, cfg: BevConfig) -> np.ndarray:
    """BEV 위에 추적 결과를 그린 BGR 이미지."""
    bev = out.debug.get('bev')
    if bev is None:
        bev = np.zeros((cfg.bev_h, cfg.bev_w), np.uint8)
    img = cv2.cvtColor((bev > 0).astype(np.uint8) * 90, cv2.COLOR_GRAY2BGR)
    for key, color in (('left', (255, 120, 0)), ('right', (0, 120, 255))):
        tr = out.debug.get(key)
        if tr is None:
            continue
        for c, r in tr:
            if not np.isnan(c):
                cv2.circle(img, (int(c), int(r)), 4, color, -1)
    cl = out.debug.get('centerline')
    if cl is not None:
        if cl.circle is not None:
            pts = [(int(cfg.vehicle_col + x / cfg.mm_per_px_x), int(cfg.y_mm_to_row(y)))
                   for x, y in circle_sample(cl.circle, 1500.0, 40)]
            cv2.polylines(img, [np.array(pts, np.int32)], False, (0, 220, 220), 2)
        else:
            ys = np.linspace(cl.y_near, cl.y_far, 30)
            pts = [(int(cfg.vehicle_col + np.polyval(cl.coeffs, y) / cfg.mm_per_px_x),
                    int(cfg.y_mm_to_row(y))) for y in ys]
            cv2.polylines(img, [np.array(pts, np.int32)], False, (0, 220, 0), 2)
    dp = out.debug.get('direct')
    if dp is not None:
        pts = [(int(cfg.vehicle_col + x / cfg.mm_per_px_x), int(cfg.y_mm_to_row(y))) for x, y in dp]
        cv2.polylines(img, [np.array(pts, np.int32)], False, (255, 0, 255), 1)
    if out.target_mm is not None:
        tx = int(cfg.vehicle_col + out.target_mm[0] / cfg.mm_per_px_x)
        ty = int(cfg.y_mm_to_row(out.target_mm[1]))
        cv2.circle(img, (tx, ty), 7, (255, 0, 255), 2)
    for d in out.stoplines_mm:
        row = int(cfg.y_mm_to_row(d + cfg.ref_to_front_axle_mm))
        thick = 3 if d in out.crosswalk_lines_mm else 1
        cv2.line(img, (0, row), (cfg.bev_w - 1, row), (255, 255, 0), thick)
    vc = int(cfg.vehicle_col)
    cv2.line(img, (vc, cfg.bev_h - 15), (vc, cfg.bev_h - 1), (255, 255, 255), 2)
    txt = f'{out.status} {out.source} k={out.kappa:+.2f}'
    cv2.putText(img, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    if out.stoplines_mm:
        cv2.putText(img, f'stop {out.stoplines_mm[0]:.0f}mm', (6, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    return img
