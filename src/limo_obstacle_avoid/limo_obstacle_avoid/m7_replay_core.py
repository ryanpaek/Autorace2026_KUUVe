"""M7 3차선 정적 장애물 — '녹화한 차로 변경 동작 재생' 방식 (ROS 없음). 11/17 전면 재작성.

코너 진입·켜는 시점·갈림길 방향 전달·끝 처리는 예전 그대로 obstacle_node(m7_link)가 맡는다.
이 모듈은 켜진 뒤(3차선 시작선)부터 섬 끝까지를 맡는다. 노드(m7_replay_node)가 avoid_node 자리에서 같은 토픽으로 돈다.

좌표: 켜는 순간 = 원점, x = 차로 방향(앞 +), y = 왼쪽 +. 차로 번호 k: +1 = 1차선(왼쪽, A) / 0 = 2차선 / -1 = 3차선(오른쪽, B).
      차로 k 중심 y = k x lane_width. 켤 때 차는 start_lane 중심 + start_y, 차로와 start_heading만큼 틀어져 있다고 봄.
위치: odom 이동 거리 + IMU 방향 (path_track.PoseFusion, 녹화 도구와 같은 계산) + 카메라 양쪽 선으로 옆 위치·방향 보정.

우선순위 (위가 먼저)
  1. 안전: 센서 끊김 / 진행 방향 바로 앞(뒤) 차체 범위에 라이다 점 -> 정지 (사라지면 계속)
  2. 섬: 가운데 차로로 섬 끝에 닿기 전이면 정지 (출구 차로로 옮길 수 있을 때까지)
  3. 정한 동작 끝까지: 재생(EXEC)·정렬(ALIGN) 중엔 새 판단 안 함
  4. 앞 장애물: 지금 차로 앞 장애물까지 라이다 거리 <= trigger_mm (+ 지연·제동 보정) -> 차로 고르기 -> (정렬) -> 재생
  5. 출구: 섬 앞 exit_lead 안인데 출구 차로(A=+1 / B=-1)가 아니면 빈 쪽으로 재생
  6. 끝: 섬 끝 + done_margin을 바깥 차로로 지나면 DONE
  7. 평소: 지금 차로 중심 따라 직진 (CRUISE)

상태: IDLE -> CRUISE <-> (STOP ->) ALIGN -> EXEC -> (ALIGN) -> CRUISE ... -> DONE
  11/17: 기본 no_reverse -> CRUISE -> EXEC -> CRUISE (정렬 없음, 재생 전후 위치는 차선 주행이 맞춤)
  ALIGN: 가장 가까운 두 선 사이 가운데로 맞추기. 차로 중심선을 따라 align_back만큼 후진했다가 다시 전진
         (앞뒤로 움직이며 옆 오차·방향 오차를 줄임). 전진 끝 = 장애물까지 trigger_mm (재생 전) / 원래 x (재생 후).
         허용 오차 안이면 바로 끝, 최대 align_cycles번.
  EXEC : 녹화 경로(왼쪽/오른쪽 CSV, 없으면 거울상/기본 S자)를 '장애물 앞 trigger_mm 지점, 지금 차로 중심, 차로 방향'에
         붙여 그대로 따라감. 전진/후진 구간 그대로 (구간 사이 pause초 정지).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from limo_obstacle_avoid.motion_dir import AccelDirection
from limo_obstacle_avoid.path_track import MAX_CURV, Path, PoseFusion, load_segments, wrap


@dataclass
class ReplayParams:
    lane_width: float = 0.52
    # 11/17 M2 갈림길 재생: M7 끝 지점(섬 앞 꼭짓점 - m2_start_gap, 앞범퍼 기준)에서 (지금 차로, A/B)에 맞는
    #   녹화 파일 m2_{1|2|3}-{A|B}.csv 하나를 재생해 갈래에 들어간 뒤 끝(DONE) -> FSM M2가 차선 주행으로 이어 감.
    #   차로 번호 1 = 1차선(왼쪽, A 쪽, 코드 +1) / 2 = 가운데(0) / 3 = 3차선(오른쪽, B 쪽, -1)
    m2_replay: bool = True
    m2_start_gap: float = 1.00          # ★ M2 녹화 시작 자세: 앞범퍼 ~ 섬 앞 꼭짓점 거리 (m). 녹화한 거리와 같게
    m2_fork_default: str = 'A'          # 그때까지 A/B 표지를 못 보면
    m2_late_max: float = 0.40           # 장애물 때문에 늦어지면 M2 시작 지점 + 이만큼까지는 그 자리에서 시작, 넘으면 예전 방식
    # 11/18 시험주행: 갈림길 표지판(섬 앞 꼭짓점의 막대)을 라이다가 '가운데 차로 장애물'로 봐서 표지판 앞에 가까이 서서 멈춤.
    #   1) 섬 앞 꼭짓점 근처의 작은 덩어리(표지판 막대·원판)는 장애물·경로 검사에서 뺌 (닿기 직전 안전 정지에는 그대로 씀)
    #   2) M2 녹화 길이 지금 자리에선 닿는데 녹화 시작 자리(m2_x)에선 안 닿으면, 거기까지 후진해서 녹화대로 돎
    sign_ignore: bool = True
    sign_zone_before: float = 0.40      # 섬 앞 꼭짓점(section_length) 이만큼 앞부터 (위치 추정 앞뒤 오차 포함)
    sign_zone_after: float = 0.30       #   꼭짓점 이만큼 뒤까지
    sign_zone_half: float = 0.30        #   가운데(섬 중심선) ± 이만큼
    sign_max_size: float = 0.12         # 덩어리 앞뒤·옆 크기가 둘 다 이 안이면 표지판 (정차 LIMO는 폭 약 0.23)
    m2_backup: bool = True
    m2_backup_max: float = 0.80         # M2 시작 자리까지 이만큼 넘게 지나 있으면 후진 안 함
    m2_test_start: bool = False         # 시험용: 켜지는 자리를 곧바로 M2 시작 지점으로 (M7 건너뛰고 갈림길만 연습)
    start_lane_no: int = 0              # 시험용: 켤 때 차로 번호 1/2/3 (0 = start_lane 그대로, 기본 2차선)
    # 11/17 저녁: 차로 변경을 못 하고 장애물이 너무 가까워지면 후진해서 다시 시도
    # 11/18 시험: 54 -> 46 cm 사이는 그 자리에서 바로 재생(exec_from_here), 46 cm 밑이면 50 cm까지 후진.
    #   녹화 경로(m7_left/right) 시뮬레이션: 차로 중심·반듯할 때 여유 3 cm로 통과하는 최소 시작 거리 = 46 cm (44 cm 닿음)
    backup_if_mm: float = 350.0         # 지금 차로 앞 장애물(라이다 거리)이 이보다 가까우면 후진 시작 (0 = 끔). 실차 yaml 460 (11/18)
    backup_to_mm: float = 0.0             # 이 거리까지 후진 (0 = trigger_mm: 녹화 시작 거리로 돌아가 녹화 그대로 재생)
    backup_speed: float = 0.12
    backup_max: float = 0.60            # 한 번에 최대 후진 거리 (m)
    backup_k_e: float = 8.0             # 후진 중 차로 가운데 맞추기 게인 (카메라로 고친 옆 위치·방향 기준)
    backup_k_psi: float = 4.0
    exec_from_here: bool = True         # 11/18: 녹화 시작 거리(trigger_mm)보다 가까우면 녹화 경로를 '지금 위치'에 붙여 재생
                                        #   (예전: 장애물 - trigger_mm 지점에 붙임 -> 차가 이미 지나 있어 '경로가 물체에 닿음'으로 섬)
    backup_when_blocked: bool = True    # 11/18: trigger 안인데 지금 자세로 굴려 본 경로가 닿아 막히면 backup_to_mm까지 후진해서 다시
    backup_lane_center: bool = True     # 11/18: 후진 중 차로 가운데 맞추기를 차선 노드 offset/heading으로 (위치 추정 대신)
    backup_lane_fresh_s: float = 0.3
    m2_mirror: bool = True              # 파일이 없으면 좌우 대칭 파일을 뒤집어 씀 (1-A <-> 3-B, 1-B <-> 3-A, 2-A <-> 2-B)
    accel_reverse: bool = True          # 11/17: 조종기(RC) 후진도 거리에서 빼기 (출발 가속도 방향, motion_dir.py)
    accel_sign: int = 0                 #   0 = 가속도계 축을 우리 명령 출발로 배움 / 1 / -1 고정
    num_side_lanes: int = 1
    start_lane: int = 0
    # 켜는 노드가 넣어 줌
    section_length: float = 4.38        # 켜는 점 ~ 섬 앞 끝 (m)
    start_heading_deg: float = 0.0      # 켤 때 차가 차로보다 왼쪽으로 틀어진 각
    start_target: int = 0               # (안 씀: 예전 코너 미리보기용, 호환)
    start_y: float = 0.0                # 켤 때 시작 차로 중심에서 옆 (m)
    exit_lane: str = 'auto'             # auto | A | B
    # 속도
    speed: float = 0.20                 # 차로 따라가기
    exec_speed: float = 0.20            # 재생 (녹화 속도와 같게)
    exec_rev_speed: float = 0.20
    align_speed: float = 0.15
    # 트리거: 라이다 중심 -> 앞 장애물 거리가 이만큼이 되는 지점에서 녹화 동작 시작 (녹화할 때 잰 거리)
    trigger_mm: float = 540.0
    latency_s: float = 0.15             # 판단~바퀴 반응 지연 -> 속도 x 지연만큼 일찍 판단
    brake_k: float = 0.13               # 정지 명령 후 밀리는 거리 = brake_k x 속도 (m, calib 4번 13 x v cm)
    lidar_x: float = 0.10               # 위치 기준점 -> 라이다 (앞 +)
    corridor_half: float = 0.18         # 차로 중심 ± 이 안 라이다 점 = 그 차로 장애물 (LIMO 폭 약 0.23/2 + 여유)
    obs_min_pts: int = 3                # 가장 가까운 점 앞 obs_bin 안에 이만큼 있어야 장애물 (잡음 제외)
    obs_bin: float = 0.10
    look_max: float = 2.5               # 앞 이만큼까지 봄
    # 11/19 실차 '앞에 장애물이 없는데 후진': 라이다 점이 한 스캔만 튀어도 그 스캔으로 '46 cm 앞 장애물' -> 바로 후진했다.
    #   차로 장애물 앞면은 연속 obs_confirm_frames 스캔 동안 같은 자리(차로 좌표 ± obs_confirm_tol)에 보여야 판단(트리거·후진·
    #   차로 고르기)에 씀. 진짜 장애물은 2.5 m 앞부터 계속 보이므로 트리거가 늦어지지 않음. 차체 바로 앞 안전 정지는 그대로 한 스캔
    obs_confirm_frames: int = 3         # 1 = 예전 (한 스캔으로 바로)
    obs_confirm_tol: float = 0.10
    # 라이다 점으로 정하는 후진(지금 자세로 막힘·출구 쪽·M2 시작 자리)은 그 조건이 연속 이만큼 스캔 그대로여야 함. 그동안은 정지만
    reverse_confirm_frames: int = 3     # 1 = 예전
    track_half: float = 0.80            # |y| 이보다 바깥 점은 트랙 밖 (무시)
    # 차로 고르기
    hit_margin: float = 0.03            # 차로 고를 때 녹화 경로를 굴려 보며 차체 + 이만큼 안에 라이다 점이 없어야 그쪽으로 감
    exit_prefer_m: float = 1.6          # 섬까지 이만큼 안이면 출구 쪽 차로를 먼저 고름
    blocked_wait_s: float = 3.0         # 갈 곳 없을 때 기다리는 시간 (그 뒤엔 가장 멀리 빈 쪽)
    # 11/19 실차 '앞·옆 다 막혔다고 보고 영원히 서 있음(실제로는 빠질 틈 있음)': 녹화 경로 하나를 정해진 자리(장애물 - trigger)에
    #   붙여 본 것만으로 '옆 막힘'을 정했다. 막힘이 연속 reverse_confirm_frames 스캔이면 다른 길을 찾는다 (찾는 순서):
    #   1) 그 자리 ~ rescue_fwd_max 더 간 자리에서: 녹화 경로 / 만든 S자(회전반경 = 최소 x rescue_r_scale, 꺾는 각 작은 것부터)
    #   2) 1이 다 막히고 rescue_back이면: 차로 따라 rescue_step씩 rescue_back_max까지 물러난 자리에서 같은 길
    #   길은 차체 + rescue_margin(만든 S자) / hit_margin(녹화)으로 지금 라이다 점·섬에 안 닿고, 도착 뒤 rescue_tail 더 가도 안 닿아야.
    #   다 막혔으면 예전처럼 서서 rescue_period_s마다 다시 찾음.
    rescue: bool = True
    rescue_r_scale: float = 1.2
    rescue_deg_min: float = 25.0
    rescue_deg_max: float = 60.0
    rescue_deg_step: float = 5.0
    rescue_margin: float = 0.04
    rescue_island_margin: float = 0.08  # 섬(라이다에 안 보여 도면 위치로만 앎)은 위치 추정 앞뒤 오차만큼 더 넓게
    rescue_fwd_max: float = 0.20
    rescue_back: bool = True
    rescue_back_max: float = 0.50
    rescue_step: float = 0.05          # 앞으로 더 가 볼 간격
    rescue_back_step: float = 0.10     # 물러나 볼 간격
    rescue_tail: float = 0.15
    rescue_period_s: float = 0.5
    # 정렬
    # 11/17 실차 요청: 재생 전후 '앞뒤로 움직이며 가운데 맞추기'가 너무 쉽게 걸림 -> 가운데 맞추기는 빼고 차선 주행을 100% 믿음.
    #   (장애물이 backup_if_mm보다 가까울 때 후진은 그대로 함)
    #   no_reverse true: 정렬(ALIGN) 안 함, 오래 막혀도 물러나지 않음(서서 다시 봄), 트리거 때 서지 않고 바로 재생,
    #   재생 끝나면 바로 차선 주행(CRUISE). 녹화 파일 안의 후진 구간은 녹화대로 재생 (그것도 빼려면 replay_reverse false)
    no_reverse: bool = True
    replay_reverse: bool = True
    anchor_actual_y: bool = True      #   재생을 지금 옆 위치에서 시작해 목표 차로 중심에서 끝나게 (가운데로 확 끌려가 장애물에 붙지 않게)
    anchor_max_off: float = 0.10      #   그 옆 위치 차이 최대 [m]. 시뮬레이션 최소 간격: 가운데 기준 1.4 -> 지금 위치 2.6 cm (예전 후진 정렬 7.3)
    align_before: bool = False
    align_after: bool = False
    align_tol_y: float = 0.03
    align_tol_psi_deg: float = 3.0
    align_back: float = 0.35
    align_cycles: int = 3
    align_k_e: float = 30.0             # 정렬은 짧은 거리에서 맞춰야 해서 게인 큼 (시뮬레이션: 6/3.4는 3번 해도 7 cm 남음)
    align_k_psi: float = 9.0
    align_skip_if_ok: bool = True       # 트리거 때 이미 허용 오차 안이면 서지 않고 바로 재생 (지연 없음)
    # 추종 (path_follower와 같은 식)
    k_e: float = 6.0
    k_psi: float = 3.4
    preview_s: float = 0.3
    pause_s: float = 0.3
    max_err: float = 0.30
    # 출구·끝
    exit_lead: float = 0.30             # 출구 차로 변경을 '섬 앞 끝 - 동작 길이 - 이만큼'부터
    done_margin: float = 0.30           # 섬 앞 끝 + 이만큼을 바깥 차로로 지나면 끝
    island_half_width: float = 0.25     # 섬(바닥 선, 도면 가운데 차로 폭 500)의 반폭. 라이다에 안 보여 도면 위치로만 피함
    island_stop: float = 0.15           # 가운데 차로면 섬 앞 끝 - 이만큼에서 정지
    # 안전 (진행 방향 차체 앞/뒤)
    robot_front: float = 0.20
    robot_rear: float = 0.16
    robot_half_width: float = 0.12
    stop_margin: float = 0.04
    exec_stop_margin: float = 0.015     # 재생 중엔 녹화로 검증된 길이라 더 좁게 (차체 + 1.5 cm)
    safety_stop: bool = True
    safety_giveup_s: float = 2.0        # 재생 중 이만큼 계속 막혀 서 있으면 재생을 그만두고 그 자리에서 다시 판단
    # 카메라 보정 (/lane/status: 양쪽 선이 보일 때)
    cam_fix: bool = True
    cam_gain_y: float = 0.25
    cam_gain_psi: float = 0.2
    cam_gate: float = 0.15
    cam_max_deg: float = 12.0
    cam_row: float = 0.55
    # 11/17: 트리거 전(CRUISE) 조향은 차선 기반만. 차선 노드 /lane/status 곡률을 그대로 씀 (선이 보이면. 카메라 차로 번호가 달라도).
    #   차선이 cruise_lane_fresh_s 넘게 안 오거나 선을 놓쳤으면 그동안만 위치 추정(odom+IMU) 직선 따라가기
    cruise_lane: bool = True
    cruise_lane_fresh_s: float = 0.4
    # 11/17: 트리거 전에도 장애물 위치와 '어느 차로로 빠질지'를 plan_period_s마다 미리 계산해 둠 (/avoid/state plan). 조향에는 안 씀
    plan_ahead: bool = True
    plan_period_s: float = 0.3
    # 11/17 밤: '장애물 피하고 또 장애물이면 무작정 멈춤' 대책
    still_max_s: float = 1.0            # '완전히 섰나'(odom 정지)를 이만큼만 기다림. odom이 조금씩 흔들려도 그 뒤엔 다음 단계로
    # 11/18 시험 (브랜치 test/m7-exec-yellow-guard): 재생(EXEC) 중 노란선 안전장치.
    #   '노란선이 보인다'가 아니라 '지금 노란선 반대로 최대 조향해도 바퀴~노란선 여유가 eyg_clear_mm 밑으로 떨어진다'일 때만 걸림.
    #   차선 노드 /lane/status의 yellow_l_mm / yellow_r_mm(차 중심 -> 노란 실선 옆 거리, 앞 yellow_f_mm에서 잰 값)을
    #   차 방향(위치 추정 ψ)으로 차 기준점 옆 거리로 되돌린 뒤, 앞 모서리가 쓸려 나가는 양 + 반응 지연 + 방향 펴는 데 드는 옆 이동을 뺌.
    #   걸리면 재생을 끝내고 그 틱은 노란선 반대로 최대 조향, 다음부터 CRUISE(차선 노드 곡률, 차선 노드 자체 노란선 지킴 포함)
    exec_yellow_guard: bool = True
    eyg_clear_mm: float = 50.0          # 바퀴(차체 옆) ~ 노란선 최소 여유
    eyg_psi_deg: float = 2.0            # 노란선 쪽으로 이만큼 이상 틀어져 있을 때만
    eyg_front: float = 0.20             # 위치 기준점 -> 차 앞 모서리 (m)
    eyg_arm_frac: float = 0.30          # 재생 진행률(전진 구간 길이 기준)이 이만큼 지나야 켬 (0 = 처음부터)
    eyg_frames: int = 2                 # 연속 이만큼 걸려야 인정
    eyg_fresh_s: float = 0.3            # 이보다 오래된 /lane/status는 안 씀
    cam_id_need: int = 5                # 차선 노드 lane_id가 최근 7번 중 이만큼 같으면 차로 번호를 그걸로 (CRUISE 때만)
    cam_id_window: int = 7


def _straight(y: float, x0: float, x1: float, direction: int = 1) -> Path:
    n = max(2, int(abs(x1 - x0) / 0.02) + 1)
    xs = [x0 + (x1 - x0) * i / (n - 1) for i in range(n)]
    return Path(xs, [y] * n, [0.0] * n, direction)


def default_change(direction: int, lane_width: float = 0.52, length: float = 1.3, tail: float = 0.3) -> List[Path]:
    """녹화 파일이 없을 때 쓰는 기본 차로 변경 (1-cos S자). direction +1 = 왼쪽."""
    xs, ys, hs = [], [], []
    n = int((length + tail) / 0.02) + 1
    for i in range(n):
        x = i * 0.02
        if x <= length:
            y = direction * lane_width * (1 - math.cos(math.pi * x / length)) / 2
            dy = direction * lane_width * math.pi / (2 * length) * math.sin(math.pi * x / length)
        else:
            y, dy = direction * lane_width, 0.0
        xs.append(x)
        ys.append(y)
        hs.append(math.atan(dy))
    return [Path(xs, ys, hs, 1)]


def mirror(segs: List[Path]) -> List[Path]:
    return [Path(list(g.x), [-y for y in g.y], [-h for h in g.h], g.dir) for g in segs]


def load_change(path_file: Optional[str], other_file: Optional[str], direction: int,
                lane_width: float) -> Tuple[List[Path], str]:
    """녹화 경로 읽기. 없으면 반대쪽 녹화의 거울상, 그것도 없으면 기본 S자. (구간들, 출처 설명)"""
    for f, mir in ((path_file, False), (other_file, True)):
        if not f:
            continue
        try:
            segs, _ = load_segments(f, 'fused')
        except (OSError, ValueError):
            try:
                segs, _ = load_segments(f, 'odom')
            except (OSError, ValueError):
                continue
        if segs and sum(g.length for g in segs) > 0.2:
            return (mirror(segs) if mir else segs), (f'{f} 거울상' if mir else f)
    return default_change(direction, lane_width), '기본 S자 (녹화 파일 없음)'


M2_LANE_NO = {1: 1, 0: 2, -1: 3}          # 코드 차로 -> 파일 차로 번호
M2_LANE_K = {v: k for k, v in M2_LANE_NO.items()}


def m2_name(k: int, fork: str) -> str:
    return f'm2_{M2_LANE_NO[k]}-{fork}.csv'


def load_m2(dirpath: str, mirror_ok: bool = True) -> Tuple[Dict[Tuple[int, str], List[Path]], Dict[Tuple[int, str], str]]:
    """M2 녹화 6개 읽기. 반환 ({(차로 k, 'A'|'B'): 구간들}, {(k, f): 출처 설명}). 없는 건 대칭 파일 거울상."""
    import os
    got, src = {}, {}
    for k in (1, 0, -1):
        for f in ('A', 'B'):
            fn = os.path.join(dirpath, m2_name(k, f)) if dirpath else ''
            if not fn or not os.path.exists(fn):
                continue
            for mode in ('fused', 'odom'):
                try:
                    segs, _ = load_segments(fn, mode)
                except (OSError, ValueError):
                    continue
                if segs and sum(g.length for g in segs) > 0.1:
                    got[(k, f)] = segs
                    src[(k, f)] = f'{m2_name(k, f)} [{mode}]'
                    break
    if mirror_ok:
        for k in (1, 0, -1):
            for f in ('A', 'B'):
                mk, mf = -k, ('B' if f == 'A' else 'A')
                if (k, f) not in got and (mk, mf) in src and not src[(mk, mf)].endswith('거울상'):
                    got[(k, f)] = mirror(got[(mk, mf)])
                    src[(k, f)] = f'{m2_name(mk, mf)} 거울상'
    return got, src


def fit_lateral(segs: List[Path], lane_width: float) -> Tuple[List[Path], float]:
    """녹화 끝 옆 위치를 정확히 ±차로 간격으로 (녹화가 0.50·0.55 m에서 끝났어도 다음 차로 중심에 닿게).
    배율이 0.8~1.25 밖이면 손대지 않음. 반환 (구간들, 배율)"""
    ye = segs[-1].y[-1] - segs[0].y[0]
    if abs(ye) < 1e-6:
        return segs, 1.0
    k = lane_width / abs(ye)
    if not 0.8 <= k <= 1.25:
        return segs, 1.0
    y0 = segs[0].y[0]
    out = [Path(list(g.x), [y0 + (y - y0) * k for y in g.y], [math.atan(math.tan(h) * k) if abs(h) < 1.4 else h for h in g.h],
                g.dir) for g in segs]
    return out, k


def anchor(segs: List[Path], x0: float, y0: float) -> List[Path]:
    """녹화 경로(원점 = 녹화 시작, 방향 0)를 (x0, y0, 차로 방향)에 붙임."""
    out = []
    for g in segs:
        h0 = 0.0
        out.append(Path([x0 + x for x in g.x], [y0 + y for y in g.y], [h - h0 for h in g.h], g.dir))
    return out


def anchor_fade(segs: List[Path], x0: float, yc: float, off: float) -> List[Path]:
    """anchor와 같되 시작 옆 위치를 yc + off(지금 차 위치)로, 끝으로 갈수록 off를 0으로 줄여 목표 차로 중심에서 끝남.
    11/17: 후진으로 가운데를 안 맞추므로, 벗어난 채 재생을 시작해도 차로 가운데로 확 끌려가 장애물에 붙지 않게"""
    out = anchor(segs, x0, yc)
    xs_all = [x for g in out for x in g.x]
    x_lo, x_hi = min(xs_all), max(xs_all)
    span = max(1e-6, x_hi - x_lo)
    res = []
    for g in out:
        ys = [y + off * (1.0 - min(1.0, max(0.0, (x - x_lo) / span))) for x, y in zip(g.x, g.y)]
        hs = [h - math.atan(off / span) if g.dir > 0 else h for h in g.h]
        res.append(Path(list(g.x), ys, hs, g.dir))
    return res


class Track:
    """구간 목록 따라가기 (path_follower --ctrl track과 같은 식)."""

    def __init__(self, segs: List[Path], p: ReplayParams, speed: float, rev_speed: float,
                 k_e: Optional[float] = None, k_psi: Optional[float] = None):
        self.segs, self.p, self.speed, self.rev_speed = segs, p, speed, rev_speed
        self.k_e = p.k_e if k_e is None else k_e
        self.k_psi = p.k_psi if k_psi is None else k_psi
        self.i = 0
        self.idx = 0
        self.pause_until = None
        self.done = False
        self.e = self.epsi = self.s = 0.0
        self.max_e = 0.0
        self.n = self.n_sat = 0

    @property
    def seg(self) -> Path:
        return self.segs[self.i]

    def step(self, x, y, psi, t, still: bool) -> Tuple[float, float]:
        """반환 (v, 곡률). 끝나면 done."""
        if self.done:
            return 0.0, 0.0
        if self.pause_until is not None:
            if t < self.pause_until or (not still and t < self.pause_until + self.p.still_max_s):
                return 0.0, 0.0
            self.pause_until = None
        P = self.seg
        self.idx, s, e, hp = P.project(x, y, self.idx)
        epsi = wrap(psi - hp)
        self.s, self.e, self.epsi = s, e, epsi
        self.max_e = max(self.max_e, abs(e))
        d = P.dir
        v = d * (self.speed if d > 0 else self.rev_speed)
        stop_early = self.p.brake_k * abs(v)
        if s >= P.length - stop_early:
            if self.i + 1 < len(self.segs):
                self.i += 1
                self.idx = 0
                self.pause_until = t + self.p.pause_s
            else:
                self.done = True
            return 0.0, 0.0
        kff = d * P.k_at(s + abs(v) * self.p.preview_s)
        curv = kff - self.k_e * e - d * self.k_psi * math.sin(epsi)
        sat = abs(curv) >= MAX_CURV
        self.n += 1
        self.n_sat += sat
        return v, max(-MAX_CURV, min(MAX_CURV, curv))

    def off_track(self) -> bool:
        return abs(self.e) > self.p.max_err or abs(self.epsi) > math.radians(50)


class M7Replay:
    def __init__(self, p: ReplayParams, left: List[Path], right: List[Path]):
        self.p = p
        (left, self.k_left), (right, self.k_right) = fit_lateral(left, p.lane_width), fit_lateral(right, p.lane_width)
        self.change = {+1: left, -1: right}
        self.m2: Dict[Tuple[int, str], List[Path]] = {}
        self.m2_src: Dict[Tuple[int, str], str] = {}
        self.m2_run = None            # 재생 중인 M2 (k, fork)
        self.fu = PoseFusion()
        self.acc = AccelDirection(fixed_sign=p.accel_sign)
        self.state = 'IDLE'
        self.events: List[str] = []
        self.reset_pose()
        self.lane = p.start_lane
        self.target = p.start_lane
        self.exit_pref: Optional[int] = None
        self.set_exit(p.exit_lane)
        self.track: Optional[Track] = None
        self.backup = None
        self.align = None
        self.after_align = False
        self.pending = None           # 정렬 뒤 실행할 동작 (방향, 장애물 x)
        self.obs: Dict[int, Optional[float]] = {}        # 확정된 차로 장애물 앞면 x (obs_confirm_frames 스캔 연속)
        self.obs_raw: Dict[int, Optional[float]] = {}    # 이번 스캔만 본 것 (status·로그용)
        self.front_hist: List[Dict[int, List[float]]] = []   # 최근 스캔별 차로 장애물 앞면들
        self.scan_seq = 0                                # 받은 스캔 수 (연속 조건 세기)
        self.persist: Dict[str, Tuple[int, int]] = {}    # 조건 이름 -> (처음 참인 스캔, 마지막 참인 스캔)
        self.pts: List[Tuple[float, float]] = []
        self.sign_pts: list = []          # 표지판으로 보고 뺀 점 (안전 정지에만 씀)
        self.m2_backed = False
        self.scan_t = None
        self.cam_ids: List[Optional[int]] = []
        self.blocked_since = None
        self.stop_why = None
        self.cam_n = 0
        self.last_v = 0.0
        self.why = ''
        self.n_exec = 0
        self.safety_since = None
        self.backed_off = False
        self.lane_msg: Optional[dict] = None      # 마지막 /lane/status
        self.lane_t: Optional[float] = None
        self.steer_src = '-'                      # CRUISE 조향 출처: lane(차선 노드) / pose(위치 추정)
        self.plan: dict = {}                      # 미리 계산한 회피 계획
        self.plan_t = -1e9
        self.stop_since = None                    # STOP 들어간 시각 (still_max_s)
        self.eyg_n = 0                            # 노란선 안전장치 연속 걸린 수
        self.eyg_hits = 0                         # 이번 판 노란선 안전장치로 재생 끊은 횟수
        self.eyg_last: Optional[dict] = None      # 마지막 계산 값 (디버그)

    # ---------- 위치 ----------
    def reset_pose(self):
        self.x0f = self.y0f = 0.0
        self.offset = (0.0, 0.0, 0.0)       # fusion 좌표 -> 차로 좌표 (dx, dy, dpsi)

    def pose(self) -> Tuple[float, float, float]:
        dx, dy, dpsi = self.offset
        c, s = math.cos(dpsi), math.sin(dpsi)
        fx, fy = self.fu.x, self.fu.y
        return dx + c * fx - s * fy, dy + s * fx + c * fy, wrap(self.fu.yaw + dpsi)

    def _shift(self, ddx=0.0, ddy=0.0, ddpsi=0.0):
        """차로 좌표에서 지금 위치를 (ddx, ddy) 옮기고 방향 ddpsi 돌림 (지금 위치 기준 회전)."""
        x, y, psi = self.pose()
        dx, dy, dpsi = self.offset
        dpsi2 = dpsi + ddpsi
        c, s = math.cos(dpsi2), math.sin(dpsi2)
        fx, fy = self.fu.x, self.fu.y
        nx, ny = x + ddx, y + ddy
        self.offset = (nx - (c * fx - s * fy), ny - (s * fx + c * fy), dpsi2)

    def center(self, k: int) -> float:
        return k * self.p.lane_width

    def nearest_lane(self, y: float) -> int:
        n = self.p.num_side_lanes
        return max(-n, min(n, int(round(y / self.p.lane_width))))

    # ---------- 입력 ----------
    def on_imu(self, t, gz, ax=None):
        self.fu.on_imu(t, gz)
        if ax is not None and self.p.accel_reverse:
            self.acc.on_imu(t, ax)

    def on_odom(self, t, x, y, yaw, v):
        acc_rev = False
        if self.p.accel_reverse:
            acc_rev = self.acc.on_odom(t, x, y, self.last_v) < 0
        self.fu.on_odom(t, x, y, yaw, v, self.last_v, acc_rev)

    def set_m2(self, paths: Dict[Tuple[int, str], List[Path]], src: Dict[Tuple[int, str], str]):
        self.m2, self.m2_src = dict(paths), dict(src)

    def m2_on(self) -> bool:
        return bool(self.p.m2_replay and self.m2)

    def m2_possible(self, x: float) -> bool:
        """아직 M2 녹화로 갈 수 있나 (파일 있음, 시작 지점 + m2_late_max 안)"""
        return self.m2_on() and x <= self.m2_x() + self.p.m2_late_max

    def m2_hit(self, key, x0: float) -> Optional[Tuple[float, float]]:
        """M2 녹화를 붙여 굴려 봤을 때 차체 + hit_margin에 라이다 점 (섬은 녹화가 이미 피했다고 보고 안 봄)"""
        p = self.p
        m = p.hit_margin
        pts = [(lx, ly) for lx, ly, _, _ in self.pts]
        for g in anchor(self.m2[key], x0, self.center(self.lane)):
            for i in range(0, len(g), 3):
                cx, cy, h = g.x[i], g.y[i], g.h[i]
                c, s_ = math.cos(h), math.sin(h)
                for lx, ly in pts:
                    a = c * (lx - cx) + s_ * (ly - cy)
                    b = -s_ * (lx - cx) + c * (ly - cy)
                    if -p.robot_rear - m <= a <= p.robot_front + m and abs(b) <= p.robot_half_width + m:
                        return lx, ly
        return None

    def m2_x(self) -> float:
        """M2 녹화 시작 위치 (위치 기준점 x) = 섬 앞 꼭짓점 - m2_start_gap - 앞범퍼"""
        return self.p.section_length - self.p.m2_start_gap - self.p.robot_front

    def set_exit(self, s):
        s = (s or 'auto').strip().upper()
        self.exit_pref = +1 if s == 'A' else (-1 if s == 'B' else None)

    def on_scan_points(self, t, pts_car: Sequence[Tuple[float, float]]):
        """pts_car: 차 기준 (앞 +, 왼쪽 +) 위치 기준점 원점 m"""
        self.scan_t = t
        x, y, psi = self.pose()
        c, s = math.cos(psi), math.sin(psi)
        loc = []
        for px, py in pts_car:
            lx, ly = x + c * px - s * py, y + s * px + c * py
            if abs(ly) <= self.p.track_half:
                loc.append((lx, ly, px, py))
        loc, self.sign_pts = self._split_sign(loc)
        if self.sign_pts and not getattr(self, '_sign_logged', False):
            self._sign_logged = True
            sx = sum(q[0] for q in self.sign_pts) / len(self.sign_pts)
            self.events.append(f'[x={x:.2f}] 섬 앞 표지판으로 보고 장애물에서 뺌 ({len(self.sign_pts)}점, 꼭짓점 기준 '
                               f'{100 * (sx - self.p.section_length):+.0f} cm)')
        self.pts = loc
        fronts = self._lane_fronts(loc, x + self.p.lidar_x - 0.05)                # 앞 (라이다 앞)
        self.scan_seq += 1
        n = max(1, int(self.p.obs_confirm_frames))
        self.front_hist = (self.front_hist + [fronts])[-n:]
        self.obs_raw = {k: (f[0] if f else None) for k, f in fronts.items()}
        self.obs = self._confirmed(n)

    def _confirmed(self, n: int) -> Dict[int, Optional[float]]:
        """차로마다: 이번 스캔 앞면들 중 앞선 n-1 스캔 모두에 ± obs_confirm_tol 안 짝이 있는 가장 가까운 것 (없으면 None).
        차로 좌표(odom+IMU)로 비교하므로 차가 움직여도 서 있는 장애물은 같은 자리. 한 스캔만 튄 점은 짝이 없어 버려짐."""
        hist = self.front_hist
        last = hist[-1] if hist else {}
        if len(hist) < n:
            return {k: None for k in last}
        tol = self.p.obs_confirm_tol
        out = {}
        for k, fs in last.items():
            out[k] = next((f for f in fs if all(any(abs(f - g) <= tol for g in h.get(k, ())) for h in hist[-n:-1])),
                          None)
        return out

    def _persist(self, key: str, cond: bool) -> bool:
        """cond가 연속 reverse_confirm_frames 스캔 동안 참이었나 (스캔 하나라도 거짓이거나 빠지면 처음부터)."""
        if not cond:
            self.persist.pop(key, None)
            return False
        first, last = self.persist.get(key, (self.scan_seq, self.scan_seq))
        if last < self.scan_seq - 1:                 # 사이 스캔에서 참이 아니었음 (확인 안 됨) -> 다시
            first = self.scan_seq
        self.persist[key] = (first, self.scan_seq)
        return self.scan_seq - first + 1 >= max(1, int(self.p.reverse_confirm_frames))

    def _split_sign(self, loc):
        """섬 앞 꼭짓점 근처의 작은 덩어리 = 갈림길 표지판 -> (나머지 점, 표지판 점)"""
        p = self.p
        if not p.sign_ignore:
            return loc, []
        x_tip = p.section_length
        zone = [q for q in loc if x_tip - p.sign_zone_before <= q[0] <= x_tip + p.sign_zone_after
                and abs(q[1]) <= p.sign_zone_half]
        if not zone:
            return loc, []
        # 점끼리 6 cm 안이면 같은 덩어리
        groups: List[list] = []
        for q in sorted(zone, key=lambda r: (r[0], r[1])):
            for g in groups:
                if any(math.hypot(q[0] - r[0], q[1] - r[1]) <= 0.06 for r in g):
                    g.append(q)
                    break
            else:
                groups.append([q])
        sign = []
        for g in groups:
            xs, ys = [r[0] for r in g], [r[1] for r in g]
            if max(xs) - min(xs) <= p.sign_max_size and max(ys) - min(ys) <= p.sign_max_size:
                sign.extend(g)
        if not sign:
            return loc, []
        ids = set(map(id, sign))
        return [q for q in loc if id(q) not in ids], sign

    def _lane_fronts(self, loc, x_min) -> Dict[int, List[float]]:
        """차로마다 장애물 앞면 x 목록 (가까운 순). 앞면 = 그 점부터 obs_bin 안에 점이 obs_min_pts 이상.
        첫 번째가 예전 '가장 가까운 장애물'과 같음. 한 앞면을 찾으면 obs_bin 뒤부터 다음 앞면을 찾음."""
        p = self.p
        out = {}
        for k in range(-p.num_side_lanes, p.num_side_lanes + 1):
            yc = self.center(k)
            xs = sorted(lx for lx, ly, _, _ in loc if abs(ly - yc) <= p.corridor_half
                        and x_min <= lx <= x_min + p.look_max)
            fr: List[float] = []
            for i, xv in enumerate(xs):
                if fr and xv <= fr[-1] + p.obs_bin:
                    continue
                if sum(1 for xw in xs[i:i + 40] if xw - xv <= p.obs_bin) >= p.obs_min_pts:
                    fr.append(xv)
            out[k] = fr
        return out

    def _lane_obstacles(self, loc, x_min) -> Dict[int, Optional[float]]:
        """차로마다 가장 가까운 장애물 앞면 (한 스캔). 확인 없이 쓰는 곳용."""
        return {k: (f[0] if f else None) for k, f in self._lane_fronts(loc, x_min).items()}

    def on_lane(self, lane: dict, t: Optional[float] = None):
        """차선 노드 /lane/status: 양쪽 선이 보이면 옆 위치·방향 보정, 차로 번호 확인. CRUISE 조향에도 씀 (cruise_lane)."""
        p = self.p
        if lane:
            self.lane_msg, self.lane_t = lane, (t if t is not None else self.scan_t)
        if not p.cam_fix or not lane or self.state in ('IDLE', 'DONE', 'EXEC'):
            return
        lid = lane.get('lane_id')
        self.cam_ids = (self.cam_ids + [lid if lid in (-1, 0, 1) else None])[-p.cam_id_window:]
        off, h = lane.get('offset_mm'), lane.get('heading_rad')
        if lane.get('status') != 'BOTH' or lane.get('low_conf') or off is None or h is None:
            return
        h = float(h)
        if abs(math.degrees(h)) > p.cam_max_deg:
            return
        x, y, psi = self.pose()
        k = self.nearest_lane(y)
        # 카메라 차로 번호가 여러 번 같고 지금 추정과 다르면 그 차로로 (CRUISE에서만)
        if self.state == 'CRUISE':
            for cand in (-1, 0, 1):
                # 양 끝 차로끼리(1 <-> 3차선) 바꾸는 것은 두 칸 차이 -> 창 전체(7번)가 같아야 (한 칸은 5번)
                need = p.cam_id_need if abs(cand - k) == 1 else p.cam_id_window
                if cand != k and self.cam_ids.count(cand) >= need:
                    self._shift(ddy=self.center(cand) - self.center(k))
                    self.events.append(f'카메라 차로 번호 {cand} -> 차로 {k} => {cand}로 고침')
                    self.lane = self.target = cand
                    self.cam_ids.clear()
                    # 차로 번호가 통째로 (cand - k)만큼 밀림 -> 차로별 장애물 기록도 같은 만큼 밀어 둠 (x는 그대로라 확인 이어감)
                    sh, n_ = cand - k, p.num_side_lanes
                    self.front_hist = [{kk + sh: v for kk, v in h.items() if abs(kk + sh) <= n_} for h in self.front_hist]
                    self.obs = {kk + sh: v for kk, v in self.obs.items() if abs(kk + sh) <= n_}
                    self.obs_raw = {kk + sh: v for kk, v in self.obs_raw.items() if abs(kk + sh) <= n_}
                    self.persist.clear()
                    x, y, psi = self.pose()
                    k = cand
                    break
        offm = float(off) / 1000.0
        y_meas = self.center(k) - offm * math.cos(h) + p.cam_row * math.sin(h)
        dy = y_meas - y
        dpsi = wrap(-h - psi)
        if abs(dy) > p.cam_gate:
            return
        self._shift(ddy=p.cam_gain_y * dy, ddpsi=(p.cam_gain_psi * dpsi if abs(math.degrees(dpsi)) <= 5.0 else 0.0))
        self.cam_n += 1

    # ---------- 켜기/끄기 ----------
    def start(self):
        p = self.p
        self.fu.x = self.fu.y = 0.0
        self.fu.s = self.fu.dist = 0.0
        self.offset = (0.0, self.center(p.start_lane) + p.start_y, math.radians(p.start_heading_deg) - self.fu.yaw)
        if p.start_lane_no in (1, 2, 3):
            k0 = M2_LANE_K[p.start_lane_no]
            self.offset = (0.0, self.center(k0) + p.start_y, self.offset[2])
            self.lane = self.target = k0
        else:
            self.lane = self.target = p.start_lane
        if p.m2_test_start:
            p.section_length = p.m2_start_gap + p.robot_front       # 지금 자리 = M2 시작 지점
            self.events.append(f'시험: 지금 자리를 M2 시작 지점으로 (차로 {M2_LANE_NO[self.lane]})')
        self.backup = None
        self.state = 'CRUISE'
        self.track = None
        self.align = None
        self.pending = None
        self.blocked_since = None
        self.n_exec = 0
        self.safety_since = None
        self.backed_off = False
        self.m2_run = None
        self._m2_missing_logged = False
        self._m2_hit_logged = False
        self.m2_backed = False
        self._sign_logged = False
        self.cam_ids.clear()
        self.front_hist = []
        self.obs, self.obs_raw = {}, {}
        self.persist = {}
        self.rescue_t = -1e9
        self.rescue_phase = 'fwd'
        self.n_rescue = 0
        self._rescue_none_logged = False
        self.events.append(f'시작: 차로 {self.lane}, 섬까지 {p.section_length:.2f} m, 출구 {self._exit_name()}')

    def stop(self):
        self.state = 'IDLE'

    def _exit_name(self):
        return {1: 'A(1차선)', -1: 'B(3차선)'}.get(self.exit_pref, '모름')

    # ---------- 판단 ----------
    def gap(self, k: int) -> Optional[float]:
        """라이다 -> 차로 k 장애물 앞면 (m)"""
        xo = self.obs.get(k)
        if xo is None:
            return None
        x, _, _ = self.pose()
        return xo - (x + self.p.lidar_x)

    def _change_len(self, d: int) -> float:
        segs = self.change[d]
        return max(max(g.x) for g in segs) - min(min(g.x) for g in segs)

    def _x0(self, xo: Optional[float]) -> float:
        """녹화 경로 시작 x: 장애물 앞면 - 라이다 - trigger_mm. 차가 이미 지나 있으면(exec_from_here) 지금 위치."""
        p = self.p
        x, _, _ = self.pose()
        if xo is None:
            return x
        x0 = xo - p.lidar_x - p.trigger_mm / 1000.0
        return max(x0, x) if p.exec_from_here else x0

    def choose(self, cur: int, xo: Optional[float]) -> Tuple[Optional[int], str]:
        """옮길 옆 차로 (+1 왼쪽 / -1 오른쪽 방향) 고르기. xo = 지금 차로 장애물 앞면 x (없으면 출구 변경)."""
        p = self.p
        x, _, _ = self.pose()
        n = p.num_side_lanes
        cands = []
        x0 = self._x0(xo)
        x0_rec = (xo - p.lidar_x - p.trigger_mm / 1000.0) if xo is not None else x
        why_no = []
        self.pref_needs_backup = False   # 출구 쪽이 '지금 자리'에선 닿는데 녹화 시작 거리에선 됨 -> 물러나면 출구 쪽으로 갈 수 있음
        for d in (+1, -1):
            k = cur + d
            if abs(k) > n:
                continue
            hit = self.path_hit(d, x0)
            if hit is not None:
                why_no.append(f'{k}차로: 녹화 경로가 {hit[0] - x:+.2f} m 지점 물체에 닿음')
                if (self.exit_pref is not None and self.exit_pref != cur and d == (1 if self.exit_pref > cur else -1)
                        and x0 > x0_rec + 0.01 and self.path_hit(d, x0_rec + (x0 - x)) is None):
                    self.pref_needs_backup = True
                continue
            o = self.obs.get(k)
            cands.append((d, o if o is not None else math.inf))
        if not cands:
            return None, '옆 차로 막힘 (' + ', '.join(why_no) + ')'
        if len(cands) == 1:
            return cands[0][0], f'{cur + cands[0][0]}차로만 비어 있음'
        if self.exit_pref is not None and self.exit_pref != cur:
            d = 1 if self.exit_pref > cur else -1
            return d, f'양쪽 비어 있음: 출구 {self._exit_name()} 쪽'
        best = max(cands, key=lambda c: c[1])
        return best[0], '양쪽 비어 있음: 더 멀리 빈 쪽'

    def _anchored(self, d: int, x0: float) -> List[Path]:
        """재생할 경로: no_reverse면 지금 옆 위치에서 시작해 목표 차로 중심에서 끝나게(anchor_fade), 아니면 차로 중심에 붙임"""
        p = self.p
        yc = self.center(self.lane)
        if p.no_reverse and p.anchor_actual_y:
            _, y, _ = self.pose()
            off = max(-p.anchor_max_off, min(p.anchor_max_off, y - yc))
            return anchor_fade(self.change[d], x0, yc, off)
        return anchor(self.change[d], x0, yc)

    def path_hit(self, d: int, x0: float) -> Optional[Tuple[float, float]]:
        """녹화 경로를 (x0, 지금 차로 중심)에 붙여 굴려 봤을 때 차체(+여유)에 라이다 점이 들어오면 그 점 (x, y). 없으면 None."""
        return self._sweep_hit(self._anchored(d, x0), self.p.hit_margin)

    def _sweep_hit(self, segs: List[Path], m: float, step: int = 3, end: bool = False,
                   isl_m: float = 0.0) -> Optional[Tuple[float, float]]:
        """구간들을 따라 차체(+여유 m)를 굴려 라이다 점·섬(+여유 isl_m)에 닿는 첫 점 (x, y). 없으면 None. end: 구간 끝 자세도 꼭 봄"""
        p = self.p
        pts = [(lx, ly) for lx, ly, _, _ in self.pts]
        if pts and segs:
            # 경로 둘레(차 대각선 + 여유) 밖 점은 미리 뺌 (속도만, 결과 같음)
            r = math.hypot(max(p.robot_front, p.robot_rear), p.robot_half_width) + m + 0.01
            xs = [v for g in segs for v in g.x]
            ys = [v for g in segs for v in g.y]
            x_lo, x_hi, y_lo, y_hi = min(xs) - r, max(xs) + r, min(ys) - r, max(ys) + r
            pts = [q for q in pts if x_lo <= q[0] <= x_hi and y_lo <= q[1] <= y_hi]
        isl_half = p.island_half_width
        for g in segs:
            n = len(g)
            for i in list(range(0, n, step)) + ([n - 1] if end and n and (n - 1) % step else []):
                cx, cy, h = g.x[i], g.y[i], g.h[i]
                c, s_ = math.cos(h), math.sin(h)
                # 섬은 바닥 선(라이다에 안 보임) -> 도면 위치로: 섬 앞 끝(section_length)부터 가운데 차로 폭
                for a_, b_ in ((p.robot_front, p.robot_half_width), (p.robot_front, -p.robot_half_width),
                               (-p.robot_rear, p.robot_half_width), (-p.robot_rear, -p.robot_half_width)):
                    qx, qy = cx + c * a_ - s_ * b_, cy + s_ * a_ + c * b_
                    if qx >= p.section_length - isl_m and abs(qy) < isl_half + isl_m:
                        return qx, qy
                for lx, ly in pts:
                    dx, dy = lx - cx, ly - cy
                    a = c * dx + s_ * dy
                    b = -s_ * dx + c * dy
                    if -p.robot_rear - m <= a <= p.robot_front + m and abs(b) <= p.robot_half_width + m:
                        return lx, ly
        return None

    # ---------- 막혔을 때 다른 길 찾기 (rescue) ----------
    @staticmethod
    def _line(x0: float, x1: float, y: float, direction: int = 1) -> Path:
        """차로 방향 직선 (x0 -> x1, 옆 y)."""
        n = max(2, int(abs(x1 - x0) / 0.02) + 1)
        return Path([x0 + (x1 - x0) * i / (n - 1) for i in range(n)], [y] * n, [0.0] * n, direction)

    @staticmethod
    def _ray(x: float, y: float, h: float, dist: float, direction: int = 1) -> Path:
        """(x, y)에서 차 방향 h 그대로 dist만큼 앞(direction +1)·뒤(-1)로 가는 직선."""
        n = max(2, int(dist / 0.02) + 1)
        c, s_ = math.cos(h), math.sin(h)
        return Path([x + direction * dist * c * i / (n - 1) for i in range(n)],
                    [y + direction * dist * s_ * i / (n - 1) for i in range(n)], [h] * n, direction)

    @staticmethod
    def _arc_d(r: float, pa: float, pb: float) -> Tuple[float, float]:
        """반경 r로 차 방향 pa -> pb 돌 때 (앞 dx, 옆 dy)."""
        sg = 1.0 if pb >= pa else -1.0
        return sg * r * (math.sin(pb) - math.sin(pa)), -sg * r * (math.cos(pb) - math.cos(pa))

    def _s_curve(self, x0: float, y0: float, h0: float, y1: float, deg: float, r: float,
                 tail: float) -> Optional[Tuple[Path, float]]:
        """지금 자세 (x0, y0, 차 방향 h0)에서 (?, y1, 차로 방향 0)까지: 반경 r로 ±deg까지 틀고 - 직선 - 반경 r로 폄 + tail 직선.
        옆 거리가 모자라면 각을 줄여 직선 0으로 맞춤. 안 되면 None. 반환 (경로, 실제 각 deg)."""
        Y = y1 - y0
        sg = 1.0 if Y >= 0 else -1.0
        if abs(Y) < 0.01 and abs(h0) < math.radians(2.0):
            return None                                       # 옮길 게 없음 (차로 안)

        def miss(th):                                          # 직선 길이 x sin(각) (음수면 너무 많이 옮김)
            pm = sg * th
            return Y - self._arc_d(r, h0, pm)[1] - self._arc_d(r, pm, 0.0)[1]

        th = math.radians(deg)
        if sg * miss(th) < 0:                                  # 이 각이면 넘침 -> 직선 없이 맞는 각 (반으로 나눠 찾기)
            lo, hi = math.radians(1.0), th
            if sg * miss(lo) < 0:
                return None
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if sg * miss(mid) >= 0 else (lo, mid)
            th = lo
        pm = sg * th
        if abs(pm - h0) > math.radians(90.0):
            return None
        st = max(0.0, miss(th) / math.sin(pm)) if abs(math.sin(pm)) > 1e-6 else 0.0
        xs, ys, hs = [x0], [y0], [h0]

        def arc(pa, pb):
            xa, ya = xs[-1], ys[-1]
            sgn = 1.0 if pb >= pa else -1.0
            n = max(1, int(r * abs(pb - pa) / 0.02))
            for i in range(1, n + 1):
                ph = pa + (pb - pa) * i / n
                xs.append(xa + sgn * r * (math.sin(ph) - math.sin(pa)))
                ys.append(ya - sgn * r * (math.cos(ph) - math.cos(pa)))
                hs.append(ph)

        arc(h0, pm)
        xa, ya = xs[-1], ys[-1]
        n = int(st / 0.02)
        for i in range(1, n + 1):
            d = st * i / n
            xs.append(xa + d * math.cos(pm))
            ys.append(ya + d * math.sin(pm))
            hs.append(pm)
        arc(pm, 0.0)
        if abs(ys[-1] - y1) > 0.01:                           # 계산 확인 (안 맞으면 쓰지 않음)
            return None
        xe = xs[-1]
        nt = max(1, int(tail / 0.02))
        for i in range(1, nt + 1):
            xs.append(xe + tail * i / nt)
            ys.append(y1)
            hs.append(0.0)
        return Path(xs, ys, hs, 1), math.degrees(th)

    @staticmethod
    def _join(a: Path, b: Path) -> Path:
        """같은 방향 두 구간을 하나로 (사이 멈춤 없이)."""
        return Path(list(a.x) + list(b.x[1:]), list(a.y) + list(b.y[1:]), list(a.h) + list(b.h[1:]), a.dir)

    def _rescue_sides(self, cur: int) -> List[int]:
        """찾을 쪽 순서: 출구를 알면 출구 쪽 먼저, 아니면 확정 장애물이 더 먼 쪽 먼저."""
        n = self.p.num_side_lanes
        sides = [d for d in (+1, -1) if abs(cur + d) <= n]
        if self.exit_pref is not None and self.exit_pref != cur:
            first = 1 if self.exit_pref > cur else -1
            return sorted(sides, key=lambda d: d != first)
        far = lambda d: self.obs.get(cur + d) if self.obs.get(cur + d) is not None else math.inf  # noqa: E731
        return sorted(sides, key=lambda d: -far(d))

    def _shifts(self, fwd: List[float], back: List[float]):
        """찾을 시작 자리 (앞으로 가까운 것부터, 그다음 뒤로). 그 방향 직선이 막히면 그 방향 나머지는 건너뜀."""
        self._shift_stop = None
        for a in fwd:
            if self._shift_stop == 'fwd':
                break
            yield a
        self._shift_stop = None
        for a in back:
            if self._shift_stop == 'back':
                break
            yield a

    def rescue_search(self, allow_back: bool, fwd: bool = True) -> Optional[dict]:
        """막혔을 때 갈 길: {'d', 'shift'(+앞 / -뒤 m), 'shape', 'segs'(재생할 구간들), 'why'} 또는 None. 위 rescue 설명 순서.
        길은 지금 실제 자세(x, y, 차 방향)에서 시작 (차로와 틀어져 있어도 그대로 굴려 봄). 녹화 경로는 차로와 5° 안일 때만."""
        p = self.p
        x, y, psi = self.pose()
        cur = self.lane
        R = p.rescue_r_scale / MAX_CURV                       # 최소 회전반경(0.53 m, calib 3번) x 배율
        degs, dg = [], p.rescue_deg_min
        while dg <= p.rescue_deg_max + 1e-6:
            degs.append(dg)
            dg += p.rescue_deg_step
        rec_ok = abs(psi) <= math.radians(5.0)
        step = max(0.01, p.rescue_step)
        fwds = [round(i * step, 3) for i in range(int(p.rescue_fwd_max / step + 1e-6) + 1)] if fwd else []
        bstep = max(0.01, p.rescue_back_step)
        back = ([-round(i * bstep, 3) for i in range(1, int(p.rescue_back_max / bstep + 1e-6) + 1)]
                if allow_back else [])
        for a in self._shifts(fwds, back):
            lead = None
            xs0, ys0 = x, y
            if abs(a) > 0.005:
                lead = self._ray(x, y, psi, abs(a), 1 if a > 0 else -1)
                if self._sweep_hit([lead], p.rescue_margin, step=2, end=True, isl_m=p.rescue_island_margin) is not None:
                    self._shift_stop = 'fwd' if a > 0 else 'back'   # 그쪽으로 더 가면 닿음 -> 그 방향 나머지는 안 봄
                    continue
                xs0, ys0 = lead.x[-1], lead.y[-1]
            for d in self._rescue_sides(cur):
                y1 = self.center(cur + d)
                cands = []
                if rec_ok:
                    rec = self._anchored(d, xs0)
                    tail = self._line(rec[-1].x[-1], rec[-1].x[-1] + p.rescue_tail, y1)
                    hit = (self._sweep_hit(rec, p.hit_margin, isl_m=p.rescue_island_margin)
                           or self._sweep_hit([tail], p.hit_margin, end=True, isl_m=p.rescue_island_margin))
                    cands.append(('녹화', rec, hit))
                seen = set()
                for dg in degs:
                    got = self._s_curve(xs0, ys0, psi, y1, dg, R, p.rescue_tail)
                    if got is None:
                        continue
                    path, used = got
                    key = round(used, 1)
                    if key in seen:
                        continue
                    seen.add(key)
                    hit = self._sweep_hit([path], p.rescue_margin, step=2, end=True, isl_m=p.rescue_island_margin)
                    cands.append((f'S자 {used:.0f}°', [path], hit))
                for shape, segs, hit in cands:
                    if hit is not None:
                        continue
                    segs = list(segs)
                    if lead is not None and a > 0:
                        segs = [self._join(lead, segs[0])] + segs[1:]
                    elif lead is not None:
                        segs = [lead] + segs
                    how = ('그 자리' if abs(a) <= 0.005 else f'{100 * a:.0f} cm 더 가서' if a > 0 else f'{-100 * a:.0f} cm 물러나서')
                    return {'d': d, 'shift': a, 'shape': shape, 'segs': segs,
                            'why': f'다른 길: {how} {shape}로 차로 {cur + d}'}
        return None

    def _begin_rescue(self, r: dict, t: float):
        p = self.p
        x, y, psi = self.pose()
        segs = r['segs']
        if not p.replay_reverse and any(g.dir < 0 for g in segs) and r['shift'] >= 0:
            segs = [g for g in segs if g.dir > 0] or segs
        self.track = Track(segs, p, p.exec_speed, p.exec_rev_speed)
        self.target = self.lane + r['d']
        self.cam_ids.clear()
        self.state = 'EXEC'
        self.n_exec += 1
        self.n_rescue = getattr(self, 'n_rescue', 0) + 1
        self.why = r['why']
        self.blocked_since = None
        self.persist.clear()
        self.events.append(f'[x={x:.2f}] 막힘 -> {r["why"]} (옆 오차 {100 * (y - self.center(self.lane)):+.1f} cm, '
                           f'방향 {math.degrees(psi):+.1f}°)')

    def _try_rescue(self, t: float) -> Optional[Tuple[float, float]]:
        """막힘이 연속 reverse_confirm_frames 스캔이면 (rescue_period_s마다) 다른 길을 찾아 바로 시작. 시작했으면 그 틱 명령.
        한 틱이 길어지지 않게 '앞으로'와 '물러나서' 찾기는 다른 틱에 나눠 함 (서 있는 동안이라 늦어도 됨)."""
        p = self.p
        if not p.rescue or not self._persist('rescue', True):
            return None
        phase = getattr(self, 'rescue_phase', 'fwd')
        if phase == 'fwd' and t - getattr(self, 'rescue_t', -1e9) < p.rescue_period_s:
            return None
        if phase == 'fwd':
            self.rescue_t = t
            r = self.rescue_search(allow_back=False)
            if r is None and p.rescue_back:
                self.rescue_phase = 'back'                  # 다음 틱에 물러나서 찾기
                return None
        else:
            self.rescue_phase = 'fwd'
            r = self.rescue_search(allow_back=True, fwd=False)
        if r is None:
            if not getattr(self, '_rescue_none_logged', False):
                self._rescue_none_logged = True
                x, _, _ = self.pose()
                self.events.append(f'[x={x:.2f}] 다른 길도 없음 (앞 {100 * p.rescue_fwd_max:.0f} cm ~ 뒤 '
                                   f'{100 * p.rescue_back_max if p.rescue_back else 0:.0f} cm, 녹화·S자 '
                                   f'{p.rescue_deg_min:.0f}~{p.rescue_deg_max:.0f}°) -> 서서 {p.rescue_period_s:.1f}초마다 다시')
            return None
        self._rescue_none_logged = False
        self._begin_rescue(r, t)
        x, y, psi = self.pose()
        return self.track.step(x, y, psi, t, self._still(t))

    def check_paths(self, obs_len: float = 0.32, obs_half_w: float = 0.115) -> Dict[int, bool]:
        """녹화 경로가 '피하려는 장애물'(지금 차로, 라이다 trigger_mm 앞 정차 LIMO)을 스스로 비켜 가나. {+1: 왼쪽 됨, -1: 오른쪽 됨}.
        안 되면 실차에서 늘 '옆 차로 막힘'으로 서 있게 됨 (녹화 파일이 없어 기본 S자를 쓰거나, 녹화 시작 거리 != trigger_mm)"""
        p = self.p
        keep_pts, keep_lane = self.pts, self.lane
        x, y, _ = self.pose()
        yc = self.center(self.lane)
        xo = x + p.lidar_x + p.trigger_mm / 1000.0
        box = [(xo + obs_len * i / 16, yc + sgn * obs_half_w) for i in range(17) for sgn in (-1, 1)]
        box += [(xo, yc - obs_half_w + 2 * obs_half_w * j / 12) for j in range(13)]
        self.pts = [(bx, by, 0.0, 0.0) for bx, by in box]
        try:
            return {d: self.path_hit(d, x) is None for d in (+1, -1)}
        finally:
            self.pts, self.lane = keep_pts, keep_lane

    def aligned(self) -> bool:
        x, y, psi = self.pose()
        return (abs(y - self.center(self.lane)) <= self.p.align_tol_y
                and abs(math.degrees(psi)) <= self.p.align_tol_psi_deg)

    # ---------- 한 주기 ----------
    def step(self, t: float) -> Tuple[float, float]:
        """반환 (v, 곡률). 상태 IDLE/DONE이면 (0, 0)."""
        v, k = self._step(t)
        v, k = self._safety(v, k)
        if self.stop_why and self.state == 'EXEC':
            if self.safety_since is None:
                self.safety_since = t
                self.events.append(f'재생 중 정지: {self.stop_why}')
            elif t - self.safety_since > self.p.safety_giveup_s:
                x, y, _ = self.pose()
                self.events.append(f'[x={x:.2f}] {self.p.safety_giveup_s:.0f}초 넘게 막힘 -> 재생 그만, 다시 판단')
                self.lane = self.target = self.nearest_lane(y)
                self.track = None
                self.state = 'STOP'
                if self.m2_run is not None:          # M2 재생 중이면 다시 하지 않고 끝 -> FSM 차선 주행
                    self.target = self.lane = 1 if self.m2_run[1] == 'A' else -1
                    self.state = 'DONE'
                self.safety_since = None
        else:
            self.safety_since = None
        self.last_v = v
        return v, k

    def _still(self, t) -> bool:
        return self.fu.is_still(self.fu.t_odom if self.fu.t_odom is not None else t)

    def _safety(self, v, k):
        p = self.p
        self.stop_why = None
        if not p.safety_stop or v == 0.0 or self.state in ('IDLE', 'DONE'):
            return v, k
        m = p.exec_stop_margin if self.state == 'EXEC' else p.stop_margin
        hw = p.robot_half_width + m
        for _, _, px, py in list(self.pts) + list(self.sign_pts):     # 표지판도 닿기 직전엔 섬
            if abs(py) > hw:
                continue
            if v > 0 and -0.02 <= px - p.robot_front <= m + p.brake_k * abs(v):
                self.stop_why = f'앞 {100 * (px - p.robot_front):.0f} cm에 물체'
                return 0.0, 0.0
            if v < 0 and -0.02 <= -px - p.robot_rear <= m + p.brake_k * abs(v):
                self.stop_why = f'뒤 {100 * (-px - p.robot_rear):.0f} cm에 물체'
                return 0.0, 0.0
        return v, k

    def _cruise_cmd(self, t):
        p = self.p
        ln = self.lane_msg
        if (p.cruise_lane and ln and self.lane_t is not None and t - self.lane_t <= p.cruise_lane_fresh_s
                and ln.get('status') in ('BOTH', 'L', 'R') and not ln.get('low_conf')
                and ln.get('kappa') is not None and ln.get('source') in (None, 'lane', 'memory')):
            # 11/17: 카메라 차로 번호가 내부 추정과 달라도 차선 곡률을 그대로 씀 (차선 주행 100% 신뢰)
            self.steer_src = 'lane'                       # 차선 기반: 차선 노드 곡률 그대로
            return p.speed, max(-MAX_CURV, min(MAX_CURV, float(ln['kappa'])))
        self.steer_src = 'pose'
        x, y, psi = self.pose()
        tr = Track([_straight(self.center(self.lane), x - 0.5, x + 3.0)], self.p, self.p.speed, self.p.speed)
        return tr.step(x, y, psi, t, True)

    def _begin_exec(self, d: int, xo: Optional[float], t, why: str):
        p = self.p
        x, y, psi = self.pose()
        x0 = self._x0(xo)
        segs = self._anchored(d, x0)
        if not p.replay_reverse:
            segs = [g for g in segs if g.dir > 0] or segs           # 녹화 안 후진 구간 빼기 (전진만)
        self.track = Track(segs, p, p.exec_speed, p.exec_rev_speed)
        self.target = self.lane + d
        self.cam_ids.clear()
        self.state = 'EXEC'
        self.n_exec += 1
        self.why = why
        self.events.append(f'[x={x:.2f}] 차로 {self.lane} -> {self.target} 재생 시작 ({why}, '
                           f'옆 오차 {100 * (y - self.center(self.lane)):+.1f} cm, 방향 {math.degrees(psi):+.1f}°)')

    def _track_progress(self) -> float:
        """재생 진행률 0~1 (전진 구간 길이 기준)."""
        tr = self.track
        if tr is None:
            return 0.0
        tot = sum(g.length for g in tr.segs if g.dir > 0)
        if tot <= 0:
            return 1.0
        done = sum(g.length for g in tr.segs[:tr.i] if g.dir > 0)
        if tr.seg.dir > 0:
            done += max(0.0, tr.s)
        return min(1.0, done / tot)

    def _exec_yellow_check(self, t, psi) -> Optional[int]:
        """재생 중 노란선에 붙기 직전인지. 반환 +1(왼쪽 노란선) / -1(오른쪽) / None."""
        p = self.p
        if not p.exec_yellow_guard:
            return None
        ln = self.lane_msg
        if not ln or self.lane_t is None or t - self.lane_t > p.eyg_fresh_s:
            self.eyg_n = 0
            return None
        prog = self._track_progress()
        f = ln.get('yellow_f_mm')
        f = (float(f) / 1000.0) if f is not None else 0.575
        best = None
        for side, key in ((+1, 'yellow_l_mm'), (-1, 'yellow_r_mm')):
            d = ln.get(key)
            if d is None:
                continue
            a = side * psi                                   # 노란선 쪽으로 틀어진 각 (+ = 그쪽으로)
            if math.degrees(a) < p.eyg_psi_deg:
                continue
            d_ref = float(d) / 1000.0 + f * math.tan(a)      # 앞 f에서 잰 거리 -> 차 기준점 옆 거리 (선은 차로와 나란하다고 봄)
            sweep = p.eyg_front * math.sin(a)                # 앞 모서리가 그쪽으로 더 나가 있음
            lag = p.exec_speed * p.latency_s * math.sin(a)   # 반응 지연 동안 더 감
            turn = (1.0 / MAX_CURV) * (1.0 - math.cos(a))    # 최대 조향으로 방향 펴는 동안 옆 이동
            clear = d_ref - p.robot_half_width - sweep - lag - turn
            if best is None or clear < best[1]:
                best = (side, clear, a)
        if best is None:
            self.eyg_n = 0
            self.eyg_last = {'prog': prog}
            return None
        side, clear, a = best
        self.eyg_last = {'side': side, 'clear_mm': 1000.0 * clear, 'toward_deg': math.degrees(a), 'prog': prog}
        if prog < p.eyg_arm_frac or 1000.0 * clear >= p.eyg_clear_mm:
            self.eyg_n = 0
            return None
        self.eyg_n += 1
        return side if self.eyg_n >= p.eyg_frames else None

    def _begin_align(self, x_end: Optional[float], after: bool):
        x, _, _ = self.pose()
        self.align = {'cycle': 0, 'leg': 'back', 'x_start': x, 'x_end': x_end if x_end is not None else x,
                      'after': after, 'track': None}
        self.state = 'ALIGN'
        _, y, psi = self.pose()
        self.events.append(f'[x={x:.2f}] 가운데 맞추기 {"(재생 뒤)" if after else "(재생 전)"}: 옆 '
                           f'{100 * (y - self.center(self.lane)):+.1f} cm, 방향 {math.degrees(psi):+.1f}°')

    def _step(self, t) -> Tuple[float, float]:
        p = self.p
        if self.state in ('IDLE', 'DONE'):
            return 0.0, 0.0
        x, y, psi = self.pose()
        still = self._still(t)

        if self.state == 'STOP':
            if self.stop_since is None:
                self.stop_since = t
            if still or t - self.stop_since >= p.still_max_s:
                self.stop_since = None
                if self.pending is not None:
                    d, xo = self.pending
                    x0 = xo - p.lidar_x - p.trigger_mm / 1000.0
                    if not p.no_reverse and ((p.align_before and not self.aligned()) or x - x0 > 0.05):
                        # 가운데가 아니거나 녹화 시작 거리보다 가까움 -> 앞뒤로 움직여 맞추며 x0로
                        self._begin_align(x0, after=False)
                    else:
                        self.pending = None
                        self._begin_exec(d, xo, t, self.why)
                else:
                    self.state = 'CRUISE'
            return 0.0, 0.0

        if self.state == 'EXEC':
            v, k = self.track.step(x, y, psi, t, still)
            if self.track.off_track() and self.m2_run is not None:
                self.events.append(f'[x={x:.2f}] M2 재생 경로에서 벗어남 (옆 {100 * self.track.e:+.0f} cm) -> 끝내고 차선 주행으로')
                self.target = self.lane = 1 if self.m2_run[1] == 'A' else -1
                self.track = None
                self.state = 'DONE'
                return 0.0, 0.0
            if self.track.off_track():
                self.events.append(f'[x={x:.2f}] 재생 경로에서 벗어남 (옆 {100 * self.track.e:+.0f} cm) -> 멈추고 지금 차로에서 다시 판단')
                self.lane = self.target = self.nearest_lane(y)
                self.track = None
                self.state = 'STOP'
                return 0.0, 0.0
            if self.m2_run is None and not self.track.done:
                side = self._exec_yellow_check(t, psi)
                if side is not None:
                    self.lane = self.target
                    tr = self.track
                    ev = self.eyg_last or {}
                    self.events.append(f'[x={x:.2f}] 노란선 안전장치 ({"왼쪽" if side > 0 else "오른쪽"}): 여유 '
                                       f'{ev.get("clear_mm", 0):.0f} mm, 방향 {ev.get("toward_deg", 0):+.1f}°, 진행 '
                                       f'{100 * ev.get("prog", 0):.0f}% -> 재생 끝, 차로 {self.lane} 차선 주행')
                    self.track = None
                    self.cam_ids.clear()
                    self.eyg_hits += 1
                    self.state = 'CRUISE'
                    return max(v, 0.0), -side * MAX_CURV
            if self.track.done and self.m2_run is not None:
                k0, f = self.m2_run
                tr = self.track
                self.target = self.lane = 1 if f == 'A' else -1     # 갈래 A/B -> m7_exit -> FSM M2가 그 갈래 차선 주행
                self.track = None
                self.state = 'DONE'
                self.events.append(f'[x={x:.2f}] M2 갈림길 재생 끝 ({m2_name(k0, f)} -> 갈래 {f}, 최대 옆 오차 '
                                   f'{100 * tr.max_e:.1f} cm) -> 차선 주행으로')
                return 0.0, 0.0
            if self.track.done:
                self.lane = self.target
                self.cam_ids.clear()
                tr = self.track
                self.events.append(f'[x={x:.2f}] 재생 끝 -> 차로 {self.lane} (최대 옆 오차 {100 * tr.max_e:.1f} cm, '
                                   f'조향 한계 {100 * tr.n_sat / max(1, tr.n):.0f}%)')
                self.track = None
                if p.align_after and not p.no_reverse and not self.aligned():
                    self._begin_align(None, after=True)
                else:
                    self.state = 'CRUISE'
                return 0.0, 0.0
            return v, k

        if self.state == 'BACKUP':
            return self._backup_step(t, x, y, psi, still)

        if self.state == 'ALIGN':
            return self._align_step(t, x, y, psi, still)

        # ---- CRUISE ----
        self.lane = self.nearest_lane(y) if abs(y - self.center(self.lane)) > 0.6 * p.lane_width else self.lane
        island = p.section_length
        exit_ok = self.lane != 0 or p.num_side_lanes == 0
        if x >= island + p.done_margin and exit_ok:
            self.state = 'DONE'
            self.events.append(f'[x={x:.2f}] 끝: 차로 {self.lane}')
            return 0.0, 0.0
        # 1b. M2 갈림길 재생: M7 끝 지점에 오면 (지금 차로, A/B) 녹화 하나 재생 -> 갈래 안에서 끝
        if self.m2_possible(x):
            fork = 'A' if self.exit_pref == 1 else ('B' if self.exit_pref == -1 else None)
            x_m2 = self.m2_x()
            hit = None
            if x >= x_m2 - p.speed * p.latency_s:
                key0 = (self.lane, fork or p.m2_fork_default)
                hit = self.m2_hit(key0, max(x_m2, x)) if key0 in self.m2 else None
                if self._persist('m2_backup', hit is not None and p.m2_backup and not self.m2_backed and key0 in self.m2
                                 and x > x_m2 + 0.03 and x - x_m2 <= p.m2_backup_max
                                 and self.m2_hit(key0, x_m2) is None):
                    # 라이다 점 하나 튄 것으로 후진하지 않게 연속 reverse_confirm_frames 스캔 확인 (그동안은 아래처럼 장애물 먼저)
                    return self._begin_m2_backup(x, x_m2, f'M2 녹화 길이 지금 자리에선 물체에 닿음 ({hit[0] - x:+.2f} m)')
                if hit is not None and not getattr(self, '_m2_hit_logged', False):
                    self._m2_hit_logged = True
                    self.events.append(f'[x={x:.2f}] M2 녹화 길에 물체 ({hit[0] - x:+.2f} m) -> 장애물 먼저 (늦어도 '
                                       f'{100 * p.m2_late_max:.0f} cm까지는 그 자리에서 M2 시작)')
            if x >= x_m2 - p.speed * p.latency_s and hit is None:
                why_f = '표지' if fork else f'표지 못 봄 -> 기본 {p.m2_fork_default}'
                fork = fork or p.m2_fork_default
                key = (self.lane, fork)
                if key in self.m2:
                    x0 = max(x_m2, x)
                    segs = anchor(self.m2[key], x0, self.center(self.lane))
                    self.track = Track(segs, p, p.exec_speed, p.exec_rev_speed)
                    self.m2_run = key
                    self.state = 'EXEC'
                    self.n_exec += 1
                    self.why = f'M2 {m2_name(*key)} ({why_f})'
                    self.events.append(f'[x={x:.2f}] M7 끝 지점 -> M2 갈림길 재생 {self.m2_src.get(key, m2_name(*key))} '
                                       f'(차로 {M2_LANE_NO[self.lane]}, 갈래 {fork}: {why_f}, 옆 오차 '
                                       f'{100 * (y - self.center(self.lane)):+.1f} cm, 방향 {math.degrees(psi):+.1f}°)')
                    return self.track.step(x, y, psi, t, still)
                if not getattr(self, '_m2_missing_logged', False):
                    self._m2_missing_logged = True
                    self.events.append(f'[x={x:.2f}] M2 녹화 {m2_name(*key)} 없음 -> 예전 방식(바깥 차로로 옮겨 섬 지나 끝)')
        # 2. 섬 앞 가운데 차로 -> 정지
        if self.lane == 0 and x >= island - p.island_stop - p.robot_front:
            d, why = self.choose(self.lane, None)
            if d is not None:
                self._begin_exec(d, None, t, '섬 앞: ' + why)
                return 0.0, 0.0
            r = self._try_m2_backup(x, '섬 앞 가운데 차로에서 옮길 곳 없음')
            if r is not None:
                return r
            r = self._try_rescue(t)
            if r is not None:
                return r
            self.why = '섬 앞 가운데 차로: 옮길 곳 없음 -> 정지'
            return 0.0, 0.0
        v_now = p.speed
        # 미리 계산 (조향에는 안 씀): 지금 차로 앞 장애물과 그때 빠질 차로
        if p.plan_ahead and t - self.plan_t >= p.plan_period_s:
            self.plan_t = t
            g0 = self.gap(self.lane)
            if g0 is None:
                self.plan = {'obstacle_mm': None, 'to_lane': None, 'why': '지금 차로 앞 장애물 없음'}
            else:
                d0, why0 = self.choose(self.lane, self.obs[self.lane])
                self.plan = {'obstacle_mm': round(1000 * g0), 'to_lane': None if d0 is None else self.lane + d0,
                             'to_lane_no': None if d0 is None else 2 - (self.lane + d0),
                             'start_in_mm': round(1000 * max(0.0, g0 - p.trigger_mm / 1000.0)), 'why': why0}
        # 4. 앞 장애물
        g = self.gap(self.lane)
        if g is not None and p.backup_if_mm > 0 and g < p.backup_if_mm / 1000.0:
            # 차로 변경을 못 했는데(막힘·재생 중단) 장애물이 너무 가까움 -> 차로 가운데 맞추며 후진 -> 다시 판단
            self._begin_backup(x, g)
            return 0.0, 0.0
        if g is not None:
            # 바로 재생: 이미 가운데이고 녹화 시작 거리보다 많이 가깝지 않을 때 (가까우면 서서 정렬하며 뒤로 물러남)
            ok = p.no_reverse or (p.align_skip_if_ok and (self.aligned() or not p.align_before)
                                  and g >= p.trigger_mm / 1000.0 - 0.05)
            lead = v_now * p.latency_s + (0.0 if ok else p.brake_k * v_now)
            if g <= p.trigger_mm / 1000.0 + lead:
                d, why = self.choose(self.lane, self.obs[self.lane])
                to_b = (p.backup_to_mm or p.trigger_mm) / 1000.0
                if d is not None:
                    self.persist.pop('rescue', None)
                    self._rescue_none_logged = False
                else:
                    r = self._try_rescue(t)                  # 녹화 경로가 양쪽 다 닿음 -> (막힘 연속 확인 뒤) 다른 길
                    if r is not None:
                        return r
                if self._persist('blocked_backup', d is None and p.backup_when_blocked and p.backup_if_mm > 0
                                 and g < to_b - 0.02):
                    # 46~50 cm인데 지금 자세로는 경로가 닿음 -> 50 cm까지 물러나 가운데 맞추고 다시
                    #   (경로 검사는 이번 스캔 라이다 점이라 연속 reverse_confirm_frames 스캔 막혀야 후진. 그동안은 아래에서 정지)
                    self.events.append(f'[x={x:.2f}] 앞 {1000 * g:.0f} mm, 지금 자세로는 막힘 ({why}) -> 후진')
                    self._begin_backup(x, g)
                    return 0.0, 0.0
                if d is None:
                    r = self._try_m2_backup(x, f'섬 앞에서 막힘 ({why})')
                    if r is not None:
                        return r
                    if self.blocked_since is None:
                        self.blocked_since = t
                        self.events.append(f'[x={x:.2f}] 앞 막힘 ({why}) -> 정지하고 기다림')
                    if t - self.blocked_since >= p.blocked_wait_s and not self.backed_off and not p.no_reverse:
                        # 오래 막힘: 한 번만 align_back만큼 물러나서 다시 봄 (가려졌던 것이 보이거나 시작점이 바뀜)
                        self.backed_off = True
                        self.blocked_since = None
                        self.events.append(f'[x={x:.2f}] 오래 막힘 -> {p.align_back:.2f} m 물러나서 다시 판단')
                        self._begin_align(x, after=True)
                        self.align['cycle'] = p.align_cycles - 1
                        return 0.0, 0.0
                    self.why = why + (' (물러나 봐도 막힘: 정지 유지)' if self.backed_off
                                      else ' (서서 다시 봄)' if p.no_reverse else '')
                    return 0.0, 0.0
                pref_back = (getattr(self, 'pref_needs_backup', False) and p.backup_when_blocked and p.backup_if_mm > 0
                             and g < to_b - 0.02 and not getattr(self, 'backed_for_pref', False))
                if pref_back and not self._persist('pref_backup', True):
                    self.why = f'출구 {self._exit_name()} 쪽 막힘 확인 중 (후진 전 연속 {p.reverse_confirm_frames}스캔)'
                    return 0.0, 0.0                  # 확인될 때까지 서서 봄 (반대쪽으로 바로 가지 않음)
                if not pref_back:
                    self.persist.pop('pref_backup', None)
                if pref_back:
                    self.backed_for_pref = True
                    self.events.append(f'[x={x:.2f}] 출구 {self._exit_name()} 쪽은 지금 자리에선 닿음 -> 후진해서 출구 쪽으로')
                    self._begin_backup(x, g)
                    return 0.0, 0.0
                self.backed_for_pref = False
                self.blocked_since = None
                self.backed_off = False
                self.why = why
                if ok:
                    self._begin_exec(d, self.obs[self.lane], t, why)
                    return self.track.step(x, y, psi, t, still)
                self.pending = (d, self.obs[self.lane])
                self.state = 'STOP'
                self.events.append(f'[x={x:.2f}] 앞 장애물 {1000 * g:.0f} mm -> {d:+d}쪽 ({why}), 서서 가운데 맞춤')
                return 0.0, 0.0
        self.blocked_since = None
        # 5. 출구 차로 (섬 앞 끝 전에 바깥 차로로). 가운데 차로면 출구 쪽 먼저, 안 되면 반대쪽.
        #    반대쪽 바깥 차로면 가운데로 한 칸 (두 칸 거리 전부터), 안 되면 그 차로로 끝 (FSM이 그 갈래를 따름)
        L = self._change_len(1)
        want = self.exit_pref
        if self.m2_possible(x) and any((self.lane, f) in self.m2 for f in ('A', 'B')):
            return self._cruise_cmd(t)               # 출구는 M2 녹화가 맡음 (차로 그대로 M7 끝 지점까지)
        if self.lane == 0 and p.num_side_lanes > 0 and x >= island - L - p.exit_lead:
            order = ([1 if want > 0 else -1] if want else []) + [d for d in (1, -1) if not want or d != (1 if want > 0 else -1)]
            for d in order:
                if self.path_hit(d, x) is None:
                    self._begin_exec(d, None, t, '섬 앞 바깥 차로로' + (' (출구 쪽)' if want and d == want else ''))
                    return self.track.step(x, y, psi, t, still)
        elif want and self.lane == -want and x >= island - 2 * L - p.exit_lead and x < island - L - p.exit_lead + 0.2:
            d = 1 if want > 0 else -1
            if self.path_hit(d, x) is None and self.path_hit(d, x + L) is None:
                self._begin_exec(d, None, t, f'출구 {self._exit_name()} 쪽으로 (두 칸 중 첫 칸)')
                return self.track.step(x, y, psi, t, still)
        return self._cruise_cmd(t)

    def _try_m2_backup(self, x, why):
        """섬 앞(M2 시작 자리를 지나침)에서 막혔을 때: M2 녹화 시작 자리까지 후진해서 녹화대로 돌 수 있으면 후진."""
        p = self.p
        if not (p.m2_backup and self.m2_on() and not self.m2_backed):
            return None
        x_m2 = self.m2_x()
        fork = 'A' if self.exit_pref == 1 else ('B' if self.exit_pref == -1 else p.m2_fork_default)
        key = (self.lane, fork)
        if key not in self.m2 or not (x_m2 + 0.03 < x <= x_m2 + p.m2_late_max + p.m2_backup_max):
            return None
        if self.m2_hit(key, x_m2) is not None:
            return None
        if not self._persist('m2_try', True):         # 막힘(라이다 점)이 연속 reverse_confirm_frames 스캔일 때만 후진, 그동안 정지
            return None
        return self._begin_m2_backup(x, x_m2, why)

    def _begin_m2_backup(self, x, x_m2, why):
        p = self.p
        self.m2_backed = True
        self._begin_backup(x, None, to_x=x_m2, max_m=p.m2_backup_max + p.m2_late_max)
        self.events.append(f'[x={x:.2f}] {why} -> M2 녹화 시작 자리까지 {x - x_m2:.2f} m 후진해서 갈림길 녹화대로')
        return 0.0, 0.0

    def _begin_backup(self, x, g, to_x=None, max_m=None):
        p = self.p
        xb = x - (p.backup_max if max_m is None else max_m)
        n = 31
        line = Path([x - (x - xb) * i / (n - 1) for i in range(n)], [self.center(self.lane)] * n, [0.0] * n, -1)
        self.backup = {'x0': x, 'track': Track([line], p, p.backup_speed, p.backup_speed, p.backup_k_e, p.backup_k_psi),
                       'stopping': False, 'to_x': to_x, 'max': (p.backup_max if max_m is None else max_m)}
        self.state = 'BACKUP'
        self.blocked_since = None
        if to_x is not None:
            return
        to = (p.backup_to_mm or p.trigger_mm)
        self.events.append(f'[x={x:.2f}] 장애물 {1000 * g:.0f} mm (< {p.backup_if_mm:.0f}) -> {to:.0f} mm까지 후진 '
                           f'(차로 {M2_LANE_NO.get(self.lane)} 가운데 맞추며)')

    def _backup_step(self, t, x, y, psi, still):
        p = self.p
        b = self.backup
        g = self.gap(self.lane)
        to = (p.backup_to_mm or p.trigger_mm) / 1000.0
        if b.get('to_x') is not None:          # M2 시작 자리까지 (앞 장애물 거리 대신 위치)
            done = x <= b['to_x'] + p.brake_k * p.backup_speed or b['track'].done or b['x0'] - x >= b['max']
        else:
            done = (g is None or g >= to - p.brake_k * p.backup_speed or b['track'].done
                    or b['x0'] - x >= p.backup_max)
        if done or b['stopping']:
            if b['stopping'] is False:
                b['stopping'] = t
            if still or t - b['stopping'] >= p.still_max_s:
                self.events.append(f'[x={x:.2f}] 후진 끝: 앞 {"없음" if g is None else f"{1000 * g:.0f} mm"}, '
                                   f'옆 {100 * (y - self.center(self.lane)):+.1f} cm, 방향 {math.degrees(psi):+.1f}°, 카메라로 맞춘 틱 {b.get("cam", 0)} -> 다시 판단')
                self.backup = None
                self.state = 'CRUISE'
            return 0.0, 0.0
        v, k = b['track'].step(x, y, psi, t, still)
        kc = self._backup_lane_kappa(t) if v < 0 else None
        if kc is not None:
            b['cam'] = b.get('cam', 0) + 1
            return v, kc
        return v, k

    def _backup_lane_kappa(self, t) -> Optional[float]:
        """후진 중 차선 노드 offset/heading으로 차로 가운데 맞추기 곡률 (Track 후진과 같은 식). 못 쓰면 None."""
        p = self.p
        ln = self.lane_msg
        if (not p.backup_lane_center or not ln or self.lane_t is None or t - self.lane_t > p.backup_lane_fresh_s
                or ln.get('status') != 'BOTH' or ln.get('low_conf')
                or ln.get('offset_mm') is None or ln.get('heading_rad') is None):
            return None
        h = float(ln['heading_rad'])
        if abs(math.degrees(h)) > p.cam_max_deg:
            return None
        offm = float(ln['offset_mm']) / 1000.0
        e = -offm * math.cos(h) + p.cam_row * math.sin(h)      # 차 기준 왼쪽 + 옆 오차 (on_lane의 y_meas와 같은 식)
        epsi = -h
        if abs(e) > p.cam_gate + 0.05:
            return None
        curv = -p.backup_k_e * e + p.backup_k_psi * math.sin(epsi)   # Track 후진(d = -1): -k_e e + k_psi sin(epsi)
        return max(-MAX_CURV, min(MAX_CURV, curv))

    def _align_step(self, t, x, y, psi, still):
        p = self.p
        a = self.align
        if a['track'] is None:
            if not still:
                return 0.0, 0.0
            if a['leg'] == 'back':
                if self.aligned() and a['cycle'] > 0 or (self.aligned() and a['after']):
                    return self._align_done(t)
                if a['cycle'] >= p.align_cycles:
                    self.events.append(f'[x={x:.2f}] 가운데 맞추기 {p.align_cycles}번 했는데 아직 '
                                       f'옆 {100 * (y - self.center(self.lane)):+.1f} cm -> 그대로 진행')
                    return self._align_done(t)
                xb = min(x, a['x_end']) - p.align_back
                a['track'] = Track([Path([x - (x - xb) * i / 30 for i in range(31)], [self.center(self.lane)] * 31,
                                         [0.0] * 31, -1)], p, p.align_speed, p.align_speed, p.align_k_e, p.align_k_psi)
            else:
                xe = a['x_end']
                if xe - x < 0.03:
                    a['leg'] = 'back'
                    a['cycle'] += 1
                    return 0.0, 0.0
                a['track'] = Track([_straight(self.center(self.lane), x, xe)], p, p.align_speed, p.align_speed,
                                   p.align_k_e, p.align_k_psi)
        v, k = a['track'].step(x, y, psi, t, still)
        if a['track'].done:
            a['track'] = None
            if a['leg'] == 'back':
                a['leg'] = 'fwd'
            else:
                a['leg'] = 'back'
                a['cycle'] += 1
            return 0.0, 0.0
        return v, k

    def _align_done(self, t):
        x, y, psi = self.pose()
        a = self.align
        self.align = None
        self.events.append(f'[x={x:.2f}] 가운데 맞추기 끝: 옆 {100 * (y - self.center(self.lane)):+.1f} cm, '
                           f'방향 {math.degrees(psi):+.1f}° ({a["cycle"]}번)')
        if self.pending is not None and not a['after']:
            d, xo = self.pending
            self.pending = None
            self._begin_exec(d, xo, t, self.why)
        else:
            self.state = 'CRUISE'
        return 0.0, 0.0

    def _cam_lane_no(self) -> Optional[int]:
        """카메라(차선 노드 lane_id: 노란 실선·흰 점선으로 구분) 최근 값 중 과반인 차로 번호 1·2·3, 없으면 None"""
        ids = [c for c in self.cam_ids if c is not None]
        if not ids:
            return None
        best = max((-1, 0, 1), key=ids.count)
        return 2 - best if ids.count(best) * 2 > len(self.cam_ids) else None

    # ---------- 상태 ----------
    def status(self) -> dict:
        x, y, psi = self.pose()
        return {'state': 'RUN' if self.state not in ('IDLE', 'DONE') else self.state,
                'phase': self.state, 'x': round(x, 3), 'y': round(y, 3), 'psi_deg': round(math.degrees(psi), 1),
                'now_lane': self.lane, 'now_lane_no': M2_LANE_NO.get(self.lane), 'target': self.target,
                'exit_pref': self.exit_pref,
                'm2': (None if not self.m2_on() else
                       {'run': None if self.m2_run is None else m2_name(*self.m2_run),
                        'start_in_m': round(self.m2_x() - x, 2), 'files': len(self.m2)}),
                # 11/17: 지금 몇 차선 (1 = 왼쪽 A쪽, 2 = 가운데, 3 = 오른쪽 B쪽), 카메라 선 종류로 본 차로와 같은지
                'lane_no': 2 - self.lane, 'cam_lane_no': self._cam_lane_no(),
                'lane_agree': (None if self._cam_lane_no() is None else self._cam_lane_no() == 2 - self.lane),
                'steer': self.steer_src if self.state == 'CRUISE' else self.state.lower(), 'plan': self.plan,
                'gaps_mm': {str(k): (None if self.gap(k) is None else round(1000 * self.gap(k))) for k in self.obs},
                'why': self.why, 'stop': self.stop_why, 'n_exec': self.n_exec, 'cam_fix_n': self.cam_n,
                'n_rescue': getattr(self, 'n_rescue', 0),
                'gaps_raw_mm': {str(k): (None if v is None else round(1000 * (v - x - self.p.lidar_x)))
                                for k, v in self.obs_raw.items()},
                'acc_dir': self.acc.state(),
                'backup': None if not getattr(self, 'backup', None) else round(self.backup['x0'] - x, 2),
                'align': None if not self.align else f'{self.align["leg"]}{self.align["cycle"]}',
                'lane_why': {'reason': self.why},
                'eyg': (None if not self.eyg_last else {k: (round(v, 1) if isinstance(v, float) else v)
                                                        for k, v in self.eyg_last.items()}),
                'eyg_hits': self.eyg_hits}
