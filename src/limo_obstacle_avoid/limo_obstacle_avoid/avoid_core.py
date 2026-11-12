"""
avoid_core.py  —  3차선 정적 장애물(정차 LIMO) 회피 핵심 로직

★ ROS를 전혀 쓰지 않는 순수 파이썬 파일입니다.
  → ROS 노드(s2~s6)도 이 파일을 쓰고,
  → 집에서 LIMO 없이 돌리는 tools/offline_sim.py 도 이 파일을 씁니다.

[좌표계: "미션 로컬 좌표"]
  미션 시작(enable) 순간의 로봇 위치를 원점, 그때의 진행 방향을 +x 로 잡습니다.
  y 는 왼쪽이 +.  (3차선 구간은 직선이라 이 좌표 하나로 충분)
  GPS 없이 '시작점으로부터 얼마나 왔나'만 오도메트리(거리) + IMU(방향)로 계산합니다.

[차선 번호]  (진행 방향 기준)
  +1 = 왼쪽 차선 (도면 A 쪽)
   0 = 가운데 차선 (진입 차선)
  -1 = 오른쪽 차선 (도면 B 쪽)
  차선 중심 y = 차선번호 × lane_width
"""
import math
from dataclasses import dataclass

from limo_obstacle_avoid.fork_exit import shift_distance, lane_name

INF = float('inf')


def wrap(a):
    """각도를 -pi ~ +pi 로"""
    return math.atan2(math.sin(a), math.cos(a))


def parse_exit(s):
    """'A' → +1(왼쪽), 'B' → -1(오른쪽), 그 외 → None(자동)"""
    s = str(s).strip().upper()
    if s in ('A', 'LEFT', 'L', '+1', '1'):
        return 1
    if s in ('B', 'RIGHT', 'R', '-1'):
        return -1
    return None


def _with_lookahead(p, ld):
    """shift_distance 계산용: lookahead만 바꾼 얕은 복사."""
    import copy
    q = copy.copy(p)
    q.lookahead = ld
    return q


# ──────────────────────────────────────────────────────────────
# 파라미터 (config/avoid_params.yaml 에서 덮어씀)
# ──────────────────────────────────────────────────────────────
@dataclass
class AvoidParams:
    # ── 트랙 ──
    lane_width: float = 0.52          # 이웃 차로 중심 간격 (도면: 선 사이 500 + 점선 20 mm)
    num_side_lanes: int = 1           # 가운데 기준 양옆 차선 수 → 3차선
    start_lane: int = 0               # enable 순간 로봇이 있는 차선
    start_heading_deg: float = 0.0    # enable 순간 차가 3차선 직선 방향에서 틀어진 각 (왼쪽 +). 팀 스택: obstacle_node가
                                      #   IMU로 '코너 진입 방향 - 90°'를 직선으로 보고 켤 때마다 넣음 (코너 끝에서 아직 덜 펴져도 기준이 맞음)
    section_length: float = 4.3       # enable 지점 ~ 3차선 끝(가운데 섬 시작)까지 거리 [실측!]
    island_length: float = 1.0        # 가운데 섬(육각형) 길이 → 가상 장애물로 넣음
    use_island: bool = True           # 섬을 가상 장애물로 넣을지
    exit_lane: str = 'auto'           # 'auto' | 'A'(왼쪽 +1) | 'B'(오른쪽 -1)
    done_margin: float = 0.3          # 섬 시작 + 이만큼 지나고 차선에 붙으면 완료
    done_max_psi_deg: float = 8.0     # 11/12 밤: 끝낼 때 차가 이보다 비스듬하면 조금 더 가며 바로잡고 끝냄 (차선 노드가 비스듬한 채 받아 차선을 벗어나던 것)
    # ── 차량 (LIMO PRO) ──
    robot_front: float = 0.20         # base_link → 앞 범퍼
    robot_rear: float = 0.16          # base_link → 뒤 범퍼
    robot_half_width: float = 0.12    # 차폭/2 (+약간)
    side_margin: float = 0.05         # 옆 여유
    # ── 라이다 ──
    lidar_x: float = 0.10             # base_link 기준 라이다 앞쪽 위치
    scan_yaw_offset_deg: float = 0.0  # 라이다 0도가 정면이 아니면 보정 (s1에서 확인)
    scan_flip: bool = False           # 각도 방향이 반대면 True (s1에서 확인)
    range_min: float = 0.10
    range_max: float = 3.0
    roi_front: float = 2.5            # 앞으로 이만큼까지만 봄
    roi_back: float = 0.6             # 뒤로 이만큼까지 (옆 차 지나가는 중 확인용)
    roi_side_extra: float = 0.10      # 3차선 바깥으로 이만큼만 더 봄 (트랙 밖 사람 무시)
    obs_max_center_y: float = 0.67    # 11/12 밤: 장애물 덩어리 중심이 차로 중심선에서 이보다 밖이면 트랙 밖 물체로 보고 버림 [m]
    cluster_gap: float = 0.12         # 점 사이가 이보다 멀면 다른 물체
    cluster_min_pts: int = 3          # 점이 이보다 적으면 노이즈
    merge_dist: float = 0.20          # 같은 장애물로 합치는 거리
    max_obs_width: float = 0.45       # 장애물 하나의 최대 폭 (LIMO 0.22)
    max_obs_len: float = 0.70         # 한 번에 본 덩어리가 앞뒤로 이보다 길면 벽·트랙 테두리로 보고 버림 (LIMO 0.32)
    min_obs_size: float = 0.08        # 덩어리 가로·세로 중 긴 쪽이 이보다 작으면 버림 (기둥·다리. 리모 면 0.22)
    obs_min_len: float = 0.33         # 앞면만 보여도 이 길이만큼 차가 있다고 가정 (LIMO 길이)
    confirm_hits: int = 3             # 이 횟수 이상 보여야 진짜 장애물로 인정
    # 11/12 밤: 지도에 한 번 들어온 장애물은 지워지지 않았다 -> 사람 다리·손·튄 점·위치가 어긋난 채 찍힌 가짜가
    #   차로를 영원히 막아 '멈춤 엔딩'. 장애물이 있어야 할 앞면을 라이다 빔이 forget_rays개 이상 뚫고 지나가고
    #   (그 너머에서 반사) 그 스캔에 덩어리로도 안 보이면 miss+1, forget_scans번 연속이면 지운다.
    #   앞쪽 forget_min_x ~ forget_max_x 안의 것만 (옆·뒤는 빔이 비스듬해 판단 안 함)
    forget: bool = True
    forget_scans: int = 4
    forget_rays: int = 3
    forget_pass: float = 0.15         # 앞면보다 이만큼 더 멀리서 반사돼야 '뚫고 지나감' [m]
    forget_min_x: float = 0.25        # 차 기준 앞 이 거리부터 [m] (base_link 기준)
    forget_max_x: float = 2.2
    # ── 판단 ──
    trigger_dist: float = 1.3   # 내 차선 앞 장애물이 (앞범퍼 기준) 이 안이면 차선 변경. 11/12 1.0 -> 1.3 (더 일찍)
    change_len: float = 1.0           # 차선 1칸 옮기는 데 필요한 앞쪽 여유
    clear_back: float = 0.0           # 옮길 차선의 차 끝이 (뒤범퍼 - 이값)보다 뒤여야 들어감 (음수=더 일찍)
    clear_rollout: bool = True        # 11/12: 옆 장애물을 지나는 중이면 꺾어 들어가는 길을 굴려 봐서 안 부딪히면 바로 옮김
    clear_rollout_margin: float = 0.04  #   그때 차체 둘레 추가 여유 [m] (path_margin 0.02 + 이것)
    # 11/12: 바깥 노란선 지킴. 바깥 차로에서 바깥쪽으로 향해 있고 edge_look 앞에서 차체 모서리가 선(가장자리 - edge_margin)을
    #   넘을 것 같으면 안쪽으로 끝까지 꺾음 (회피 중 노란선 밖으로 나간 것, 11/12 실차)
    edge_guard: bool = True
    edge_margin: float = 0.03
    edge_look: float = 0.30
    edge_from: float = 0.85           # 차로 중심에서 바깥 차로 쪽으로 lane_width x 이만큼 넘어온 뒤에만 지킴
    better_margin: float = 0.3        # 옮길 차선이 지금보다 이만큼 더 비어야 옮김
    rechoose_dy: float = 0.12         # 차로를 옮기기 시작했어도 원래 차로 중심에서 이만큼 안이면, 새로 확정된
                                      #   장애물로 다른 차로가 더 비어 보일 때 목표를 바꿈 (켜자마자 고른 쪽이 다음 줄에서 막힘 방지)
    rechoose_sec: float = 0.6         # 위 다시 고르기는 차로를 고른 뒤 이 시간 안에 한 번만 (계속 바꾸면 왔다 갔다 함)
    # 11/12: 옮기는 도중 가던 차로 앞이 막힌 걸 알면 멈추지 않고 그 다음 차로까지 이어서 한 번에 (예: 1차선 -> 2차선 가다 -> 3차선).
    #   안전 조건: 다음 차로 앞이 cont_min_free 넘게 비고 지금 가던 차로보다 better_margin 더 비어야, 섬까지 cont_island_m 넘게,
    #   지나갈 길(path_clear) 비어야, 차가 max_heading_deg 까지 안 틀어졌어야. 한 번 옮기는 동안 한 번만.
    cont_change: bool = True
    cont_min_free: float = 1.80       # 다음 차로 앞 최소 빈 거리 [m]. 11/12 밤 1.3 -> 1.8 (두 차로 한 번에는 정말 필요할 때만)
    cont_island_m: float = 2.00       # 섬 앞 끝까지 이보다 가까우면 안 함 [m]. 11/12 밤 1.7 -> 2.0
    # 11/12 밤 실차 요청: 두 차로를 한 번에 건너는 일을 줄이고 가능하면 2차로(가운데)로 달림.
    #   고르기 점수 = 앞 빈 거리(최대 3 m) - two_lane_penalty x (두 칸이면 1) + center_bonus x (가운데면 1)
    two_lane_penalty: float = 1.2
    center_bonus: float = 0.3
    #   옆 차로(1·3)에서 장애물을 지난 뒤 가운데 앞이 center_return_free 넘게 비고(옆에 지나가는 것도 없음) 갈 길이 비면
    #   가운데로 돌아옴. 섬 앞 center_from_m 안(출구·표지 판단 구간)에서는 안 함. 차로를 바꾼 뒤 center_return_sec 지나서만
    center_return: bool = True
    center_return_free: float = 1.6
    center_return_sec: float = 1.0
    center_before_island: bool = True  # 11/12 밤: 표지를 모르면 섬 앞에서 가능하면 2차로로 모아 표지를 보게 함
    center_from_m: float = 2.6        #   섬 앞 끝까지 이 거리 안에서
    center_until_m: float = 1.6       #   이 거리까지 (그보다 가까우면 2차로로 들어가지 않음: 1차로 옮겨 나갈 거리 1.1 m + 여유)
    center_leave_m: float = 0.95      #   가운데에서 표지를 기다리다 끝내 모르면 섬 앞 이 거리에서 옆 차로로 (1차로 옮겨 나갈 최소 0.84 m + 여유)
    rollback_need_sharp: bool = True  # 후진을 멈출 '틀 공간'을 크게 꺾는 기준(recover_lookahead)으로 잼
    hold_time: float = 0.4   # 차선 바꾼 뒤 최소 유지 시간 [s]. 11/12 0.8 -> 0.4 (옮기는 중에도 다음 판단을 빨리)
    flip_block_sec: float = 1.5       # 11/12 밤: 방금 떠난 차로로 이 시간 안에는 안 돌아감 (실차 -1 <-> 0 왔다갔다)
    flip_ok_free: float = 0.5         #   지금 가는 차로 앞이 이보다 짧으면(정말 막힘) 예외로 허용 [m]
    stop_dist: float = 0.30           # 지금 조향대로 이만큼 더 가면 차체가 부딪힐 때만 정지(→ 후진). 옆 차로·옆에 붙은 장애물은 안 봄
    # 11/12: 막혔을 때 바로 서서 후진하지 말고 먼저 조향을 끝까지 꺾어 빠져 본다. 끝까지 꺾은 길로도
    #   squeeze_min_dist 안에서 부딪히면(장애물이 0.2 m까지 붙음) 그때만 서고 예전처럼 후진·탈출
    squeeze: bool = True
    squeeze_min_dist: float = 0.10    # 끝까지 꺾은 길로 이만큼 더 가도 안 부딪혀야 꺾어서 감 [m]. 11/12 밤 0.2 -> 0.1 (후진 없앰)
    squeeze_speed: float = 0.30   # 끝까지 꺾어 빠질 때 속도. 11/12 0.12 -> 0.30
    path_margin: float = 0.02         # 위 판단에서 차체 둘레 여유 [m]
    path_rollout: bool = True         # 위 판단의 길: 목표 차로로 가다 똑바로 서는 실제 조향을 굴려 봄 (false: 지금 곡률 유지)
    emerg_lane_band: bool = True      # 범퍼 앞 비상: 지금 차로~가려는 차로 안(도면 차로 경계)의 점만 셈. 그 밖 차로 장애물 무시
    emerg_min_pts: int = 2            # 비상정지 박스 안에 점이 이만큼 있어야 비상 (라이다 점 하나 튀어서 서는 것 방지)
    # ── 제어 (pure pursuit) ──
    v_cruise: float = 0.30   # 차선 안 속도. 11/12 0.25 -> 0.30 (팀 base_speed, 규정 권장 0.3~0.4의 아래쪽)
    v_change: float = 0.30   # 차선 변경 중 속도. 11/12 0.20 -> 0.30
    lookahead: float = 0.35   # pure pursuit 전방주시. 11/12 0.40 -> 0.35: 더 크게 꺾어 확 피함 (0.30은 속도 0.3과 같이 쓰면 코너 끝에서 켤 때 이탈 1/30)
    min_turn_radius: float = 0.53     # LIMO 아커만 최소 회전반경 실측값 (이보다 작게 두면 못 도는 경로를 계획)
    max_heading_deg: float = 40.0     # 이 이상 비틀어지면 더 꺾지 않음 (30으로 줄이면 붙은 배치 시험 4개 실패 -> 그대로)
    reach_tol: float = 0.08           # 차선 중심에서 이 안이면 '도착'
    # ── 갈림길(미션 2) 출구 차선 미리 이동 ──
    fork_shift: bool = True           # 갈림길 방향을 알면 섬 앞 끝 전에 출구 차선(A=+1, B=-1)으로 미리 옮김 (0/1/2차선)
    exit_margin: float = 0.20         # 섬 앞 끝까지 남겨 둘 여유 [m] (조향 지연·차체 모서리)
    exit_min_time: float = 0.6        # 시작 후 이 시간 동안은 미리 이동 안 함 (라이다 지도가 쌓일 때까지)
    # ── 정렬 보정 (정차 LIMO 중심이 차선 중심선에 오도록 내 좌표를 돌리고 옮김) ──
    lane_align: bool = True           # 출발 방향/위치 오차 보정 (라이다만 사용)
    align_gain: float = 0.5           # 한 번에 반영하는 비율 (0~1)
    align_sigma_meas: float = 0.03    # 차 중심 측정 오차 [m] (차가 차선 중앙에서 벗어나 서 있는 정도 포함)
    align_sigma_dy: float = 0.05      # 한 번 보정에서 옆 오프셋 사전 크기 [m]
    align_sigma_heading_deg: float = 2.0  # 한 번 보정에서 방향 사전 크기 [deg]
    align_max_dy: float = 0.12        # 누적 옆 보정 한계 [m]
    align_max_heading_deg: float = 6.0    # 누적 방향 보정 한계 [deg]
    align_w_min: float = 0.14         # 폭이 이 범위인 장애물만 사용 (일부만 보인 것 제외)
    align_w_max: float = 0.32
    align_min_span: float = 0.8       # 방향 보정은 앞뒤로 이만큼 떨어진 장애물들이 있을 때만 (가까우면 옆 위치만)
    align_max_psi_deg: float = 15.0   # 차가 이보다 비틀어져 있거나 후진(탈출) 중이면 정렬 보정 안 함
    # 11/12: 정렬 보정은 'LIMO가 차로 한가운데에 똑바로 서 있다'고 가정한다. 실제로 손으로 놓은 LIMO가 옆으로 10 cm쯤
    #   비켜 있으면 방향을 최대 6°까지 잘못 돌려, 3 m 앞 LIMO가 옆으로 25 cm 옮겨 보임 -> 내 차로에 없는 장애물이 생겨
    #   '갈 차로 없음'으로 앞뒤 반복 (시뮬레이션 재현). 이번 회피에서 카메라 차선 보정이 한 번이라도 됐으면 정렬 보정은 끈다
    #   (카메라가 더 정확. '최근 1초'만 쉬게 하면 차로 바꾸는 동안 정렬이 다시 틀어 카메라 문턱 밖으로 밀려남 -> 시뮬레이션에서 더 나빴음)
    align_skip_with_camera: bool = True
    # ── 카메라 차선 보정 (팀 lane_node의 /lane/status 를 그대로 받아 옆 위치·방향을 맞춤) ──
    #   차선 코드를 복사하지 않고 같은 노드의 결과를 쓰므로 차선 주행과 항상 같은 기준
    lane_fix: bool = True
    lane_fix_gain: float = 0.30       # 한 번에 옆 오차의 이만큼 반영 (차선 노드 약 20Hz). 11/10 0.15 -> 0.30
    lane_fix_heading_gain: float = 0.2   # 11/10 0.1 -> 0.2
    # 차로 번호: 차선 노드가 선 종류로 본 3차선 번호(lane_id)가 연속 lane_id_need/lane_id_window번 같고
    #   지금 추정 차로와 다르면 옆 위치를 그 차로로 바로 옮긴다 (시작 위치·방향 오차로 차로를 잘못 아는 것 방지)
    lane_id_fix: bool = True
    lane_id_window: int = 7
    lane_id_need: int = 5
    # 차로 안에서 똑바로 갈 때 조향 = (1-a) x 회피 pure pursuit + a x 차선 노드 곡률 (양쪽 선 보임 + 같은 차로일 때만)
    lane_keep_blend: float = 1.0      # 11/10 오후 0.5 -> 1.0: 차로 안 직진은 팀 차선 주행과 완전히 같은 조향 (v x kappa)
    lane_keep_blend_single: float = 0.7   # 11/12 밤: 한쪽 선(L/R)만 보일 때 차선 노드 조향 비율 (예전 0 = 안 섞음)
    lane_fresh_sec: float = 0.3
    lane_fix_gate: float = 0.15       # 지금 추정과 이만큼 넘게 다르면 다른 차로를 본 것 → 안 씀 [m]
    lane_fix_max_deg: float = 10.0    # 차선 방향이 이보다 틀어져 보이면 안 씀 (차로 변경 중·3차선 입구 사선)
    lane_fix_while_changing: bool = True   # 11/12: 옮기는 중·10~20° 틀어져도 양쪽 선이 보이면 반만 반영
    lane_fix_change_deg: float = 20.0      #   이보다 틀어지면 안 씀
    lane_fix_change_scale: float = 0.5     #   옮기는 중·10° 넘을 때 반영 비율
    lane_row: float = 0.55            # 차선 노드 offset_mm 을 재는 행 (기준점 앞 m)
    # ── 멈춤 탈출: 막혀서 서 있으면 상황이 안 바뀌어 계속 선다 → 조금 후진하며 크게 꺾어 다시 잡음 ──
    recover: bool = False             # 11/12 밤: 후진(되돌아가기·탈출) 완전히 끔. 막히면 끝까지 꺾어 빠지기(squeeze)만, 안 되면 서서 기다림
    recover_after: float = 0.4        # 이만큼 계속 막혀 있으면 탈출 시작 [s] (0.8 -> 0.4: 서 있는 시간 줄임)
    recover_dist: float = 0.40   # 탈출 후진 거리 [m]. 11/12 0.25 -> 0.40
    recover_speed: float = 0.30   # 후진 속도 [m/s]. 11/12 0.10 -> 0.30
    recover_time: float = 4.0         # 이 시간 안에 못 끝내도 멈추고 다시 계획 [s]
    recover_max: int = 3              # trail 방식 횟수
    recover_swing_max: int = 6        # trail을 다 쓰고도 막히면: 후진하며 최대 조향으로 머리를 빈 쪽으로 돌리기 횟수 (0이면 예전처럼 정지)
    # 온 길 되돌아가기(11/10): 막히면 지나온 길(trail)을 그대로 후진하면서, 갈 수 있는 차로가 생기는 순간 멈추고 그리로 간다
    recover_mode: str = 'trail'       # 'trail' | 'swing'(예전: 25cm 후진하며 크게 꺾기)
    trail_step: float = 0.05          # 지나온 길 점 간격 [m]
    rollback_lookahead: float = 0.30  # 후진할 때 따라갈 뒤쪽 점 거리 [m]
    recover_lookahead: float = 0.20   # 후진 뒤 다음 차로까지 전방 주시. 11/12 0.25 -> 0.20 (최소 회전반경까지 꺾음)
    rollback_max_dist: float = 0.40   # 한 번 되돌아갈 최대 거리 [m]. 11/12 0.3 -> 0.4
    rollback_time: float = 15.0       # 한 번 되돌아가기 최대 시간 [s]


# ──────────────────────────────────────────────────────────────
# 1) 로컬 위치 추정 : 오도메트리 거리 + IMU 방향
# ──────────────────────────────────────────────────────────────
class LocalPose:
    def __init__(self):
        self.ready = False
        self.x = self.y = self.psi = 0.0
        self.dist = 0.0

    def reset(self, ox, oy, yaw, y0=0.0, psi0=0.0):
        self._ox, self._oy = ox, oy      # 직전 odom 위치
        self.yaw0 = yaw - psi0           # 기준 방향 = 3차선 직선 방향 (psi0: 켤 때 차가 직선에서 틀어진 각)
        self.x, self.y, self.psi = 0.0, y0, psi0
        self.dist = 0.0
        self.ready = True

    def update(self, ox, oy, yaw, forward=True):
        if not self.ready:
            return
        ds = math.hypot(ox - self._ox, oy - self._oy)   # odom 이 움직인 거리만 사용
        if not forward:
            ds = -ds
        psi_new = wrap(yaw - self.yaw0)                 # 방향은 IMU(또는 odom) yaw
        mid = self.psi + wrap(psi_new - self.psi) / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.psi = psi_new
        self.dist += abs(ds)
        self._ox, self._oy = ox, oy


# ──────────────────────────────────────────────────────────────
# 2) 라이다 → 로봇 좌표 점
# ──────────────────────────────────────────────────────────────
class ScanPts(list):
    """scan_to_base 결과. list 그대로 + .far = 반사 없음(inf·0·range_max 넘음) 빔들의 라이다 기준 방향 [rad]"""
    far = ()


def scan_to_base(ranges, angle_min, angle_inc, p):
    """LaserScan ranges → [(x, y) 또는 None, ...]  (base_link 기준, 순서 유지)"""
    off = math.radians(p.scan_yaw_offset_deg)
    out = ScanPts()
    far = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_inc
        if p.scan_flip:
            a = -a
        a += off
        if not (p.range_min <= r <= p.range_max):      # nan / inf 도 여기서 걸러짐
            out.append(None)
            if not (r == r) or r == 0.0 or r > p.range_max:   # 반사 없음 (빔이 끝까지 감). nan 포함
                far.append(math.atan2(math.sin(a), math.cos(a)))
            continue
        out.append((p.lidar_x + r * math.cos(a), r * math.sin(a)))
    out.far = far
    return out


class Obstacle:
    """로컬 좌표의 사각형(bbox) 장애물"""
    def __init__(self, xmin, xmax, ymin, ymax, virtual=False, tag=''):
        self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax
        self.hits = 1
        self.virtual = virtual
        self.tag = tag
        self.miss = 0          # 있어야 할 자리를 라이다가 뚫고 지나간 연속 횟수 (forget)
        self.seen = False      # 이번 스캔에서 덩어리와 합쳐졌나

    def x_end(self, min_len):
        """뒤쪽 끝.  앞면만 봤어도 차 길이만큼은 있다고 가정"""
        if self.virtual:
            return self.xmax
        return max(self.xmax, self.xmin + min_len)


def _rect_hits_box(poly, xmin, xmax, ymin, ymax):
    """돌아간 사각형(꼭짓점 4개)과 축 정렬 상자가 겹치나 (분리축)."""
    xs = [q[0] for q in poly]
    ys = [q[1] for q in poly]
    if max(xs) < xmin or min(xs) > xmax or max(ys) < ymin or min(ys) > ymax:
        return False
    box = ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))
    for i in range(2):
        ex, ey = poly[i + 1][0] - poly[i][0], poly[i + 1][1] - poly[i][1]
        ax, ay = -ey, ex
        a = [qx * ax + qy * ay for qx, qy in poly]
        b = [qx * ax + qy * ay for qx, qy in box]
        if max(a) < min(b) or max(b) < min(a):
            return False
    return True


# ──────────────────────────────────────────────────────────────
# 3) 회피 플래너
# ──────────────────────────────────────────────────────────────
class AvoidPlanner:
    def __init__(self, params=None):
        self.p = params or AvoidParams()
        self.pose = LocalPose()
        self.reset()

    # ---------- 상태 ----------
    def reset(self):
        self.state = 'IDLE'          # IDLE → RUN → DONE
        self.t_start = 0.0
        self.obs = []
        self.target = self.p.start_lane
        self.last_switch = -1e9
        self.emergency = False
        self.blocked = False
        self.hit_dist = INF          # 지금 조향대로 가면 부딪히는 거리 (status)
        self.exit_pref = parse_exit(self.p.exit_lane)
        self.note = ''
        self.events = []             # (x, '설명') 기록
        self.align_dy = 0.0          # 누적 옆 보정 [m]
        self.align_th = 0.0          # 누적 방향 보정 [rad]
        self.exit_info = {}          # 출구 이동 판단 (status 로 내보냄)
        self.flip_from = None        # 마지막으로 떠난 차로 (왔다갔다 막기)
        self._cont_done = False      # 이번 옮기기에서 '이어서 한 번에'를 이미 했나
        self.lane_why = {}           # 마지막 차로 고르기에서 차로별 판단 (status·멈춤 로그로 내보냄, 11/12 진단용)
        self._late_logged = False
        self._given_up_for = None
        self.stuck_since = None      # 막혀서 서 있기 시작한 시각
        self.rec = None              # 탈출 중이면 {'t0', 'x0', 'y0', 'side'}
        self.rb = None               # 되돌아가기 중이면 {'t0', 'x0', 'y0', 'dist', 'blocked'}
        self.sharp = False           # 후진 뒤 다음 차로에 들어갈 때까지 크게 꺾기 (recover_lookahead)
        self.rechosen = True         # 이번 차로 선택에서 다시 고르기를 이미 했나 (처음엔 고른 적 없으니 True)
        self.rec_n = 0               # 탈출 횟수 (trail + swing)
        self.rec_side_last = None    # 마지막 swing 방향
        self.rec_same_side = 0
        self.lane_fix_n = 0          # 카메라 차선으로 보정한 횟수 (status)
        self.lane_fix_dy = 0.0       # 마지막 보정량 [m]
        self.trail = []              # 지나온 길 [(x, y)] (되돌아가기)
        self.lane_ids = []           # 차선 노드 차로 번호 기록
        self.lane_id = None          # 확정된 카메라 차로 번호
        self.lane_id_fix_n = 0
        self.lane_last = None        # 마지막 /lane/status
        self.lane_t = None
        self.rollback_m = 0.0

    def start(self, ox, oy, yaw, t=0.0):
        self.reset()
        self.t_start = t
        psi0 = math.radians(self.p.start_heading_deg)
        if abs(psi0) > math.radians(35.0):
            psi0 = 0.0                                   # 말이 안 되는 값은 버림
        self.pose.reset(ox, oy, yaw, y0=self.p.start_lane * self.p.lane_width, psi0=psi0)
        self.state = 'RUN'
        self.last_switch = t - self.p.hold_time

    def set_exit(self, s):
        new = parse_exit(s)
        if new is not None and self.exit_info.get('plan') == 'given_up' and new == self._given_up_for:
            return                                   # 이미 포기한 방향을 다시 받아도 또 시도하지 않음
        if self.exit_info.get('plan') == 'given_up':
            self.exit_info = {}
        self.exit_pref = new

    @property
    def lanes(self):
        n = self.p.num_side_lanes
        return list(range(-n, n + 1))

    def center(self, lane):
        return lane * self.p.lane_width

    def nearest_lane(self, y):
        n = self.p.num_side_lanes
        return max(-n, min(n, int(round(y / self.p.lane_width))))

    # ---------- 라이다 처리 ----------
    def update_scan(self, pts_base):
        """base_link 점들 → (비상정지 확인) → 로컬 좌표 → 묶기 → 지도에 합치기"""
        p = self.p
        x, y, psi = self.pose.x, self.pose.y, self.pose.psi
        c, s = math.cos(psi), math.sin(psi)
        ylim = (p.num_side_lanes + 0.5) * p.lane_width + p.roi_side_extra

        emerg_n = 0
        lo_lane, hi_lane = sorted((self.nearest_lane(y), self.target))
        band_lo = self.center(lo_lane) - 0.5 * p.lane_width
        band_hi = self.center(hi_lane) + 0.5 * p.lane_width
        clusters, cur = [], []
        prev = None

        def close():
            if len(cur) >= p.cluster_min_pts:
                xs = [q[0] for q in cur]
                ys = [q[1] for q in cur]
                lx, wy = max(xs) - min(xs), max(ys) - min(ys)
                # 너무 크면 벽·테두리(길이 max_obs_len, 폭 max_obs_width 초과), 너무 작으면 기둥·다리 -> 버림 (11/10)
                yc = 0.5 * (max(ys) + min(ys))
                # 11/12 밤 실차: 트랙 밖(노란 실선 밖) 물체·사람이 장애물로 들어옴 -> 중심이 바깥 차로 중심보다
                #   obs_max_center_y 넘게 밖이면 버림 (바깥 차로 LIMO 중심 0.52 m, 위치 오차 15 cm까지 받음)
                if (lx <= p.max_obs_len and wy <= p.max_obs_width and max(lx, wy) >= p.min_obs_size
                        and abs(yc) <= p.obs_max_center_y):
                    clusters.append((min(xs), max(xs), min(ys), max(ys)))
            cur.clear()

        for pt in pts_base:
            if pt is None:
                close(); prev = None
                continue
            px, py = pt
            # 비상정지 박스: 앞범퍼 바로 앞, 차폭 안
            X = x + px * c - py * s
            Y = y + px * s + py * c
            if (p.robot_front < px < p.robot_front + p.stop_dist and abs(py) < p.robot_half_width
                    and (not p.emerg_lane_band or band_lo <= Y <= band_hi)):
                emerg_n += 1
            if not (x - p.roi_back <= X <= x + p.roi_front and -ylim <= Y <= ylim):
                close(); prev = None
                continue
            if prev is not None and math.hypot(X - prev[0], Y - prev[1]) > p.cluster_gap:
                close()
            cur.append((X, Y))
            prev = (X, Y)
        close()

        self.emergency = emerg_n >= max(1, p.emerg_min_pts)
        self.last_clusters = clusters
        for o in self.obs:
            o.seen = False
        self._merge(clusters)
        if p.forget:
            self._forget(pts_base)
        self._align()
        return clusters

    def _forget(self, pts_base):
        """있어야 할 앞면을 빔이 뚫고 지나가면(그 너머에서 반사) 그 장애물은 없는 것 -> 몇 번 연속이면 지움"""
        p = self.p
        P = self.pose
        c, s = math.cos(P.psi), math.sin(P.psi)
        lx, ly = P.x + p.lidar_x * c, P.y + p.lidar_x * s
        rays = []
        for pt in pts_base:
            if pt is None:
                continue
            bx, by = pt[0] - p.lidar_x, pt[1]
            rays.append((math.atan2(by, bx), math.hypot(bx, by)))
        rays.extend((a, INF) for a in getattr(pts_base, 'far', ()))
        if not rays:
            return
        keep = []
        for o in self.obs:
            if o.virtual or o.seen:
                o.miss = 0 if o.seen else o.miss
                keep.append(o)
                continue
            # 앞면 양 끝을 라이다 기준 (방향, 거리)로
            ends = []
            for yy in (o.ymin, o.ymax):
                dx, dy = o.xmin - lx, yy - ly
                fx, fy = dx * c + dy * s, -dx * s + dy * c
                ends.append((math.atan2(fy, fx), math.hypot(fx, fy), fx))
            fx_min = min(e[2] for e in ends)
            if not (p.forget_min_x <= fx_min + p.lidar_x <= p.forget_max_x):
                keep.append(o)
                continue
            a0, a1 = sorted(e[0] for e in ends)
            pad = 0.25 * (a1 - a0)                    # 가장자리 빔은 빼고 가운데만
            a0, a1 = a0 + pad, a1 - pad
            near = min(e[1] for e in ends)
            far = max(e[1] for e in ends)
            n_pass = n_hit = 0
            for a, r in rays:
                if a0 <= a <= a1:
                    if r > far + p.forget_pass:
                        n_pass += 1
                    elif r > near - 0.10:
                        n_hit += 1
            if n_pass >= p.forget_rays and n_hit == 0:
                o.miss += 1
            elif n_hit > 0:
                o.miss = 0
            if o.miss >= p.forget_scans:
                self.events.append((self.pose.x, f'장애물 지움 (x {o.xmin:.2f}, y {0.5 * (o.ymin + o.ymax):+.2f}: 빔이 뚫고 지나감)'))
                continue
            keep.append(o)
        self.obs = keep

    # ---------- 정렬 보정 ----------
    def _align(self):
        """확정된 정차 LIMO들의 중심 y 가 차선 중심(k × lane_width)에 오도록
        잔차 r = y중심 - k·w 를  r ≈ a + b·x  로 맞춘 뒤(사전값이 있는 최소제곱),
        좌표 전체를 원점 기준 δ=-b 만큼 회전 + t=-a 만큼 옆으로 옮긴다."""
        p = self.p
        if not p.lane_align:
            return
        if p.align_skip_with_camera and self.lane_fix_n > 0:
            return
        if self.rec is not None or self.rb is not None or abs(math.degrees(self.pose.psi)) > p.align_max_psi_deg:
            return      # 후진·크게 비틀어진 동안은 장애물 옆면이 비스듬히 보여 중심이 틀어짐 -> 정렬에 안 씀
        xs, rs = [], []
        for o in self.obs:
            if o.virtual or o.hits < p.confirm_hits:
                continue
            w = o.ymax - o.ymin
            if not (p.align_w_min <= w <= p.align_w_max):
                continue
            yc = 0.5 * (o.ymin + o.ymax)
            k = self.nearest_lane(yc)
            r = yc - k * p.lane_width
            if abs(r) > 0.5 * p.lane_width - 0.05:      # 차선 경계 근처면 어느 차선인지 애매 → 제외
                continue
            xs.append(o.xmin + 0.5 * p.obs_min_len)
            rs.append(r)
        if not xs:
            return
        im = 1.0 / p.align_sigma_meas ** 2
        s0 = len(xs) * im + 1.0 / p.align_sigma_dy ** 2
        s1 = sum(xs) * im
        s2 = sum(x * x for x in xs) * im + 1.0 / math.radians(p.align_sigma_heading_deg) ** 2
        t0 = sum(rs) * im
        t1 = sum(x * r for x, r in zip(xs, rs)) * im
        det = s0 * s2 - s1 * s1
        if det <= 0:
            return
        a = (t0 * s2 - s1 * t1) / det
        b = (s0 * t1 - s1 * t0) / det
        if max(xs) - min(xs) < p.align_min_span:
            # 장애물들이 앞뒤로 가까이 붙어 있으면 방향(기울기)은 잴 수 없다 (5cm 오차 / 15cm 간격 = 18°) -> 옆 위치만
            a, b = t0 / s0, 0.0
        g = p.align_gain
        d = -g * b
        t = -g * a
        mth, mdy = math.radians(p.align_max_heading_deg), p.align_max_dy
        d = max(-mth - self.align_th, min(mth - self.align_th, d))
        t = max(-mdy - self.align_dy, min(mdy - self.align_dy, t))
        if abs(d) < 1e-5 and abs(t) < 1e-5:
            return
        self._apply_transform(d, t)
        self.align_th += d
        self.align_dy += t

    def _apply_transform(self, d, t):
        """로컬 좌표 전체를 원점 기준 d[rad] 회전 + y 로 t[m] 이동 (위치·방향·장애물 모두)"""
        c, s = math.cos(d), math.sin(d)
        P = self.pose
        P.x, P.y = c * P.x - s * P.y, s * P.x + c * P.y + t
        P.psi = wrap(P.psi + d)
        P.yaw0 -= d                      # 이후 odom/IMU 갱신도 같은 좌표로 이어지게
        self.trail = [(c * tx - s * ty, s * tx + c * ty + t) for tx, ty in self.trail]
        for o in self.obs:               # 장애물은 중심만 옮김 (크기는 그대로 → 반복 회전에도 안 커짐)
            cx, cy = 0.5 * (o.xmin + o.xmax), 0.5 * (o.ymin + o.ymax)
            nx, ny = c * cx - s * cy, s * cx + c * cy + t
            dx, dy = nx - cx, ny - cy
            o.xmin += dx; o.xmax += dx
            o.ymin += dy; o.ymax += dy

    def _merge(self, clusters):
        p, md = self.p, self.p.merge_dist
        for (x0, x1, y0, y1) in clusters:
            merged = False
            for o in self.obs:
                if x0 <= o.xmax + md and x1 >= o.xmin - md and y0 <= o.ymax + md and y1 >= o.ymin - md:
                    ny0, ny1 = min(o.ymin, y0), max(o.ymax, y1)
                    if ny1 - ny0 > p.max_obs_width:
                        continue
                    if max(o.xmax, x1) - min(o.xmin, x0) > p.max_obs_len:   # 벽 조각이 이어 붙어 길어지는 것 방지
                        continue
                    o.xmin, o.xmax = min(o.xmin, x0), max(o.xmax, x1)
                    o.ymin, o.ymax = ny0, ny1
                    o.hits += 1
                    o.seen = True
                    merged = True
                    break
            if not merged:
                n = Obstacle(x0, x1, y0, y1)
                n.seen = True
                self.obs.append(n)

    def obstacles(self, with_pref=True):
        """확정된 실제 장애물 + 가상 장애물(섬, 출구 선호)"""
        p = self.p
        out = [o for o in self.obs if o.hits >= p.confirm_hits]
        L, lw = p.section_length, p.lane_width
        if p.use_island:
            out.append(Obstacle(L, L + p.island_length, -0.3 * lw, 0.3 * lw, True, 'island'))
        if with_pref and self.exit_pref is not None:
            other = -self.exit_pref                      # 반대쪽 차선을 섬 구간부터 막음
            out.append(Obstacle(L, L + p.island_length,
                                other * lw - 0.3 * lw, other * lw + 0.3 * lw, True, 'exit'))
        return out

    # ---------- 차선별 여유 ----------
    def _in_corridor(self, o, lane):
        c = self.center(lane)
        hw = self.p.robot_half_width + self.p.side_margin
        return not (o.ymax < c - hw or o.ymin > c + hw)

    def free_ahead(self, lane, obs, skip_beside=None):
        """그 차선에서 앞범퍼 ~ 가장 가까운 장애물까지 거리 (없으면 INF, 옆에 붙어있으면 음수)"""
        # 11/12 밤 실차: 차로 고르기는 옆(지나가는 중) 장애물을 빼고 보는데, 고른 뒤 '지금 목표 차로'는 빼지 않고 봐서
        #   옮기자마자 그 차로가 막힌 것(앞 거리 음수)으로 보여 0.4초마다 되돌아갔다 (bag: -1<->0 반복, 1->-1 뒤집힘).
        #   기본(None) = 차가 아직 그 차로에 안 들어갔으면 옆 장애물을 뺌 (들어갈 수 있는지는 path_clear·path_hit가 봄)
        if skip_beside is None:
            skip_beside = self.p.clear_rollout and lane != self.nearest_lane(self.pose.y)
        p = self.p
        x = self.pose.x
        xf = x + p.robot_front
        d = INF
        for o in obs:
            if not self._in_corridor(o, lane):
                continue
            if o.x_end(p.obs_min_len) < x - p.robot_rear:   # 이미 지나감
                continue
            if skip_beside and o.xmin < xf:                  # 옆(지나가는 중): 들어갈 수 있는지는 path_clear가 봄
                continue
            d = min(d, o.xmin - xf)
        return d

    def path_clear(self, frm, to, obs):
        """frm → to 로 옮길 때 지나가는 차선들이 '지금 옆 + 앞쪽 change_len' 동안 비었나"""
        p = self.p
        x = self.pose.x
        ahead = x + p.robot_front + p.change_len * abs(to - frm)
        back = x - p.robot_rear - p.clear_back
        step = 1 if to > frm else -1
        beside = False
        for lane in range(frm + step, to + step, step):
            for o in obs:
                if not self._in_corridor(o, lane):
                    continue
                if o.xmin < ahead and o.x_end(p.obs_min_len) > back:
                    if p.clear_rollout and o.xmin < x + p.robot_front:
                        beside = True                       # 옆에 있는(지나가는 중인) 장애물 -> 아래 굴려 보기로 판단
                        continue
                    return False
        if not beside:
            return True
        # 11/12: 옆 장애물을 다 지나기 전이라도, 그 차로로 실제로 꺾어 들어가는 길(차체 전체)을 굴려 봐서
        #   안 부딪히면 지금 옮기기 시작 (S자로 일찍 꺾음). 예전엔 뒤범퍼가 옆 장애물을 다 지나야 시작 ->
        #   다음 장애물이 가까워져 서고 후진
        saved, saved_sharp = self.target, self.sharp
        self.target = to
        try:
            m = p.path_margin
            p.path_margin = m + p.clear_rollout_margin
            hit = self.path_hit(obs, 0.0, p.change_len * abs(to - frm))
        finally:
            p.path_margin = m
            self.target, self.sharp = saved, saved_sharp
        return hit == INF

    def choose(self, cur, obs):
        """막힌 차선 cur 대신 갈 차선 고르기 (없으면 None). 차로별 판단을 self.lane_why 에 남긴다."""
        p = self.p
        f_cur = self.free_ahead(cur, obs)
        best, best_key = None, None
        why = {cur: f'지금 앞 {min(f_cur, 9.9):.2f}m'}
        for lane in self.lanes:
            if lane == cur:
                continue
            if not self.path_clear(cur, lane, obs):
                why[lane] = '지나갈 길 막힘(옆·앞 장애물)'
                continue
            # 11/12: 옆에 붙어 지나가는 중인 장애물은 '앞'이 아님 (들어가도 되는지는 위 path_clear가 굴려 봄).
            #   예전엔 옆 장애물 때문에 앞 거리가 음수로 나와 그 차로를 안 골랐다 -> 다음 장애물 앞에서 서고 후진
            f = self.free_ahead(lane, obs, skip_beside=p.clear_rollout)
            if f < f_cur + p.better_margin:
                why[lane] = f'앞 {min(f, 9.9):.2f}m (지금보다 {p.better_margin:.1f}m 더 비어야 함)'
                continue
            why[lane] = f'갈 수 있음 앞 {min(f, 9.9):.2f}m'
        self.lane_why = why
        for lane in self.lanes:
            if lane == cur or not why.get(lane, '').startswith('갈 수 있음'):
                continue
            f = self.free_ahead(lane, obs, skip_beside=p.clear_rollout)
            score = (min(f, 3.0) - p.two_lane_penalty * (abs(lane - cur) - 1)
                     + (p.center_bonus if lane == 0 else 0.0))
            key = (round(score, 3), -abs(lane - cur), 1 if lane == self.exit_pref else 0)
            if best is None or key > best_key:
                best, best_key = lane, key
        return best

    # ---------- 갈림길 출구 차선 이동 ----------
    def _lanes_between(self, frm, to):
        """frm 에서 to 로 갈 때 새로 지나가는 차선들 (frm 제외, to 포함)"""
        step = 1 if to > frm else -1
        return list(range(frm + step, to + step, step))

    def exit_plan(self, obs):
        """출구 차선으로 지금 옮겨도 되나 → ('keep'|'go'|'wait'|'too_late', 정보)
        keep    : 이미 출구 차선 (또는 방향 모름)
        go      : 지금 옮기기 시작 (지나갈 차선이 비었고, 섬 앞 끝까지 거리가 충분)
        wait    : 장애물 때문에 아직 못 옮김 (계속 다시 봄)
        too_late: 섬 앞 끝까지 남은 거리가 이동에 필요한 거리보다 짧음"""
        p = self.p
        ex, cur = self.exit_pref, self.target
        if ex is None or cur == ex:
            return 'keep', {'need_lanes': 0}
        n = ex - cur
        clear, settle = shift_distance(p, n)
        x = self.pose.x
        rem = p.section_length - (x + p.robot_front)
        info = {'need_lanes': n, 'need_m': round(clear + p.exit_margin, 2), 'rem_m': round(rem, 2)}
        if rem < clear + p.exit_margin:
            return 'too_late', info
        # 미리 옮기는 판단은 보수적으로: 아직 확정 전(1~2번 본) 장애물도 막힌 것으로 본다
        real = [o for o in self.obs if not o.virtual]
        back = x - p.robot_rear - p.clear_back
        ahead = x + p.robot_front + settle
        for lane in self._lanes_between(cur, ex):          # 지나갈 차선: 옮기는 동안 옆·앞이 비어야 함
            for o in real:
                if self._in_corridor(o, lane) and o.xmin < ahead and o.x_end(p.obs_min_len) > back:
                    info['blocked_by'] = lane
                    return 'wait', info
        for o in real:                                      # 출구 차선: 섬 앞 끝까지 쭉 비어야 함 (옮긴 뒤 또 막히지 않게)
            if self._in_corridor(o, ex) and o.x_end(p.obs_min_len) > back and o.xmin < p.section_length:
                info['blocked_by'] = ex
                return 'wait', info
        return 'go', info

    # ---------- 한 주기 ----------
    # ---------- 카메라 차선 보정 ----------
    def lane_fix(self, lane, t=None):
        """팀 lane_node 의 /lane/status (dict) 로 옆 위치·방향 보정. 쓰면 True.

        offset_mm  : 기준점 앞 lane_row 행에서 차로 중심의 횡위치 (왼쪽 +, 차 기준)
        heading_rad: 차로 방향 (왼쪽 +, 차 기준)  →  차 기준 차로 직선에서 차의 옆 위치
                     y_car = -off*cos(h) + row*sin(h),  차의 방향(차로 기준) = -h
        같은 차로를 봤다고 확신할 때만 쓴다: 양쪽 선(BOTH), 방향 작음, 지금 추정과 가까움."""
        p = self.p
        if self.state != 'RUN' or not lane:
            return False
        self.lane_last, self.lane_t = lane, t
        self._lane_id_fix(lane)
        if not p.lane_fix:
            return False
        off, h = lane.get('offset_mm'), lane.get('heading_rad')
        if lane.get('status') != 'BOTH' or lane.get('low_conf') or off is None or h is None:
            return False
        h = float(h)
        deg = abs(math.degrees(h))
        # 11/12: 차로를 옮기는 중(목표 차로 중심 밖)이거나 10~20°로 틀어져 있어도 양쪽 선이 잡히면 반만 반영 (늘 차선을 봄).
        #   20° 넘으면 안 씀. 다른 차로를 본 것 같으면(gate) 안 씀은 그대로
        changing = abs(self.pose.y - self.center(self.target)) >= p.reach_tol
        if deg > (p.lane_fix_change_deg if p.lane_fix_while_changing else p.lane_fix_max_deg):
            return False
        scale = p.lane_fix_change_scale if (changing or deg > p.lane_fix_max_deg) else 1.0
        if not p.lane_fix_while_changing and changing and deg > p.lane_fix_max_deg:
            return False
        off = float(off) / 1000.0
        y_in_lane = -off * math.cos(h) + p.lane_row * math.sin(h)
        lane_idx = self.nearest_lane(self.pose.y)
        dy = self.center(lane_idx) + y_in_lane - self.pose.y
        if abs(dy) > p.lane_fix_gate:
            return False
        dpsi = wrap(-h - self.pose.psi)
        d = p.lane_fix_heading_gain * dpsi if abs(math.degrees(dpsi)) <= 5.0 else 0.0
        if not p.lane_fix_while_changing:
            scale = 1.0
        self._apply_transform(scale * d, scale * p.lane_fix_gain * dy)
        self.lane_fix_n += 1
        self.lane_fix_dy = dy
        return True

    def _lane_id_fix(self, lane):
        """차선 노드의 3차선 번호가 여러 번 같게 나오면, 지금 추정 차로가 다를 때 옆 위치를 그 차로로 옮긴다."""
        p = self.p
        lid = lane.get('lane_id')
        self.lane_ids.append(lid if lid in self.lanes else None)
        self.lane_ids = self.lane_ids[-p.lane_id_window:]
        best = None
        for k in self.lanes:
            if self.lane_ids.count(k) >= p.lane_id_need:
                best = k
        self.lane_id = best
        if not p.lane_id_fix or best is None or self.rb is not None:
            return
        now = self.nearest_lane(self.pose.y)
        if best == now or self.target != now:          # 같은 차로 / 차로 바꾸는 중이면 안 건드림
            return
        h = lane.get('heading_rad')
        off = lane.get('offset_mm')
        if h is None or off is None or abs(math.degrees(float(h))) > p.lane_fix_max_deg:
            return
        off = float(off) / 1000.0
        y_in_lane = -off * math.cos(float(h)) + p.lane_row * math.sin(float(h))
        dy = self.center(best) + y_in_lane - self.pose.y
        self._apply_transform(0.0, dy)
        self.target = best
        self.lane_id_fix_n += 1
        self.lane_ids = []
        self.events.append((self.pose.x, f'카메라 선 종류로 차로 확인: {lane_name(now)} → {lane_name(best)} (옆 {dy * 100:+.0f}cm)'))

    # ---------- 온 길 되돌아가기 ----------
    def _record_trail(self):
        x, y = self.pose.x, self.pose.y
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= self.p.trail_step:
            self.trail.append((x, y))
            if len(self.trail) > 400:
                self.trail = self.trail[-400:]

    def _rollback_step(self, t):
        """지나온 길을 따라 후진. 갈 차로가 생기면 그 차로로 정하고 끝(None). 너무 멀리·오래면 끝."""
        p, rb = self.p, self.rb
        x, y, psi = self.pose.x, self.pose.y, self.pose.psi
        rb['dist'] = max(rb['dist'], math.hypot(x - rb['x0'], y - rb['y0']))
        self.rollback_m = rb['dist']
        obs = self.obstacles(True)
        cur = self.nearest_lane(y)
        nxt = self.choose(cur, obs)
        if nxt is None and self.free_ahead(cur, obs) >= p.trigger_dist and cur != rb['blocked']:
            nxt = cur                                  # 지금 차로 앞이 충분히 비었으면 그대로
        if nxt is not None and nxt != cur:
            # 옆 차로로 틀 공간이 생길 때까지는 더 물러난다 (차체가 지금 차로 장애물 옆을 벗어나는 데 필요한 앞 거리)
            # 후진 뒤에는 짧은 전방 주시(recover_lookahead)로 크게 꺾으므로 그 기준으로 필요한 거리를 잼 (11/12)
            q = _with_lookahead(p, p.recover_lookahead) if p.rollback_need_sharp else p
            need = shift_distance(q, nxt - cur)[0] + 0.05
            if self.free_ahead(cur, obs) < need:
                nxt = None
        if nxt is not None and rb['dist'] >= 0.05:
            self.events.append((x, f'되돌아가기 {rb["dist"]:.2f}m → 갈 길 찾음: {lane_name(nxt)}'))
            self.target = nxt
            self.last_switch = t
            self.rb = None
            self.sharp = True
            self.stuck_since = None
            return None
        while len(self.trail) > 1 and math.hypot(self.trail[-1][0] - x, self.trail[-1][1] - y) < p.rollback_lookahead:
            self.trail.pop()
        if (rb['dist'] >= p.rollback_max_dist or t - rb['t0'] >= p.rollback_time or not self.trail):
            self.events.append((x, f'되돌아가기 끝 ({rb["dist"]:.2f}m, 갈 길 없음) → 다시 계획'))
            self.rb = None
            self.sharp = True
            self.stuck_since = None
            self.last_switch = t - p.hold_time
            return None
        tx, ty = self.trail[-1]
        dx, dy = tx - x, ty - y
        xr = dx * math.cos(psi) + dy * math.sin(psi)
        yr = -dx * math.sin(psi) + dy * math.cos(psi)
        d2 = xr * xr + yr * yr
        k = 0.0 if d2 < 1e-6 else 2.0 * yr / d2
        kmax = 1.0 / p.min_turn_radius
        k = max(-kmax, min(kmax, k))
        v = -p.recover_speed
        return v, v * k

    # ---------- 멈춤 탈출 ----------
    def _recover_side(self, obs):
        """후진하며 머리를 돌릴 쪽 (+1 왼쪽, -1 오른쪽).

        차로를 옮기는 중이면(목표 차로 ≠ 지금 차로) 가던 차로 쪽 (옮기다 모서리에 걸린 것).
        아니면 갈 수 있는 차로 쪽, 그것도 없으면 더 비어 있는 옆 차로 쪽."""
        y = self.pose.y
        now = self.nearest_lane(y)
        want = self.target if self.target != now else self.choose(now, obs)
        if want is None or want == now:
            best, best_f = None, -1.0
            for lane in (now - 1, now + 1):
                if lane in self.lanes:
                    f = self.free_ahead(lane, obs)
                    if f > best_f:
                        best, best_f = lane, f
            want = best
        if want is None:
            return 1 if y < 0 else -1
        d = self.center(want) - y
        if abs(d) < 1e-6:
            return 1 if want > now else -1
        return 1 if d > 0 else -1

    def _recover_limit(self):
        p = self.p
        if p.recover_mode == 'trail':
            return p.recover_max + p.recover_swing_max
        return p.recover_max

    def _recover_step(self, t):
        """후진하며 크게 꺾어 머리를 갈 쪽으로 돌린다. 끝나면 None."""
        p, r = self.p, self.rec
        back = math.hypot(self.pose.x - r['x0'], self.pose.y - r['y0'])
        if back >= p.recover_dist or t - r['t0'] >= p.recover_time:
            self.events.append((self.pose.x, f'탈출 끝 ({back:.2f}m 후진) → 다시 계획'))
            self.rec = None
            self.sharp = True
            self.stuck_since = None
            self.last_switch = t - p.hold_time        # 바로 차로를 다시 고를 수 있게
            return None
        v = -p.recover_speed
        k = -r['side'] / p.min_turn_radius            # 후진 + 반대 조향 → 머리가 side 쪽으로 돈다
        return v, v * k

    def step(self, t):
        """→ (v, w).  20Hz 정도로 호출"""
        p = self.p
        if self.state != 'RUN':
            return 0.0, 0.0
        if self.rec is not None:
            out = self._recover_step(t)
            if out is not None:
                return out
        if self.rb is not None:
            out = self._rollback_step(t)
            if out is not None:
                return out
        self._record_trail()
        x, y = self.pose.x, self.pose.y

        obs = self.obstacles(True)
        cur = self.target
        f_cur = self.free_ahead(cur, obs)

        # ① 내 차선 앞이 막혔나? → ② 갈 차선 고르기
        # 가운데 차로에서 표지를 기다리는 중: 앞을 막는 게 섬뿐이면 center_leave_m 까지는 안 비킴 (표지를 알면 ②'가 1차로만 옮김)
        wait_sign = False
        if p.center_before_island and self.exit_pref is None and cur == 0:
            real = [o for o in obs if not o.virtual]
            rem = p.section_length - (x + p.robot_front)
            wait_sign = rem > p.center_leave_m and self.free_ahead(0, real) >= p.trigger_dist
        if f_cur < p.trigger_dist and t - self.last_switch >= p.hold_time and not wait_sign:
            nxt = self.choose(cur, obs)
            if nxt is None and self.exit_pref is not None:
                obs2 = self.obstacles(False)            # 출구 선호 때문에 막혔으면 선호 포기
                nxt = self.choose(cur, obs2)
                if nxt is not None:
                    self._given_up_for = self.exit_pref
                    self.exit_pref = None
                    self.exit_info = {'plan': 'given_up'}
                    self.events.append((x, '출구선호 포기'))
                    obs = obs2
            if nxt is None and self.exit_pref is not None:
                # 지금 차선이 '출구 선호' 가상벽 때문에만 막힌 거면 → 선호 포기하고 그대로 직진
                obs2 = self.obstacles(False)
                if self.free_ahead(cur, obs2) > self.free_ahead(cur, obs) + 1e-6:
                    self._given_up_for = self.exit_pref
                    self.exit_pref = None
                    self.exit_info = {'plan': 'given_up'}
                    self.events.append((x, '출구선호 포기(현재 차선 유지)'))
                    obs = obs2
            # 11/12 밤 실차: 방금 떠난 차로로 0.4초마다 되돌아가는 왔다갔다(-1 <-> 0)가 있었다 -> 그 차로로는
            #   flip_block_sec 동안 안 돌아감 (지금 가는 차로가 flip_ok_free 안으로 정말 막혔을 때만 허용)
            if (nxt is not None and nxt == self.flip_from and t - self.last_switch < p.flip_block_sec
                    and f_cur >= p.flip_ok_free):
                nxt = None
            if nxt is not None:
                self.events.append((x, f'차선 {cur:+d} → {nxt:+d}  (앞 {f_cur:.2f}m)'))
                self.flip_from = cur
                self.target = nxt
                self.last_switch = t
                self.rechosen = False

        # ①' 다시 고르기: 막 옮기기 시작했는데(원래 차로에서 rechoose_dy 안) 다른 차로가 더 비면 목표를 바꿈.
        #     켜자마자 첫 장애물만 보고 고르면 뒤 줄이 아직 확정 전이라 왼쪽·오른쪽이 같아 보인다(11/11 시뮬레이션).
        origin = self.nearest_lane(y)
        if (p.rechoose_dy > 0 and self.target != origin and abs(y - self.center(origin)) < p.rechoose_dy
                and t - self.last_switch <= p.rechoose_sec and not self.rechosen
                and self.rb is None and self.rec is None):
            alt = self.choose(origin, obs)
            if (alt is not None and alt != self.target
                    and self.free_ahead(alt, obs) > self.free_ahead(self.target, obs) + p.better_margin):
                self.events.append((x, f'다시 고름: 차선 {self.target:+d} → {alt:+d} '
                                       f'(앞 {self.free_ahead(self.target, obs):.2f}m → {min(self.free_ahead(alt, obs), 9.9):.2f}m)'))
                self.target = alt
                self.rechosen = True

        # ①'' 이어서 한 번에: 옮기는 도중(아직 목표 차로 밖) 목표 차로 앞이 막혔으면 그 다음 차로까지 계속
        if p.cont_change and self.rb is None and self.rec is None and not self._cont_done:
            dyt = self.center(self.target) - y
            if abs(dyt) >= p.reach_tol:                       # 아직 옮기는 중
                d = 1 if dyt > 0 else -1                      # 옮겨 가는 방향
                nxt2 = self.target + d
                f_tg = self.free_ahead(self.target, obs)
                rem = p.section_length - (x + p.robot_front)
                if (nxt2 in self.lanes and 0.0 <= f_tg < p.trigger_dist and rem > p.cont_island_m
                        and d * self.pose.psi < math.radians(p.max_heading_deg)
                        and self.path_clear(self.target, nxt2, obs)):
                    f2 = self.free_ahead(nxt2, obs)
                    if f2 > p.cont_min_free and f2 > f_tg + p.better_margin:
                        self.events.append((x, f'옮기는 중 {lane_name(self.target)} 앞 {f_tg:.2f}m 막힘 -> 이어서 '
                                               f'{lane_name(nxt2)}까지 한 번에 (앞 {min(f2, 9.9):.2f}m, 섬까지 {rem:.2f}m)'))
                        self.target = nxt2
                        self.last_switch = t
                        self._cont_done = True
            else:
                self._cont_done = False                       # 목표 차로에 도착 -> 다음 옮기기 때 다시 쓸 수 있음

        # ②' 갈림길 방향을 알면 섬 앞 끝 전에 출구 차선으로 미리 옮김 (0/1/2차선)
        if p.fork_shift and self.exit_pref is not None and self.target != self.exit_pref \
                and t - self.last_switch >= p.hold_time and t - self.t_start >= p.exit_min_time:
            decision, info = self.exit_plan(obs)
            self.exit_info = dict(info, plan=decision)
            if decision == 'go':
                self.events.append((x, f'갈림길 출구로 이동: {lane_name(self.target)} → {lane_name(self.exit_pref)} '
                                        f'({abs(info["need_lanes"])}차선, 섬까지 {info["rem_m"]:.2f}m / 필요 {info["need_m"]:.2f}m)'))
                self.target = self.exit_pref
                self.last_switch = t
            elif decision == 'too_late' and not self._late_logged:
                self._late_logged = True
                self.events.append((x, f'갈림길 출구 이동 불가: 섬까지 {info["rem_m"]:.2f}m < 필요 {info["need_m"]:.2f}m '
                                        f'({lane_name(self.target)} 유지)'))
        elif self.exit_pref is not None and self.target == self.exit_pref:
            self.exit_info = {'need_lanes': 0, 'plan': 'keep'}

        # ②'' 11/12 밤 실차: 1·3차로에서 섬 앞 갈림길 표지(섬 앞 끝 가운데)를 잘 못 봄 -> 표지를 아직 모르면 섬 앞
        #   center_from_m ~ center_until_m 사이에서는 가능하면 2차로(가운데)로 모아 표지를 보게 한다.
        #   2차로 앞이 섬까지 비고 지나갈 길이 비었을 때만 (특별한 경우 = 장애물이 있으면 그대로).
        #   표지를 알게 되면 위 ②'가 1차로만 옮기면 되고, 끝까지 모르면 섬(가상 장애물) 때문에 ①에서 옆 차로를 고른다.
        if (p.center_before_island and self.exit_pref is None and self.target != 0 and 0 in self.lanes
                and t - self.last_switch >= p.hold_time and self.rb is None and self.rec is None):
            rem = p.section_length - (x + p.robot_front)
            if p.center_until_m < rem <= p.center_from_m and self.nearest_lane(y) == self.target:
                f0 = self.free_ahead(0, obs, skip_beside=False)   # 옆에서 지나가는 중인 것도 없어야 (가운데가 정말 빔)
                if f0 >= rem - 0.05 and self.path_clear(self.target, 0, obs):
                    self.events.append((x, f'섬 앞 {rem:.2f}m: 표지를 보려고 가운데 차로로 ({lane_name(self.target)} → 2차선)'))
                    self.flip_from = self.target
                    self.target = 0
                    self.last_switch = t
                    self.rechosen = True             # 다시 고르기가 '앞이 더 빈 옆 차로'로 되돌리지 않게 (섬 때문에 가운데는 늘 짧음)

        # ②''' 가운데로 돌아오기 (위 center_return 설명)
        if (p.center_return and self.target != 0 and 0 in self.lanes and self.rb is None and self.rec is None
                and t - self.last_switch >= max(p.hold_time, p.center_return_sec)
                and abs(y - self.center(self.target)) < p.reach_tol
                and not (self.flip_from == 0 and t - self.last_switch < p.flip_block_sec)):
            rem = p.section_length - (x + p.robot_front)
            if rem > p.center_from_m:
                f0 = self.free_ahead(0, obs, skip_beside=False)
                if f0 >= p.center_return_free and self.path_clear(self.target, 0, obs):
                    self.events.append((x, f'가운데로 돌아옴: {lane_name(self.target)} → 2차선 (가운데 앞 {min(f0, 9.9):.2f}m)'))
                    self.flip_from = self.target
                    self.target = 0
                    self.last_switch = t
                    self.rechosen = True

        # ③ 속도
        in_lane = abs(y - self.center(self.target)) < p.reach_tol
        if in_lane and self.sharp:
            self.sharp = False                          # 새 차로에 들어옴 -> 보통 조향
        v = p.v_cruise if in_lane else p.v_change
        # 정지(→ 후진)는 '지금 조향대로 stop_dist 더 가면 차체가 부딪힘'일 때만.
        #   예전: 가려는 차로 전체(옆에 붙은 장애물 포함, 거리 음수)를 봐서 옆 장애물만으로도 서고 후진했다.
        k_now = self.control(1.0, t, in_lane)
        self.hit_dist = self.path_hit(obs, k_now, p.stop_dist)
        self.blocked = self.hit_dist < INF
        self.squeezing = False
        if (self.blocked or (self.emergency and not p.recover)) and p.squeeze:
            out = self._squeeze(obs, t)
            if out is not None:
                self.stuck_since = None
                return out
        if self.blocked or self.emergency:
            v = 0.0
            # 서 있으면 상황이 안 바뀐다 → 잠시 뒤 후진하며 크게 꺾어 다시 잡기 (횟수 제한)
            if self.stuck_since is None:
                self.stuck_since = t
            elif (p.recover and p.recover_mode == 'trail' and self.rec_n < p.recover_max
                  and t - self.stuck_since >= p.recover_after and len(self.trail) > 1):
                self.choose(self.nearest_lane(y), obs)       # 차로별 판단을 로그에 남기려고 (결정은 안 바꿈)
                self.events.append((x, '  차로 판단: ' + self._why_text()))
                self.rb = {'t0': t, 'x0': x, 'y0': y, 'dist': 0.0, 'blocked': self.target}
                self.rec_n += 1
                why = '앞 비상' if self.emergency else '갈 차선 막힘'
                self.events.append((x, f'멈춤 {t - self.stuck_since:.1f}s ({why}) → 온 길 되돌아가기 '
                                       f'{self.rec_n}/{p.recover_max} (최대 {p.rollback_max_dist:.1f}m, 갈 길 생기면 멈춤)'))
                out = self._rollback_step(t)
                if out is not None:
                    return out
            elif (p.recover and t - self.stuck_since >= p.recover_after
                  and self.rec_n < self._recover_limit()):
                # swing: trail을 다 썼거나(또는 recover_mode: swing) 지나온 길이 없음 -> 후진하며 최대 조향으로 머리를 빈 쪽으로
                side = self._recover_side(obs)
                if self.rec_side_last == side and self.rec_same_side >= 2:
                    side = -side                       # 같은 쪽으로 두 번 돌려도 안 되면 반대쪽도 해 본다
                self.rec_same_side = self.rec_same_side + 1 if side == self.rec_side_last else 1
                self.rec_side_last = side
                self.rec = {'t0': t, 'x0': x, 'y0': y, 'side': side}
                self.rec_n += 1
                why = '앞 비상' if self.emergency else '갈 차선 막힘'
                self.events.append((x, f'멈춤 {t - self.stuck_since:.1f}s ({why}) → 탈출 {self.rec_n}/{self._recover_limit()}: '
                                       f'{p.recover_dist:.2f}m 후진하며 {"왼쪽" if side > 0 else "오른쪽"}으로 최대 조향'))
                return self._recover_step(t)
        else:
            self.stuck_since = None

        # ④ 끝났나?
        L = p.section_length
        settled = in_lane and abs(math.degrees(self.pose.psi)) <= p.done_max_psi_deg
        if (x > L + p.done_margin and settled) or x > L + p.island_length:
            self.state = 'DONE'
            self.events.append((x, f'완료 (차선 {self.target:+d})'))
            return 0.0, 0.0

        return v, self.control(v, t, in_lane)

    # ---------- 바로 앞 충돌 판단 ----------
    def path_hit(self, obs, k, dist, fixed=False):
        """지금 계획대로 dist[m] 더 갈 때 차체(여유 path_margin)가 장애물과 처음 겹치는 거리. 없으면 INF.
        실제 장애물과 섬만 본다 (출구 선호 가상벽은 차로 고르기용이라 제외).
        길은 '목표 차로 중심으로 가다가 똑바로 서는' pure pursuit을 그대로 굴려서 만든다 (11/10 밤).
        예전엔 지금 곡률 k를 0.3 m 동안 유지한다고 봐서, 1->2차선으로 꺾어 들어갈 때 계속 꺾는 것으로 보고
        3차선 장애물에 닿는다고 판단 -> 정지·후진·출발을 반복했다. k는 rollout을 끄면(path_rollout false) 쓴다."""
        p = self.p
        boxes = [o for o in obs if not (o.virtual and o.tag == 'exit')]
        if not boxes:
            return INF
        x0, y0, psi0 = self.pose.x, self.pose.y, self.pose.psi
        hw = p.robot_half_width + p.path_margin
        front, rear = p.robot_front + p.path_margin, p.robot_rear
        n = max(1, int(math.ceil(dist / 0.05)))
        ds = dist / n
        x, y, h = x0, y0, psi0
        for i in range(n + 1):
            sd = ds * i
            if i > 0:
                kk = self._pursuit_k(x, y, h) if (p.path_rollout and not fixed) else k
                hm = h + 0.5 * kk * ds
                x, y, h = x + ds * math.cos(hm), y + ds * math.sin(hm), h + kk * ds
            c, s = math.cos(h), math.sin(h)
            car = [(x + a * c - b * s, y + a * s + b * c) for a, b in ((front, hw), (front, -hw), (-rear, -hw), (-rear, hw))]
            for o in boxes:
                if _rect_hits_box(car, o.xmin, o.x_end(p.obs_min_len), o.ymin, o.ymax):
                    return sd
        return INF

    def _squeeze(self, obs, t):
        """막혔을 때 조향을 끝까지 꺾어 빠지기. 갈 쪽(목표 차로 / 갈 수 있는 차로 / 더 빈 옆 차로) 먼저,
        안 되면 반대쪽. 끝까지 꺾은 원호로 squeeze_min_dist 더 가도 안 부딪힐 때만. 못 하면 None (-> 정지·후진)."""
        p = self.p
        kmax = 1.0 / p.min_turn_radius
        mh = math.radians(p.max_heading_deg)
        first = self._recover_side(obs)
        now = self.nearest_lane(self.pose.y)
        for side in (first, -first):
            if now + side not in self.lanes and self.target == now:
                continue                               # 바깥 차로에서 바깥쪽으로는 안 꺾음 (트랙 밖)
            if side * self.pose.psi > mh:              # 이미 그쪽으로 많이 틀어짐 -> 더 꺾으면 차로 밖
                continue
            if p.edge_guard and side * self.pose.y > 0.5 * p.lane_width:
                continue                               # 바깥 차로에서 바깥쪽으로 끝까지 꺾지 않음 (노란선 밖)
            k = side * kmax
            if self.path_hit(obs, k, p.squeeze_min_dist, fixed=True) < INF:
                continue
            want = now + side
            if want in self.lanes and self.target != want and self.target == now:
                self.events.append((self.pose.x, f'막힘 -> 끝까지 꺾어 빠짐: {lane_name(now)} → {lane_name(want)}'))
                self.target = want
                self.last_switch = t
            elif not getattr(self, '_sq_logged', False):
                self.events.append((self.pose.x, f'막힘 -> 끝까지 {"왼쪽" if side > 0 else "오른쪽"}으로 꺾어 빠짐'))
            self._sq_logged = True
            self.squeezing = True
            v = min(p.squeeze_speed, p.v_change)
            return v, v * k
        self._sq_logged = False
        return None

    def _ld(self):
        """전방 주시 거리. 후진(되돌아가기·탈출)한 뒤 다음 차로에 들어갈 때까지는 짧게 -> 더 크게 꺾어 빨리 빠짐."""
        return self.p.recover_lookahead if self.sharp else self.p.lookahead

    def _pursuit_k(self, x, y, psi):
        """(x, y, psi)에서 목표 차로 중심을 따라가는 곡률 (control과 같은 식, 차선 노드 섞기 없음)."""
        p = self.p
        Ld = self._ld()
        dy = self.center(self.target) - y
        xr = Ld * math.cos(psi) + dy * math.sin(psi)
        yr = -Ld * math.sin(psi) + dy * math.cos(psi)
        k = 2.0 * yr / (xr * xr + yr * yr)
        kmax = 1.0 / p.min_turn_radius
        k = max(-kmax, min(kmax, k))
        mh = math.radians(p.max_heading_deg)
        if (psi > mh and k > 0) or (psi < -mh and k < 0):
            k = 0.0
        return k

    # ---------- pure pursuit (직선 y = 목표차선중심 따라가기) ----------
    def control(self, v, t=None, in_lane=False):
        p = self.p
        y, psi = self.pose.y, self.pose.psi
        Ld = self._ld()
        dy = self.center(self.target) - y
        # 목표점 (Ld, dy) 을 로봇 기준으로 회전
        xr = Ld * math.cos(psi) + dy * math.sin(psi)
        yr = -Ld * math.sin(psi) + dy * math.cos(psi)
        k = 2.0 * yr / (xr * xr + yr * yr)
        kmax = 1.0 / p.min_turn_radius
        k = max(-kmax, min(kmax, k))
        mh = math.radians(p.max_heading_deg)
        if (psi > mh and k > 0) or (psi < -mh and k < 0):
            k = 0.0
        if self._edge_danger(y, psi):
            if not getattr(self, 'edge_guarded', False):
                self.events.append((self.pose.x, f'바깥 노란선 앞 (옆 {y:+.2f}m, 방향 {math.degrees(psi):+.0f}°) -> 안쪽으로 끝까지 꺾음'))
            self.edge_guarded = True
            return v * (-kmax if y > 0 else kmax)      # 바깥 노란선 앞: 안쪽으로 끝까지
        self.edge_guarded = False
        # 차로 안에서 똑바로 갈 때는 팀 차선 노드 곡률을 섞는다 (양쪽 선 보임, 같은 차로, 최근 값)
        ln = self.lane_last
        self.blend_used = False
        if (p.lane_keep_blend > 0 and in_lane and ln and t is not None and self.lane_t is not None
                and t - self.lane_t <= p.lane_fresh_sec and ln.get('status') in ('BOTH', 'L', 'R')
                and not ln.get('low_conf')
                and ln.get('kappa') is not None and self.target == self.nearest_lane(y)
                and (self.lane_id is None or self.lane_id == self.target)):
            # 11/12 밤: 차로 안에서는 차선 인식 우선. 한쪽 선만 보여도 lane_keep_blend_single 만큼 차선 노드 조향
            a = p.lane_keep_blend if ln.get('status') == 'BOTH' else p.lane_keep_blend_single
            k = max(-kmax, min(kmax, (1.0 - a) * k + a * float(ln['kappa'])))
            self.blend_used = True
        return v * k

    def _edge_danger(self, y, psi):
        """바깥 차로에서 바깥쪽으로 가는데 edge_look 앞에서 차체 앞 모서리가 노란선(가장자리 - edge_margin)을 넘나."""
        p = self.p
        if not p.edge_guard or abs(y) < p.edge_from * p.lane_width:
            return False                                # 바깥 차로 안쪽까지 들어온 뒤에만 (옮기는 중간엔 안 끼어듦)
        side = 1.0 if y > 0 else -1.0
        out = side * psi                                # + 면 바깥쪽으로 향함
        edge = (p.num_side_lanes + 0.5) * p.lane_width - p.edge_margin
        corner = abs(y) + p.robot_half_width * math.cos(psi) + p.robot_front * max(0.0, math.sin(out))
        pred = corner + p.edge_look * max(0.0, math.sin(out))
        return corner > edge or pred > edge

    def _why_text(self):
        return ' | '.join(f'{lane_name(k)} {v}' for k, v in sorted(self.lane_why.items(), reverse=True))

    def status(self):
        return {
            'state': self.state,
            'x': round(self.pose.x, 3), 'y': round(self.pose.y, 3),
            'psi_deg': round(math.degrees(self.pose.psi), 1),
            'target': self.target,
            'now_lane': self.nearest_lane(self.pose.y),
            'emergency': self.emergency, 'blocked': self.blocked, 'squeezing': getattr(self, 'squeezing', False), 'edge_guard': getattr(self, 'edge_guarded', False),
            'hit_m': None if self.hit_dist == INF else round(self.hit_dist, 2),
            'recovering': self.rec is not None or self.rb is not None, 'recover_n': self.rec_n,
            'rollback': self.rb is not None, 'rollback_m': round(self.rollback_m, 2),
            'cam_lane': self.lane_id, 'lane_id_fix_n': self.lane_id_fix_n,
            'lane_blend': getattr(self, 'blend_used', False),
            'lane_fix_n': self.lane_fix_n, 'lane_fix_dy_cm': round(self.lane_fix_dy * 100, 1),
            'exit_pref': self.exit_pref,
            'lane_why': {str(k): v for k, v in self.lane_why.items()},
            # 확정 장애물 (앞 1.5 m 안, 차 기준 x, 옆 y 범위). 실차에서 '어느 차로에 있다고 보는지' 확인용
            'obs_near': [[round(o.xmin - self.pose.x, 2), round(o.ymin, 2), round(o.ymax, 2)]
                         for o in self.obs if o.hits >= self.p.confirm_hits
                         and -0.5 < o.xmin - self.pose.x < 1.5][:6],
            'n_obs': len([o for o in self.obs if o.hits >= self.p.confirm_hits]),
            'align_dy_cm': round(self.align_dy * 100, 1),
            'align_deg': round(math.degrees(self.align_th), 2),
            'section_length': round(self.p.section_length, 3),
            'exit_plan': self.exit_info.get('plan'),
            'exit_need_lanes': self.exit_info.get('need_lanes'),
            'exit_need_m': self.exit_info.get('need_m'),
            'exit_rem_m': self.exit_info.get('rem_m'),
        }
