# 11/20 M7 점선 방식 (m7_impl:=dash, 기본)

기존 M7(녹화 재생 replay)을 **점선 기준 회피 + M2 녹화 재생**으로 바꿈. 되돌리기: `m7_impl:=replay`.

## 하는 일
- 3차선 시작선에서 `obstacle_node`(m7_link)가 켬 (replay와 같은 방식·같은 파라미터: `section_length`, `start_heading_deg`, `start_y`)
- 가운데 차로 중심에서 **가운데|오른쪽 차로 사이 점선**으로 0.9 m 동안 옮긴 뒤 점선 따라 직진
- 정차 LIMO: 가운데 차로 → 오른쪽으로, 오른쪽 차로 → 왼쪽으로 붙어서 비킴 (옆 여유 5 cm). 연속 회피는 점선 복귀 없이 바로 S자
- 왼쪽 차로는 회피에 안 씀. 같은 거리에 양쪽 막힘 = 다가가 정지 후 재판단 반복
- 위치: odom + IMU, 라이다 LIMO 중심(차로 중심 맞춤)·상자 방향, 카메라 점선, 노란선, **섬 앞 표지판 막대(앞뒤 위치)**
- 안전: 차 외곽선을 지금 곡률 + 조향 지연 동안 낸 곡률로 굴려 라이다 점에 닿으면 정지
- 멈추면 0.8 s 뒤 재판단: 길 있으면 감 / 다른 곡률로 비켜 감 / 머리를 목표 쪽으로 돌리며 후진 20 cm (최대 4번, 0.5 m 나아가면 다시 채움) / 같은 자리면 계획 여유 1 cm씩 줄임 (충돌 검사 여유는 그대로)
- **M2**: 섬 앞 꼭짓점 - 1.0 m(replay와 같은 녹화 시작 거리) 0.8 m 전부터 갈래 쪽 차로 중심(A = 가운데, B = 오른쪽)으로 옮기고, `paths/m2_2-A.csv` / `m2_3-B.csv`를 남은 옆 오차를 줄이며 재생 → 갈래 안에서 끝 (`/avoid/done`, 끝 차로 +1/-1 → FSM M2 차선 주행)

## 파일
| 파일 | 내용 |
|---|---|
| `limo_obstacle_avoid/m7_dash_core.py` | 판단 (ROS 없음) |
| `limo_obstacle_avoid/m7_dash_node.py` | avoid_node 자리 노드 (`/avoid/*`, `/fork_direction`, 카메라 점선) |
| `limo_obstacle_avoid/m7_dash_sim.py` | 오프라인 시뮬 (`--orient side/long/mixed`, `run(..., integrated=True)`) |
| `config/m7_dash_params.yaml` | 값 (`/**` 키 → 노드 이름 상관없음) |
| `launch/m7_dash.launch.py` | 단독 시험 (점선 위에서 켬, `/cmd_vel` 직접) |
| `autorace_bringup/launch/autorace.launch.py`, `full.launch.py` | `m7_impl` 기본 `dash` |

## 시뮬레이션 (11/20)
- 팀 주행 그대로(가운데 차로 중심에서 켬 → 점선 → M2 A/B 갈래), 1차 대회 배치: 흔들기 0 / 0.4에서 96/96 성공, 접촉 0, 노란선·3차로 침범 0, 최소 간격 4.5 cm
- 크게 흔든 배치(±0.35 m): 42/48. 실패 6판은 모두 **가운데 차로 첫 장애물이 시작선 0.8 m 안 + 같은 거리 왼쪽 차로도 막힘** — 회전반경으로 오른쪽으로 못 비킴 (접촉 없이 정지·재판단 반복)
- 단독(점선 위에서 켬): 크게 흔든 배치 30/30, 번갈아 간격 0.8 m까지 통과
- 값: lookahead 0.25, latency 0.2, shift_len 0.7, speed 0.20 (좁을 때 0.14), clear 0.05 (54조합 x 20판 탐색)

## 실차에서 맞출 것
- `yellow_r_y`(오른쪽 노란선 위치), 실차에서 좌우로 떨면 `lookahead 0.30`
- M2 녹화 시작 거리 `m2_start_gap`(1.0 m)은 replay와 같음 — 녹화를 다시 하면 둘 다 바꿀 것
- 점선에서 A로 갈 때는 가운데 차로로 미리 옮긴 뒤 `m2_2-A`를 씀: 섬 앞 가운데 차로에 장애물이 있으면 최대 0.4 m 미루고 그 뒤엔 그냥 시작 (안전 정지는 그대로)
