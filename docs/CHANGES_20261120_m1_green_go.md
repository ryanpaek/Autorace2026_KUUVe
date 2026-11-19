# 11/20 M1 신호등: 서 있다가 초록에 출발 안 함 → 처음 30 cm 방향 유지 (`m1_go_heading_mm`)

근거: 캡틴 실차 증상 + 코드 추정 + 오프라인 시험. **실차 확인 전** (`/lane/status` source 확인 필요).

## 증상 (캡틴 실차)
- 빨강·노랑에 정지선에서 잘 섬. 빨·노·초 검출도 됨.
- 초록으로 바뀌어도 차가 안 움직임. 다른 구간(M3 등)의 정지·출발은 정상.

## 원인 (코드 추정)
1. 정지선에 서 있는 동안 카메라에 차선이 안 보이면 차선 노드 `lost_frames`가 계속 늘어남 (차가 서 있어도).
2. `lost_hold_frames`(30, 약 1초)를 넘으면 `source=stop`, 속도 0 (`lane_core.py` 선 놓침 처리).
3. 초록 → 미션 `go`는 차선 모드 `normal` 0.30 m/s를 보냄. 정지 때도 `normal`이라 모드가 안 바뀌어 차선 노드가 초기화되지 않음.
4. 차가 안 움직이니 선이 다시 보일 일도 없음 → 영원히 정지.
- 인식(`class_commands`)의 빨강→초록 해제는 시험으로 확인: 초록 7프레임 뒤 `RESUME_NORMAL` 정상.
- 차선 노드 쪽 재현: `src/autorace_lane/test/test_m1_go_lost_1120.py` (차선 코드는 안 고침).

## 고침 (`fsm.py` `_s_M1_TRAFFIC`, 신호등만)
- **서 있다가(stopped) 초록으로 출발할 때만** 처음 `m1_go_heading_mm`(300 mm) 동안 차선 모드 `heading`, `yaw_ref` = 선 때 방향.
  - 모드가 바뀌면 차선 노드가 `reset()` → `lost_frames` 0. heading은 선 없이도 방향 유지로 속도가 나감.
  - 300 mm 뒤 `normal`로 돌아갈 때 또 초기화 → 선이 안 보여도 1초 버팀(hold) 동안 차선을 다시 찾음.
- 달리면서 초록으로 통과(서지 않음)하면 예전처럼 차선 그대로.
- 다른 구간은 안 바뀜.
- 300 mm 근거: 횡단보도 깊이 약 570 mm의 절반쯤. 추정값이며 실차로 맞출 것.
- 끄기: `ros2 param set /mission_node m1_go_heading_mm 0.0`

## 시험
- autorace_mission 412 통과 (새 4개: 섰다 출발 → heading 300 mm 뒤 차선 / 끄면 예전 / yaw 없을 때 / 달리며 통과는 차선)
- autorace_lane 100 통과 (새 2개: 서서 선 놓침 → normal로는 계속 stop / heading으로 바꾸면 출발)
- autorace_obstacle 106, limo_obstacle_avoid 180 통과
