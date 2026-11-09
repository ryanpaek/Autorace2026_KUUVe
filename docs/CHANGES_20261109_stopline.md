# 변경 내역 (2026-11-09): 횡단보도 끝선에서 서는 문제, '천천히' 글자에 끌려가는 도면 위치

근거: 11/09 통주행 bag `1109_152817_full_01` (main 486c50f). 분석 전문은 프로젝트 문서 `claude/run-1109-analysis.md`.

## 증상과 원인

| 증상 | 원인 (bag) |
| --- | --- |
| M3·M1이 정지선이 아니라 횡단보도 끝선 근처에서 섬 (정지선보다 +47cm / +67cm) | 정지선이 카메라 사각지대(앞바퀴 약 350mm 안)로 들어간 뒤, 뒤따르는 횡단보도 블록이 BEV 맨 아래 행에 잘려 "348mm 앞 선"으로 계속 잡힘. 예측 거리와 150mm 안이라 남은 거리가 350 근처에서 계속 리셋 -> 블록을 다 지나서야 카운트다운 |
| 도면 위치(route s)가 M3 앞에서 약 +55cm 틀어짐 | 정지선 400~600mm 앞 '천천히' 글자를 흰 가로선으로 봄. 순서 짝짓기 범위(1200mm)라 글자가 `m3_stop_line`으로 맞춰짐. 막대 판정 비율 0.3이라 글자 행도 "횡단보도 선"이 되기도 함 |

## 바꾼 것

| 파일 | 내용 |
| --- | --- |
| `autorace_lane/lane_core.py` `detect_stoplines` | BEV 맨 아래·맨 위 행에 닿은 띠는 버림 (`stopline_skip_edges`). 행마다 흰 구간 평균이 3.9 넘는 띠(글자·블록 가장자리)는 선이 아님 (`stopline_max_segments`). 막대 판정 비율 0.3 -> 0.6 (`bar_row_ratio`; bag: 정지선 0.76~1.0, 글자 0.47 이하) |
| `autorace_mission/fsm.py` `StopApproach` | 남은 거리 450mm 아래부터 카메라 갱신을 끊고 오도메트리로만 셈 (`stop_freeze_mm`). 처음 잡는 선이 도면 예측과 300mm 넘게 다르면 버림, 사각지대에서 시작하면 도면 위치로 바로 셈 (`stop_route_gate_mm`, 0이면 끔). 디버그용 `how` (camera/route) |
| `autorace_mission/route.py` `RouteTracker` | 한 프레임에서 가장 가까운 선 대신 도면과 가장 잘 맞는 선 하나를 씀. `kind: crosswalk` 도면 선(정지선)은 막대 있는 선으로만, 도면 선이 본 선보다 400mm 넘게 뒤면 안 맞춤 (`crosswalk_gate_mm`, 지그재그 odom 과대 방향은 그대로 허용). 하나 건너뛰기는 건너뛸 선을 이미 지났을 때만 순서 짝짓기 범위 |
| `autorace_bringup/config/track_route.yaml` | `m3_stop_line`, `m1_stop_line`에 `kind: crosswalk` |
| `autorace_mission/mission_node.py` | 경로 추적에 `crosswalk_lines_mm`도 넘김 |
| 테스트 6개 추가 | 옛 코드에서 6개 모두 실패, 새 코드에서 통과. 전체 119 passed |

## bag 재생 결과 (옛 코드 vs 새 코드, 같은 영상으로 차선 노드부터 다시 돌림)

| 항목 | 옛 | 새 |
| --- | --- | --- |
| M3 정지선 900mm 앞 도면 위치 오차 | +609mm | +161mm |
| M3 정지선 300mm 앞 도면 위치 오차 | +419mm | -121mm |
| M3 정지 명령 위치 (정지선 기준) | +505mm | +50mm |
| M1 정지 명령 위치 | -667mm | -47mm |
| 처음 맞춘 `m3_stop_line` 오차 | +529 (글자) | -173 (정지선) |
| 놓친 도면 선 | - | 없음 (11개 모두 맞춤) |

- 기준 "실제 정지선 위치"는 카메라가 약 0.9m 앞에서 본 거리로 잡아서 자체 오차가 ±5cm쯤 있다.
- 개루프 재생이라 실제로 멈춘 뒤 밀리는 거리는 안 들어감.
- BEV 거리가 멀수록 짧게 나오는 것으로 보임 (같은 프레임에서 정지선~끝선 380mm, 도면 540mm). 줄자로 확인 필요.

## 실차에서 할 것

1. M3·M1 정지 후 앞바퀴 ~ 정지선 거리를 줄자로 재서 `stop_target_mm`(+면 선 앞)·`stop_brake_mm` 조정.
2. `ros2 topic echo /mission/state`의 `route.last_fix`가 M3 앞에서 `m3_stop_line`으로 맞춰질 때 `err_mm`가 수백 mm로 튀지 않는지.
3. 예전 동작으로 되돌리기: `stop_freeze_mm:=0.0`, `stop_route_gate_mm:=0.0` (mission_node), `stopline_skip_edges:=false`, `stopline_max_segments:=99.0`, `bar_row_ratio:=0.3` (lane_node).
