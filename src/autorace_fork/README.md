# autorace_fork — 미션 2 갈림길(A/B)

YOLO가 본 갈림길 표지(`fork_a`/`fork_b`)로 A/B를 정하고, **지금 몇 차선에 있는지**를 기준으로
**몇 차선을 옮겨야 하는지** 판단해 회피 노드에 출구 방향을 넘깁니다. 실제 차선 이동은 회피 노드
(`limo_obstacle_avoid`, v0.3의 "갈림길 출구 차선 이동")가 합니다.

## 차선 번호 (진행 방향 기준)

| 번호 | 이름 | 갈림길 |
| --- | --- | --- |
| +1 | 1차선 (왼쪽) | A |
| 0 | 2차선 (가운데) | 가운데 섬이 막음 |
| -1 | 3차선 (오른쪽) | B |

| 지금 차선 | A 표지 | B 표지 |
| --- | --- | --- |
| 1차선 | 유지 | **오른쪽 2차선** |
| 2차선 | 왼쪽 1차선 | 오른쪽 1차선 |
| 3차선 | **왼쪽 2차선** | 유지 |

## 왜 회피 노드가 옮기나
3차선 구간은 가운데 섬 앞 끝에서 A/B 두 길로 갈라지고, 섬을 지나면 반대 길로 옮길 수 없습니다.
그 직전까지 바퀴는 회피 노드가 잡고 있으므로(정차 LIMO가 아직 옆에 있을 수 있음), 다른 노드가 같이
조향하면 명령이 충돌합니다. 그래서 이 패키지는 **판단과 전달**, 회피 노드는 **장애물을 보면서 이동**을 맡습니다.

## 필요한 거리 (실측 회전반경 0.53m, 차선 간격 0.52m, 회피 노드와 같은 제어식으로 계산)

| 이동 | 차체가 섬 구역을 벗어날 때까지 | 차선 중심에 자리 잡을 때까지 |
| --- | --- | --- |
| 1차선 | 약 0.74 m | 약 1.41 m |
| 2차선 | 약 1.32 m | 약 1.96 m |

여기에 여유(`exit_margin` 0.2m)와 앞범퍼(0.2m)를 더해, **2차선 이동은 섬 앞 끝 약 1.7m 전, 1차선은 약 1.1m 전**까지
표지를 알아야 합니다. 그보다 늦으면 무리하지 않고 지금 쪽을 유지합니다(충돌·차로 이탈 없이).

## 동작
1. `/perception/detections`(팀 YOLO, JSON)에서 `fork_a`/`fork_b`를 최근 8프레임 중 5번 이상 보면 결정 (FSM과 같은 기준)
2. FSM이 이미 `fork_choice`를 정했으면 그걸 따름 (`follow_fsm`, 같은 YOLO를 두 곳에서 따로 정해 엇갈리지 않게)
3. `/fork_direction`에 A/B 발행 → 회피 노드가 섬 앞 끝 전에 출구 차선으로 이동
   - 지나갈 차선과 출구 차선이 비고, 섬까지 거리가 충분하면 바로 이동 (2차선은 한 번에)
   - 정차 LIMO가 막고 있으면 지나갈 때까지 기다렸다가 이동
   - 거리가 모자라면 지금 쪽 유지
4. `/fork/state`(JSON)에 결정, 지금 차선, 이동 차선 수, 섬까지 남은 거리/필요 거리, 상태 문장 발행

## 토픽

| 방향 | 토픽 | 형식 |
| --- | --- | --- |
| 입력 | `/perception/detections` | String JSON `{"detections":[{"cls":"fork_a","conf":0.8,"bbox":[...]}]}` |
| 입력 (`input_mode: text`) | `/fork/sign` | String `A` / `B` (`fork_a`, `left`, `좌`도 됨) |
| 입력 | `/avoid/state` | 회피 노드 상태 (지금 차선, 목표 차선, 섬까지 거리) |
| 입력 | `/mission/state` | 팀 FSM (지금 미션, `fork_choice`) — 없으면 항상 동작 |
| 출력 | `/fork_direction` | String `A` / `B` → 회피 노드 |
| 출력 | `/fork/state` | String JSON (상황) |

## 시험 결과 (ROS 없는 가상 환경)
- 갈림길 시뮬레이션(`limo_obstacle_avoid/tools/fork_sim.py`, 무작위 7대 배치, 마지막 차~섬 2.5m):
  표지를 섬 2.0m 이상 전에 알면 **16/16** (2차선 이동이 필요한 5건 모두 성공). 예전 방식은 11/16(2차선 이동 0건).
  섬 1.5m 이하로 늦으면 2차선 이동은 물리적으로 불가 → 지금 쪽 유지. 모든 경우 충돌 0.
- 노드 통합(가짜 ROS: 회피 노드 + cmd_mux + 브리지 흉내 + fork_node): 1차선에서 B → 2차선 이동, 1차선에서 A → 유지,
  가운데에서 섬 2m 전 B → 1차선 이동, FSM 결정 따르기 모두 통과. 너무 늦은 B는 충돌 없이 1차선 유지.
- 레포 전체 `python3 -m pytest src -q` 통과. **실차에서는 아직 안 돌려 봤습니다.**

## 실행

빌드 (환경 메뉴 1번):
```
cd ~/autorace_ws && colcon build --symlink-install --packages-select autorace_fork limo_obstacle_avoid autorace_bringup
source install/setup.bash
```

**① 판단만 (차 정지, 회피 노드 없이)**
```
ros2 launch autorace_fork fork.launch.py input_mode:=text
```
```
ros2 topic pub -t 6 -r 10 /fork/sign std_msgs/msg/String "{data: B}"
```
```
ros2 topic echo /fork/state
```
통과: 로그에 `갈림길 B 결정`, `/fork/state`의 `decision`이 `B`

**② 회피와 함께 (단독 시험, 바퀴 움직임)** — 정차 LIMO를 1차선으로 밀려나게 배치하고
```
ros2 launch limo_obstacle_avoid s6_avoid_run.launch.py
```
```
ros2 launch autorace_fork fork.launch.py input_mode:=text
```
```
ros2 topic pub -t 3 /avoid/enable std_msgs/msg/Bool "{data: true}"
```
차가 1차선으로 밀려난 뒤(섬 앞 끝 2m 이상 전):
```
ros2 topic pub -t 6 -r 10 /fork/sign std_msgs/msg/String "{data: B}"
```
통과: 회피 노드 로그 `갈림길 출구로 이동: 1차선(A쪽) → 3차선(B쪽) (2차선, ...)`, 섬 앞 끝에서 3차선 쪽

**③ 팀 스택 (YOLO 포함)**
```
ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true use_fork_node:=true
```
```
ros2 topic echo /fork/state
```

## 설정 (`config/fork_params.yaml`)

| 값 | 기본 | 설명 |
| --- | --- | --- |
| `input_mode` | detections | `text`면 `/fork/sign` 문자열 |
| `cls_a`, `cls_b` | fork_a, fork_b | YOLO 클래스 이름 (detector_node와 같게) |
| `det_conf` | 0.5 | 신뢰도 하한 |
| `min_box_h` | 0 | 표지 상자 높이 하한(픽셀). 멀리 있는 표지 무시 |
| `vote_window`, `vote_need` | 8, 5 | 다수결 |
| `follow_fsm` | true | FSM이 정했으면 따름 |
| `allow_change` | false | 한 번 정하면 유지 |

회피 노드 쪽 (`limo_obstacle_avoid/config/avoid_params.yaml`): `fork_shift`(켜기), `exit_margin`(섬 앞 여유 0.2m), `exit_min_time`(0.6초).

## 실차에서 확인할 것
1. **표지가 섬 앞 끝 몇 m 전에 안정적으로 인식되는지** — 2차선 이동은 약 1.7m 전까지 필요. 모자라면 YOLO 쪽 입력 해상도·`det_conf`·`vote_need`를 조정
2. 2차선 이동 때 차가 실제로 섬 앞 끝 전에 반대편으로 넘어가는지 (회피 노드의 `max_heading_deg`, `lookahead`와 실제 조향이 맞는지)
3. 실제 표지 위치가 정해지면 `fork_sim.py`의 표지 시점으로 다시 시험
