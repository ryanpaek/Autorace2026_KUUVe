# limo_obstacle_avoid v0.2 — 3차선 정적 장애물(정차 LIMO) 회피

미션 7(정차 LIMO 7대를 차선 변경으로 회피, 10점)을 위한 ROS 2 Humble 패키지입니다.
GPS 없이 **미션 시작 순간을 원점으로 하는 구간 좌표** 안에서 차선 단위로 판단하고,
**정차 LIMO들의 위치로 내 좌표의 방향·옆 오차를 계속 보정**합니다.

## 동작 원리

1. **시작(`/avoid/enable true`)**: 그 순간 위치를 원점, 진행 방향을 +x, 가운데 차선을 y=0 으로 잡음. 차선 중심 y = −0.55 / 0 / +0.55
2. **내 위치**: odom 이동거리 × IMU 방향으로 누적 (odom 방향은 조향 중 틀려서 안 씀)
3. **장애물 지도**: 라이다 점을 구간 좌표로 옮겨 묶고, 3번 이상 보인 것만 차로 인정해 기억 (가려져 안 보여도 유지)
4. **정렬 보정 (v0.2 추가)**: 정차 LIMO 중심이 차선 중심선에 오도록 내 좌표를 조금씩 돌리고 옮김 → 출발 방향이 몇 도 틀어져도 바로잡힘
5. **차선 선택**: 내 차선 앞 여유 < `trigger_dist` 이면, 옮겨 가는 길이 비어 있고 더 멀리 빈 옆 차선으로. 갈 곳이 없으면 정지
6. **주행**: 목표 차선 중심선을 pure pursuit 로 추종 (최소 회전반경 0.53m 제한)
7. **종료**: 가운데 섬(가상 장애물)을 지나 차선에 붙으면 `/avoid/done`, 제어권을 차선추종에 넘김

## v0.1 대비 바뀐 점

| 구분 | 내용 | 이유 |
| --- | --- | --- |
| 정렬 보정 | 라이다로 본 정차 LIMO 위치로 방향·옆 오차 보정 (`lane_align`) | 출발 방향 3°만 틀어져도 무작위 배치 성공률 23/40 → 40/40 |
| 실측값 | `min_turn_radius` 0.45 → **0.53**, `wheelbase` 0.20 → **0.222** | 실측(회전반경 약 53cm)과 팀 브리지 값에 맞춤 |
| 센서 끊김 | `/odom`·`/imu`·`/scan` 이 끊기면 정지하고 기다림, 다시 들어오면 계속 | 이전엔 마지막 판단대로 계속 달림 |
| cmd_mux 안전 | 회피 중 회피 명령이 끊기면(노드 죽음) 차선추종으로 넘기지 않고 **정지** | 이전엔 0.3초 뒤 차선추종이 장애물 구간을 직진 |
| 출력 방식 | `out_mode: twist`(/cmd_vel → 팀 브리지) 또는 `ack`(/limo/ack_cmd 직접) | 아커만 드라이버가 /limo/ack_cmd 만 받음 |
| IMU | `imu_mode: auto` — 방향값이 비었거나 안 변하면 자이로 적분, 자이로 편향은 정지 중 자동 측정. 미션 중에는 방식 고정 | 이전엔 손으로 설정, 편향 보정 없음 |
| Ctrl+C | 모든 명령 노드가 정지 명령(+ active=false)을 보내고 종료 | Humble 에선 통신이 먼저 닫혀 정지 명령이 안 나갈 수 있었음 |
| 연결 점검 | cmd_mux·s4 가 출력 토픽 받는 노드 없음/다른 발행자 충돌을 알려 줌. s4 는 문제 있으면 출발 안 함 | 명령이 차에 안 닿는 상황을 바로 알 수 있게 |
| 파일명 | `s4_lane_change_test.py` → `s4_lane_change.py` (실행 이름은 그대로) | `*_test.py` 이름 때문에 레포 전체 pytest 가 멈춤 |
| 시험 | `test/` (pytest 8개), `tools/stress_sim.py` (무작위 배치 × 실차 오차) | 오류 없이 넣을 수 있는지 확인 |

## 팀 스택(autorace_bringup)에 넣어 쓰기

단계별 시험(아래 ⑤~⑦)은 이 패키지 혼자 `cmd_mux`로 하고, 본 주행은 팀 스택에 붙인다.

```
ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true                  # 회피까지 실제로
ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true m7_dry_run:=true # 판단만 (바퀴는 차선 주행)
```

```
mission_node ──/mission/state──▶ obstacle_node (m7_link.py)
   ▲                               │ M7 + 앞바퀴가 3차선 시작선 → section_length 설정 후 /avoid/enable
   │                               │ fork_choice → /fork_direction
   │                               │ /avoid/active·done → /obstacle/state의 m7_active·m7_done·m7_exit
   └──/obstacle/cmd_vel ◀── avoid_node (/avoid/cmd_vel 을 remap)
```

| 걸리던 점 | 처리 |
| --- | --- |
| `cmd_mux`와 `mission_node`가 둘 다 `/cmd_vel` 발행 | 본 주행에선 `cmd_mux`를 안 켬. `/cmd_vel`은 `mission_node` 하나 (비상정지 `/mission/estop`도 그대로 동작) |
| `obstacle_node`가 `/obstacle/cmd_vel`에 0을 냄 | M8 라바콘에서만 냄 |
| FSM은 시작선 1.68 m 앞(코너 전)에서 M7이 됨 | 앞바퀴가 시작선에 닿을 때 켬: 카메라 흰 선 + 오도메트리 → 없으면 도면 위치 s → 없으면 M7 진입 후 거리. 차선 방향이 3° 넘게 틀어져 있으면 최대 30 cm 미룸 |
| 켜는 위치가 늦으면 섬까지 거리가 달라짐 | 켜기 직전에 `section_length` = 섬 끝 − 지금 위치로 넣음 |
| FSM이 섬 끝 30 cm 앞에서 M2로 넘어감 (회피는 섬 끝 30 cm 뒤에 끝남) | M7은 `m7_done`으로 끝냄. 안 오면 섬 끝 + 50 cm에서 강제로 넘기고 회피 노드를 끔 |
| 갈림길 방향 | FSM 투표 결과를 `/fork_direction`으로. 끝까지 못 읽으면 회피가 끝난 차로를 M2가 그대로 씀 (`fork_from_m7_exit`) |
| 회피 노드가 켜지기 전 명령 | M7이어도 회피 노드가 실행 중(`m7_active`)일 때만 그 명령을 쓰고, 그 전엔 차선 주행 |

설정은 `autorace_bringup/config/params.yaml`의 `obstacle_node: m7_*`, `mission_node: m7_*`.
상태 보기: `ros2 topic echo /obstacle/state` → `m7_phase`(idle→armed→running→done), `m7_how`(line/route/distance), `m7_section_m`.

**실측해서 맞출 값**
- 3차선 한 칸 폭: 이 패키지 `lane_width`(0.55)와 팀 FSM `three_lane_width_mm`(500)을 같은 실측값으로.
  0.50인데 0.55로 두면 바깥 차로 중심을 5 cm 바깥으로 잡는다 (차 바깥 끝 ~ 노란 선 8 cm). 정렬 보정은 폭 비율 오차를 못 고친다.
- `autorace.launch.py`의 브리지는 `wheelbase`를 넘기지 않아 0.2로 돈다 (캘리브레이션 유효값 0.222).
  회피 노드는 곡률(angular.z = v × κ)로 명령하므로 브리지 축거가 틀리면 실제 회전이 약 11% 커진다.

**옆 여유 값에 대해 (시뮬레이션 확인)**: `side_margin`을 0.05~0.10으로 바꿔도 무작위 배치 140건 결과·최소 간격이 같았다
(차로 판단에만 쓰이고 지나가는 간격은 차로 폭이 정함). `clear_back`을 0.05/0.10/0.15로 올리면 최소 간격이 4.6/7.9/5.3 cm,
성공이 140/133/127건으로 오히려 나빠졌다 → 기본값(0.05, 0.0) 유지.

## 시험 결과 (ROS 없이, 실제 회전반경 0.53m 가상 차량)

| 조건 (무작위 7대 배치 40개, 지나갈 수 있는 배치만) | 성공 | 충돌 |
| --- | --- | --- |
| 이상적 | 40/40 | 0 |
| 조향 지연 0.15초 | 40/40 | 0 |
| 출발 방향 +3° | 40/40 | 0 |
| 출발 방향 −5° | 40/40 | 0 |
| 출발 옆 6cm | 40/40 | 0 |
| 정차 LIMO 위치 ±6cm, 각도 ±10° | 25/25 | 0 |
| 복합 (지연 + 3° + 6cm + odom 4% + IMU 드리프트 2°) | 40/40 | 0 |

노드 통합 시험(가짜 ROS 환경에서 avoid_node + cmd_mux + 브리지·차선추종 흉내): 브리지 경유, ack 직접, IMU 방향값 0/고정, 출발 오차, 회피 노드 사망 시 정지 유지, 라이다 2초 끊김 후 재개, 실행 중 파라미터 변경, 갈림길 A/B, dry_run, Ctrl+C 정지 전송, s1~s4 단계 노드 모두 통과.
**실제 LIMO 에서는 아직 돌려 보지 않았습니다.** 아래 단계 순서대로 확인하세요.

## 설치 (LIMO, 한 번)

접속 후 환경 메뉴 **1번(Global)** 에서 (Conda 에서 빌드하면 파이썬이 섞일 수 있음):
```
cd ~/autorace_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select limo_obstacle_avoid
source ~/autorace_ws/install/setup.bash
ros2 pkg executables limo_obstacle_avoid
```
7개(avoid_node, cmd_mux, s0~s4)가 나오면 성공. `.py`/`yaml` 수정은 다시 빌드 불필요, 파일을 새로 추가하면 다시 빌드.

## 명령 출력 방식 정하기

| | `out_mode: twist` (기본) | `out_mode: ack` |
| --- | --- | --- |
| 나가는 곳 | `/cmd_vel` (Twist) | `/limo/ack_cmd` (AckermannDrive) |
| 팀 브리지 | **켜기** `ros2 run autorace_mission ackermann_bridge --ros-args -p wheelbase:=0.222 -p cmd_timeout_sec:=1.0` | **끄기** |
| 장점 | 팀 차선추종·브리지 구성 그대로 | 노드 하나 덜 거침 |

차선추종 노드는 출력 토픽을 `/lane_follow/cmd_vel` 로 remap 해야 cmd_mux 가 둘 중 하나만 내보냅니다.

## 단계별 시험 (창 T1 드라이버 / T2 시험 노드 / T3 명령)

**T1 — 드라이버(계속 켜 둠), 조종기는 끄기**
```
ros2 launch limo_base limo_base_ackerman.launch.py
```
라이다가 따로면 같이 켜기 (`ros2 pkg list | grep -i -E "ydlidar|lidar"` 로 찾기).

**① 토픽 확인 [T3]** → `/scan /odom /imu` 가 있어야 함
```
ros2 topic list | grep -E "scan|odom|imu"
```

**② 라이다 방향 (정지) [T2]** → 앞에 둔 물체가 "앞", 왼쪽이 "왼쪽"
```
ros2 launch limo_obstacle_avoid s1_scan_check.launch.py
```
앞↔뒤 반대면 `scan_yaw_offset_deg: 180.0`, 왼↔오 반대면 `scan_flip: true`

**③ 차선 판단 (정지, 진짜 LIMO 로) [T2]** → 가운데 앞 LIMO 면 "+1 또는 −1 으로 회피"
```
ros2 launch limo_obstacle_avoid s2_lane_view.launch.py
```

**④ 위치·IMU 확인 [T2][T3]** → 똑바로 3m 가면 x≈3.0, y≈0. 출력의 `IMU=` 값 확인
```
ros2 launch limo_obstacle_avoid s3_local_pose.launch.py
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
원점 다시 잡기(teleop 끄고): `ros2 topic pub -t 3 /avoid/reset std_msgs/msg/Empty`
코너를 한 번 돌면 IMU 방향값 판별 로그가 나옵니다. 결과를 `imu_mode` 에 고정해 두면 가장 안전합니다.

**⑤ 차선 변경만 (★ 바퀴 움직임) [T2][T3]** → 옆 이동 0.55m
```
ros2 launch limo_obstacle_avoid s4_lane_change_test.launch.py change_lanes:=1
```
브리지 없이: `... change_lanes:=1 out_mode:=ack`
```
ros2 topic pub -t 3 /avoid/enable std_msgs/msg/Bool "{data: true}"
```
멈추기: T2 에서 `Ctrl+C` (또는 `{data: false}`). `change_lanes:=-1` 도 확인.

**⑥ 회피 판단만 (바퀴 안 움직임) [T2][T3]** → LIMO 약 1m 앞에서 "차선 0 → ±1" 로그
```
ros2 launch limo_obstacle_avoid s5_avoid_dryrun.launch.py
```
```
ros2 topic pub -t 3 /avoid/enable std_msgs/msg/Bool "{data: true}"
```
```
ros2 topic echo /avoid/state
```

**⑦ 실제 회피 [T2][T3]** → "회피 종료: 구간 통과"
```
ros2 launch limo_obstacle_avoid s6_avoid_run.launch.py
```
브리지 없이: `ros2 launch limo_obstacle_avoid s6_avoid_run.launch.py out_mode:=ack`
첫 주행은 느리게 (T3):
```
ros2 param set /avoid_node v_cruise 0.15
```
```
ros2 param set /avoid_node v_change 0.12
```
```
ros2 topic pub -t 3 /avoid/enable std_msgs/msg/Bool "{data: true}"
```
상태 보기: `ros2 topic echo /avoid/state` — `align_deg`, `align_dy_cm` 이 정렬 보정량(보통 수 도, 수 cm 이내)

## 대회장에서 꼭 할 것

- `section_length`: 회피 시작 지점(enable 순간 앞바퀴 위치)부터 **가운데 섬 앞 끝까지** 줄자로 재서 yaml 에 입력 (도면 4.38 m. 팀 스택에서는 `obstacle_node`가 자동으로 넣음)
- 시작 전 **1초 이상 정지**: 자이로 편향 자동 측정 (출발선에서 대기하는 동안이면 충분)
- 갈림길 방향을 알면 `/fork_direction` 에 `A`/`B` 를 보내거나 `ros2 param set /avoid_node exit_lane A`

## 튜닝 가이드

| 증상 | 바꿀 값 |
| --- | --- |
| 차선 변경이 늦어 앞차에 너무 붙음 | `trigger_dist` ↑ (1.0 → 1.2) |
| 옆 차선 차를 스치듯 지나감 | `side_margin` ↑, `clear_back` ↑ |
| 차선 변경 후 좌우로 흔들림 | `lookahead` ↑ (0.5 → 0.6) |
| 옮기는 데 너무 오래 걸림 | `lookahead` ↓ (회전반경 0.53 아래로는 안 꺾임) |
| 정렬 보정이 이상하게 큼 (`align_deg` 5° 이상) | 정차 LIMO 가 차선 중앙에서 많이 벗어난 경우 → `lane_align: false` 로 비교 |
| 갈 곳 없다며 멈춤 | 실제 배치가 회전반경으로 지나갈 수 없는지 확인 (앞줄 차가 있던 차선으로 들어가려면 줄 간격 약 1.35m 필요) |

## ROS 없이 시험 (노트북)

```
cd src/limo_obstacle_avoid
python3 -m pytest -q test                     # 8 passed
python3 tools/offline_sim.py --all --no_pdf   # 기존 배치 20건
python3 tools/stress_sim.py --n 40            # 무작위 배치 × 실차 오차 (수 분)
python3 tools/stress_sim.py --only 6          # 복합 오차 조건만
```

## 안전 수칙

- ⑤단계부터 차를 손으로 잡을 수 있는 거리에서. 와이파이가 끊기면 SSH 로 정지 명령을 못 보냄
- 회피 노드가 죽으면 cmd_mux 가 정지를 유지함 → 풀려면 `ros2 topic pub --once /avoid/active std_msgs/msg/Bool "{data: false}"`
- cmd_mux 까지 죽으면 드라이버가 마지막 속도를 유지할 수 있음 → 드라이버 창 Ctrl+C 또는 조종기
- `-t 3` 은 1초 간격으로 3번 보내므로 명령이 들어가기까지 최대 1초 정도 걸릴 수 있음
