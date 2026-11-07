# M7 3차선 회피 명령어 정리

창은 모두 LIMO에 SSH 접속 → 환경 메뉴 **1번(Global)**. 조종기(RC)는 끈다.
`[T1]` 같은 표시는 터미널 창 번호. 자세한 설명은 [README.md](README.md).

## 0. 코드 받기 · 빌드 (한 번, 코드가 바뀌었을 때)

```bash
# 레포로 받는 경우 (워크스페이스 = 레포 루트)
cd ~/autorace_ws && git fetch origin && git checkout claude/blissful-bardeen-ovgg6s && git pull
```

```bash
# 압축 파일로 받는 경우: src/ 폴더를 워크스페이스에 덮어쓰기 (기존 src는 먼저 백업)
cd ~/autorace_ws && cp -r src src_backup_$(date +%m%d) && unzip -o ~/autorace_m7_avoid.zip -d ~/autorace_ws
```

```bash
# 빌드 (바뀐 4개 패키지만)
cd ~/autorace_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select limo_obstacle_avoid autorace_obstacle autorace_mission autorace_bringup
source ~/autorace_ws/install/setup.bash          # 새 창마다
ros2 pkg executables limo_obstacle_avoid         # avoid_node, cmd_mux, s0~s4 (7개)
```

## 1. 차 없이 시험 (LIMO나 노트북)

```bash
cd ~/autorace_ws && python3 -m pytest src -q                    # 102 passed
cd ~/autorace_ws/src/limo_obstacle_avoid
python3 tools/offline_sim.py --all --no_pdf                      # 배치 20건
python3 tools/stress_sim.py --n 20                               # 무작위 배치 × 실차 오차 (수 분)
```

## 2. 센서 켜기 (단독 시험·본 주행 공통)

```bash
ros2 launch limo_base limo_base_ackerman.launch.py               # [T1] 드라이버 (계속 켜 둠)
ros2 launch limo_base open_ydlidar_launch.py                     # [T2] 라이다
ros2 topic list | grep -E "scan|odom|imu"                        # [T3] /scan /odom /imu 있어야 함
```

## 3. 회피 패키지 단독 시험 (팀 스택은 끈 상태)

| 단계 | 명령 | 확인할 것 | 바퀴 |
| --- | --- | --- | --- |
| ② 라이다 방향 | `ros2 launch limo_obstacle_avoid s1_scan_check.launch.py` | 앞 물체가 "앞", 왼쪽이 "왼쪽" | 안 움직임 |
| ③ 차선 판단 | `ros2 launch limo_obstacle_avoid s2_lane_view.launch.py` | 가운데 앞 LIMO면 "+1 또는 −1로 회피" | 안 움직임 |
| ④ 위치·IMU | `ros2 launch limo_obstacle_avoid s3_local_pose.launch.py` + teleop | 3 m 직진에 x≈3.0, y≈0 | 손으로 조종 |
| ⑤ 차선 변경 | `ros2 launch limo_obstacle_avoid s4_lane_change_test.launch.py change_lanes:=1` | 옆으로 0.55 m | **움직임** |
| ⑥ 회피 판단 | `ros2 launch limo_obstacle_avoid s5_avoid_dryrun.launch.py` | LIMO 1 m 앞에서 "차선 0 → ±1" | 안 움직임 |
| ⑦ 실제 회피 | `ros2 launch limo_obstacle_avoid s6_avoid_run.launch.py` | "회피 종료: 구간 통과" | **움직임** |

```bash
# ⑤⑦은 팀 브리지가 필요 (out_mode 기본 twist). 브리지 없이 하려면 launch 뒤에 out_mode:=ack
ros2 run autorace_mission ackermann_bridge --ros-args -p wheelbase:=0.222 -p cmd_timeout_sec:=1.0
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard                    # ④ 손 조종
ros2 topic pub -t 3 /avoid/reset std_msgs/msg/Empty                     # ④ 원점 다시 잡기
ros2 topic pub -t 3 /avoid/enable std_msgs/msg/Bool "{data: true}"      # ⑤⑥⑦ 시작
ros2 topic echo /avoid/state                                            # 상태 보기
ros2 param set /avoid_node v_cruise 0.15                                # ⑦ 첫 주행은 느리게
ros2 param set /avoid_node v_change 0.12
```

## 4. 팀 스택에 붙여서 (본 주행)

카메라도 켠다: `ros2 launch orbbec_camera dabai.launch.py`. 단독 시험용 `s6`·`cmd_mux`·브리지 창은 **끈다**
(`autorace.launch.py`가 브리지를 켜고, `/cmd_vel`은 `mission_node` 하나만 낸다).

```bash
# 판단만 (바퀴는 차선 주행) — 실차 첫 확인용
ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true m7_dry_run:=true
```

```bash
# 실제 회피
ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true
```

```bash
# M7 구간만 연습: 차 앞바퀴를 3차선 시작선 30 cm 앞, 가운데 차로에 놓고
ros2 launch autorace_bringup autorace.launch.py m7_avoid:=true
```

```bash
# 다른 창에서. 보내는 즉시 출발한다 (도면 위치도 시작선 - 30 cm로 맞춰짐)
ros2 topic pub --once /mission/set_state std_msgs/msg/String "{data: M7_OBSTACLE}"
```
`start_state:=M7_OBSTACLE`로 시작하면 도면 위치가 안 맞춰져서 회피가 약 1.4 m 늦게 켜진다. 위처럼 `set_state`를 쓸 것.

```bash
ros2 topic echo /obstacle/state          # m7_phase: idle→armed→running→done, m7_how: line/route/distance, m7_section_m, m7_hold
ros2 topic echo /mission/state           # state, source(lane/obstacle), fork_choice
ros2 topic echo /avoid/state             # x, y, target(차로), align_deg, align_dy_cm
```

## 5. 자주 바꿀 값

| 값 | 어디서 | 명령 / 파일 |
| --- | --- | --- |
| 3차선 한 칸 폭 (실측) | 회피 `lane_width` + FSM `three_lane_width_mm` | `limo_obstacle_avoid/config/avoid_params.yaml`, `autorace_bringup/config/params.yaml` (둘 다 같은 값) |
| 회피 속도 | avoid_node | `ros2 param set /avoid_node v_cruise 0.2` |
| 차로 변경 시작 거리 | avoid_node | `ros2 param set /avoid_node trigger_dist 1.1` |
| 갈림길 직접 지정 | avoid_node | `ros2 topic pub --once /fork_direction std_msgs/msg/String "{data: A}"` |
| 켜는 위치 미세 조정 | obstacle_node `m7_enable_at_mm` | `params.yaml` (+면 시작선을 지나서) |
| 표지 못 읽을 때 갈림길 | obstacle_node `m7_fork_fallback` | `params.yaml`: `auto` / `A` / `B` |

## 6. 비상

```bash
ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}"      # 본 주행 (mission_node가 정지)
ros2 topic pub --once /avoid/enable std_msgs/msg/Bool "{data: false}"      # 회피만 중단
ros2 topic pub --once /avoid/active std_msgs/msg/Bool "{data: false}"      # 단독 시험에서 cmd_mux 정지 풀기
```

- 본 주행 중 회피 노드가 끝나기 전에 멈추면(Ctrl+C·죽음) FSM이 정지를 유지한다 (`/obstacle/state`의 `m7_hold: true`).
  다시 하려면: ① `set_state`로 `"{data: WAIT_START}"` (정지, 연결 초기화) ② 회피 노드를 다시 켜고 차를 시작선 30 cm 앞에 놓기
  ③ `"{data: M7_OBSTACLE}"` (바로 출발). M7 상태에서 곧바로 `M7_OBSTACLE`을 다시 보내면 정지가 안 풀린다.
  구간을 건너뛰려면 `"{data: M2_FORK}"`.

- 실행 중인 창에서 `Ctrl+C` (정지 명령을 보내고 종료). 그래도 움직이면 T1 드라이버 `Ctrl+C` 또는 본체 전원.
- ⑤단계부터는 차를 손으로 잡을 수 있는 거리에서.
