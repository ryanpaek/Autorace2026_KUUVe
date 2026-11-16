# 실차 실행 가이드 (LIMO, 2026-11-10 기준)

자주 쓰는 명령을 한곳에 모았다. 리모 IP는 그날 `hostname -I`로 확인 (11/10: `10.101.171.68`).

## 0. 시작 전 (매번)
```bash
cd ~/autorace_ws && git checkout main && git pull
colcon build --symlink-install && source install/setup.bash
ros2 pkg prefix autorace_bringup      # ~/autorace_ws/install/... 이어야 함 (catkin_ws면 옛 코드)
```
남은 프로세스 정리 (중복 실행되면 상태가 꼬이고 차체 통신이 깨짐):
```bash
ps -eo pid,cmd | grep -E "ackermann|limo|obstacle_node|parking_node|lane_node|detector_node|mission_node|class_command|fork_node|avoid_node|ydlidar|component_container|rosbag" | grep -v grep | awk '{print $1}' | xargs -r kill -9
ros2 daemon stop; ros2 daemon start
ros2 node list          # 비어 있어야 함
```

## 1. 띄우기
**한 번에 (차체 + 라이다 + 카메라 + 전체 미션, 주차·M7 회피·갈림길 노드 켬)**
```bash
ros2 launch autorace_bringup full.launch.py
```
- 센서를 이미 따로 켰으면 `sensors:=false` (안 붙이면 중복 실행)
- 끄기: `parking:=false` `m7_avoid:=false` `use_fork_node:=false` `use_detector:=false`

**하나씩 (통합 launch가 안 될 때)**
```bash
ros2 launch limo_base limo_base_ackerman.launch.py          # 차체. "Invalid frame" 경고 없어야 함
ros2 launch ydlidar_ros2_driver ydlidar.launch.py           # 라이다. "Now lidar is scanning"
ros2 launch orbbec_camera dabai.launch.py enable_depth:=false enable_ir:=false   # 카메라 컬러만
ros2 launch autorace_bringup autorace.launch.py parking:=true m7_avoid:=true use_fork_node:=true
```

**띄운 뒤 확인**
```bash
ros2 node list                          # 이름이 한 번씩만
ros2 topic hz /camera/color/image_raw   # 일정하게
ros2 topic hz /scan                     # 약 10Hz
ros2 topic echo /mission/state --once   # WAIT_START, dist_mm 0 근처
```

## 2. 녹화 → 출발 → 정지
```bash
# 녹화 (출발 전, 다른 터미널). -a 금지: depth 압축 구독 때문에 카메라 드라이버가 죽은 적 있음 (11/09)
ros2 bag record -o 1110_10_full /camera/color/image_raw/compressed /scan /odom /imu /cmd_vel /lane/cmd_vel /lane/status /mission/state /perception/detections /perception/commands /obstacle/state /avoid/state /fork/state /parking/state /system/usage
# 출발 (한 번만)
ros2 topic pub --once /mission/start std_msgs/msg/Empty
# 비상 정지 / 해제
ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: false}"
```
녹화는 Ctrl+C로 끝내야 저장된다. 이름은 `날짜_번호_내용` (예: `1110_11_park`).

## 3. 미션 하나만 연습
출발 신호 없이 상태를 바로 넘긴다. **보내는 순간 움직이니** 차를 그 미션 시작 위치에 먼저 놓는다.
```bash
ros2 topic pub --once /mission/set_state std_msgs/msg/String "{data: M4_PARKING}"
```
| 상태 | 차 놓는 곳 |
|---|---|
| `M5_BUMP` | 출발 체커 |
| `M6_ZIGZAG` | 지그재그 시작선 1.5m 앞 |
| `M3_PEDESTRIAN` | 지그재그 끝 |
| `M7_OBSTACLE` | 3차선 시작선 30cm 앞 |
| `M2_FORK` | 섬 끝 30cm 앞 |
| `M8_CONES` | 라바콘 시작선 50cm 앞 |
| `M1_TRAFFIC` | 신호등 앞 마름모 시작선 50cm 앞 |
| `M4_PARKING` | 주차 앞 선 30cm 앞 (두 번째 횡단보도 지난 곳) |

## 4. 주행 중 보기
```bash
ros2 topic echo /mission/stage      # 11/15: 지금 몇 번 미션인지 (mission: 7 / 확실하지 않으면 null, why에 이유)
ros2 topic pub --once /mission/resume std_msgs/msg/Empty   # 11/15: 끊긴 뒤 이어 하기 (마지막 기록의 미션·갈림길·위치로)
ros2 param set /mission_node stop_target_mm 200.0          # 11/15: 정지선 앞 멈추는 거리 바로 바꾸기 (빌드 없이)
ros2 topic echo /mission/state      # state/sub, note(왜 바뀌었나), parking_armed
ros2 topic echo /parking/state      # phase, row, stop_how
ros2 topic echo /avoid/state        # M7 회피
ros2 topic echo /fork/state         # 갈림길 A/B
ros2 topic echo /perception/commands   # YOLO 확인된 명령
```
로그에 `FALLBACK:`이 보이면 표지·선을 못 보고 도면 위치나 거리로 넘어간 것.

## 5. bag을 노트북으로 (Claude 분석용)
노트북 PowerShell:
```powershell
scp -r wego@10.101.171.68:~/1110_10_full C:\Users\shpae\Downloads\
```
`.db3`가 400MB를 넘으면 300MB씩 쪼갠다 (원본은 그대로 둠):
```powershell
cd C:\Users\shpae\Downloads\1110_10_full
$f = Get-ChildItem *.db3 | Select-Object -First 1
$in = [IO.File]::OpenRead($f.FullName); $buf = New-Object byte[] (300MB); $i = 0
while (($n = $in.Read($buf, 0, $buf.Length)) -gt 0) { [IO.File]::WriteAllBytes("$($f.FullName).part$i", $buf[0..($n-1)]); $i++ }
$in.Close(); dir
```
scp가 안 되면: 리모에서 `hostname -I`로 IP 확인 → 노트북에서 `ping` → `ssh wego@IP`로 따로 시험.

## 6. 자주 바꾸는 값 (`src/autorace_bringup/config/params.yaml`)
`--symlink-install`로 빌드했으면 yaml만 고치고 launch를 다시 띄우면 된다 (재빌드 불필요).
| 값 | 노드 | 뜻 |
|---|---|---|
| `base_speed` / `slow_speed` | mission_node | 기본 / 서행 속도 (서행 0.2) |
| `stop_brake_mm` | mission_node | 정지선 앞 이만큼에서 정지 명령 (밀리는 거리) |
| `odom_scale` | mission_node | 실제 거리 / odom 거리 (직선 3m로 실측) |
| `bump_imu_rate` | mission_node | 방지턱 감지 IMU 기준 (rad/s, 0.3 추정) |
| `fork_default` | mission_node | 갈림길 표지 끝까지 못 읽을 때 (A) |
| `turn_lead_mm` | parking_node | 칸 가운데가 앞바퀴보다 이만큼 앞일 때 꺾기 시작 (320). 늦게 꺾으면 키움 |
| `car_min_size_mm` | parking_node | 리모로 인정하는 덩어리 최소 크기 (60) |
| `vote_need` / `vote_window` | class_command_node | YOLO 명령 확정 (10프레임 중 7) |
실행 중 바꾸기: `ros2 param set /parking_node turn_lead_mm 350.0` (재시작하면 yaml 값으로 돌아감)

## 7. 문제 생겼을 때
| 증상 | 원인 / 조치 |
|---|---|
| `ros2 node list`에 같은 이름 두 번 | 예전 launch가 살아 있음 → 0번 정리 |
| 출발도 안 했는데 `FINISHED`, `dist_mm` 수백만 | odom 쓰레기값 (차체 통신 오류). 지금은 무시하게 고쳤지만 0번 정리 + 리모 전원 재시작 |
| 차체 `Invalid frame! Check sum failed!` 반복 | 차체 시리얼 꼬임. 중복 실행 확인, 리모 전원 껐다 켜기, 배터리 확인 |
| 카메라 `16UC1 is not a color format` → `OutOfMemoryError` → 죽음 | depth/ir 켜짐 + 전체 녹화(`-a`). depth/ir 끄고 녹화는 위 목록만. USB 뽑았다 꽂기 |
| `lane_node: 카메라 영상이 안 들어옴` | 카메라 죽음 → 위 |
| `xmlrpc ... !rclpy.ok()` | ROS 데몬 꼬임 → `ros2 daemon stop; ros2 daemon start` |
| 출발 신호에 `Waiting for at least 1 matching subscription` | mission_node가 아직 안 뜸. launch 확인 후 다시 |
| 라이다가 콘·컵을 못 봄 | 2D 라이다는 한 높이만 봄. 물체가 라이다 높이에 걸려야 함 |

## 8. 시뮬레이션 (노트북·서버, ROS 없이)
```bash
python3 tools/course_sim.py          # 출발~주차 상태 전환 6가지 상황
python3 tools/parking_sim_plot.py out.png   # 주차만: 6가지 상황 궤적 그림
python3 -m pytest -q src             # 전체 시험 (위치 추정: test_route_motion.py)
```

## 9. 위치 확인 수동 주행 (자율주행 끄고 도면 위 위치만 보기, 11/11)
맵 기반 모드 전환을 계속 쓰려면 "도면 위 내 위치"가 맞아야 한다. 차를 직접 몰면서 디버그 화면으로 확인한다.
`manual:=true`면 mission_node가 `/cmd_vel`을 **내지 않는다**. 위치 추정·FSM·차선·YOLO는 그대로 돌아서 미션이 어디서 바뀌는지도 같이 보인다.

```bash
ros2 launch autorace_bringup full.launch.py manual:=true
# 차를 출발 체커에 차로 방향으로 반듯이 놓고 (방향을 이때 맞춤 = 도면 경로 방향)
ros2 topic pub --once /mission/start std_msgs/msg/Empty        # s = 0. 차는 안 움직임
# 다른 터미널: 키보드 조종 (없으면 sudo apt install ros-humble-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- teleop 속도는 mission_node 상한(0.45)을 거치지 않는다. 기본 0.5 m/s라 시작하자마자 `z`를 여러 번(한 번에 10%) 눌러 **0.2 m/s 근처**로 낮춘다
- 애커만이라 멈춘 채 `j`/`l`은 조향이 안 먹는다. `i` 전진, `,` 후진, 꺾을 때는 `u`/`o`(전진), `m`/`.`(후진), `k` 정지
- 중간 구간부터: 그 미션 시작 위치에 차를 놓고 `/mission/set_state` (3번 표) 또는 `ros2 topic pub --once /route/set_s std_msgs/msg/Float32 "{data: 12000.0}"`

**디버그 화면** (`http://<리모IP>:8080`)
- 초록 선 = 전진 중 추정 궤적, 빨강 선 = 후진. 실제로 달린 길(차로 가운데)과 겹치는지 본다
- 점 + 숫자 = 흰 선으로 위치를 고친 곳과 그때 오차(mm). 초록 < 100, 노랑 < 250, 빨강 그 이상
- 오른쪽 '진행': 후진이면 무엇으로 알았는지 (`odom_v` 속도 부호 / `odom_xy` 위치 변화 / `cmd` 조종 명령)
- '좌우 d / 방향 차': 경로 중심선 기준 좌우 위치와 차 방향 - 경로 방향
- '길 잃음: 예'면 FSM이 도면 위치를 안 쓰는 상태
- '궤적 지우기' 버튼으로 초기화

**확인 순서 (각각 bag 녹화, 3번 녹화 명령에 `/route/preview` 추가)**
1. 직선 3 m 전진: 줄자 거리와 s 증가가 같은지 → 다르면 `odom_scale` 조정
2. 후진 50 cm: s가 약 500 줄고 '진행'이 후진으로 바뀌는지. 근거가 `cmd`로만 나오면 LIMO odom이 후진을 안 알려 주는 것
3. 지그재그를 일부러 크게 흔들며 통과: 지그재그 끝 정지선 보정 오차가 작은지 (예전엔 최대 80 cm)
4. 주차처럼 앞뒤로 꺾기: 같은 자리로 돌아오면 s도 돌아오는지
5. 출발~주차 한 바퀴: 보정 점 오차가 커지는 구간 기록

**이번에 바뀐 위치 추정** (`params.yaml`, 문제 있으면 false로 예전 방식)
| 값 | 뜻 |
|---|---|
| `route_signed_dist` | 후진 거리는 s에서 뺀다. 후진 판단을 odom 속도 부호 하나에서 → 속도 부호 / odom 위치 변화 방향 / 최근 `/cmd_vel` 중 하나라도로 넓힘 |
| `route_project_heading` | 차가 경로와 비스듬히 간 거리는 cos(방향 차)만 s에, sin 성분은 좌우 d에 (조향 반영). 방향 차가 70° 넘으면 예전처럼 거리 그대로 |
| `route_align_on_reset` | 출발선·연습 시작점·`/route/set_s` 때 차가 경로 방향으로 놓였다고 보고 방향을 맞춤 (직선에서 두 선을 보기 전에도 투영 가능) |

## 10. 센서 융합 화면 + '왜?' 설명 (디버그 화면 위쪽, 11/11)
디버그 화면(`http://<리모IP>:8080`) 위쪽에 실제 카메라와 라이다를 겹쳐 보여 주고, 오른쪽 맨 위 칸에 **지금 왜 이렇게 하는지**를 문장으로 보여 준다.
리모는 카메라 JPEG를 그대로 넘기기만 하고(`camera_max_hz`, 기본 8) 그림은 노트북 브라우저가 그린다.

**카메라 위 그림** (차선 노드 BEV 캘리브레이션 `bev_limo.yaml`을 거꾸로 써서 바닥 → 영상)
| 그림 | 뜻 |
|---|---|
| 세로 막대 점 (빨강 가까움 → 하늘 멂) | 라이다 점이 **바닥에 닿는 자리**. 콘·리모·기둥 밑동에 겹쳐야 정상 |
| 상자 (실선 = 확정, 점선 = 이번 프레임만) | YOLO. 글자: 클래스·신뢰도·표 수·**그 방향 라이다 거리** (카메라 + 라이다 융합) |
| 분홍 가로선 | 차선 노드가 본 흰 가로선과 거리 (진한 분홍 = 횡단보도 정지선) |
| 노랑 선 | 카메라가 본 차로 가운데 |
| 하늘 점선 | 도면이 예상하는 차로 가운데 (위치 추정이 맞으면 노랑과 겹침) |
| 초록 선 | 지금 속도 명령대로 가면 갈 궤적 |

라이다 점이 물체 밑동보다 계속 옆으로 밀려 있으면 `laser_yaw`, 앞뒤로 밀려 있으면 `front_axle_from_lidar_mm`가 틀린 것.

**라이다 위에서 본 그림**: 차 기준 (앞 = 위, 원 0.5 m), 차로 폭(±250) 안 가장 가까운 점, YOLO 상자 방향(노랑 점선), 명령 궤적.

**'왜?' 칸** (`autorace_mission/explain.py`): 지금 상태·조향 주체·속도 / 경고(FALLBACK, 길 잃음, 토픽 끊김, 차선 한쪽) /
이 상태가 된 근거 / **다음 단계로 넘어가는 조건과 지금 값** (표 수 7개 중 몇 개, 정지선까지 카메라·도면 거리, 대기 시간 등, 막대로 진행률).
FSM 조건을 바꾸면 explain.py도 같이 고쳐야 설명이 맞다.

**지도**: 3차선 taper·점선, 가운데 섬, 갈림길 A/B 경로(고른 쪽 진하게). 차가 노랑이면 차선으로 확인 못 하고 오도메트리로 추정한 위치.
위치 추정은 차선 offset을 도면상 차로 후보(3차선·A/B) 중 가까운 차로에 맞추고, 추정과 200mm 넘게 튀는 값은 10프레임 연속일 때만 받는다
(차로를 벗어나거나 옆 차로에 있을 때 차가 가운데로 끌려가던 문제).

**주차 완료 소리**: 화면의 `🔔 소리` 버튼을 한 번 누르면(브라우저는 클릭 전엔 소리를 못 냄) M4 주차 → FINISHED 때 노트북에서 '띠링' 두 음이 난다 (직접 합성한 소리).

## 11. 주차 완료 '띠링' (차 스피커, 11/11)
`chime_node`가 M4 주차 → FINISHED 순간 LIMO 차체 스피커로 두 음 '띠링'을 낸다 (직접 합성한 소리, launch에 기본 포함, 끄기 `chime:=false`).
```bash
ros2 topic pub --once /chime/play std_msgs/msg/Empty        # 시험: 바로 울림
```
소리가 안 나면 리모에서:
```bash
aplay -l                                    # 스피커 카드 번호 확인
speaker-test -t sine -f 1000 -l 1           # 기본 장치로 소리 나는지
alsamixer                                   # 음소거(MM)·볼륨 확인
ros2 param set /chime_node device plughw:1,0    # 기본 장치가 아니면 지정 (aplay로 재생)
```
`volume`(0~1, 기본 0.6)은 시작할 때만 읽는다. 디버그 화면의 🔔 버튼(노트북 소리)과는 따로 동작한다.

## 12. 디버그 화면에서 미션 조작 (11/11)
오른쪽 '미션 조작' 칸. 버튼을 누르면 확인 창이 뜨고, **보내는 순간 차가 움직인다**.
| 버튼 | 보내는 것 | 언제 |
|---|---|---|
| 시작 위치에서 시작 | `/mission/set_state` | 차를 3번 표의 그 미션 시작 위치에 놓았을 때. 도면 위치도 그 시작점으로 다시 맞춤 |
| 지금 자리에서 이 미션 | `/mission/skip` | 주행 중 지금 자리에서 그 미션으로. 도면 위치는 그대로 |
| 다음 미션 ▶ | `/mission/skip` | 지금 미션을 끝난 것으로 보고 다음으로 (위치 그대로) |
| 출발 | `/mission/start` | 출발 체커에서 |
| 비상 정지 (**Space 키**) / 해제 | `/mission/estop` | 비상 정지는 확인 없이 바로 |

- 대회 때는 `debug_control:=false`로 끈다 (같은 와이파이 누구나 누를 수 있음). 꺼도 **비상 정지는 된다**
- 비밀번호: `ros2 launch ... ` 전에 params.yaml `debug_view.ros__parameters.control_password` 또는 `ros2 param set /debug_view control_password 원하는값`
- 누른 기록은 '상태 기록'과 리모 로그에 `[debug 조작]`으로 남는다

## 13. 차선을 놓쳤을 때 도면 곡률로 조향 (11/11)
예전엔 차선을 놓치면 IMU 방향을 유지해 **직진**했다 (도면상 코너인데도 안 꺾고 경로 이탈).
이제 `lane_node.route_ff_weight_lost: 0.8` — 놓친 동안 조향을 도면 곡률 쪽으로 80% 섞는다.
- 미션 노드가 '길 잃음'이면 도면 곡률을 안 섞는다 (틀린 위치에서 꺾지 않게)
- 차선이 보일 때는 예전과 같다 (카메라 차선만)
- 디버그 화면 '차선 영상'에 source가 `hold`일 때 꺾이는지 보면 된다. 엉뚱한 데서 꺾으면 위치 추정을 먼저 확인 (9절), 급하면 `ros2 param set /lane_node route_ff_weight_lost 0.0`
