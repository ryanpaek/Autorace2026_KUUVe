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
ros2 bag record -o 1110_10_full /camera/color/image_raw/compressed /scan /odom /imu /cmd_vel /lane/cmd_vel /lane/status /mission/state /perception/detections /perception/commands /obstacle/state /avoid/state /fork/state /parking/state
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
python3 -m pytest -q src             # 전체 시험
```
