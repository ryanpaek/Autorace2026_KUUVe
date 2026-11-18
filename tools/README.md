# tools/ : LIMO 경로 기록·재생 스크립트

ROS2 패키지가 아니라 `python3`로 직접 실행하는 스크립트입니다 (colcon 빌드 필요 없음).
CSV는 **명령을 친 폴더**에 저장되므로 데이터 폴더(예: `~/autorace_tools`)에서 실행하세요.

| 파일 | 역할 | 실행 |
| --- | --- | --- |
| `path_recorder.py` | 키보드 주행 중 odom 위치·속도·명령 + 재생 기준 위치(`xs, ys, yaws, s` = odom 거리 + IMU 방향)를 CSV로 기록 | `python3 path_recorder.py line1.csv` |
| `path_follower.py` | 기록한 CSV 경로를 따라 주행 (옆 오차·방향 오차·기록 곡률로 조향) | `python3 path_follower.py line1.csv 0.2 0.023` |
| `path_check.py` | 기록 CSV 점검: 끝 위치/방향, 최대 곡률, 뒤로 간 구간, odom·IMU 부호 (ROS 불필요) | `python3 path_check.py line1.csv --png` |
| `path_common.py` | 위 셋이 같이 쓰는 위치 계산·경로 처리. **같은 폴더에 같이 복사** | — |
| `stop_diag.py` | 직진 중 갑자기 멈춤 진단: 1초 요약 + 멈추면 STOP 스냅샷을 `/diag/stop_reason`으로 발행 ([RUN_GUIDE 14절](../docs/RUN_GUIDE.md)) | `python3 stop_diag.py` |

> 11/17 변경: 위치를 odom **이동 거리**(위치 변화량) + IMU 자이로 **방향**으로 계산하고(기록·재생 같은 코드),
> 재생 조향은 위치 목표점 하나(pure pursuit) 대신 **옆 오차 + 방향 오차 + 기록 곡률 미리 넣기**로 바꿨다.
> 돌 때 odom 방향과 IMU 방향 부호가 반대면 기록 중 경고, 재생 중 정지한다 (한쪽으로 끝까지 꺾이던 증상의 원인 후보).
> 예전 CSV(`xs` 열 없음)는 자동으로 `--pose odom`으로 재생된다.
> **후진 지원**: 기록에 후진이 있으면 전진↔후진이 바뀌는 곳(3 cm 넘게 반대로 간 곳)마다 구간을 나눠
> 그 순서대로 재생한다. 구간 끝에서 멈추고 `--pause`초 쉰 뒤 다음 방향으로 출발.
> 후진 판단은 odom 위치 변화 방향 / odom 속도값 부호 / `/cmd_vel` 명령 부호 중 하나라도 후진이면 후진
> (LIMO 드라이버가 후진을 전진처럼 쌓는 경우 대비, mission_node와 같은 규칙). 재생할 때 경로는 CSV의 원본 열
> (odom·명령·자이로)로 다시 계산하므로 예전 규칙으로 기록한 CSV도 후진이 살아난다.
> 후진이 있는 경로는 `--pose` 기본값(fused)으로 재생할 것 (`odom`은 드라이버가 후진 방향을 틀리면 못 따라감).

> 정지 시간·거리 측정 스크립트(`stop_test_ack.py`)는 [`calib/20261102_stop_test_ack.py`](../calib/20261102_stop_test_ack.py)에 있다.
> 사용법은 [calib/USAGE.md](../calib/USAGE.md), 결과는 [calib/README.md](../calib/README.md) 4번.

## 실행 순서 (경로 기록 → 재생)

조종기(RC)는 끄고 시작한다. 각 블록은 **터미널 하나**씩이다.
스크립트는 `~/autorace_tools`에 복사해 두고 쓴다 (CSV도 거기에 쌓인다):

```bash
mkdir -p ~/autorace_tools && cp ~/autorace_ws/tools/*.py ~/autorace_tools/    # path_common.py도 같이 복사됨
```

### 0. 드라이버 (T1, 기록·재생 내내 켜 둠)

```bash
# 애커만 드라이버. /limo/ack_cmd만 받는다 (/cmd_vel은 안 받음)
ros2 launch limo_base limo_base_ackerman.launch.py
```

### 1. 경로 기록

```bash
# T2: 브리지. /cmd_vel → /limo/ack_cmd 변환
#   wheelbase 0.222 = 캘리브레이션 유효 축거 (calib/README 3번)
#   cmd_timeout_sec 1.0 = teleop 키를 꾹 누르지 않아도 1초간 명령 유지 (기본 0.3초)
ros2 run autorace_mission ackermann_bridge --ros-args -p wheelbase:=0.222 -p cmd_timeout_sec:=1.0
```

```bash
# T3: 기록기. 처음 1초는 차를 움직이지 말 것 (자이로 편향 보정)
cd ~/autorace_tools
python3 path_recorder.py line1.csv
```

```bash
# T4: 키보드 조종. 애커만이라 제자리 회전(j/l)은 안 됨 → u/o로 커브
#   다 달리면 k로 세우고, T3에서 Ctrl+C → CSV 저장
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
# 기록 확인: 줄 수 = 헤더 1 + odom 샘플 수 (50 Hz면 10초에 약 500줄)
wc -l ~/autorace_tools/line1.csv
```

### 2. 경로 재생

기록기(T3)와 teleop(T4)은 **끄고**, 차를 기록할 때와 같은 출발 자세에 놓는다.

```bash
# T2: 브리지 다시 켜기. 재생은 명령이 계속 나오므로 타임아웃은 기본값(0.3초)
#   → follower가 죽으면 0.3초 안에 차가 선다
ros2 run autorace_mission ackermann_bridge --ros-args -p wheelbase:=0.222
```

```bash
# T3: 재생. 인자 = CSV, 속도 0.2 m/s, 끝점 2.3 cm 앞에서 정지 명령(0.2 m/s 제동 거리)
#   약 2초 동안 /cmd_vel 연결 확인 후 출발
cd ~/autorace_tools
python3 path_follower.py line1.csv 0.2 0.023
```

### 3. 정리

```bash
# 저장된 기록 목록 보기
ls -la ~/autorace_tools/*.csv
```

```bash
# 필요 없는 기록 지우기 (되돌릴 수 없음)
rm ~/autorace_tools/line1.csv
```

### path_follower.py 인자

| 인자 | 기본값 | 뜻 |
| --- | --- | --- |
| `csv` | — | `path_recorder.py`로 만든 파일 |
| `speed` | 0.2 | m/s |
| `stop_early` | 0.0 | m, 끝점 이만큼 앞에서 정지 명령 (제동 거리 보정. 0.2 m/s면 0.023~0.024) |
| `--pose` | `auto` | `auto`: 새 기록이면 `fused`, 예전 기록이면 `odom` / `fused`: odom 거리 + IMU 방향 (`xs, ys`) / `odom`: odom 위치·방향 / `imu`: 예전 `xf, yf` 열 |
| `--ctrl` | `track` | `track`: 곡률 = 기록 곡률(앞 `preview`초) - `k_e`×옆 오차 - `k_psi`×방향 오차 / `pp`: 예전 pure pursuit |
| `--k_e` | 6.0 | 옆 오차 게인 (1/m²). 크면 빨리 붙지만 출렁임 |
| `--k_psi` | 3.4 | 방향 오차 게인 (1/m) |
| `--preview` | 0.3 | s, 조향 지연 보정용으로 기록 곡률을 미리 봄 |
| `--rev_speed` | = speed | m/s, 후진 구간 속도 |
| `--pause` | 0.5 | s, 전진↔후진 바뀔 때 멈춰 있는 시간 |
| `--max_err` | 0.30 | m, 옆 오차가 이보다 크거나 방향 오차 50° 넘으면 정지 |
| `--topic` | `/cmd_vel` | 발행 토픽 |
| `--type` | `twist` | `twist` 또는 `ackermann` |
| `--lookahead` | 0.3 | m, `--ctrl pp`일 때만 |

- 끝나면 `최대 옆 오차`, `조향 한계에 붙은 비율`을 출력한다. 한계 비율이 높으면 기록이 너무 급한 것 → 더 완만하게 다시 기록.
- 재생 속도는 `path_check.py`가 알려 주는 기록 평균 속도에 맞춘다.
- 브리지 없이 재생하려면 `--type ackermann --topic /limo/ack_cmd` (축거 0.222 사용).

## 주의

- 실행하면 차가 실제로 움직인다. 앞쪽을 비우고 차 옆에서 `Ctrl+C`를 칠 수 있게 한다.
- 기록한 CSV는 레포에 올리지 않는다.

## course_sim.py — 통주행 미션 로직 시뮬레이션 (ROS 없이)
`python3 tools/course_sim.py` — 출발부터 주차까지 상태 전환 위치·이유·정지선 오차를 시나리오별로 출력
(정상 / 표지 전부 놓침 / 방지턱 표지만 놓침 / odom 튐 / odom 5% 길게 / 카메라 선 없음).
FSM·도면 위치·YOLO 명령 노드는 진짜 코드, 차선 주행·카메라·YOLO·IMU·M7 회피·라바콘·주차는 단순 모형.
