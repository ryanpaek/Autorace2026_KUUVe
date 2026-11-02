# tools/ : LIMO 경로 기록·재생 스크립트

ROS2 패키지가 아니라 `python3`로 직접 실행하는 스크립트입니다 (colcon 빌드 필요 없음).
CSV는 **명령을 친 폴더**에 저장되므로 데이터 폴더(예: `~/autorace_tools`)에서 실행하세요.

| 파일 | 역할 | 실행 |
| --- | --- | --- |
| `path_recorder.py` | 키보드 주행 중 odom 위치·속도·명령 + IMU 융합 위치(`xf, yf, yawf`)를 CSV로 기록 | `python3 path_recorder.py line1.csv` |
| `path_follower.py` | 기록한 CSV 경로를 따라 주행(pure pursuit) | `python3 path_follower.py line1.csv 0.2 0.023` |

> 정지 시간·거리 측정 스크립트(`stop_test_ack.py`)는 [`calib/20261102_stop_test_ack.py`](../calib/20261102_stop_test_ack.py)에 있다.
> 사용법은 [calib/USAGE.md](../calib/USAGE.md), 결과는 [calib/README.md](../calib/README.md) 4번.

## 실행 순서 (경로 기록 → 재생)

조종기(RC)는 끄고 시작한다. 각 블록은 **터미널 하나**씩이다.
스크립트는 `~/autorace_tools`에 복사해 두고 쓴다 (CSV도 거기에 쌓인다):

```bash
mkdir -p ~/autorace_tools && cp ~/autorace_ws/tools/*.py ~/autorace_tools/
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
| `--pose` | `odom` | `odom`: odom 위치·방향 / `imu`: odom 속도 + IMU 방향으로 만든 위치(`xf, yf`) |
| `--topic` | `/cmd_vel` | 발행 토픽 |
| `--type` | `twist` | `twist` 또는 `ackermann` |
| `--lookahead` | 0.3 | m, pure pursuit 전방 주시 거리 |

- 방향은 IMU가 더 정확하다(odom yaw는 중간 조향에서 약 12% 틀림, calib 결과 3번) → 커브가 있는 경로는 `--pose imu` 권장.
- 브리지 없이 재생하려면 `--type ackermann --topic /limo/ack_cmd`.
  이때 `path_follower.py`의 `WHEELBASE`(0.21)를 브리지 유효 축거(0.222)와 맞춰야 한다.

## 주의

- 실행하면 차가 실제로 움직인다. 앞쪽을 비우고 차 옆에서 `Ctrl+C`를 칠 수 있게 한다.
- 기록한 CSV는 레포에 올리지 않는다.
