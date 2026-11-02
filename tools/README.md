# tools/ : LIMO 경로 기록·재생 스크립트

ROS2 패키지가 아니라 `python3`로 직접 실행하는 스크립트입니다 (colcon 빌드 필요 없음).
CSV는 **명령을 친 폴더**에 저장되므로 데이터 폴더(예: `~/autorace_tools`)에서 실행하세요.

| 파일 | 역할 | 실행 |
| --- | --- | --- |
| `path_recorder.py` | 키보드 주행 중 odom 위치·속도·명령 + IMU 융합 위치(`xf, yf, yawf`)를 CSV로 기록 | `python3 path_recorder.py line1.csv` |
| `path_follower.py` | 기록한 CSV 경로를 따라 주행(pure pursuit) | `python3 path_follower.py line1.csv 0.2 0.023` |

> 정지 시간·거리 측정 스크립트(`stop_test_ack.py`)는 [`calib/20261102_stop_test_ack.py`](../calib/20261102_stop_test_ack.py)에 있다.
> 사용법은 [calib/USAGE.md](../calib/USAGE.md), 결과는 [calib/README.md](../calib/README.md) 4번.

## 실행 순서

1. 조종기를 끄고, 아커만 드라이버를 켠다: `ros2 launch limo_base limo_base_ackerman.launch.py`
2. 브리지(`ackermann_bridge`)를 켠다 (`/cmd_vel` -> `/limo/ack_cmd` 변환).
   기록할 때는 `-p cmd_timeout_sec:=1.0`을 주면 키를 꾹 누르지 않아도 된다.
3. 기록: `path_recorder.py` 실행 → **처음 1초는 차를 움직이지 않는다**(자이로 보정) → teleop으로 주행, `k`로 세우고 `Ctrl+C`
4. 재생: 같은 출발 자세에 놓고 `path_follower.py` 실행 (teleop·recorder 끄기)

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
