# tools/ : LIMO 단독 실행 스크립트

ROS2 패키지가 아니라 `python3`로 직접 실행하는 스크립트입니다 (colcon 빌드 필요 없음).
CSV는 **명령을 친 폴더**에 저장되므로 데이터 폴더(예: `~/autorace_tools`)에서 실행하세요.

| 파일 | 역할 | 실행 |
| --- | --- | --- |
| `stop_test_ack.py` | 속도별(0.2/0.3/0.4m/s) 정지 시간·거리 측정. `/limo/ack_cmd`로 직접 발행(브리지 불필요) | `python3 stop_test_ack.py` 또는 `python3 stop_test_ack.py 0.3 5` |
| `path_recorder.py` | 키보드 주행 중 odom 위치·속도·명령을 CSV로 기록 | `python3 path_recorder.py line1.csv` |
| `path_follower.py` | 기록한 CSV 경로를 따라 주행(pure pursuit) | `python3 path_follower.py line1.csv 0.2 0.023` |

## 실행 순서

1. 조종기를 끄고, 아커만 드라이버를 켠다: `ros2 launch limo_base limo_base_ackerman.launch.py`
2. 정지 측정(`stop_test_ack.py`)은 **브리지·teleop 없이** 실행한다 (자체적으로 `/limo/ack_cmd`에 발행).
3. 경로 기록·재생은 브리지(`ackermann_bridge`)를 켜고 실행한다 (`/cmd_vel` -> `/limo/ack_cmd` 변환).
   - 기록: `path_recorder.py` 실행 후 teleop으로 주행, `k`로 세우고 `Ctrl+C`
   - 재생: 같은 출발 자세에 놓고 `path_follower.py` 실행 (teleop·recorder 끄기)
4. 브리지 없이 재생하려면 `--type ackermann --topic /limo/ack_cmd` 옵션을 쓴다.
   이때 `path_follower.py`의 `WHEELBASE`(0.21)를 브리지 값(0.222)과 맞춰야 한다.

## 주의

- `stop_test_ack.py`의 결과(`stop_결과.csv`, `stop_raw/`)는 `~/calib_test/`에 저장된다. 레포에는 올리지 않는다.
- 파일 맨 위 주석에 예전 파일명·경로(`~/calib_test/20261102_stop_test_ack.py`)가 남아 있다. 동작에는 영향 없다.
- 실행하면 차가 실제로 움직인다. 앞쪽을 비우고 차 옆에서 `Ctrl+C`를 칠 수 있게 한다.
