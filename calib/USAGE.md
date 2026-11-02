# calib — LIMO 센서·주행 캘리브레이션 도구

2026-11-02 윤서가 실차(LIMO Pro, 애커만 모드)에서 쓴 시험 스크립트 모음.
결과는 [README.md](README.md)에 있다.

> 이 폴더는 `src/` 밖에 있어서 `colcon build` 대상이 아니다. 그냥 `python3`로 실행한다.
> 팀 코드(`src/`)는 하나도 고치지 않았다.

## 파일

| 파일 | 하는 일 | 로봇 움직임 |
|---|---|---|
| `20261102_yaw_monitor.py` | `/odom` x·y·yaw, `/imu` yaw를 0.5초마다 출력 | ❌ |
| `20261102_imu_drift_test.py` | 정지 상태 60초×5회 IMU yaw 드리프트·자이로 바이어스 | ❌ |
| `20261102_drive_straight_test.py` | 0.2 m/s로 odom 1 m 직진 ×5회, 줄자 입력 → 거리 비율 | ⚠️ 움직임 |
| `20261102_drive_curve_test.py` | 0.2 m/s, 최대 조향(0.40) 오른쪽 커브, IMU 90°에서 정지 ×5회 → odom÷IMU, 회전 반지름, 유효 축거 | ⚠️ 움직임 |
| `20261102_stop_test_ack.py` | 0.2/0.3/0.4 m/s 각 5회, 0.5 m 이후 정속에서 정지 → 지연·정지 시간·제동 거리 | ⚠️ 움직임 |
| `20261102_센서검증_기록표_5회.xlsx` | 결과 입력용 엑셀 (노란 칸 입력 → 오차·비율·판정 자동) | — |

움직이는 스크립트는 전부 `/limo/ack_cmd`로 **직접** 명령한다 (브리지 안 거침).
안전장치: 제한 시간 초과·`/odom` 끊김(정지 시험) 시 자동 정지, `Ctrl+C` 시 정지 명령 후 종료.

## 준비 (매번)

1. LIMO에서 터미널을 열 때 환경 선택은 **1번(Global)**. 3번(dl_env)은 딥러닝 노드만.
2. **조종기(RC) 끄기.** 수동 모드면 명령이 거의 안 먹는다 (0.2 m/s 명령에 약 0.04 m/s).
3. 결과 저장 폴더: `mkdir -p ~/calib_test`
4. 붙여 넣기에 `^[[200~`가 섞이면: `bind 'set enable-bracketed-paste off'`

## 터미널 구성

| 터미널 | 명령 | 비고 |
|---|---|---|
| T1 드라이버 | `ros2 launch limo_base limo_base_ackerman.launch.py` | 항상 켜 둠. 차동용 `limo_base.launch.py` 아님 |
| T2 모니터 | `python3 calib/20261102_yaw_monitor.py` | 선택 |
| T3 비상정지 | `ros2 topic pub --once /limo/ack_cmd ackermann_msgs/msg/AckermannDrive "{speed: 0.0, steering_angle: 0.0}"` | 쳐 두고 필요할 때 엔터 |
| T4 시험 | 아래 시험 스크립트 | 브리지·teleop·다른 제어 노드는 **끄기** |

## 시험별 실행

```bash
python3 calib/20261102_imu_drift_test.py        # 약 5분, 건드리지 말 것
python3 calib/20261102_drive_straight_test.py   # 앞 2 m 비우기, 줄자 1.5 m
python3 calib/20261102_drive_curve_test.py      # 오른쪽 앞 1.5×1.5 m 비우기
python3 calib/20261102_stop_test_ack.py         # 0.2/0.3/0.4 각 5회, 앞 1.5 m
python3 calib/20261102_stop_test_ack.py 0.3 5   # 한 속도만
```

중간 조향(0.20) 커브 시험은 사본을 만들어 실행했다:
```bash
sed -e 's/^STEER = 0.40.*/STEER = 0.20/' -e 's/curve_결과.csv/curve_steer020_결과.csv/' \
  calib/20261102_drive_curve_test.py > ~/calib_test/20261102_drive_curve_test_steer020.py
python3 ~/calib_test/20261102_drive_curve_test_steer020.py   # 오른쪽 앞 2×2 m 비우기
```

결과 CSV는 `~/calib_test/`(정지 시험 원자료는 `~/calib_test/stop_raw/`)에 저장된다.

## 측정 요령

- 출발할 때 **앞바퀴 중심**을 출발선(타일 이음선이 정확)에 맞추고, 방향은 옆면을 타일선에 나란히.
- 줄자 눈금은 앞바퀴 중심에서 자를 직각으로 대고 읽는다.
- 애커만 odom은 조향 0이면 y가 항상 0 → 옆 쏠림은 줄자로만 확인 가능.
