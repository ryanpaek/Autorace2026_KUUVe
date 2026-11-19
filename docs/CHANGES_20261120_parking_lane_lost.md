# 11/20 M4 주차: 선 잡은 뒤 차선이 멈춰도 선까지 감 (fixed_line_creep)

근거: 시뮬 simcheck M4 (main c64162f, 차량 real_1119_06). 시뮬 결과이며 실차 확인 전.

## 증상
- `/parking/state` phase `F_LINE`, `end_mm` 380에서 116초 동안 그대로 (Time-out).
- `/lane/status` source: lane → hold → stop. 주차 구역에 들어가 차선(양쪽 선)이 안 보이자 차선 노드가 속도 0.
- F_LINE은 선을 잡은 뒤에도 속도를 `/lane/cmd_vel`(lane_v) 그대로 써서, 차선이 서면 같이 섬. 선은 이미 카메라 아래라 다시 안 보임 → 영원히 정지.

## 고침 (`parking_node.py` `_step_fixed` F_LINE)
- 선을 잡았고(end_d 있음) lane_v < `fixed_line_creep_min_v`(0.03 m/s)면 `fixed_slow_speed`(0.10 m/s)로 감.
  조향은 차선이 살아 있을 때 마지막 IMU 방향 유지 (`fixed_heading_gain`, F_MOVE와 같은 식). 앞에 물체(blocked)면 기다림.
- 선을 아직 못 잡았으면 예전처럼 차선 따라 섬 (아무 데로나 가지 않음).
- `/parking/state`에 `creep`(지금 이 모드인지), `creep_n`(몇 번 들어갔나).
- 끄기: `ros2 param set /parking_node fixed_line_creep false`

## 고침 2: 켜질 때 낡은 선 무시 (`fixed_line_fresh_n` 2)
- 증상: 주차 모드가 켜지는 순간 `/lane/status`가 0.4초 전(순간이동 전 출발선 자리)의 선 1636 mm를 들고 있었음 →
  '1.6 m 앞 주차선'으로 잡고 주차장 한가운데(앞바퀴 x 7.09)에서 섬 → 칸 검사 0칸 → 가짜 선 → 다시 찾다 정지.
- 고침: F_LINE에 들어온 뒤 새로 받은 `/lane/status`가 2개 이상일 때부터 처음 선을 잡음 (노드가 메시지 번호 `lane_seq`를 넘김).
  서 있어도 새 메시지면 잡으므로 연습 자리(선 352 mm, 카메라 아래 한계 근처)에서도 바로 잡음.
  (처음엔 '3 cm 움직이는 동안 다시 보이면'으로 했는데, 352 mm 선은 움직이자마자 화면 밖이라 못 잡아 바꿈)
- 실차 영향: 켜진 뒤 0.1초 정도(25 fps 2장) 선을 안 씀. 통주행은 선이 1 m 넘게 앞에서 보여 영향 없음.

## 시뮬에서 같이 찾은 것 (팀 코드 아님, 박재승 님 시뮬)
- 시뮬 `/imu` orientation이 odom 방향(`veh.odom[2]`)을 그대로 써서, 11/20 실차 프로필(odom 요레이트 ×1.14)부터
  IMU도 1.14배로 돎 → 주차가 'IMU 87.6°'에서 끝났는데 실제 76.5° → 비스듬히 서서 모서리 1곳 이탈(7/15).
  실차 IMU는 자이로라 해당 없음. 시뮬 패치(sim_imu_yaw_fix.patch)를 박재승 님께 전달.
- 시뮬 M4 시작 자리는 앞바퀴가 주차선 9 cm 앞이라 '25 cm보다 가까운 선은 버림' 규칙에 걸림 → simcheck에서 런 가이드대로 30 cm 앞에 놓음.

## 시험
- pytest autorace_mission 414 통과 (새 6개: 선 놓친 뒤 선까지 감 / 끄면 예전처럼 정지 / 앞 물체 기다림 / 선 못 잡았으면 안 감 /
  켜질 때 낡은 선 무시 / 연습 자리 352 mm 서서 잡기)
- 시뮬(simcheck M4만, 시뮬 IMU 패치 + 런 가이드 시작 자리): 이 브랜치 3판 15/15 ×3, main 0/15 (낡은 선 → 차선 놓침 정지).
  시뮬 IMU 패치 없이: 이 브랜치 7/15 (회전 부족, 시뮬 문제). ※ 실차 확인 전.
