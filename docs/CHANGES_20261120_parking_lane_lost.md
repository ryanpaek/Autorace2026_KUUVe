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

## 시험
- pytest autorace_mission 412 통과 (새 4개: 선 놓친 뒤 선까지 감 / 끄면 예전처럼 정지 / 앞 물체 기다림 / 선 못 잡았으면 안 감)
