# 11/16 주차 '고정 거리' 방식 추가 (turn_ref: fixed, 캡틴)

기본값은 그대로 `turn_ref: path`. 실행 중 `ros2 param set /parking_node turn_ref fixed` (M4 들어가기 전) 또는 params.yaml에서 바꿈.

```
① F_LINE    차선 따라가며 주차선 찾기 → 앞바퀴가 선 15cm 앞(fixed_line_target_mm)에서 정지
            (선이 카메라 아래로 사라지면 마지막 거리 − 오도메트리. 시작 30cm는 선 무시, 4m 못 보면 FAIL_NO_LINE)
② F_PAUSE   0.3초 정지 (fixed_pause_sec)
③ F_MOVE    오도메트리 104cm 직진 (fixed_move_mm, 멈출 때 방향 유지, 앞이 막히면 기다림)
④ F_TPAUSE  0.3초 정지 + 끝 0.4초 조향 미리 꺾기 (presteer)
⑤ F_TURN    오른쪽 최대 조향 → IMU 87도(fixed_turn_deg) 또는 fixed_turn_mm(>0) 가면 끝, 앞 15cm 막히면 그 자리 정지
⑥ F_STR     바퀴 펴고 fixed_straight_mm 직진 (기본 0)
⑦ DONE      PARKED_FIXED (/parking/state: turn_at.line_mm = 선에서 선 위치, q_s = 직진 거리, last_err = 돈 각도·거리)
```

- 시뮬레이션 (test_parking_node.py drive, 차로 왼쪽 5cm = 미션 m4_target_offset_mm 50):
  반경 560/590/620/650, 서보 지연 0~120mm → 모두 네 모서리 칸 안·충돌 없음. 칸 가운데 대비 −3.0 ~ +7.1cm, 앞 여유 1.2~10.7cm.
  반경 590: ±1cm, 앞 여유 7cm. 반경이 1cm 크면 칸에서 약 1cm 3번 쪽 + 1cm 깊게.
- 최대 조향 400mm(캡틴 '40cm')만 가면 반경 590에서 약 39도라 칸에 덜 들어감 → 기본은 각도(87도)로 끝, 거리는 끔(0).
- 90도 원호만으로 앞범퍼가 칸 끝 7cm 앞까지 감 → 바퀴 편 뒤 직진(fixed_straight_mm)은 0.
- 한계: 라이다로 1·3·5번을 보지 않음. 주차선 정지 위치 오차와 실제 반경 오차가 그대로 위치 오차가 됨. 실차 미검증.
