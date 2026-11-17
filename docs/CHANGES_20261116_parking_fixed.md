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

## 11/16 추가: 앞범퍼를 선에 맞춰 정지
- `fixed_line_target_mm` 150 → 56 (캡틴 그림: 앞범퍼가 주차선에 닿게 = 앞바퀴→앞범퍼 56mm). `fixed_move_mm` 1040은 캡틴이 실제 트랙에서 잰 값 그대로.
- 참고: 시험 트랙(선→주차 구역 500mm)에서는 이 조합이 칸 가운데보다 3번 쪽 +9.4cm(반경 590) → 시험은 직진 940으로 논리만 확인.
  실제 트랙과 시험 트랙이 다르면 실차 결과가 맞음. 3번 쪽으로 붙으면 fixed_move_mm를 줄이기 (1cm 붙음 = 10mm 줄임).

## 11/16 밤 실차 확정 → 기본 방식을 fixed로
- 실차 튜닝: 104 → 100 → 95 → (최대 조향 35/56cm 시험: 덜 꺾임) → **직진 87cm + 회전 IMU 87도** (`fixed_turn_mm` 0).
- `turn_ref: fixed`를 params.yaml 기본으로 (캡틴). 되돌리기: `turn_ref: path` 또는 `ros2 param set /parking_node turn_ref path`.
- 연습 놓는 곳: 주차선 40~50cm 앞 (앞범퍼를 선에 대고 시작하면 선이 카메라에 안 보여 직진만 함).
- 시험: 예전 경로 방식 시험들은 turn_ref만 path로 두고 그대로 통과 (359).

## 11/17 공지 대응: 빈 칸 랜덤 → 라이다로 빈 칸 찾기
- 공지: 한 칸만 비우고 나머지는 모두 막음, 빈 칸 위치는 심사위원이 바꿈 (1행 2열 고정 아님).
- 주차선에서 서 있는 1초(`fixed_detect_sec`) 동안 라이다 점을 앞줄 1·2·3번 칸 상자마다 셈 → 가장 적은 칸 선택
  → 직진 거리만 칸별로 (`fixed_cells_move_mm` [350, 870, 1390] = 2번 실차 870 ± 칸 간격 520). 회전은 그대로 87도.
- 상자: 선→1번 칸 455mm(도면 440·시험 트랙 500 사이), 칸 간격 520, 앞뒤 10cm 줄임, 옆 33~75cm(리모 앞면 40~50cm).
- 시험(빈 칸 1·2·3 × 반경 590·620, 차로 위치 ±5cm): 모두 빈 칸 맞힘·칸 안·충돌 없음. 빈 칸 점 0 vs 막힌 칸 4~22.
- 뒷줄(4·5·6)은 앞줄이 다 막혀 있으면 들어갈 길이 없어 고려 안 함. `/parking/state`의 cell, cell_src, cell_avg로 확인.

## 11/18 캡틴 알고리즘으로 정리 (값은 비워 둠)
```
① 주차선 정지 → ② 서서 라이다로 앞줄 빈 칸 찾기
   └ 옆 33~75cm(오른쪽) 점만, 앞 거리 범위 1번 50~102 / 2번 103~153 / 3번 154~204cm (fixed_cell_ranges_mm, 라이다 기준)
③ 칸별 직진(fixed_cells_move_mm ★선배 오돔으로 교체) → 최대 조향 87°
④ F_CHECK: 앞범퍼 앞 50cm, 차 폭 띠(±13cm)에 물체?
   ├ 있음 → PARKED_FIXED (앞칸)
   └ 없음 → 뒷줄 빈 칸 → fixed_backrow_mm(★미측정 0) 더 직진 → PARKED_FIXED_BACK
                         (0이면 앞칸에서 끝: PARKED_FIXED_NO_BACKROW)
```
- 시험: 앞칸 1·2·3 + 뒷줄 4·5·6(앞칸도 빔) 모두 맞는 칸, 칸 안, 충돌 없음 (뒷줄 거리 620으로 시험). mission 375 통과.
- fixed_straight_mm 삭제 (④로 대체).

## 11/18 빈 칸 판단: A(캡틴 덩어리) + B(점 개수) 같이
- A: 덩어리(점 간격 80mm로 나눔) 중심이 어느 칸 범위에 있나 → 절반 넘는 장면에서 덩어리가 없던 칸 = 빈 칸 (딱 1개일 때만 답)
- B: 칸마다 점 수 → 가장 적은 칸
- 같음 → 출발(agree) / A 답 없음 → B(a_fail_b) / 다름 → 1초 더 모은 뒤 또 다르면 B(mismatch_b)
- /parking/state: cell_a, cell_b, cell_src, cell_avg

## 11/18 공지: 2행 주차 없음 → 뒷줄 단계 삭제
- F_CHECK(앞 50cm 확인)·F_STR(뒷줄 직진)·fixed_check_*·fixed_backrow_mm 삭제. 87도 돌면 바로 PARKED_FIXED.
