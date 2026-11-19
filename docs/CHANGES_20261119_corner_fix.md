# 11/19 corner_fix — 코너 중 조향 풀림·뒤집힘 (실험 브랜치, 기본 꺼짐)

근거: 11/19 오전 05·06·08판 코너 분석. 성공한 코너는 도면 곡률이 끝까지 유지된 경우뿐이고,
실패한 3·4번은 코너 도중 명령 곡률이 0이나 반대로 튐 (L/R 깜빡임, lane↔heading 전환).

**세 기능 모두 기본 꺼짐 = main과 같은 동작.** `scripts/corner_fix.sh`로 주행 중 켜고 끔.

| 순서 | 파라미터 | 하는 일 |
|---|---|---|
| 1 | `/lane_node corner_lock` (+ `corner_lock_corr_max` 0.3) | 코너 미리보기 중(\|도면 k\| ≥ corner_ld_on) 기존 섞기는 그대로, **코너 방향으로 최소 \|도면 k\| − 0.3** 은 꺾게 (L/R 헷갈림으로 풀리거나 뒤집히지 않게). 더 꺾는 쪽(바깥 밀림 보정)은 그대로. 이때 코너 배율 curve_kappa_gain은 안 곱함 (선 보일 때 1.2배·놓쳤을 때 1배로 출렁이던 것) |
| 2 | `/lane_node keep_kappa_on_heading_switch` | 코너 도중 normal↔heading 전환 때 kappa_cmd·코너 속도 판단을 0으로 안 지움 (원본은 전환마다 0 -> 4번 '덜 꺾음' -> 다시 원호 반복). 코너 끝·합류 S자 전환은 기존대로 0 |
| 3 | `/mission_node m1_lane_back_frames` (기본 1) | 4번(M1) 강제 원호 중 선이 N프레임 연속 보여야 차선으로 복귀 (5 권장). M2_EXIT는 안 바뀜 |

주의: `corner_lock`을 켜면 선이 보이는 코너에서 1.2배 배율이 빠져 원본보다 덜 꺾을 수 있음 → 바깥 여유 확인.

뺀 것: 3번 코너 보호를 M8까지 유지(라바콘 노드와 동시 조향), 1번 코너 보호(코너 시작 거리 실측 없음, 근본은 노출).

테스트: `test_corner_fix.py`(lane), `test_corner_fix_fsm.py`(mission). 기존 lane+mission 테스트 471개 그대로 통과.
