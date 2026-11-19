# 11/19 노란 선 옆 거리 노드 (yellow_side_node)

## 왜
코너 전 직선에서 "차가 차로 안에서 바깥/안쪽으로 몇 mm 밀려 있나"를 따로 재고 싶다 (캡틴 로직: 밀린 만큼 코너 시작을 보정).
lane_node의 `yellow_l_mm/yellow_r_mm`은 3차선 모드에서만 계산되고, lane_core를 고치면 다른 기능과 얽힌다.
→ **lane_core.py는 그대로 두고** 별도 노드가 `/lane/yellow_side`로만 발행. 아직 아무 노드도 이 값으로 조향하지 않는다.

## 무엇
- `autorace_lane/yellow_side.py` (계산), `yellow_side_node.py` (ROS), `test/test_yellow_side.py` (20개)
- launch `yellow_side:=true`일 때만 뜸 (기본 false). params.yaml 맨 끝 `yellow_side_node` 칸
- 노란색 임계값은 params.yaml의 lane_node 값을 그대로 읽음 (두 노드 값이 어긋나지 않게)

## 흐름
```
카메라 영상 (3장 중 1장, 10 fps)
 ① 차 앞 800~1000 mm 띠만 BEV로 뽑기 (remap 표 미리 계산, 25줄 x 400열)
 ② 띠에서 노란색 마스크
 ③ 줄마다 차 중심 왼쪽/오른쪽에서 가장 가까운 노란 덩어리 (8~120 mm 폭만)
 │   └ 못 찾으면: 직전 선 +-60 mm 안의 옅은 노랑(S>=20, V>=200)만 인정 (새 선 안 만듦)
 ④ 줄별 거리 중앙값 → raw_l / raw_r (5줄 이상 잡혀야 ok)
 ⑤ 0.5초 중앙값 필터 (50 mm 넘게 튀면 버림, 3번 연속이면 인정)
 ⑥ 양쪽 다 → offset_mm = (l - r)/2 (오른쪽 +)
    한쪽만 → offset_one_mm (반폭 257.5로 추정, 참고용)
```

## 근거
- 띠 위치 800~1000 mm: bev_limo.yaml 계산. 550 mm 앞은 옆으로 -188~+220 mm만 보여 반폭 257 mm 선이 화각 밖 (시험 `test_550_band_cannot_see_lines`)
- **오프라인 (05·06·08 프레임, 자동 노출 시절, #151 이전)** — 실차 주행 시험 아님
  - 직선 양쪽 다 보일 때 l+r 중앙값 516~520 mm (도면 515)
  - lane offset_mm과 차이 중앙값 0~10 mm, 평균 절대 차이 35~45 mm
  - 직선에서 양쪽 다 보임 39~44% (옅은 노랑 다리 없으면 28~30%)
  - 한쪽만으로 추정한 값은 lane과 평균 차이 40~150 mm → offset_mm에서 뺌
  - 코너 2번 앞 3초: 세 판 모두 17~18/18 프레임 양쪽 다 봄. 3번 앞: 06판 15/18. 1번 앞: 0 (노출 포화, #151 뒤엔 몰라). 4번 앞: 콘 구간 직후라 거의 0
- 계산량 (클라우드 같은 컴퓨터에서 비교): lane_core.process 14.5 ms vs yellow_side 0.52 ms (3.6%) + 영상 변환 0.11 ms
  - 리모 추정: 처리 약 2 ms x 10 fps + 영상 메시지 받기(30 fps 전부) → 코어 하나의 약 5~8%, 전체 CPU(6코어) 약 +1~1.5%p. **실측 필요**

## 아직 안 한 것
- 이 값을 FSM/코너 시작에 연결 (결재 후 별도 PR)
- 리모 실측 (CPU·lane proc_ms on/off 비교)
