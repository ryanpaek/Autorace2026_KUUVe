# YOLO v3 (신호등·표지판 검출)

- **모델**: YOLO11n 기반으로 대회용 신호등 3종과 표지판 6종을 학습한 객체 검출 모델 (`models/autorace_yolo_v3.onnx`).
- **변경 사항**: v3 가중치를 640×640 입력의 ONNX로 변환하고, 팀 미션 코드가 쓰는 이름으로 9개 클래스를 매핑 (`params.yaml` → `detector_node`).

| ID | 원본 라벨 | 팀 이름 |
|---|---|---|
| 0 | traffic_red | red |
| 1 | traffic_yellow | yellow |
| 2 | traffic_green | green |
| 3 | sign_left(A) | fork_a |
| 4 | sign_right(B) | fork_b |
| 5 | sign_pedestrian | pedestrian |
| 6 | sign_bump | bump |
| 7 | sign_slow | slow |
| 8 | sign_parking | parking |

## 실행 방법
v3는 `params.yaml`을 읽는 팀 launch로 실행한다.

```bash
ros2 launch autorace_bringup autorace.launch.py
```

> 주의: `detector_node.py`의 `DEFAULT_CLASSES`는 v3와 순서가 달라서 그대로 두었다. `ros2 run`으로 단독 실행하는 등 `params.yaml` 없이 띄우면 클래스 이름이 잘못 매핑된다. 단독으로 실행할 때는 `--ros-args --params-file src/autorace_bringup/config/params.yaml`을 붙인다.

- **검증 결과**: 합성 입력 2개에서 PyTorch 원본과 ONNX의 출력이 허용 오차 안에서 일치했고, 팀 디코더가 9개 클래스 ID를 올바른 이름으로 해석하는 것을 확인.
- **남은 검증 (LIMO 실차 검증 아직 안 함)**: LIMO에서 실제 카메라 영상 기준으로 검출 정확도, 추론 속도, 미션 반응을 확인해야 함. 학습 성능 수치(mAP 등)는 원본 학습 로그에서 확인한 값이 있을 때만 추가.
