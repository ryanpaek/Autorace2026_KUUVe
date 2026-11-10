# YOLO v3 (신호등·표지판 검출)

> 상태: **LIMO 실차 검증 전.** 아래 수치는 검증셋, 영상, 합성 입력 기준이고, 실차 성능은 따로 기록한다.

## 모델
- Ultralytics **YOLO11n** 객체 검출 모델이다. 대회용 신호등 3종과 표지판 6종, 모두 9개 클래스를 학습했다.
- 파일은 `models/autorace_yolo_v3.onnx`이고, SHA-256은 `1c26d3aba1ca322ce889a7216fff9f360397f263ae6489c5eb1b6176d7e83c9a`이다.
- ONNX는 CPU용 FP32이고, 입력은 `[1, 3, 640, 640]` 고정, opset은 12다. 출력은 `[1, 13, 8400]`(박스 4값 + 클래스 9개)이며 NMS는 들어 있지 않다. 후처리와 NMS는 팀 디코더(`yolo_onnx.decode`)가 한다.

## 학습 기록 (GPT 작업 인수인계 기준)
| 항목 | v3 |
|---|---|
| 초기 가중치 | `auto_race_v1/weights/best.pt`에서 파인튜닝 |
| 데이터 | 학습 2,521장, 검증 704장 (객체 없는 배경 61장 포함) |
| 환경 | Mac MPS, Ultralytics 8.4.199 |
| 설정 | imgsz 640, batch 8, AdamW, seed 42, patience 20, 60 epoch |
| 증강 | 좌우·상하 반전과 HSV 색조 변화 끔 |
| 사용 가중치 | `auto_race_v3/weights/best.pt` (mAP50–95 최고는 49 epoch) |

| 평가 | 결과 |
|---|---|
| v3 검증셋 704장 | Precision 99.83%, Recall 99.58%, mAP50 99.28%, mAP50–95 91.93% |
| 단일 객체 영상 9개 (`conf=0.7`) | A/B 표지 교차 인식 없음, 회색 벽을 초록불로 잡는 오검출 없음. 노란불은 한 프레임 누락 뒤 다시 검출 |
| ONNX 변환 | 합성 입력 2종에서 PyTorch 원본과의 raw 출력 최대 절대차가 약 0.0007 |

- 위 수치의 출처는 체크포인트의 `train_metrics`와 `train_args`, 그리고 이전 영상 테스트 기록이다. 이 PR에서 다시 계산한 값은 아니다.
- 검증셋이 같은 촬영 환경에서 나왔다면 실차보다 낙관적으로 나왔을 수 있다. 실차 결과로 다시 판단한다.

## 클래스 매핑
| ID | 학습 라벨 | 발행 라벨 |
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

- `parking` 검출과 별도로 `class_command_node`가 `START_PARKING` 요청을 발행한다. mission_node가 이 요청을 받아 주차 상태에 한 번 진입하며 반복 요청으로 재시작하지 않는다. `parking_node`는 기본적으로 비활성이다. 연결 규약은 [yolo_commands.md](yolo_commands.md)를 참고한다.

## 실행
v3는 `params.yaml`을 읽는 팀 launch로 실행한다. `autorace.launch.py`는 `detector_node`에 `params`를 넘기고, `use_detector`의 기본값은 true다.

```bash
ros2 launch autorace_bringup autorace.launch.py
```

인식 노드만 단독으로 실행할 때도 **같은 params 파일**을 넣어야 한다.

```bash
ros2 run autorace_perception detector_node --ros-args \
  --params-file ~/autorace_ws/src/autorace_bringup/config/params.yaml
```

> 주의할 점
> - `detector_node.py`의 기본 클래스 순서와 기본 입력 크기는 v3에 맞췄다. 장비별 카메라·모델 경로·스레드 설정을 적용하려면 계속 params 파일을 전달한다.
> - `.pt` 백엔드의 v3 학습 라벨(`sign_slow` 등)도 팀 라벨(`slow` 등)로 정규화한다. LIMO 배포 기준은 ONNX다.
> - `model_path`는 기존 `seg_model_path`와 같은 `/home/wego/autorace_ws`를 기준으로 잡았다. **팀 LIMO의 실제 워크스페이스 경로가 맞는지는 확인이 필요하다.**

## ONNX 다시 내보내기
```python
from ultralytics import YOLO  # 8.4.199
YOLO("autorace_yolo_v3_best.pt").export(format="onnx", imgsz=640, opset=12,
    dynamic=False, simplify=False, nms=None, device="cpu")
```
다시 내보내면 SHA-256이 달라질 수 있다. 출력 shape과 메타데이터의 `names`를 다시 확인한다.

## 원본 가중치
- `autorace_yolo_v3_best.pt`의 SHA-256은 `0604d4d460b976c35c772a7b7fc93995224d8ba5da797321adba87451a577b3e`이다.
- 레포는 `*.pt`를 무시하도록 되어 있어 커밋하지 않았다. 어디로 공유할지(GitHub Release 등)는 아직 정하지 않았다.
- optimizer가 제거된 배포용 가중치다. 파인튜닝의 초기값으로는 쓸 수 있지만, 중단된 학습을 그대로 이어가는 데는 쓸 수 없다.

## LIMO 실차에서 확인할 것
1. 모델이 로드되는지 확인한다. 정지 상태에서 `ros2 topic echo /perception/detections`를 띄우고, 서행 표지가 `slow`로, 좌·우 표지가 `fork_a`/`fork_b`로 나오는지 본다.
2. `ros2 topic hz /perception/detections`와 메시지의 `infer_ms`를 본다. 현재 설정은 `infer_hz: 5.0`, `ort_threads: 2`다. 과거 2코어 클라우드 CPU에서 약 81ms였다는 기록은 LIMO 수치가 아니다.
3. 검출 임계값과 FSM `det_conf`는 둘 다 0.5다. 0.7로 돌린 영상 시험과 결과가 다를 수 있다.
4. 주행이 허용된 조건에서 서행, 신호등, 갈림길 반응을 확인한다. 느리면 해상도나 주기를 조정한다. 해상도를 바꾸면 ONNX도 같은 크기로 다시 내보낸다.
