# YOLO v4 (신호등·표지판 검출) — 실차 시험 후보

> 과거 v4 적용 기록. 현재 기본 선택/운영 가이드는 [v4.5 문서](yolo_v45.md)를 따른다. 아래 상태 설명은 v4 당시의 기록이다.

> 상태: **LIMO 실차 검증 전.** 기본 실행 모델은 여전히 **v3**이다. v4는 LIMO에서 v3와 비교해 보기 위한 후보로만 추가했고, 기본 모델 전환은 측정 결과를 보고 후속 변경에서 정한다. 추가 학습도 예정되어 있어서 이 파일은 중간 버전이다.

## v3와 달라진 점
- v3 가중치에서 시작해 **새 신호등 형태**를 추가로 학습했다. 기존 데이터에 신호등 영상 3종에서 추출한 132장을 더해 총 2,653장으로 30 epoch 학습했다. 검증셋은 v3와 같은 704장이다.
- 클래스, 입력 크기, 출력 형식은 v3와 같다. 그래서 `params.yaml`의 `class_names`와 `input_size: 640`을 바꾸지 않고 `model_path`만 바꿔서 쓴다.
- 새 신호등 형태를 학습한 것이어서 검증셋 점수에는 큰 차이가 나지 않는다. 차이는 실제 대회 모형에서 나타날 수 있으므로 실차에서 확인해야 한다.

## ONNX 사양
| 항목 | 값 |
|---|---|
| 파일 | `models/autorace_yolo_v4.onnx` |
| SHA-256 | `4d6b1118254eb15e7ccb50ca0d00e2c08e3bf479ff24ea6baad46a8c98b71ce8` |
| 원본 체크포인트 SHA-256 | `372ee96ba35fb28b0b211d57506ff5b04a931bc6947a071e2f18d5c8a0a61c20` (`auto_race_v4/weights/best.pt`) |
| 모델 / 버전 | YOLO11n, Ultralytics 8.4.199로 export |
| 입력 | `images`, `[1, 3, 640, 640]`, FP32 고정 |
| 출력 | `output0`, `[1, 13, 8400]` (박스 4값 + 클래스 9개, NMS 없음) |
| opset | 12 |

NMS와 후처리는 v3와 같이 팀 디코더(`yolo_onnx.decode`)가 한다.

## 클래스 매핑
v3와 같다. 모델 메타데이터의 `names`가 아래 순서와 일치하는 것을 확인했다.

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

## 검증 점수 (v3와 v4)
같은 검증셋 704장에서 각 실행의 **최고 mAP50–95 회차**를 비교한 값이다.

| 지표 | v3 | v4 |
|---|---:|---:|
| 최고 mAP50–95 | 91.93% (49회차) | 92.02% (15회차) |
| 해당 회차 mAP50 | 99.28% | 99.40% |
| 해당 회차 Precision | 99.83% | 99.50% |
| 해당 회차 Recall | 99.58% | 98.91% |

- v4의 mAP50–95·mAP50·Precision·Recall은 v4 체크포인트 안의 `train_metrics`와 같은 값인 것을 확인했다. 15회차라는 정보와 v3 수치는 학습 담당자의 기록을 옮긴 것이다.
- 차이가 0.1%p 안팎이어서 v4가 더 좋다고 판단할 근거는 아니다.
- 혼동행렬에서 정답과 매칭되지 않은 예측 박스는 **655개에서 309개**로 줄었다. 반면 실제 초록불을 노란불로 분류한 경우는 **1건에서 5건**으로 늘었다. 두 값은 학습 담당자의 기록이며, 이 PR에서 다시 계산하지 않았다.

## 영상 시험에서 본 것 (학습 담당자 기록)
- 표지판은 대체로 안정적이었다.
- 320 입력, `conf=0.7`은 기존 640, `conf=0.7`보다 **멀리 있거나 어두운 곳의 신호등 누락**이 많았다. 두 크기 모두 **빨간불을 초록불로 분류한 사례**가 있었다. 두 실행의 `rect` 설정이 달라서 해상도만 바꾼 비교로 보기는 어렵다.
- 640, `conf=0.25`에서는 노란불과 초록불이 일부 더 일찍 잡혔지만, 기둥 오검출과 빨강에서 초록으로의 오분류도 보였다. 0.25는 후보를 확인해 본 조건이다.
- 위 점수와 영상 결과는 **ONNX를 LIMO에서 돌린 성능이 아니다.**

## 이 PR에서 한 호환성 검사
- ONNX Runtime CPU에서 모델이 로드되고, 0 입력으로 `[1, 13, 8400]`이 유한한 값으로 나왔다.
- 메타데이터(입력 shape, 출력 shape, `names`, opset 12)가 위 사양과 일치한다.
- 팀 디코더에 클래스 ID 0~8을 하나씩 넣었을 때 발행 라벨이 위 표대로 나왔다.
- `src/autorace_perception/test/test_yolo_onnx.py` 3개가 통과했다.
- **하지 않은 것**: v4 `.pt`와 ONNX를 같은 영상 프레임에서 비교하는 검사, LIMO 실행, 처리 지연 측정.

## v4로 시험하는 방법
기본 launch는 v3를 쓴다. v4만 지정해 시험하려면 **LIMO의 팀 저장소 루트**에서 인식 launch에 `model_path:=`를 준다. 이 launch는 인식 노드와 행동 요청 발행 노드만 실행하고 차량 제어 노드는 실행하지 않는다.

```bash
ros2 launch autorace_bringup perception_commands.launch.py \
  model_path:="$PWD/models/autorace_yolo_v4.onnx" \
  params:="$PWD/src/autorace_bringup/config/params.yaml"
```

다른 터미널에서 확인한다. 로그의 `model loaded (onnx)` 경로가 v4인지 본다.

```bash
ros2 topic echo /perception/detections
ros2 topic hz /perception/detections
```

전체 launch(`autorace.launch.py`)로 시험하려면 `params.yaml`의 **시험용 사본**을 만들어 `detector_node.model_path`를 v4 절대 경로로 바꾸고 `params:=`로 넘긴다. 저장소의 기본 `params.yaml`은 v3 경로로 둔다.

## LIMO에서 확인할 것
같은 카메라 조건과 같은 설정에서 v3와 v4를 **한 번에 하나씩** 실행해 비교한다. 현재 설정은 검출 `conf_threshold: 0.5`, `infer_hz: 5.0`, `ort_threads: 2`이고, `class_command_node`도 신뢰도 0.5와 최근 10프레임 중 7프레임 투표를 쓴다. 비교하는 동안 이 값은 같게 둔다.

1. `infer_ms`(노드 내부의 전처리, 추론, 후처리 시간)와 검출 발행 주기를 기록한다. 카메라 입력부터 제어까지의 전체 지연은 따로 잰다. 전체 노드를 돌릴 때는 차선 노드의 처리 주기와 검출 지연도 같이 본다.
2. `/camera/color/image_raw`로 들어오는 실제 영상 크기를 확인한다. 카메라 앞단에서 이미 줄인 영상은 640 입력에 넣어도 원본 디테일이 복원되지 않는다.
3. 대회 모형으로 다음을 기록한다.
   - 정지선 앞에서 **빨간불이 처음 안정적으로 검출되는 거리**
   - **빨간불인데 초록불로 나오는 구간**
   - **검출이 끊기는 길이**
   - 새 신호등 형태에서 v3와 v4의 검출 차이
4. 표지판 A/B와 서행이 팀 라벨(`fork_a`, `fork_b`, `slow`)로 나오는지 확인한다.
5. 640 입력이 느리면 320으로 별도 export해서 시험할 수 있다. 같은 체크포인트를 `imgsz=320`으로 다시 내보내고, 그 ONNX와 `input_size: 320`을 함께 적용한다. CPU 스레드를 제한해도 640 입력의 프레임당 연산량은 그대로다.

## ONNX 다시 내보내기
```python
from ultralytics import YOLO  # 8.4.199
YOLO("auto_race_v4/weights/best.pt").export(
    format="onnx", imgsz=640, batch=1, opset=12, dynamic=False,
    simplify=False, nms=False, half=False, device="cpu")
```
다시 내보내면 SHA-256이 달라질 수 있다. 출력 shape과 메타데이터의 `names`를 다시 확인한다. `best.pt`는 레포가 `*.pt`를 무시하도록 되어 있어 커밋하지 않았다.
