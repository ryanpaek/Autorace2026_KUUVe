# YOLO v4.5 — 신호등 보강 모델

YOLO는 카메라 영상에서 객체의 위치(박스), 종류(클래스), 신뢰도를 한 번에 예측하는 객체 검출 모델이다. 이 프로젝트는 YOLO11n을 사용해 신호등 3종과 표지판 6종을 인식한다. v4.5는 v4에 신호등 데이터를 추가 학습한 버전이다. 이번 변경은 이미 학습된 가중치를 적용하는 작업이며 추가 학습을 실행하지 않았다.

이번 적용 범위는 **카메라 → YOLO → 클래스별 행동 요청 발행**이다. 미션 상태 머신, 속도·정지·주차·방지턱·조향 제어는 팀원의 기존 구현을 유지한다.

## 성능

v4.5 체크포인트의 학습 실행 이름 `auto_race_v45`와 저장된 `train_metrics`를 확인했다.

| 검증 지표 | v4.5 |
|---|---:|
| Precision | 99.469% |
| Recall | 99.221% |
| mAP50 | 99.365% |
| mAP50–95 | 91.954% |

학습 입력 크기는 640이다. 위 수치는 기존 검증셋에서의 체크포인트 지표이며 **320 추론 또는 LIMO 실차에서 conf=0.7로 측정한 성능이 아니다.** 기존 검증셋 점수가 높아도 멀고 작은 LED 신호등·노출·색감 차이의 문제는 남을 수 있다. 실제 LIMO 녹화 영상으로 미검출·오인식·최초 인식 거리·명령 발행 지연을 확인해야 한다.

## 모델 파일

| 파일 | 고정 FP32 입력 | 출력 | 용도 |
|---|---|---|---|
| `models/autorace_yolo_v45.onnx` | `[1,3,640,640]` | `[1,13,8400]` | 저장소 기본 선택. SHA-256 `cfeb24a74a91b740dacba689cbb8bb95b5974630f9368912cde89c93a3627a4c` |
| `autorace_yolo_v45_320.onnx` (**저장소에 없음**) | `[1,3,320,320]` | `[1,13,2100]` | 실차 비교용으로 별도 전달. 쓰려면 `models/`에 복사 |
| `models/autorace_yolo_v45_manifest.json` | — | — | 640 ONNX의 SHA256, 입력/출력, 변환 검증 기록 |

640과 320 ONNX는 **같은 v4.5 가중치**를 각각 export한 것이다. 320용으로 재학습한 모델은 아니다. Ultralytics 8.4.199 / PyTorch 2.15.0 CPU / ONNX 1.23.2, opset 12, IR 7, batch 1, dynamic=False, NMS 미포함으로 생성했다. 박스 후처리는 기존 팀 디코더가 수행한다.

각 크기마다 빈 영상 1개와 업로드 영상에서 추출한 12개 프레임, 총 13개 입력으로 PyTorch 원본과 ONNX Runtime 1.31.1 CPU 출력을 비교했다. 최대 박스 좌표 차이는 0.003px 미만, 클래스 점수 차이는 0.000005 미만이었고 같은 팀 디코더/conf=0.7에서 검출 클래스 목록이 같았다. 이는 변환 호환성 검사다. 라벨이 있는 실차 테스트 또는 전체 영상 성능 평가는 아니다. LIMO의 기존 ONNX Runtime에서도 실제 로딩·지연을 확인한다.

## 클래스와 발행 이름

| ID | 학습 라벨 | ROS 검출 라벨 |
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

ID 순서는 v3/v4와 같다. 모델 파일 이름만 바꾼 v3가 아니라 v4.5 체크포인트에서 직접 변환한 파일이다. v3/v4 ONNX는 비교·복구용으로 그대로 보관한다.

## 320/640과 CPU 제한

- 팀 전달 사항은 “현재 320을 사용 중”이지만, **저장소 기본 params.yaml은 input_size=640, infer_hz=5.0, ort_threads=2**다. 실제 실행 중인 파라미터와 카메라 원본 해상도는 별도로 확인해야 한다.
- 카메라 원본 크기, YOLO에 들어가는 크기, 디버그 화면 크기는 서로 다를 수 있다. 원본 640을 YOLO 입력 전에 320으로 줄이면 작은 신호등의 픽셀 정보가 줄어든다. 원본부터 320이면 나중에 640으로 확대해도 잃은 정보가 복구되지는 않는다.
- `ort_threads=2`는 ONNX 연산 스레드 수, `infer_hz=5`는 새 영상을 처리하는 목표 주기다. 차선·센서 노드와 CPU 경합을 줄이려는 설정이다. RAM 제한이나 CPU 사용률 25% 고정이 아니며, 640 한 장의 연산량을 320 한 장 수준으로 줄이지도 않는다. 처리 시간이 200ms보다 길면 실제 5Hz에 못 미칠 수 있다.
- 원본 해상도와 처리 시간이 충분하다면 640을 유지하면서 주기·스레드를 제한하는 편이 작은 신호등의 공간 정보를 보존하기 좋다. 다만 지연이 늘면 반응 시점은 나빠질 수 있으므로 정확도만으로 결정하지 않는다.
- 차선 주기 저하, 발열, 지속적인 처리 지연 또는 필요한 반응 거리 부족이 생기면 동일 영상/동일 conf/동일 전처리로 320과 비교한다. 320이 필요한 인식 거리와 오인식 기준을 만족할 때 전환한다. 작은 객체를 놓치면 원본 해상도·ROI·실제 카메라 데이터 보강도 함께 검토한다.
- RAM을 강제로 제한하는 방식은 권하지 않는다. 연산 정확도를 유지하면서 자동으로 1/4 비용으로 만드는 기능이 아니며 메모리 부족/프로세스 종료가 생길 수 있다.

공식 참고: [Ultralytics export](https://docs.ultralytics.com/modes/export/), [ONNX Runtime 스레드 관리](https://onnxruntime.ai/docs/performance/tune-performance/threading.html).

## 선택·실행

아래는 **LIMO 저장소 루트**에서 실행한다. 스크립트는 YOLO 모델 경로·입력 크기·인식/발행 conf 4개 값만 수정한다. 다른 노드의 설정과 주석을 보존한다.

```bash
# 640 선택. 실제 운영이 320이면 아래 대체 명령 사용.
python3 tools/configure_yolo_v45.py --size 640 --conf 0.7

# 320을 선택하려면 별도 전달받은 autorace_yolo_v45_320.onnx를 models/에 복사한 뒤, 위 명령 대신:
# python3 tools/configure_yolo_v45.py --size 320 --conf 0.7

git diff -- src/autorace_bringup/config/params.yaml
colcon build --symlink-install --packages-up-to autorace_bringup
source install/setup.bash

# 카메라 드라이버를 먼저 실행하고 인식/발행만 시험
ros2 launch autorace_bringup perception_commands.launch.py \
  params:="$PWD/src/autorace_bringup/config/params.yaml"
```

단독 launch는 차량 제어와 mission_node를 실행하지 않는다. 실제 `/mission/state`가 없으면 인식은 되지만 행동 요청은 나오지 않는다. 전체 스택과 같은 노드를 중복 실행하지 않는다. 전체 주행은 팀 기존 launch/절차를 따른다. 이 PR은 주차 노드 enabled=false도 그대로 유지한다.

```bash
ros2 param get /detector_node model_path
ros2 param get /detector_node input_size
ros2 param get /detector_node infer_hz
ros2 param get /detector_node ort_threads
ros2 param get /detector_node conf_threshold
ros2 param get /class_command_node conf_threshold
ros2 topic echo /perception/detections
ros2 topic echo /perception/commands
```

카메라 원본 해상도는 detector 입력 크기와 별도로 `/camera/color/image_raw` 메시지의 width/height를 확인한다. 예: `ros2 topic echo --once /camera/color/image_raw --field width`와 `--field height` (해당 ROS CLI 지원 여부에 따라 한 번 받은 메시지 헤더를 확인).

`model_path`, `input_size`, `class_names`, `infer_hz`, `ort_threads`는 노드 시작 시 적용하므로 변경 후 재시작한다. 이번 변경에서 읽기 전용 파라미터로 표시해, 실행 중 param set은 성공했으나 실제 모델/타이머는 안 바뀌는 혼동을 막았다. 고정 640 ONNX에 input_size만 320으로 바꾸면 시작 시 명확한 오류를 낸다.

## conf 조정

시험 conf는 모델 가중치에 저장되어 운영에 자동 적용되는 값이 아니다. 운영 ROS 노드에서 지정한다. 기본은 detector와 class_command 모두 0.7이다.

```bash
# 실제 테스트에서 0.7이 너무 높으면, 예를 들어 두 곳을 같이 0.65로 변경
ros2 param set /detector_node conf_threshold 0.65
ros2 param set /class_command_node conf_threshold 0.65
ros2 param get /detector_node conf_threshold
ros2 param get /class_command_node conf_threshold
```

둘 중 하나가 0.7이면 0.65 검출이 행동 요청까지 통과하지 못할 수 있다. 명령 노드의 conf 변경은 기존 투표를 초기화하되 적색 정지 상태는 유지한다. `ros2 param set`은 이 실행에만 적용된다. 확정값은 YAML 두 곳에 저장하거나 위 스크립트를 사용한다.

팀의 별도 원시 검출 경로(`mission_node.det_conf`), 갈림길 노드(`fork_node.det_conf`), 장애물 종료 판단(`obstacle_node.m7_off_sign_conf`)은 기존 0.5를 유지했다. 이들을 쓰는 운영 모드에서는 각 담당자와 함께 설정을 확인한다. 기본 명령 경로의 투표는 최근 10프레임을 채운 뒤 구간별 필요 횟수(신호등 7회 등)를 확인한다. 5Hz라면 첫 창에 약 2초가 필요하므로 conf뿐 아니라 최초 인식 거리와 발행 지연을 같이 측정해야 한다.
