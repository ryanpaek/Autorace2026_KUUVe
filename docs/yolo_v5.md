# YOLO v5 모델 안내

기본 카메라 인식 모델을 v4.5에서 v5로 교체함. YOLO11n 기반이며 9개 클래스와 ROS 명령 규약은 유지함. 빨강+노랑 동시 점등은 별도 클래스를 추가하지 않고 `traffic_red`로 학습함.

## 모델과 실행 설정

- 배포 파일: `models/autorace_yolo_v5.onnx` (10,631,710 bytes).
- 입력: FP32 RGB NCHW `[1, 3, 640, 640]`, batch 1 고정. 출력: `[1, 13, 8400]`.
- ONNX opset 12, NMS 미포함. 기존 `yolo_onnx.py`의 letterbox·정규화·decode·NMS를 사용함.
- 학습 크기 640, 현재 배포 입력도 640임. 이 파일의 `input_size`만 320으로 바꾸면 안 됨. 320 시험 시 별도 320 ONNX 및 경로 변경이 필요함.
- detector와 class_command_node의 `conf_threshold`는 모두 0.7 유지함.
- CPU 설정은 기존 `infer_hz: 5.0`, `ort_threads: 2`; GPU 설정은 기존 `infer_hz_gpu: 15.0` 유지함. 처리 빈도와 스레드 제한은 자원 사용을 조정하며 입력 해상도를 줄이지 않음. 실제 처리율은 장비에서 확인해야 함.

| ID | 학습 클래스 | ROS 클래스 |
|---:|---|---|
| 0 | traffic_red (빨강+노랑 포함) | red |
| 1 | traffic_yellow | yellow |
| 2 | traffic_green | green |
| 3 | sign_left(A) | fork_a |
| 4 | sign_right(B) | fork_b |
| 5 | sign_pedestrian | pedestrian |
| 6 | sign_bump | bump |
| 7 | sign_slow | slow |
| 8 | sign_parking | parking |

## 학습 및 결과

v4.5 best.pt를 시작점으로 기존 학습 자료와 신호등 추가 자료 292장을 사용함. 추가 자료는 빨강 73장, 빨강+노랑 73장, 노랑 73장, 초록 73장임. 전체 train 3,161장, 기존 val 704장으로 30 epoch 학습했으며 best는 26 epoch임.

| 기존 검증 704장 | v4.5 | v5 |
|---|---:|---:|
| Precision | 99.469% | 99.135% |
| Recall | 99.221% | 98.453% |
| mAP50 | 99.365% | 99.227% |
| mAP50–95 | 91.954% | 91.898% |

기존 검증 지표는 소폭 하락함. 이 검증셋은 기존 종이 모형 중심으로, 새 신호등 환경 성능을 단독으로 대표하지 않음. 위 지표는 검증 과정에서 계산된 값이며 고정 conf 0.7의 결과로 해석하면 안 됨.

별도 신호등 영상 4개(각 240프레임, 총 960프레임)를 640/conf 0.7로 확인한 결과는 다음과 같음.

| 영상의 정답 클래스 | v4.5 정답 클래스 검출 프레임 비율 | v5 정답 클래스 검출 프레임 비율 |
|---|---:|---:|
| 빨강 → red | 40.42% | 99.58% (239/240) |
| 노랑 → yellow | 34.58% | 98.75% (237/240) |
| 초록 → green | 39.17% | 97.50% (234/240) |
| 빨강+노랑 → red | 5.42% | 100% (240/240) |

v5는 위 영상에서 conf 0.7 이상 다른 클래스 검출 프레임이 없었음. 일부 큰 각도 구간에서 미검출이 있었음. 이는 프레임별 예상 클래스 검출 비율이며, 박스 정답으로 계산한 mAP·precision·recall이 아님. 합성 학습·시험 배경이 유사하므로 실제 LIMO 카메라, 원거리, 조도 변화에서 추가 확인이 필요함.

## 명령과 주행 로직

모델 경로만 바꾸며 기존 명령 발행·미션·차선·갈림길·장애물·주차 소스는 유지함. 빨강+노랑은 red 검출로 기존 적색 처리를 따름. 황색은 기존 `yellow_action: none`을 유지함. 황색 자체가 새 주행 명령을 만들지는 않으며, 실제 정지·출발 판단은 기존 미션 상태와 정지선 조건을 따름.

현재 설정에서 M1 신호등 투표는 최근 10프레임 중 4회이며 최초 10프레임을 기다림. 투표 설정을 모델 교체와 함께 변경하지 않음. 팀원의 기존 대기 시간과 정지 거리 설정도 유지함.

## LIMO 적용

저장소 루트가 `/home/wego/autorace_ws`인 기존 환경을 기준으로 함. 다른 위치라면 `params.yaml`의 절대 모델 경로를 실제 경로에 맞춰야 함.

주행 노드를 종료한 상태에서 새 ONNX로 실제 LIMO의 TensorRT 엔진을 생성함.

```bash
cd ~/autorace_ws
bash tools/build_trt.sh "$PWD/models/autorace_yolo_v5.onnx"
colcon build --symlink-install --packages-up-to autorace_bringup
source install/setup.bash
```

성공 시 `models/autorace_yolo_v5.engine`이 만들어짐. 기존 v4.5 엔진을 이름만 바꿔 사용하면 안 됨. 엔진은 장비에서 생성하며 Git에 올리지 않음. 현재 `engine_path: ''`이면 ONNX와 같은 이름의 엔진을 찾음. 실제 설정에 engine_path가 따로 지정돼 있다면 v5 엔진을 가리키는지 확인해야 함.

이후 팀에서 사용하는 기존 launch 명령으로 재시작하고 아래 값을 확인함.

```bash
ros2 param get /detector_node model_path
ros2 param get /detector_node input_size
ros2 param get /detector_node conf_threshold
ros2 param get /class_command_node conf_threshold
ros2 param get /detector_node engine_path
ros2 param get /detector_node use_gpu
```

모델 경로가 v5 ONNX이고 입력 640, 양쪽 conf 0.7인지 확인함. 로그에서 v5.engine 로드와 GPU 사용을 확인함. 엔진이 없거나 로드에 실패하면 CPU로 돌아갈 수 있으므로 파일 교체만으로 GPU 전환이 완료됐다고 판단하지 않음.

conf를 현장에서 변경한다면 두 노드 모두 확인해야 함. 예를 들어 아래 값은 재시작 전까지만 적용되므로 최종 확정값은 YAML에도 기록함.

```bash
ros2 param set /detector_node conf_threshold 0.7
ros2 param set /class_command_node conf_threshold 0.7
```

## 검증 범위

ONNX checker 통과. 실제 저장소 전처리·decoder를 사용한 21개 입력에서 PyTorch와 ONNX Runtime CPU의 클래스 및 박스 출력을 허용 오차 내에서 확인함(최대 좌표 차이 0.003 px 미만, score 차이 0.000004 미만). 기존 명령 처리 테스트 39개, 명령 연동 테스트 10개 통과함. TensorRT 변환과 ROS2 실차 주행은 이 패키지에서 검증하지 않았음.

정확한 파일 SHA256, 형식, 지표와 검증 기록은 `models/autorace_yolo_v5_manifest.json`에 있음. 이전 모델은 비교·복구용으로 유지함.
