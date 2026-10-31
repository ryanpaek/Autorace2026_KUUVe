# 차선 분할 모델 (딥러닝)

HSV 대신 학습한 모델로 차선 마스크를 만든다. 대회장 조명을 모르기 때문.
마스크 뒤 단계(BEV, 쌍 선택, 중앙선, 정지선, Pure Pursuit)는 HSV와 똑같다.

**2클래스 모델**: 0 = 노란 차선, 1 = 흰 표시(정지선·횡단보도·3차선 점선·마름모·'천천히').
차선 노드는 HSV 때와 같이 `lane_color`(구간별: 보통 yellow, 3차선 both)로 차선 마스크를 고르고,
정지선은 흰 표시 채널에서 찾는다. 한 클래스로 섞으면 마름모·글자가 차선 후보로 들어간다.

라벨 PNG는 컬러: 노랑 (0,255,255) / 흰색 (255,255,255) / 검정. 그림판으로 고칠 때 이 두 색으로 칠한다.

## 10/31 데이터셋
녹화 4개(hsv_straight, manual_lap, curve_v2, curve_v3)에서 5프레임마다 뽑은 사진 + 수동 수집 1,141장
-> 중복·트랙 밖 제외 2,973장. `pack_dataset.py`로 320x240 zip 3개(약 62MB)로 묶음.
라벨 규칙: 덩어리 바로 바깥이 대부분 검정 매트인 것만 표시로 인정(바닥·벽·상자 제외),
좌우 끝에 닿은 지평선 근처 흰 띠(매트 너머 바닥)는 제외.
알려진 한계: 먼 거리(지평선 근처)의 가는 선은 라벨이 빠진 경우가 있다.

## 순서
1. 모의 맵에서 수동 주행하며 사진 수집 (LIMO)
   `ros2 run autorace_perception image_collector --ros-args -p every_n:=3 -p save_dir:=~/autorace_images/day1`
   - 조명: 형광등 켜고 끄기, 창가 햇빛, 폰 플래시 반사, 그림자
   - 구간: 곡선, 지그재그, 횡단보도, 3차선, 갈림길, 주차 구역 전부
   - 차 위치: 가운데만 말고 선에 붙기, 비스듬히, 선 밟기
   - 목표: 생 이미지 약 5천 장
2. 자동 라벨 (PC): `python3 autolabel.py --images ~/autorace_images/day1 ~/autorace_images/day2 --out ~/lane_dataset --every 2`
   - 녹화(bag)에서 사진 뽑기는 ROS 없이도 된다: `offline_replay`의 `iter_bag` (pip install rosbags)
   - `contact/*.jpg`: 미리보기 20장을 한 장에. 빨리 넘겨 보기용
   - 업로드용으로 줄이기: `python3 pack_dataset.py --data ~/lane_dataset --out packs --max-mb 25`
   - `~/lane_dataset/preview`를 넘겨 보고 틀린 건 images·masks에서 같이 지우거나 masks를 고친다
   - 조명이 나쁜 사진은 HSV 라벨이 틀리기 쉬움 → 지우고, 학습 증강에 맡긴다
   - 목표: 깨끗한 라벨 1,000~2,000장
3. 학습 (PC, VRAM 4GB 가능): `python3 train_lane_seg.py --data ~/lane_dataset --out runs/v1 --pretrained`
   - 메모리가 모자라면 `--batch 4`
   - `runs/v1/val_preview`: 원본 | 노란 차선 | 흰 표시. 초록 맞춤, 빨강 놓침, 파랑 오검출
   - ImageNet 사전학습 가중치(`--pretrained`)를 내려받을 수 있어야 한다 (Colab은 됨)
4. LIMO에 올리기: `lane_seg.onnx`를 복사하고 params.yaml에서
   `lane_mask_source: model`, `seg_model_path: <경로>`
5. 비교: 같은 bag을 `offline_replay --model <onnx>`와 HSV로 각각 돌려 결과 비교
6. 실차에서 틀리는 장면을 다시 모아 2번부터 반복

## Colab으로 학습 (설치 없이)
`colab_train.ipynb`를 Colab에서 열고(파일 → 노트 업로드 또는 GitHub 탭) T4 GPU로 차례대로 실행.
데이터는 Drive의 `autorace/lane_dataset.zip`, 결과는 `autorace/models/<날짜>`에 저장된다.

## 설치 (PC)
`pip install torch torchvision onnx opencv-python numpy pyyaml` (CUDA 버전은 pytorch.org 안내대로)
LIMO에는 `onnxruntime`(가능하면 GPU 버전)만 있으면 된다.
