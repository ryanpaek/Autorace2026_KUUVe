# Autorace2026_KUUVe — Autorace 2026 (LIMO Pro, ROS2 Humble)

국민대 Autorace 2026 본선용 ROS2 워크스페이스 초안이다. 팀 인수인계서(Docs)와 함께 본다.
차선 노드는 D-Racer(SEA:ME 해커톤) 차선 인식을 옮긴 **초안**이고, 실차 검증 전이다.

## 패키지

| 패키지 | 노드 / 도구 | 상태 |
| --- | --- | --- |
| `autorace_lane` | `lane_node`, `bev_calib`, `offline_replay` | 초안 동작 (오프라인 테스트 통과, 실차 튜닝 필요) |
| `autorace_mission` | `mission_node` (FSM + /cmd_vel 선택기), `parking_node` | FSM 동작, 미션별 거리값 TODO, 주차 계획 TODO |
| `autorace_obstacle` | `obstacle_node` | 라이다 클러스터·라바콘 추종 초안, 3차선 회피 TODO |
| `autorace_perception` | `detector_node`, `image_collector` | ONNX/ultralytics 추론 틀, 모델 학습 TODO |
| `training/lane_seg` | `autolabel.py`, `train_lane_seg.py` (PC에서 실행) | 차선 분할 모델 수집·라벨·학습·ONNX. 사용법은 그 폴더 README |
| `autorace_bringup` | `autorace.launch.py`, `lane_test.launch.py`, 설정, 스크립트 | 실행 틀 |

## 폴더

| 폴더 | 내용 |
| --- | --- |
| `src/` | ROS2 패키지 (colcon 빌드 대상) |
| `calib/` | LIMO 캘리브레이션 결과([README](calib/README.md))와 시험 스크립트([USAGE](calib/USAGE.md)) |
| `tools/` | 경로 기록·재생 등 `python3`로 직접 실행하는 도구 ([README](tools/README.md)) |
| `training/` | PC에서 돌리는 학습 코드 |
| `models/` | 학습된 ONNX 모델 |
| `docs/` | 트랙 도면 등 자료 |

## 빌드

```bash
# 레포 루트가 곧 colcon 워크스페이스다 (src/가 바로 아래)
git clone https://github.com/ryanpaek/Autorace2026_KUUVe.git ~/autorace_ws
cd ~/autorace_ws
colcon build --symlink-install      # symlink: config/*.yaml 수정이 바로 반영됨
source install/setup.bash
```

## 첫날 순서 (요약)

1. LIMO 기본 bringup으로 드라이버·카메라·라이다를 띄운다.
2. `bash src/autorace_bringup/scripts/check_topics.sh` → 토픽 이름이 다르면 `config/params.yaml` 수정.
3. 캘리브레이션 사진: `ros2 run autorace_perception image_collector --ros-args -p every_n:=1 -p max_images:=1`
4. `ros2 run autorace_lane bev_calib --image <사진> --rect 500 600 --near <mm> --out src/autorace_bringup/config/bev_limo.yaml`
5. 바퀴를 띄운 상태에서 `ros2 launch autorace_bringup lane_test.launch.py speed:=0.0` → `/lane/debug/compressed` 확인.
6. 바닥에서 `lane_test.launch.py speed:=0.2` → 차선 주행.

비상 정지: `ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}"`

## 토픽

| 토픽 | 타입 | 내는 쪽 → 받는 쪽 |
| --- | --- | --- |
| `/cmd_vel` | Twist | mission_node → LIMO (**이 노드만 발행**) |
| `/lane/cmd_vel` | Twist | lane_node → mission_node |
| `/obstacle/cmd_vel`, `/parking/cmd_vel` | Twist | 각 노드 → mission_node |
| `/lane/config` | String(JSON) | mission_node → lane_node: `{"mode","speed","lane_width_mm",...}` |
| `/lane/status` | String(JSON) | lane_node → mission_node: status, offset_mm, stoplines_mm, crosswalk_lines_mm |
| `/perception/detections` | String(JSON) | detector_node → mission_node: `{"detections":[{"cls","conf","bbox"}]}` |
| `/obstacle/state` | String(JSON) | obstacle_node → mission_node |
| `/mission/state` | String(JSON) | mission_node → 모두 |
| `/mission/start` / `/mission/estop` / `/mission/set_state` | Empty / Bool / String | 사람 → mission_node |
| `/route/set_s` | Float32 | 사람 → mission_node: 경로 위치 s(mm) 직접 지정 (연습용) |
| `/limo/ack_cmd` | AckermannDrive | ackermann_bridge → LIMO 애커만 드라이버 (`/cmd_vel`을 변환) |
| `/lane/debug/compressed`, `/perception/debug/compressed` | CompressedImage | 디버그 화면 (rqt_image_view, Foxglove) |

## 도면 경로 지도 (track_route.yaml)

라이다 지도 대신 도면(2026Track DXF)에서 뽑은 차로 중심선과 흰 가로선 위치를 `autorace_bringup/config/track_route.yaml`에 저장해 두었다.
mission_node가 오도메트리로 경로상 위치 `s`(mm, 출발 체커 = 0)를 늘리고, 차선 노드가 흰 가로선을 보면(`stoplines_mm`) 도면의 그 선 위치로 `s`를 다시 맞춘다.
미션 구간 전환(지그재그 시작·끝, 3차선 끝, 갈림길 합류, 라바콘 끝)과 정지선 예비 거리를 이 `s`로 판단한다. 좌우 위치는 차선 인식이 맡는다.

- 현재 위치 보기: `ros2 topic echo /mission/state` → `"route": {"s_mm", "next", "next_in_mm", "last_fix"}`
- 미션 하나만 연습: `/mission/set_state`로 상태를 바꾸면 `s`도 그 미션 시작점으로 맞춰진다. 차를 그 자리에 놓을 것.
- 끄기: `ros2 launch autorace_bringup autorace.launch.py route_file:=''` (예전 거리 기준으로 동작)
- 실측할 것: `odom_scale` (직선 3 m를 줄자로 재고 /odom 거리와 비교)

## 규약

- 곡률 `kappa`(1/m)와 `angular.z`는 **왼쪽이 +** (ROS 규약). Twist는 `linear.x = v`, `angular.z = v * kappa`.
  애커만·사륜 차동 모드 모두 이 식으로 맞다 (LIMO 드라이버가 애커만에서 r = v/ω로 조향각을 계산).
- 거리 단위는 mm, 속도는 m/s.
- 정지선 거리 `stoplines_mm`는 **앞바퀴 → 선의 가까운 가장자리** (규정 판정 기준점이 앞바퀴).
- 헤딩은 `/imu` orientation yaw를 쓴다 (`/odom` yaw는 드라이버가 작은 변화를 버림).

## 테스트 (ROS 없이)

```bash
cd ~/autorace_ws
python3 -m pytest src -q          # 38개: 차선·정지선·지그재그·FSM·라이다·ONNX 디코딩
```
차선 알고리즘을 고치면 `src/autorace_lane/test/test_lane_core.py`에 합성 트랙 케이스를 추가해 확인한다.
녹화한 주행으로 튜닝 전후를 비교하려면 `offline_replay`를 쓴다.

## TODO 찾기

```bash
grep -rn "TODO" src --include=*.py --include=*.yaml
```

## 출처

차선 인식 구조·Pure Pursuit·다수결 투표·ONNX 디코딩은 D-Racer(`ryanpaek/D-Racer-Kit`, release/v1.0.0)에서 옮겼다.
LIMO 토픽·애커만 처리는 `agilexrobotics/limo_ros2`(humble) 드라이버 코드 기준이다.
