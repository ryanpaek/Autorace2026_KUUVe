# YOLO 클래스 → 구간별 명령 발행

담당 범위는 **객체 확인 → 현재 구간에 맞는 행동 요청**이다. 속도·정지 여유 거리·대기 시간·방지턱 통과·주차·조향은 기존 팀원 코드가 실행한다. 이 노드는 `/cmd_vel`을 발행하거나 신호 때문에 종료하지 않는다.

```text
/perception/detections ─┐
                       ├→ class_command_node → /perception/commands → 팀원 수신부
/mission/state ────────┘
   현재 구간 + 카메라 선/지도 매칭 결과
```

**연결 상태:** 전체 launch의 기본 경로는 `/perception/commands` → `mission_node` → 기존 FSM/실행 노드다. 원시 검출과 새 명령을 동시에 차량 제어에 적용하지 않는다. `use_class_commands:=false`는 명령 발행을 끄고 이전 원시 검출 경로로 함께 전환하는 비교용 설정이다.

확인된 명령은 FSM에서 다시 투표하지 않는다. 7/10 판단은 발행부 한 곳에서 수행하고, 수신부는 확인된 상태를 기존 행동에 전달한다. 반복 수신 콜백은 `_goto`나 주차 초기화를 호출하지 않는다. 주차 시작, 서행 시작, 녹색 출발은 기존 하위 상태가 처음 진입할 때만 시간·거리 기준을 설정한다. 초록불·주차 표지가 사라져도 이미 시작한 행동은 기존 완료 조건까지 이어진다.

로컬 기준 커밋은 `9b1dcc325057b8b8d49b8d944d115259a7f1dd5a`. 2026-11-10 `git fetch`는 DNS 오류, GitHub 연결 도구는 404여서 최신 원격 변경은 확인하지 못했다.

## 구간별 허용 객체

`/mission/state`의 기존 `state` 이름을 사용한다. 기본 목록은 `class_commands.py`의 `MODE_CLASSES` 한 곳에 있다.

| 구간 | 발행에 사용하는 클래스 |
|---|---|
| M1_TRAFFIC | red, yellow, green, pedestrian |
| M2_FORK | fork_a, fork_b |
| M3_PEDESTRIAN | pedestrian |
| M4_PARKING | parking |
| M5_BUMP | slow, bump |
| M6_ZIGZAG | 없음 |
| M7_OBSTACLE | fork_a, fork_b (기존 FSM도 다음 갈림길 선택을 미리 기억) |
| M8_CONES | 없음 |

M1의 pedestrian은 사용자 설명의 신호 횡단보도에 대응하는 요청 슬롯이다. 기존 FSM의 M1 제어는 신호등을 기준으로 동작한다. WAIT_START·FINISHED·알 수 없는 모드·모드 미수신은 명령 없음이다. 구간이 바뀌면 이전 표와 신호등 정지 요청을 비운다. 다른 구간의 빨간불은 투표에도 들어가지 않는다.

`context_timeout_sec`(기본 1초) 동안 구간 메시지가 없으면 새 명령을 차단한다. 다시 수신하면 새 투표부터 시작한다. 이 설정은 시작 시 지정한다. 차량의 통신 중단 대응은 수신 제어기의 책임이다.

## 명령과 지속 규약

| 클래스 (v3 ID) | 명령 | 의미 |
|---|---|---|
| red (0) | `PAUSE_UNTIL_GREEN` | `stop_target: before_stop_line`. 기존 정지선 접근 제어로 여유를 두고 일시정지, 녹색까지 유지 |
| yellow (1) | 기본 명령 없음 | 검출·투표 정보만 기록. `yellow_action: pause`로 변경 가능 |
| green (2) | `RESUME_NORMAL` | 신호등 정지 해제. 다른 제약이 없다면 정상 주행 지속 |
| fork_a (3), fork_b (4) | `SELECT_ROUTE`, `route: A/B` | V자 갈림길의 지정 경로. 90도 회전 아님 |
| pedestrian (5) | `STOP_AT_PEDESTRIAN_LINE` | `traffic_policy: obey_if_present`. M3은 기존 무신호 정지, M1은 신호 준수 |
| bump (6) | `CROSS_BUMP_SLOWLY` | 기존 방지턱 통과 전략 요청. 통과 뒤 정상 복귀도 기존 실행부 담당 |
| slow (7) | `SLOW` | 현재 표지 + 투표 통과 + 최근 바닥 서행선 확인이 모두 필요 |
| parking (8) | `START_PARKING` | 기존 주차 전략 요청 |

- 빨간불·녹색이 함께 확인되면 빨간불이 우선한다. 정지는 적색이 사라져도 유지한다. 정지 이전 녹색 표는 폐기하며, 이후 새 녹색 표가 기준을 충족하고 현재에도 녹색이 있어야 해제한다.
- **초록불이 안 보이게 되어도 정지 명령을 내지 않는다.** 빈 `commands`는 취소·정상 복귀·정지 명령이 아니다. 실행부가 정상 주행을 유지한다. 녹색은 다른 감속·보행자·주차 제약을 취소하는 전역 가속 명령이 아니다.
- 같은 요청을 매 프레임 보내는 것은 지속 상태로 취급한다. 주차·방지턱·서행의 거리나 타이머를 매 메시지마다 초기화하지 않는다. 수신부는 같은 구간에서 실행 중/완료한 행동을 반복 요청으로 재시작하지 않아야 한다.
- A/B가 모두 확인되면 선택을 보류하고 `conflicts: ["fork_direction"]`을 보낸다. 최종 선택 유지와 실행은 기존 미션에서 맡는다.
- 정상 속도는 하드코딩하지 않는다. 로컬 기존 `base_speed`는 0.30m/s다.

## 바닥 서행선: 기존 코드에서 찾은 연결점

별도 바닥 토픽을 새로 만들지 않는다. 현재 흐름은 다음과 같다.

1. `lane_core.py`가 흰 가로선을 검출해 `/lane/status.stoplines_mm`에 담는다.
2. `mission_node.py`가 새 차선 프레임을 `RouteTracker.update()`에 전달한다.
3. `route.py`가 선을 지도와 매칭한다. 지도 `track_route.yaml`에는 `m5_slow_line`이 있다.
4. `/mission/state`의 `route.last_fix.name == "m5_slow_line"`이며 `route.fixes`가 이전 메시지보다 증가했으면 새 바닥 확인으로 사용한다.

`last_fix`는 과거 결과도 계속 전송되므로 그 이름만 보고 계속 true로 취급하지 않는다. 새 매칭이 없으면 기본 1초 뒤 만료한다. 구간 진입 시 처음 받은 횟수는 기준으로만 저장하며, 다음 새 매칭부터 인정한다. 다른 선 매칭·경로 정보 소실·구간 변경은 기존 확인을 폐기한다.

이것은 **코스 지도 + 카메라의 선 검출**이며 노면 글자 '서행'을 읽는 기능은 아니다. 일반 정지선이 보였다는 이유만으로 서행선으로 인정하지 않는다. 지도 매칭이 꺼져 있거나 실패하면 이 발행부의 `SLOW`는 나오지 않는다. `bump` 요청은 바닥 서행선 조건과 독립적이다.

명령 경로에서는 M5의 지도 위치만으로 감속을 시작하는 우회를 끄고 확인된 서행/방지턱 요청을 사용한다. M4는 확인된 주차 요청 전까지 접근 주행하며, 요청을 받으면 한 번 주차 상태에 진입한다. parking_node는 M4이면서 미션의 source가 parking일 때 기존 주차 로직을 실행한다. 활성 구간에서는 같은 로직 객체를 계속 사용하며, 반복 요청이 상태·타이머를 초기화하지 않는다. 기존 `parking_node.enabled` 설정은 유지하므로 실차 실행에는 해당 설정이 필요하다.

## 오검출 필터와 현장 조정

처리 순서는 **구간 허용 → confidence·크기 → N프레임 중 K회 → 명령별 조건**이다.

- 기본 `vote_window: 10`, `vote_need: 7`. 처음 10개 새 추론 프레임을 모두 모은 뒤 7회 이상 확인한다. 7회 연속일 필요는 없다. 5Hz 추론이면 초기 확인까지 약 2초다. 카메라 원본 FPS가 아니라 검출기가 발행한 프레임 수 기준이다.
- 빈 검출도 한 프레임으로 센다. 같은 프레임의 같은 클래스 박스 여러 개는 한 표다. 같은 stamp의 반복 입력은 버린다.
- `min_bbox_height_px.<클래스>`는 원본 영상 박스 높이(px)다. 모두 0.0으로 시작해 크기 필터를 끈다. 실차 촬영 거리와 해상도에 맞춰 조정한다. 0보다 크면 bbox가 없거나 잘못된 검출도 거른다.
- 투표 수·신뢰도·크기는 실행 중 변경 가능하다. 바꾸면 예전 표를 비우되 이미 확인한 신호등 정지는 유지한다. 부적절한 파라미터 조합은 거부한다.

```bash
# 10프레임 중 8프레임으로 강화 / 다시 7로
ros2 param set /class_command_node vote_need 8
ros2 param set /class_command_node vote_need 7
# 예시 값일 뿐, 실제 권장 크기가 아님
ros2 param set /class_command_node min_bbox_height_px.slow 50.0
ros2 param set /class_command_node min_bbox_height_px.red 0.0
ros2 param set /class_command_node yellow_action none
```

창 크기를 줄일 때는 `vote_need`를 먼저 낮춰 항상 `1 <= need <= window`를 만족시킨다. 영구 저장은 `src/autorace_bringup/config/params.yaml`을 수정한다. 검출기가 이미 버린 낮은 confidence의 객체는 발행부 임계값만 낮춰 복구할 수 없다.

## 시간 역행이 주행에 주는 문제와 처리

카메라 재연결·기록 영상 재생 초기화로 프레임의 stamp가 100에서 0으로 바뀔 수 있다. 실제 시간이 거꾸로 흐르는 뜻은 아니다. 이전 방식은 100을 넘을 때까지 새 프레임을 버려 녹색 전환을 놓치고 정지 상태가 계속될 수 있었다.

이제 stamp가 작아지면 그 첫 프레임은 버리고 투표를 비운 뒤 새 시간축에서 다시 모은다. 기존 빨간불 정지는 유지하고 녹색도 다시 투표한다. 별도 노드 재시작은 필요 없다. 드문 역순 패킷도 같은 처리라 재확인 지연이 생길 수 있다. 하나의 검출기만 연결한다. 시간 역행과 중복은 경고 로그에 남긴다.

## 메시지와 실행

입력·출력은 `std_msgs/msg/String` JSON이다. 검출 입력 예:

```json
{"stamp":123.4,"detections":[{"cls":"slow","conf":0.92,"bbox":[100,80,60,60]}]}
```

출력은 `schema_version: 2`, `mode`, `stamp`, `observed_classes`, `confirmed_classes`, `vote`, `traffic_stop_active`, `commands`, `conflicts`를 포함한다. observed는 **구간·신뢰도·크기 필터 통과 후**의 현재 관측이다. 전체 원시 검출은 `/perception/detections`에서 확인한다. 명령에 마지막 관측의 `source_stamp`, `confidence`, 선택적 `bbox`, 현재 미검출 여부인 `held`가 들어 있다.

전체 launch는 기존 `/mission/state`를 그대로 사용한다. 검출·발행만 시험할 때:

```bash
colcon build --symlink-install --packages-up-to autorace_bringup
source install/setup.bash
ros2 launch autorace_bringup perception_commands.launch.py \
  model_path:="$PWD/models/autorace_yolo_v3.onnx"
ros2 topic echo /perception/commands
```

이 단독 launch는 미션 노드를 띄우지 않는다. **모드 입력 없이 명령이 안 나오는 것이 정상**이다. 제어 노드 없는 발행부 시험에서만 별도 터미널로 시험 모드를 주기적으로 보낸다:

```bash
ros2 topic pub -r 10 /mission/state std_msgs/msg/String '{data: "{\"state\":\"M1_TRAFFIC\"}"}'
```

실제 mission_node와 시험 모드 발행자를 동시에 실행하지 않는다. 서행 시험은 실제 지도 매칭을 함께 쓰거나 `route.fixes`가 증가하는 모의 메시지를 보내야 한다. 전체 `autorace.launch.py`에서 이미 발행부를 켰다면 따로 실행하지 않는다. 별도 발행부를 쓸 때는 전체 launch의 `use_class_commands:=false`로 중복 실행을 막는다.

## 검증

```bash
python3 -m unittest discover -s src/autorace_perception/test -p 'test_class_command*.py' -v
```

현재 단위·ROS API 모의 테스트 39개 통과. 모드 차단·전환, 7/10 투표, 크기 설정, 바닥 매칭의 갱신·만료, 시간 초기화 후 복구, 빨간불 유지 등을 확인했다. 실제 ROS/DDS·카메라·차량 시험은 수행하지 못했다. 확인된 요청을 기존 FSM에 연결했으며 주행 속도·주차 궤적 계산은 기존 구현을 사용한다. 리뷰 기록은 저장소 바깥 작업 폴더의 review-wiki에 보관한다.

## 반복 요청 식별 (추가)

각 명령에 `request_id`와 `new_request`가 들어간다. 같은 구간의 같은 주차 요청은 표지가 잠시 사라졌다 다시 보여도 같은 ID를 유지하며 `new_request: false`다. 녹색도 반복 검출 때 같은 ID를 유지한다. 새 빨간불 정지 후 녹색으로 바뀌면 새로운 해제 요청 ID를 만든다. 실제 모드 전환 후 다시 진입하면 새 요청이다. 필터 수치 변경이나 카메라 시간 초기화만으로 주차 요청을 새로 만들지는 않는다.

수신부는 `new_request`가 false인 유지 메시지부터 받아도 시작할 수 있다. 반복 ID, new_request=true의 재전송, 같은 구간의 다른 ID가 와도 이미 진행 중인 미션을 다시 진입시키지 않는다. 새 빨간불은 주행 상태에서 정지선 접근으로 전환하며, 이미 접근/정지 중이면 정지 제어와 대기 타이머를 다시 만들지 않는다. 새로운 구간 진입은 기존 미션 전환으로 처리한다.

## 반복 실행 방지 검증

명령 변환/발행 39개와 수신/FSM 연결 9개 테스트 통과. 실제 MissionNode 콜백에 주차 요청을 100번 넣어 시작 시간·이동 거리 기준이 유지되는지, 완료 후 반복 요청에도 주차가 재시작되지 않는지 확인했다. 녹색 수백 회 반복 및 미검출 뒤에도 출발 거리가 초기화되지 않아 다음 구간으로 넘어가는지, 적색 반복이 정지 접근을 다시 만들지 않는지 확인했다. 이 검증은 실제 코드와 ROS API 모의 객체를 사용하며 실차 시험은 아니다.

```bash
python3 -m unittest discover -s src/autorace_mission/test -p 'test_command*.py' -v
```
