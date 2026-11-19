# 11/19 카메라 노출 고정 (노란 실선이 흰색으로 날아가는 문제)

**한 줄 요약:** 출발 전에 노출을 한 번 정해서 고정한다. 단계는 LIMO에 한 번만 저장해 두면 계속 유지된다. 평소 실행 명령은 그대로다.

- 문제: 1번 코너 앞 직선에서 카메라 자동 노출이 노출을 올림 → 노란 실선이 흰색으로 포화 → 차선 NONE → 차가 멈춤
- 해결: `exposure_node`가 카메라가 켜지고 약 4초 뒤 노출·게인·화이트밸런스를 고정함. 그 뒤로는 영상을 안 봐서 주행 중 CPU를 쓰지 않음
- 단계: 0(끔, 예전과 같음) ~ 4(가장 어두움). 단계가 클수록 노란선이 안전해지는 대신 신호등·표지판이 조금 어두워짐

---

## 1. 처음 한 번: 코드 받고 빌드 (LIMO에 ssh 접속 후 통째로 붙여넣기)

```bash
source /opt/ros/humble/setup.bash
PKG=$(find ~ -maxdepth 5 -type d -path "*/src/autorace_bringup" -not -path "*/install/*" -not -path "*/build/*" 2>/dev/null | head -1)
REPO=$(git -C "$PKG" rev-parse --show-toplevel)
WS="$PKG"; while [ "$WS" != "/" ] && [ ! -d "$WS/install" ]; do WS=$(dirname "$WS"); done
echo "레포: $REPO / 워크스페이스: $WS"
cd "$REPO" && if [ -n "$(git status --porcelain --untracked-files=no)" ]; then echo "!! LIMO에 저장 안 된 수정이 있어서 멈춤 (아래 목록 확인)"; git status --short --untracked-files=no; else git fetch origin claude/camera-exposure-1119 && git checkout claude/camera-exposure-1119 && git reset --hard origin/claude/camera-exposure-1119 && git log -1 --oneline && cd "$WS" && colcon build --packages-select autorace_bringup && source install/setup.bash && ros2 pkg executables autorace_bringup; fi
```

- 성공하면 마지막 줄에 `autorace_bringup cam_level`과 `autorace_bringup exposure_node`가 보임
- `저장 안 된 수정이 있어서 멈춤`이 나오면: LIMO에서 누가 고친 파일이 있다는 뜻. 아무것도 안 바뀌었으니 그 목록을 담당자에게 전달

---

## 2. 단계 저장 (연습 때 정해서 한 번만)

```bash
source /opt/ros/humble/setup.bash; source ~/autorace_ws/install/setup.bash
ros2 run autorace_bringup cam_level 2
```

- 맨 끝 숫자만 바꾸면 됨 (0~4). 저장 위치는 `~/.autorace/cam_exp_level`
- git pull, 빌드, 재부팅을 해도 유지됨
- 다음에 카메라(launch)를 켤 때부터 적용됨. 이미 켜져 있으면 launch를 껐다 켜야 함

지금 저장된 단계 확인 (숫자 없이 실행하면 단계표도 같이 나옴):

```bash
source /opt/ros/humble/setup.bash; source ~/autorace_ws/install/setup.bash
ros2 run autorace_bringup cam_level
```

---

## 3. 평소 실행: 바뀐 것 없음

```bash
ros2 launch autorace_bringup full.launch.py
```

- 저장된 단계가 자동으로 적용됨. 출발할 때 따로 입력할 것 없음
- 카메라가 켜지고 약 4초 안에 고정됨 (출발 대기 중에 끝남)
- 연습 중 **이번 실행만** 다른 단계로 시험하려면 끝에 `cam_exp_level:=3`처럼 붙임. 저장된 값은 그대로 유지됨

```bash
ros2 launch autorace_bringup full.launch.py cam_exp_level:=3
```

---

## 4. 잘 적용됐는지 확인

launch 창 로그에 아래 두 줄이 나오면 정상:

```
exposure_node: 단계 2 (/home/wego/.autorace/cam_exp_level)
exposure_node: 노출 고정: 자동 312 -> 175 (출발선 바닥 V 130, 목표 100), 게인 ..., WB ...
```

(숫자는 예시)

다른 창에서 확인하는 방법:

```bash
source /opt/ros/humble/setup.bash; source ~/autorace_ws/install/setup.bash
ros2 topic echo --once /camera/exposure_state
```

`"state": "fixed"`면 정상.

| 로그 / state | 뜻 | 할 일 |
|---|---|---|
| `fixed` | 고정 완료 | 없음 |
| `off` | 0단계 (자동 노출 그대로) | 의도한 거면 없음 |
| `failed` / `실패` | 카메라 서비스 호출 실패 → 자동 노출로 돌아감 (예전과 같은 상태) | 카메라가 두 번 켜져 있는지 확인 (아래 6번) |
| 아무 로그 없음 | `exposure_node`가 안 뜸 | 1번 빌드를 다시 하고, `source ~/autorace_ws/install/setup.bash` 했는지 확인 |

---

## 5. 단계 고르는 기준

| 단계 | 목표 바닥 밝기 V | 노란선 인식 (픽셀 절반 이상 / 거의 전부) | 신호등·표지판 프레임 밝기 |
|---|---|---|---|
| 0 | 자동 노출 그대로 | 예전과 같음 (1번 코너에서 날아감) | x1.0 |
| 1 | 115 | 83% / 63% | x0.86 |
| **2 (기본)** | 100 | 96% / 82% | x0.75 |
| 3 | 90 | 99% / 95% | x0.68 |
| 4 | 80 | 100% / 98% | x0.60 |

증상별로 이렇게 조정:

- **1번 코너 앞에서 차선을 놓침 (NONE, 정지)** → 한 단계 올림 (2 → 3)
- **신호등·표지판 인식이 늦거나 놓침** → 한 단계 내림 (2 → 1)
- **둘 다 문제거나 급할 때** → 0 (예전 상태로 복귀)
- 단계를 바꾼 뒤에는 `cam_level N`으로 저장하고 launch를 재시작

(표의 숫자는 11/19 bag 05·06 프레임으로 추정한 값. 감마 2.2와 홀 조명이 고르다는 가정이라 실차에서 확인 필요)

---

## 6. 문제 생겼을 때

카메라가 두 번 켜져 있으면 노출 고정이 실패할 수 있음. 아래로 확인:

```bash
ps aux | grep component_container | grep -v grep
```

2줄 이상 나오면 전부 끄고 launch를 다시 켬:

```bash
pkill -f component_container; sleep 2
ros2 launch autorace_bringup full.launch.py
```

완전히 예전처럼 되돌리기 (노출 고정 끔):

```bash
source /opt/ros/humble/setup.bash; source ~/autorace_ws/install/setup.bash
ros2 run autorace_bringup cam_level 0
```

---

## 7. 녹화할 때 (중요)

지금 쓰는 `ros2 bag record` 토픽 목록에 **`/camera/exposure_state`를 추가**해야 함. 그래야 그 판에 노출이 몇으로 고정됐는지 bag에 남음. 11/19 bag에는 노출 값이 하나도 없어서 원인 분석이 어려웠음.

예시 (기존 명령 끝에 한 단어만 추가):

```bash
ros2 bag record -o ~/bags/run ... 기존 토픽들 ... /camera/exposure_state
```

`scripts/record_bag.sh`에는 이미 추가돼 있음.

---

## 참고: 바뀐 파일 (`autorace_bringup`)

| 파일 | 내용 |
|---|---|
| `exposure_node.py` | 출발 전 노출 고정 노드 |
| `exposure_core.py` | 계산부, 단계표 |
| `cam_level.py` | 단계 저장·확인 명령 |
| `launch/sensors.launch.py`, `launch/full.launch.py` | `exposure_node` 자동 실행, 인자 `cam_exp_level` |
| `config/params.yaml` | `exposure_node` 설정 (단계 기본 2) |
| `scripts/record_bag.sh` | `/camera/exposure_state` 추가 |
| `test/test_exposure_core.py` | 테스트 5개 |
