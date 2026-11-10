# 변경 내역: M2 갈림길(A/B) 출구 차선 이동

## 한 줄 요약
YOLO 갈림길 표지(fork_a/fork_b)로 A/B를 정하고, 지금 차선 기준으로 0/1/2차선을 옮겨 **가운데 섬 앞 끝 전에** 출구 차선에 들어가게 했다.
A = 1차선(+1, 왼쪽), B = 3차선(-1, 오른쪽).

## 발견한 문제 (기존 main 124e881)
| # | 문제 | 결과 |
| --- | --- | --- |
| ① | 회피 노드는 `/fork_direction`을 받아도 지금 차선이 막혀야만 움직임 | 표지를 일찍 알아도 섬 바로 앞에서야 판단 |
| ② | 2차선 이동(1차선→B, 3차선→A)은 가운데 차선 검사 구간(2.0m)이 섬과 겹쳐 항상 실패 → "출구선호 포기" | 1차선에서 B를 받으면 그대로 A로 감 |
| ③ | 섬을 지나면(M7 종료 = 섬 앞 끝 + 30cm) 반대 길로 옮길 수 없음 | 차선 이동은 M7 구간 안(섬 앞 끝 전)에서 끝나야 함 |

## 바뀐 것
| 파일 | 내용 |
| --- | --- |
| `limo_obstacle_avoid/fork_exit.py` (새) | 차선 n개 이동에 필요한 거리 계산 (회피 노드와 같은 제어식으로 모의) |
| `limo_obstacle_avoid/avoid_core.py` | `exit_plan()` + 매 주기 출구 차선 미리 이동 (0/1/2차선), 포기한 방향 재시도 방지, 상태값 추가 |
| `limo_obstacle_avoid/config/avoid_params.yaml` | `fork_shift`, `exit_margin`, `exit_min_time` |
| `limo_obstacle_avoid/tools/fork_sim.py` (새) | 표지 인식 시점별 시험 |
| `limo_obstacle_avoid/test/...` | 시험 3개 추가 (거리 계산, 판단 4가지, 2차선 이동 시뮬레이션) |
| `autorace_fork/` (새 패키지) | `fork_node`: YOLO 투표 → 이동 차선 수 판단 → `/fork_direction`, `/fork/state` |
| `autorace_bringup/launch/autorace.launch.py` | `use_fork_node` 인자 (기본 false) |
| `autorace_bringup/package.xml` | `autorace_fork` 의존 |

## 필요한 거리 (회전반경 0.53m, 차선 간격 0.52m)
1차선 이동: 차체가 섬 구역을 벗어나기까지 0.74m / 2차선: 1.32m. 여유 0.2m + 앞범퍼 0.2m를 더해
**2차선은 섬 앞 끝 약 1.7m 전, 1차선은 약 1.1m 전**까지 표지를 알아야 한다.

## 확인 (ROS 없는 환경)
- 레포 전체 `python3 -m pytest src -q`: 120개 통과 (기존 113 + 회피 3 + 갈림길 4)
- `fork_sim.py --gap 2.5`: 표지를 섬 2.0m 이상 전에 알면 16/16 (예전 11/16, 2차선 이동 5건 0 → 5). 충돌 0
- 기존 `offline_sim --all` 20/20, `stress_sim` 복합 오차 20/20 (출구 방향이 없을 땐 동작 변화 없음)
- 가짜 ROS 통합: 1차선 B → 2차선 이동, 1차선 A → 유지, 가운데 늦은 B → 1차선 이동, FSM 결정 따르기 통과

**아직 안 한 것**: LIMO 실차 주행. 실제 표지가 섬 앞 몇 m에서 인식되는지 확인 필요.

## 점검 (11/10)

최신 main(PR #21~#30: YOLO 명령 경로, 미션 트리거, M7 차로 판별 포함) 기준으로 M2 흐름을 다시 확인했다.

- 표지 경로: detector → `class_command_node`(`/perception/commands`, M7·M2 구간에서만 `SELECT_ROUTE A/B`) → `CommandInbox` → FSM 투표(8 중 5). `fork_node`는 원시 `/perception/detections`를 따로 투표하고 FSM 결정이 있으면 그걸 따름 → 엇갈리지 않음.
- M7 → M2: 회피가 끝난 차로는 `/obstacle/state`의 `m7_exit`(M7이 끝난 뒤에도 유지)와 `/m7/exit_lane`(latched)으로 나간다. 표지를 못 읽으면 FSM이 `m7_exit`를 그대로 씀.
- **고친 것: `tools/course_sim.py`가 `m7_exit`를 M7 동안에만 넘겨** 실제와 달랐음. 표지를 놓친 시나리오에서 M2가 12초 기다린 뒤
  기본값 A(회피는 B 차로에서 끝났는데)를 고르는 것처럼 보였다. 실제 obstacle_node처럼 M2에서도 넘기게 고친 뒤 → 바로 B.
- **추가: 표지와 회피 차로가 엇갈릴 때**(표지 A인데 섬 전에 못 옮겨 B 차로에서 끝남) FSM이 로그를 남긴다 (`M2 fork conflict: …`).
  기본은 **표지를 따름** (`fork_follow_m7_exit: false`): 규정상 반대 갈림길 진입은 M2 실패(10점)이고, 섬은 시설물이 아닌 바닥 선이라
  가로지르면 차로 이탈 감점(1회 1점). `true`로 하면 차로를 따름.
- 코스 시뮬레이션 시나리오 `fork_lane_conflict` 추가. 7개 시나리오 모두 완주.
- 확인: mission·fork·obstacle·avoid 시험 195 passed, course_sim 7/7.
