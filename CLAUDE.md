# CLAUDE.md — Autorace 2026 KUUVe 팀 공통 지침

Claude Code가 이 저장소를 열 때마다 읽는다. 팀 전체 공통 규칙만 둔다.
개인 설정(호칭, 내 담당 미션)은 각자 저장소 루트의 `CLAUDE.local.md`에 쓴다 (git에 안 올라감).

## 환경
- 차: LIMO PRO, 아커만 모드 (`limo_base_ackerman.launch.py`), Ubuntu 22.04, ROS 2 Humble, rclpy
- 리모 작업 폴더: `~/autorace_ws`. 팀 저장소 main = 대회 코드
- 실행 방법: `docs/RUN_GUIDE.md`를 먼저 읽는다
- 패키지: `autorace_bringup`(launch·params.yaml) / `autorace_lane`(차선) / `autorace_mission`(FSM·주차) /
  `autorace_obstacle`(M7 연결·라바콘) / `limo_obstacle_avoid`(M7 회피·M2 녹화 재생) / `autorace_fork` / `autorace_perception`
- 미션 순서: M5 방지턱 → M6 지그재그 → M3 보행자 → M7 장애물(3차선) → M2 갈림길 → M8 라바콘 → M1 신호등 → M4 주차

## 일하는 순서 (매번)
1. `git fetch` → `git log --oneline -15 origin/main`으로 팀원 변경 확인. 지금 작업과 겹치면 먼저 보고
2. main에서 새 브랜치 `claude/<짧은-이름>`
3. 값은 파라미터로: 코드 기본값 + `src/autorace_bringup/config/params.yaml` 둘 다. 주석에 날짜·이유·근거(실차/시뮬/도면/계산)
4. 테스트 전부 통과: 고친 패키지 폴더에서 `python3 -m pytest -q test/`. 새 기능엔 시험 추가
5. `docs/CHANGES_YYYYMMDD_*.md`에 무엇을·왜 바꿨는지 기록
6. 커밋 → push → PR (제목·본문 한국어, 흐름도·diff·시험 결과)
7. **머지는 담당자가 "머지"라고 할 때만.** 머지 전 main 다시 받아 테스트
8. PR 전에 main을 다시 받아 충돌 확인. `params.yaml`은 여러 명이 고치므로 자기 노드 부분만 고친다

## 절대 규칙
- 다른 사람 담당 미션 코드는 고치기 전에 먼저 묻는다. 충돌은 임의로 풀지 않는다
- **설정값도 차에서 직접 고치지 않는다.** GitHub에서 바꾸고 리모가 `git pull`. 실행 중 시험은 `ros2 param set`만
- 설치(sudo apt 등)·삭제·원본 수정·리모/ROS 실행·외부 공유·머지는 결재 먼저
- 모르면 "몰라". 값을 추측해서 단정하지 않는다. **시뮬레이션 결과와 실차 결과를 구분**해서 말한다
- 장애물·트랙 조건을 바꾸라고 하지 않는다. 코드가 맞춘다
- `pkill -f ros2` 금지 (ros2 daemon까지 죽음)
- launch는 tmux 안에서 켜거나 리모에서 직접 (대회장 와이파이 ping 1.5~2.7초, SSH 끊기면 차 멈춤)
- `ros2 bag record -a` 금지 (카메라 드라이버 죽은 적 있음). 필요한 토픽만

## 실차 시험 전 확인
```bash
cd ~/autorace_ws
git branch --show-current     # 팀원 시험 브랜치에 있어서 최신 코드가 안 돈 적 있음
git log --oneline -1
```

## 문제 볼 때
- 먼저 데이터: `/mission/state`, `/lane/status`, `/parking/state`, `/avoid/state`, 리모 `git log -1`
- 녹화 예: `ros2 bag record -o <이름> /mission/state /lane/status /odom /scan /imu` (미션에 맞게 토픽 추가)
- 알려진 함정
  - 코너 lookahead 판단(`corner_ld_on` 0.6)은 직선 끝·흔들림에서도 켜짐 → 코너 값 올리면 직선도 바뀜
  - 축거 0.222(약 1.11배) × 코너 배율이 곱해짐 (`params.yaml` 주석)
  - 바닥 현수막이 울면 카메라 BEV(평평한 바닥 가정)가 틀어져 차선·가로선 인식 흔들림

## 답변 형식
- 한국어, 존댓말. 결론(추천 1개)부터 짧게, 중고등학생 수준
- 채팅엔 바뀐 부분(diff)만. 코드엔 글자 흐름도(①②③ 세로 가지)
- 할 일은 표: 할 일 / 스텝 / 마감 / 근거 / 상태
- 왜 이렇게 하는지(원리·근거) 한두 줄. 문제는 돌려 말하지 않고 바로 지적
- 답할 때 리모 터미널 명령어까지 (터미널 A/B/C, 순서: 받기 → 빌드 → 실행 → 확인)

## 규정·도면
- 도면: `docs/2026Track_dimensions.pdf`, 경로 지도: `src/autorace_bringup/config/track_route.yaml`
- 규정 판정 기준점은 차량 **뒷바퀴**. 미션 진입·발진 15초 제한
