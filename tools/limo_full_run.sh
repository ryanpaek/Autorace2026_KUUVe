#!/usr/bin/env bash
# LIMO에서 최신 main 받기 -> 빌드 -> 남은 노드 정리 -> 전체 launch -> 녹화 -> 출발 -> 정지·저장 (11/20)
#
#   bash ~/autorace_ws/tools/limo_full_run.sh                 # M7 = dash (main 기본)
#   bash ~/autorace_ws/tools/limo_full_run.sh waypoint        # M7 = 웨이포인트
#   bash ~/autorace_ws/tools/limo_full_run.sh dash 1120_03    # bag 이름 직접
#   NO_PULL=1 bash ...   # git pull·빌드 건너뜀 (방금 받았을 때)
#
# 출발: 차를 출발 체커에 놓고 Enter.  정지: 주행 중 아무 때나 Enter (비상 정지 -> 녹화 저장 -> 전부 종료).
# Ctrl+C로 끝내도 같은 정리를 함.
set -u
IMPL="${1:-dash}"
NAME="${2:-$(date +%m%d_%H%M)_full_${IMPL}}"
WS="${WS:-$HOME/autorace_ws}"
BAGDIR="$HOME/bags"; LOGDIR="$HOME/runlogs"
mkdir -p "$BAGDIR" "$LOGDIR"
LOG="$LOGDIR/$NAME.launch.log"

case "$IMPL" in dash|waypoint|replay) ;; *) echo "M7 방식은 dash / waypoint / replay 중 하나"; exit 1;; esac
say() { echo -e "\n\033[1;36m[$1]\033[0m $2"; }
die() { echo -e "\033[1;31m중단: $1\033[0m"; exit 1; }

set +u; source /opt/ros/humble/setup.bash; set -u   # ROS setup은 set -u에서 깨짐 (AMENT_TRACE_SETUP_FILES)
cd "$WS" || die "$WS 없음"

# ------------------------------------------------------------ 1. 최신 main
if [ -z "${NO_PULL:-}" ]; then
  say 1/6 "최신 main 받기"
  if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
    git status --short --untracked-files=no
    die "리모에서 고친 파일이 있음 -> 'git stash' 로 치워 두고 다시 (지울 거면 'git checkout -- .')"
  fi
  git fetch origin main || die "git fetch 실패 (인터넷 확인)"
  git checkout -q main && git merge --ff-only -q origin/main || die "main을 최신으로 못 맞춤 (리모에 따로 만든 커밋?) -> 'git log --oneline -3' 확인"
  git log -1 --format='  지금 코드: %h %s'

  say 2/6 "빌드 (--symlink-install)"
  colcon build --symlink-install 2>&1 | tail -4
  [ "${PIPESTATUS[0]}" -eq 0 ] || die "빌드 실패 (위 오류 확인)"
else
  say 1/6 "pull·빌드 건너뜀 (NO_PULL)"; git log -1 --format='  지금 코드: %h %s'
fi
set +u; source install/setup.bash; set -u
case "$(ros2 pkg prefix autorace_bringup)" in "$WS"/install/*) ;; *) die "autorace_bringup이 $WS 것이 아님 (catkin_ws 등 옛 코드 source됨)";; esac

# ------------------------------------------------------------ 3. 정리
say 3/6 "남은 노드 정리"
# 팀 노드 전부(launch 파일의 executable) + 차체·라이다·카메라 + 예전 launch
PAT="ros2 launch|ackermann|limo_base|ydlidar|orbbec|component_container|rosbag|avoid_node|chime_node|class_command|debug_view|detector_node|exposure_node|fork_node|lane_node|m7_dash_node|m7_replay_node|m7_waypoint_node|mission_node|obstacle_node|parking_node|sys_monitor|yellow_side|$WS/install"
ps -eo pid,cmd | grep -E "$PAT" | grep -v -E "grep|limo_full_run" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
sleep 1
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1; sleep 3
n=$(ros2 node list 2>/dev/null | wc -l)
if [ "$n" -ne 0 ]; then
  ros2 node list
  echo "  위 노드가 아직 보임 -> 프로세스 찾기:"; ps -eo pid,cmd | grep -E "$(ros2 node list | sed 's#^/##' | paste -sd'|')" | grep -v grep | head
  die "노드가 남아 있음. 다른 터미널(다른 사람 노트북의 ros2 포함) 끄기. 같은 와이파이의 다른 팀이면 'export ROS_DOMAIN_ID=<우리 번호>'"
fi

# ------------------------------------------------------------ 4. launch
say 4/6 "전체 launch (m7_impl:=$IMPL)  로그: $LOG"
ros2 launch autorace_bringup full.launch.py m7_impl:="$IMPL" > "$LOG" 2>&1 &
LPID=$!
cleanup() {
  trap - INT TERM EXIT
  say 끝 "비상 정지 -> 녹화 저장 -> 종료"
  timeout 5 ros2 topic pub --once /mission/estop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
  [ -n "${BPID:-}" ] && kill -INT "$BPID" 2>/dev/null && wait "$BPID" 2>/dev/null
  kill -INT "$LPID" 2>/dev/null; sleep 3; kill -9 "$LPID" 2>/dev/null
  ps -eo pid,cmd | grep -E "$PAT" | grep -v -E "grep|limo_full_run" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
  echo "  bag: $BAGDIR/$NAME   launch 로그: $LOG"
  echo "  노트북으로: scp -r wego@$(hostname -I | awk '{print $1}'):~/bags/$NAME C:\\Users\\shpae\\Downloads\\"
  exit 0
}
trap cleanup INT TERM

echo -n "  mission_node 기다리는 중"
for i in $(seq 1 60); do
  timeout 3 ros2 topic echo /mission/state --once >/dev/null 2>&1 && break
  kill -0 $LPID 2>/dev/null || { tail -30 "$LOG"; die "launch가 죽음"; }
  echo -n "."; sleep 1
done
echo
timeout 3 ros2 topic echo /mission/state --once >/dev/null 2>&1 || { tail -30 "$LOG"; cleanup; }
dup=$(ros2 node list | sort | uniq -d)
[ -z "$dup" ] || { echo "$dup"; echo -e "\033[1;31m같은 노드가 두 벌!\033[0m"; cleanup; }
grep -q "M7 웨이포인트\|m7_waypoint" "$LOG" && echo "  M7 노드: 웨이포인트" || true
grep -q -i "m7_dash\|점선" "$LOG" && echo "  M7 노드: dash" || true
for tp in /camera/color/image_raw /scan /odom; do
  hz=$(timeout 4 ros2 topic hz "$tp" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
  echo "  $tp ${hz:-없음} Hz"
  [ -n "$hz" ] || { echo -e "\033[1;31m$tp 안 들어옴\033[0m (카메라 USB/라이다 확인)"; cleanup; }
done

# ------------------------------------------------------------ 5. 녹화
say 5/6 "녹화 시작: $BAGDIR/$NAME"
ros2 bag record -o "$BAGDIR/$NAME" /camera/color/image_raw/compressed /scan /odom /imu /cmd_vel /lane/cmd_vel \
  /lane/status /mission/state /perception/detections /perception/commands /obstacle/state /avoid/state \
  /fork/state /parking/state /diag/stop_reason /fork_direction /m7/exit_lane /rosout > "$LOGDIR/$NAME.bag.log" 2>&1 &
BPID=$!
sleep 2

# ------------------------------------------------------------ 6. 출발
say 6/6 "차를 출발 체커에 놓고 Enter = 출발"
read -r _
ros2 topic pub --once /mission/start std_msgs/msg/Empty >/dev/null && echo "  출발! (정지: Enter)"
( while sleep 2; do
    s=$(timeout 2 ros2 topic echo /mission/state --once --field data 2>/dev/null | head -1 | python3 -c "
import sys, json
try:
    d = json.loads(sys.stdin.read()); print(d.get('state'), d.get('sub') or '', 'dist', d.get('dist_mm'), (d.get('note') or '')[:60])
except Exception:
    pass" 2>/dev/null)
    [ -n "$s" ] && echo "  $s"
  done ) &
WPID=$!
read -r _
kill "$WPID" 2>/dev/null
cleanup
