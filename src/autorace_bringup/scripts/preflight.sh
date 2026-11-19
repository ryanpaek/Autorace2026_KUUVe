#!/usr/bin/env bash
# 출발 전 점검 (11/19 본선: 따로 띄운 카메라 런치가 카메라를 잡아 경기 런치에 영상이 0장이었음)
#
#   띄우기 전 정리:  bash ~/autorace_ws/src/autorace_bringup/scripts/preflight.sh clean
#   띄운 뒤 점검:    bash ~/autorace_ws/src/autorace_bringup/scripts/preflight.sh
#
# 점검 결과 마지막 줄이 "출발 가능"이어야 출발. "실패"가 하나라도 있으면 그 줄부터 고친다.
set -u
source ~/autorace_ws/install/setup.bash 2>/dev/null

R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; N=$'\e[0m'
fail=0; warn=0
ok()   { echo "  ${G}[OK]${N}   $*"; }
bad()  { echo "  ${R}[실패]${N} $*"; fail=$((fail+1)); }
note() { echo "  ${Y}[주의]${N} $*"; warn=$((warn+1)); }

# ---------------------------------------------------------------- clean
if [ "${1:-}" = "clean" ]; then
  echo "== 남은 ROS 프로세스 모두 끄기 =="
  if pgrep -f "ros2 bag record" >/dev/null; then
    echo "  ${Y}bag 녹화가 돌고 있음. 녹화 터미널에서 먼저 Ctrl+C 하고 다시 실행하세요.${N}"; exit 1
  fi
  pkill -INT -f "ros2 launch" 2>/dev/null; sleep 3
  for p in "ros2 launch" component_container orbbec limo_base ydlidar autorace_ws/install wego_ws/install; do
    pkill -9 -f "$p" 2>/dev/null
  done
  sleep 1
  ros2 daemon stop >/dev/null 2>&1
  left=$(ps aux | grep -E "component_container|orbbec|ros2 launch|autorace_ws/install|wego_ws/install|ydlidar|limo_base" | grep -v grep)
  if [ -n "$left" ]; then
    echo "$left"; bad "아직 남은 프로세스가 있음 (위 목록)"; exit 1
  fi
  ok "다 꺼짐. 이제 띄우세요:  ros2 launch autorace_bringup full.launch.py"
  echo "  (SSH가 끊겨도 살아 있게 tmux 안에서 띄우는 걸 권장: tmux new -s run)"
  exit 0
fi

# ---------------------------------------------------------------- check
echo "== 1. 네트워크·도메인 =="
[ "${ROS_DOMAIN_ID:-}" = "99" ] && ok "ROS_DOMAIN_ID=99" || bad "ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-}' (99여야 함. source ~/.bashrc 후 다시 띄우기)"
case "${ROS_LOCALHOST_ONLY:-}" in ""|0) ok "ROS_LOCALHOST_ONLY 꺼짐";; *) bad "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY} (대회 공지: 주석 처리)";; esac
ssid=$(iwgetid -r 2>/dev/null || nmcli -t -f active,ssid dev wifi 2>/dev/null | grep '^yes' | cut -d: -f2)
ip=$(hostname -I 2>/dev/null)
if [ "$ssid" = "wego_edu_5G" ] && echo "$ip" | grep -q "10\.42\.0\."; then ok "와이파이 $ssid / $ip"
else note "와이파이 '$ssid' / IP $ip (대회장이면 wego_edu_5G, 10.42.0.xxx)"; fi

echo "== 2. 중복 실행 =="
nl=$(pgrep -fc "ros2 launch")
[ "$nl" = "1" ] && ok "ros2 launch 1개" || bad "ros2 launch ${nl}개 (1개여야 함. preflight.sh clean 후 다시)"
nc=$(pgrep -fc "component_container")
[ "$nc" = "1" ] && ok "카메라 컨테이너 1개" || bad "카메라 컨테이너 ${nc}개 (따로 띄운 카메라 런치가 있으면 끄기)"
dup=$(ros2 node list 2>/dev/null | sort | uniq -d)
[ -z "$dup" ] && ok "같은 이름 노드 없음" || bad "같은 이름 노드: $(echo $dup)"

echo "== 3. 센서 주기 (각 5초) =="
hz() { timeout 6 ros2 topic hz "$1" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}'; }
check_hz() {  # topic min
  local r; r=$(hz "$1")
  if [ -z "$r" ]; then bad "$1 데이터 없음"
  elif awk "BEGIN{exit !($r >= $2)}"; then ok "$1 ${r} Hz"
  else bad "$1 ${r} Hz (${2} Hz 이상이어야 함)"; fi
}
check_hz /camera/color/image_raw 20
check_hz /scan 7
check_hz /odom 20
mode=$(timeout 3 ros2 topic echo --once /limo_status 2>/dev/null | awk '/motion_mode/{print $2}')
[ "$mode" = "1" ] && ok "LIMO 애커만 모드" || bad "LIMO motion_mode='${mode}' (1=애커만이어야 함. 차체 모드 스위치 확인)"

echo "== 4. 주행 값 =="
pv() { ros2 param get "$1" "$2" 2>/dev/null | sed 's/.*is: //'; }
expect() {  # node param want
  local v; v=$(pv "$1" "$2")
  [ "$v" = "$3" ] && ok "$1 $2 = $v" || bad "$1 $2 = '${v:-없음}' (기대 $3)"
}
expect /ackermann_bridge steer_gain 1.2
expect /lane_node white_bridge_mm 100.0
expect /mission_node m1_red_camera_max_mm 2200.0
expect /mission_node start_state WAIT_START
expect /parking_node enabled True
echo "  코드: $(cd ~/autorace_ws && git log --oneline -1 2>/dev/null)"

echo "== 5. 제어 경로 =="
pubs=$(ros2 topic info /cmd_vel 2>/dev/null | awk '/Publisher count/{print $3}')
if [ -z "$pubs" ]; then bad "/cmd_vel 없음"
elif [ "$pubs" -le 3 ]; then ok "/cmd_vel 보내는 노드 ${pubs}개"
else bad "/cmd_vel 보내는 노드 ${pubs}개 (다른 팀 노드가 섞였을 수 있음. ros2 topic info -v /cmd_vel)"; fi
st=$(timeout 3 ros2 topic echo --once /mission/state 2>/dev/null | head -3 | tr -d '\n' | cut -c1-120)
[ -n "$st" ] && ok "미션 상태 수신: $st" || bad "/mission/state 수신 안 됨"
pgrep -f "ros2 bag record" >/dev/null && ok "bag 녹화 중" || note "bag 녹화 안 함 (bash ~/autorace_ws/src/autorace_bringup/scripts/record_bag.sh race)"

echo
if [ $fail -eq 0 ]; then echo "${G}== 출발 가능 (주의 ${warn}건) ==${N}"
else echo "${R}== 실패 ${fail}건: 출발하지 마세요. 위 [실패] 줄부터 고치기 ==${N}"; exit 1; fi
