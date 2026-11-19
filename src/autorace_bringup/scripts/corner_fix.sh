#!/usr/bin/env bash
# 11/19 corner_fix 켜기/끄기 (주행 중에도 바로 반영). 사용: corner_fix.sh lock|keep|m1|all|off|status
set -e
lane() { ros2 param set /lane_node "$1" "$2"; }
mis()  { ros2 param set /mission_node "$1" "$2"; }
case "$1" in
  lock)   lane corner_lock true ;;                       # 1순위: 코너 중 덜 꺾기 하한
  keep)   lane keep_kappa_on_heading_switch true ;;      # 2순위: 코너 중 모드 전환 때 조향 유지
  m1)     mis m1_lane_back_frames 5 ;;                   # 3순위: 4번 코너 원호->차선 복귀 5프레임
  all)    lane corner_lock true; lane keep_kappa_on_heading_switch true; mis m1_lane_back_frames 5 ;;
  off)    lane corner_lock false; lane keep_kappa_on_heading_switch false; mis m1_lane_back_frames 1 ;;
  status) for p in corner_lock keep_kappa_on_heading_switch corner_lock_corr_max; do
            echo -n "lane_node $p: "; ros2 param get /lane_node $p; done
          echo -n "mission_node m1_lane_back_frames: "; ros2 param get /mission_node m1_lane_back_frames ;;
  *) echo "사용: $0 lock|keep|m1|all|off|status"; exit 1 ;;
esac
