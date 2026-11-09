#!/usr/bin/env bash
# 주행 기록: bash record_bag.sh run1
# 원본 영상은 초당 수십 MB라, 저장 공간을 보고 RAW=0으로 압축 토픽만 남길 수 있다.
#   RAW=0 bash record_bag.sh run1
set -u
name=${1:-run}
dir=~/bags/$(date +%m%d_%H%M%S)_${name}
mkdir -p ~/bags

topics=(/scan /odom /wheel/odom /imu /limo_status /cmd_vel /limo/ack_cmd /lane/cmd_vel /lane/status /lane/config
        /lane/debug/compressed /mission/state /route/preview /perception/detections /obstacle/state
        # M7 3차선 회피 (limo_obstacle_avoid): 언제 켜졌는지, 회피 중 옆 위치(y)·차로, 회피 명령
        /obstacle/cmd_vel /avoid/state /avoid/active /avoid/done /avoid/enable /fork_direction)
if [ "${RAW:-1}" = "1" ]; then
  topics+=(/camera/color/image_raw)
else
  topics+=(/camera/color/image_raw/compressed)
fi

echo "recording -> $dir"
ros2 bag record -o "$dir" "${topics[@]}"
