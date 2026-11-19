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
        # M7 3차선 회피 (limo_obstacle_avoid) + M8 라바콘 디버그 그림
        /obstacle/cmd_vel /obstacle/debug/compressed /avoid/state /avoid/active /avoid/done /avoid/enable /fork_direction
        # M7 차로 (늘) / M7 끝 차로 -> M2
        /m7/lane /m7/exit_lane
        # M7 직전 코너 진단 (tools/curve_diag.py가 켜져 있을 때)
        /diag/curve
        # 11/12 밤: 노드 로그(회피 판단·미션 전환·[코너 보완] 등)와 표지 투표 결과도 같이 -> bag 하나로 원인 분석
        /rosout /perception/commands)
if [ "${RAW:-1}" = "1" ]; then
  topics+=(/camera/color/image_raw)
else
  topics+=(/camera/color/image_raw/compressed)
fi

echo "recording -> $dir"
ros2 bag record -o "$dir" "${topics[@]}"
