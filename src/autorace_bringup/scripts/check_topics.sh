#!/usr/bin/env bash
# 첫날 점검: LIMO 드라이버·센서 토픽 이름과 주기를 확인한다.
#   bash ~/autorace_ws/src/autorace_bringup/scripts/check_topics.sh
# 결과를 보고 config/params.yaml의 image_topic, odom_topic 등을 맞출 것.
set -u

echo "== ROS_DISTRO: ${ROS_DISTRO:-?} =="
echo "== topics =="
ros2 topic list

for t in /camera/color/image_raw /camera/color/image_raw/compressed /scan /odom /wheel/odom /imu /limo_status; do
  echo "== $t =="
  timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m 1 "average rate" || echo "  (데이터 없음)"
done

echo "== /limo_status (motion_mode: 0 사륜 차동, 1 애커만, 2 메카넘) =="
timeout 3 ros2 topic echo --once /limo_status 2>/dev/null || echo "  (/limo_status 없음)"

echo "== /scan 정면 확인: 차 정면 50cm에 물체를 두고 ranges 중 0.5 근처 값의 각도를 확인 =="
timeout 3 ros2 topic echo --once /scan --field angle_min 2>/dev/null || true
