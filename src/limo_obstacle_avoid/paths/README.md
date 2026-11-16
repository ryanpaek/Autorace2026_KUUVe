# M7 녹화 경로

`m7_left.csv`(가운데 → 왼쪽 차로), `m7_right.csv`(가운데 → 오른쪽 차로)를 여기에 넣고 빌드한다.
녹화: `tools/path_recorder.py` (tools/README.md). 시작 자세 = 차로 가운데·반듯이, 앞 장애물까지 라이다 거리 = `trigger_mm`(546 mm).

```bash
cp ~/autorace_tools/m7_left.csv ~/autorace_tools/m7_right.csv ~/autorace_ws/src/limo_obstacle_avoid/paths/
cd ~/autorace_ws && colcon build --symlink-install --packages-select limo_obstacle_avoid
```
