"""리모 연산량 측정 (11/16): CPU·GPU·메모리·온도·전력 + 노드별 CPU, 차선·YOLO 처리 속도.

/system/usage (std_msgs/String, JSON) 를 rate_hz(기본 2)로 낸다. 녹화 목록에 넣으면 bag에 남고,
디버깅 화면(debug_view)에 '연산량' 패널로 보인다.

- 노드별 CPU: psutil로 프로세스마다 재서 명령줄 이름으로 묶음 (GROUPS). 100% = 코어 하나
- GPU·온도·전력: Jetson tegrastats (없으면 빈 값)
- 처리 속도: /lane/status 개수(차선 Hz), /perception/detections 개수와 infer_ms(YOLO 1장 시간)
"""
import json
import re
import subprocess
import threading
import time
from typing import Dict, Optional

# (묶음 이름, 명령줄에 들어 있는 글자들) - 위에서부터 먼저 맞는 것
GROUPS = [
    ('YOLO', ('detector_node',)),
    ('YOLO 투표', ('class_command',)),
    ('차선', ('lane_node',)),
    ('미션', ('mission_node',)),
    ('회피(M7)', ('avoid_node', 's5_avoid')),
    ('장애물·라바콘', ('obstacle_node',)),
    ('주차', ('parking_node',)),
    ('갈림길', ('fork_node',)),
    ('디버깅 화면', ('debug_view',)),
    ('조향 변환', ('ackermann_bridge',)),
    ('차체', ('limo_base',)),
    ('라이다', ('ydlidar',)),
    ('카메라', ('orbbec', 'component_container', 'astra', 'dabai')),
    ('녹화', ('ros2 bag', 'rosbag2', 'bag record')),
    ('알림음', ('chime_node',)),
    ('측정', ('sys_monitor',)),
]


def group_of(cmdline: str) -> Optional[str]:
    for name, keys in GROUPS:
        if any(k in cmdline for k in keys):
            return name
    return None


_RE_RAM = re.compile(r'RAM (\d+)/(\d+)MB')
_RE_CPU = re.compile(r'CPU \[([^\]]*)\]')
_RE_GPU = re.compile(r'GR3D_FREQ (\d+)%(?:@\[?(\d+))?')
_RE_TEMP = re.compile(r'(\w+)@(-?[\d.]+)C')
_RE_PWR = re.compile(r'(VDD_IN|POM_5V_IN|VIN_SYS_5V0)\s+(\d+)mW')


def parse_tegrastats(line: str) -> Dict:
    """tegrastats 한 줄 -> {ram_used_mb, ram_total_mb, cpu_cores, gpu_pct, gpu_mhz, temp:{}, power_mw}"""
    out: Dict = {}
    m = _RE_RAM.search(line)
    if m:
        out['ram_used_mb'], out['ram_total_mb'] = int(m.group(1)), int(m.group(2))
    m = _RE_CPU.search(line)
    if m:
        cores = []
        for c in m.group(1).split(','):
            mm = re.match(r'\s*(\d+)%', c)
            cores.append(int(mm.group(1)) if mm else None)      # off = 꺼진 코어
        out['cpu_cores'] = cores
    m = _RE_GPU.search(line)
    if m:
        out['gpu_pct'] = int(m.group(1))
        if m.group(2):
            out['gpu_mhz'] = int(m.group(2))
    temps = {k.lower(): float(v) for k, v in _RE_TEMP.findall(line)}
    if temps:
        out['temp'] = temps
    m = _RE_PWR.search(line)
    if m:
        out['power_mw'] = int(m.group(2))
    return out


class Tegra:
    """tegrastats를 따로 띄워 마지막 줄만 들고 있음 (없으면 조용히 빈 값)."""

    def __init__(self, interval_ms: int = 500):
        self.last: Dict = {}
        self.ok = False
        try:
            self.proc = subprocess.Popen(['tegrastats', '--interval', str(interval_ms)],
                                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            self.ok = True
            threading.Thread(target=self._read, daemon=True).start()
        except (OSError, ValueError):
            self.proc = None

    def _read(self):
        for line in self.proc.stdout:
            try:
                self.last = parse_tegrastats(line)
            except Exception:       # noqa: BLE001 - 형식이 달라도 측정 노드는 죽지 않게
                pass

    def stop(self):
        if self.proc is not None:
            self.proc.terminate()


class ProcTable:
    """psutil 프로세스별 CPU %를 묶음으로 합침 (첫 호출은 0, 다음부터 사이 구간 평균)."""

    def __init__(self):
        import psutil
        self.psutil = psutil
        self.procs = {}

    def sample(self) -> Dict[str, Dict]:
        ps = self.psutil
        seen = set()
        groups: Dict[str, Dict] = {}
        for pr in ps.process_iter(['pid', 'cmdline', 'name']):
            pid = pr.info['pid']
            cmd = ' '.join(pr.info.get('cmdline') or []) or (pr.info.get('name') or '')
            name = group_of(cmd)
            if name is None:
                continue
            seen.add(pid)
            p = self.procs.get(pid)
            if p is None:
                p = self.procs[pid] = pr
                try:
                    p.cpu_percent(None)
                except (ps.NoSuchProcess, ps.AccessDenied):
                    pass
                continue
            try:
                cpu = p.cpu_percent(None)
                mem = p.memory_info().rss / 1e6
            except (ps.NoSuchProcess, ps.AccessDenied):
                continue
            g = groups.setdefault(name, {'cpu': 0.0, 'mem_mb': 0.0, 'n': 0})
            g['cpu'] += cpu
            g['mem_mb'] += mem
            g['n'] += 1
        for pid in list(self.procs):
            if pid not in seen:
                del self.procs[pid]
        for g in groups.values():
            g['cpu'] = round(g['cpu'], 1)
            g['mem_mb'] = round(g['mem_mb'])
        return groups


class Rate:
    def __init__(self):
        self.n = 0
        self.ms = []

    def take(self, dt: float):
        hz = self.n / dt if dt > 0 else 0.0
        ms = round(sum(self.ms) / len(self.ms), 1) if self.ms else None
        self.n, self.ms = 0, []
        return round(hz, 1), ms


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SysMonitor(Node):
        def __init__(self):
            super().__init__('sys_monitor')
            self.declare_parameter('rate_hz', 2.0)
            self.declare_parameter('tegrastats', True)
            self.pub = self.create_publisher(String, '/system/usage', 10)
            import psutil
            self.psutil = psutil
            psutil.cpu_percent(None, percpu=True)
            self.table = ProcTable()
            self.tegra = Tegra() if self.get_parameter('tegrastats').value else None
            self.lane, self.yolo = Rate(), Rate()
            self.create_subscription(String, '/lane/status', lambda m: setattr(self.lane, 'n', self.lane.n + 1), 10)
            self.create_subscription(String, '/perception/detections', self._on_det, 10)
            self.t_last = time.monotonic()
            self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
            self.get_logger().info(f'sys_monitor: /system/usage (tegrastats {"on" if self.tegra and self.tegra.ok else "off"})')

        def _on_det(self, msg):
            self.yolo.n += 1
            try:
                ms = json.loads(msg.data).get('infer_ms')
                if isinstance(ms, (int, float)):
                    self.yolo.ms.append(float(ms))
            except ValueError:
                pass

        def _tick(self):
            now = time.monotonic()
            dt, self.t_last = now - self.t_last, now
            vm = self.psutil.virtual_memory()
            cores = self.psutil.cpu_percent(None, percpu=True)
            lane_hz, _ = self.lane.take(dt)
            yolo_hz, yolo_ms = self.yolo.take(dt)
            out = {
                'stamp': round(self.get_clock().now().nanoseconds / 1e9, 2),
                'cpu_total': round(sum(cores) / max(1, len(cores)), 1),
                'cpu_cores': [round(c) for c in cores],
                'mem_used_mb': round((vm.total - vm.available) / 1e6),
                'mem_total_mb': round(vm.total / 1e6),
                'nodes': dict(sorted(self.table.sample().items(), key=lambda kv: -kv[1]['cpu'])),
                'rates': {'lane_hz': lane_hz, 'yolo_hz': yolo_hz, 'yolo_ms': yolo_ms},
            }
            if self.tegra is not None and self.tegra.last:
                tg = self.tegra.last
                for k in ('gpu_pct', 'gpu_mhz', 'temp', 'power_mw'):
                    if k in tg:
                        out[k] = tg[k]
            self.pub.publish(String(data=json.dumps(out, ensure_ascii=False)))

        def destroy_node(self):
            if self.tegra is not None:
                self.tegra.stop()
            super().destroy_node()

    rclpy.init()
    node = SysMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
