"""주차 완료 '띠링' 소리를 차(LIMO) 스피커로 낸다 (11/11).

LIMO는 차체 좌우에 스피커(2W x 2)가 있다. 소리 파일은 시작할 때 직접 합성해 /tmp에 만든다 (외부 파일 없음).
  구독  /mission/state (String JSON)  M4_PARKING -> FINISHED 가 되는 순간 한 번 울림
        /chime/play    (std_msgs/Empty) 시험용: 바로 울림
  재생  paplay(PulseAudio) -> aplay(ALSA) 순서로 시도. player 파라미터로 고정 가능
        소리가 안 나면: 리모에서  aplay -l  로 장치 확인 후  device:=plughw:카드,장치

  ros2 run autorace_mission chime_node
  ros2 topic pub --once /chime/play std_msgs/msg/Empty        # 시험
"""

import json
import math
import os
import shutil
import struct
import subprocess
import tempfile
import wave
from typing import Optional

RATE = 44100
# (주파수 Hz, 시작 s): 아래 음 -> 위 음 두 번 '띠링'. 배음(2배)을 조금 섞어 맑게
NOTES = ((1046.5, 0.0), (1568.0, 0.13))


def chime_samples(volume: float = 0.6, length: float = 1.1):
    n = int(RATE * length)
    out = [0.0] * n
    for f, t0 in NOTES:
        i0 = int(t0 * RATE)
        for i in range(i0, n):
            t = (i - i0) / RATE
            env = min(1.0, t / 0.012) * math.exp(-t * 6.0)       # 빠른 시작, 0.9 s 정도에 사라짐
            out[i] += env * (0.8 * math.sin(2 * math.pi * f * t) + 0.18 * math.sin(4 * math.pi * f * t))
    peak = max(abs(v) for v in out) or 1.0
    return [v / peak * volume for v in out]


def write_chime_wav(path: str, volume: float = 0.6) -> str:
    data = b''.join(struct.pack('<h', int(max(-1.0, min(1.0, v)) * 32767)) for v in chime_samples(volume))
    with wave.open(path, 'wb') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(data)
    return path


def player_command(path: str, player: str = 'auto', device: str = ''):
    """재생 명령 (리스트). 쓸 수 있는 재생기가 없으면 None."""
    if player in ('auto', 'paplay') and shutil.which('paplay') and not device:
        return ['paplay', path]
    if player in ('auto', 'aplay') and shutil.which('aplay'):
        return ['aplay', '-q'] + (['-D', device] if device else []) + [path]
    return None


def parked(prev: str, cur: str, result: Optional[str] = None) -> bool:
    """M4_PARKING -> FINISHED 이고 주차 결과가 성공(PARKED...)일 때만. result가 없으면(옛 미션 노드) 예전처럼.
    11/14: 주차 실패(FAIL_NO_CAR)에도 FINISHED라 띠링이 울려 성공한 줄 착각했다."""
    if not (prev == 'M4_PARKING' and cur == 'FINISHED'):
        return False
    return result is None or str(result).startswith('PARKED')


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty, String

    class ChimeNode(Node):
        def __init__(self):
            super().__init__('chime_node')
            self.declare_parameter('player', 'auto')     # auto | paplay | aplay
            self.declare_parameter('device', '')         # aplay -D 장치 (예: plughw:1,0). 비우면 기본 장치
            self.declare_parameter('volume', 0.6)        # 0~1
            self.path = write_chime_wav(os.path.join(tempfile.gettempdir(), 'autorace_chime.wav'),
                                        float(self.get_parameter('volume').value))
            self.prev = None
            self.create_subscription(String, '/mission/state', self._on_state, 10)
            self.create_subscription(Empty, '/chime/play', lambda _m: self.play('test'), 10)
            cmd = self._cmd()
            self.get_logger().info(f'chime ready: {" ".join(cmd) if cmd else "재생기 없음 (aplay/paplay 설치 필요)"}')

        def _cmd(self):
            return player_command(self.path, self.get_parameter('player').value,
                                  self.get_parameter('device').value)

        def _on_state(self, msg):
            try:
                d = json.loads(msg.data)
                cur = d.get('state')
            except (ValueError, AttributeError):
                return
            if parked(self.prev, cur, d.get('parking_result')):
                self.play('parked')
            elif self.prev == 'M4_PARKING' and cur == 'FINISHED':
                self.get_logger().warning(f"주차 실패 ({d.get('parking_result')}) -> 띠링 안 울림")
            self.prev = cur

        def play(self, why):
            cmd = self._cmd()
            if cmd is None:
                self.get_logger().warning('재생기 없음: sudo apt install alsa-utils')
                return
            try:   # 기다리지 않음 (미션 노드와 따로 도니 제어에 영향 없음)
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().info(f'띠링 ({why})')
            except OSError as e:
                self.get_logger().warning(f'재생 실패: {e}')

    rclpy.init(args=args)
    node = ChimeNode()
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
