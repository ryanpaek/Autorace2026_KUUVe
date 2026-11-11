"""주차 완료 소리 (chime_node): 소리 파일 합성, 울리는 시점, 재생기 선택."""

import os
import sys
import wave

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.chime_node import RATE, parked, player_command, write_chime_wav  # noqa: E402


def test_wav_is_valid_and_short(tmp_path):
    p = write_chime_wav(str(tmp_path / 'c.wav'))
    with wave.open(p) as w:
        assert w.getframerate() == RATE and w.getnchannels() == 1 and w.getsampwidth() == 2
        assert 0.8 < w.getnframes() / RATE < 1.5


def test_rings_only_when_parking_finishes():
    assert parked('M4_PARKING', 'FINISHED')
    assert not parked('M1_TRAFFIC', 'M4_PARKING')
    assert not parked('FINISHED', 'FINISHED')
    assert not parked(None, 'FINISHED')          # 노드를 늦게 켰을 때 이미 FINISHED면 안 울림


def test_player_command_with_device(monkeypatch):
    monkeypatch.setattr('shutil.which', lambda name: '/usr/bin/' + name)
    assert player_command('x.wav') == ['paplay', 'x.wav']
    assert player_command('x.wav', device='plughw:1,0') == ['aplay', '-q', '-D', 'plughw:1,0', 'x.wav']
    monkeypatch.setattr('shutil.which', lambda name: None)
    assert player_command('x.wav') is None
