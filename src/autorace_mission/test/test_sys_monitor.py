"""11/16: 연산량 측정 노드 - tegrastats 줄 읽기, 명령줄 -> 노드 묶음."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from autorace_mission.sys_monitor import ProcTable, group_of, parse_tegrastats  # noqa: E402

ORIN = ('11-16-2026 13:20:01 RAM 3842/7620MB (lfb 2x4MB) SWAP 0/3810MB (cached 0MB) '
        'CPU [35%@1510,22%@1510,off,48%@1510,10%@1510,7%@1510] EMC_FREQ 12%@2133 GR3D_FREQ 61%@[624] '
        'NVENC off VIC_FREQ 0%@115 cpu@52.1C soc2@49.5C soc0@50.2C gpu@51.8C tj@52.6C soc1@49.9C '
        'VDD_IN 7210mW/7100mW VDD_CPU_GPU_CV 1980mW/1900mW VDD_SOC 1500mW/1490mW')


def test_parse_tegrastats_orin():
    t = parse_tegrastats(ORIN)
    assert t['ram_used_mb'] == 3842 and t['ram_total_mb'] == 7620
    assert t['cpu_cores'] == [35, 22, None, 48, 10, 7]
    assert t['gpu_pct'] == 61 and t['gpu_mhz'] == 624
    assert t['temp']['gpu'] == 51.8 and t['temp']['cpu'] == 52.1
    assert t['power_mw'] == 7210


def test_parse_tegrastats_old_format_and_garbage():
    t = parse_tegrastats('RAM 1000/4000MB CPU [10%@100,20%@100] GR3D_FREQ 5% cpu@40C')
    assert t['gpu_pct'] == 5 and 'gpu_mhz' not in t and t['cpu_cores'] == [10, 20]
    assert parse_tegrastats('nothing here') == {}


def test_group_of():
    assert group_of('/usr/bin/python3 /ws/install/autorace_perception/lib/autorace_perception/detector_node '
                    '--ros-args -r __node:=detector_node') == 'YOLO'
    assert group_of('python3 .../class_command_node --ros-args') == 'YOLO 투표'
    assert group_of('python3 .../lane_node --ros-args --params-file /x/params.yaml') == '차선'
    assert group_of('/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record -o 1116_01_data /scan') == '녹화'
    assert group_of('/usr/bin/bash') is None


def test_proc_table_runs():
    pt = ProcTable()
    pt.sample()
    assert isinstance(pt.sample(), dict)
