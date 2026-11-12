#!/usr/bin/env python3
"""YOLO 모델/입력 크기/conf만 변경. 팀 미션 설정과 YAML 주석은 보존한다."""
import argparse
import json
from pathlib import Path
import re


def configured_text(text, model_path, size, conf):
    if size not in (320, 640) or not 0.0 <= conf <= 1.0:
        raise ValueError('size must be 320/640 and conf must be in [0, 1]')
    replacements = {
        'detector_node': {'model_path': json.dumps(str(model_path), ensure_ascii=False),
                          'input_size': str(size), 'conf_threshold': str(float(conf))},
        'class_command_node': {'conf_threshold': str(float(conf))},
    }
    current, changed, out = None, set(), []
    for line in text.splitlines(keepends=True):
        m = re.match(r'^([a-zA-Z_][a-zA-Z0-9_]*):\s*(?:#.*)?$', line.rstrip())
        if m:
            current = m.group(1)
        m = re.match(r'^(    )(model_path|input_size|conf_threshold):\s*([^#\n]*)(.*)$', line.rstrip('\n'))
        if m and m.group(2) in replacements.get(current, {}):
            key = m.group(2)
            line = f'{m.group(1)}{key}: {replacements[current][key]}'
            if m.group(4):
                line += '  ' + m.group(4)
            line += '\n'
            if (current, key) in changed:
                raise ValueError(f'duplicate parameter: {current}.{key}')
            changed.add((current, key))
        out.append(line)
    if len(changed) != 4:
        raise ValueError('expected exactly 4 YOLO settings; params.yaml layout has changed')
    return ''.join(out)


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--size', type=int, choices=(320, 640), required=True)
    parser.add_argument('--conf', type=float, default=0.7)
    parser.add_argument('--model-dir', type=Path, default=repo / 'models')
    args = parser.parse_args()
    name = 'autorace_yolo_v45.onnx' if args.size == 640 else 'autorace_yolo_v45_320.onnx'
    model = (args.model_dir / name).resolve()
    if not model.is_file():
        parser.error(f'model not found: {model}')
    config = repo / 'src/autorace_bringup/config/params.yaml'
    try:
        text = configured_text(config.read_text(), model, args.size, args.conf)
    except ValueError as exc:
        parser.error(str(exc))
    config.write_text(text)
    print(f'Updated {config}\nmodel={model}\ninput_size={args.size}, conf={args.conf}')
    print('Mission parameters unchanged. Restart detector and use this source YAML via params:=.')


if __name__ == '__main__':
    main()
