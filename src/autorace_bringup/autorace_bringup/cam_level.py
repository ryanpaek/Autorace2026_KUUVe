"""카메라 노출 단계 저장/확인 (11/19).

  ros2 run autorace_bringup cam_level 3   # 3단계로 저장 (다음 실행부터 계속 적용)
  ros2 run autorace_bringup cam_level     # 지금 저장된 단계 보기
  ros2 run autorace_bringup cam_level 0   # 끔 (카메라 자동 노출 그대로)
"""
import sys

from autorace_bringup.exposure_core import LEVEL_FILE, LEVEL_FLOOR_V, parse_level, read_saved_level, save_level

TABLE = """  단계  목표 바닥 V  노란선 인식(절반 이상/거의 전부)  신호등·표지판 밝기
  0     자동 노출 그대로 (예전과 같음)
  1     115          83% / 63%                       x0.86
  2     100          96% / 82%                       x0.75
  3      90          99% / 95%                       x0.68
  4      80         100% / 98%                       x0.60"""


def main():
    args = sys.argv[1:]
    args = [a for a in args if not a.startswith('--ros-args')][:1]
    if args:
        lv = parse_level(args[0])
        if lv is None:
            print(f'0~4 중 하나로: {args[0]!r} 은 안 됨\n{TABLE}')
            sys.exit(1)
        save_level(lv)
        print(f'저장: {lv}단계 ({LEVEL_FILE}). 카메라를 다시 켜면 적용됨 (지금 켜져 있으면 launch 재시작).')
        return
    lv = read_saved_level()
    if lv is None:
        print(f'저장된 단계 없음 -> params.yaml exposure_node.level 사용 (기본 2).\n{TABLE}')
    else:
        tgt = '자동 노출' if lv == 0 else f'바닥 V {LEVEL_FLOOR_V[lv]:.0f}'
        print(f'지금 저장된 단계: {lv} ({tgt}) - {LEVEL_FILE}\n{TABLE}')


if __name__ == '__main__':
    main()
