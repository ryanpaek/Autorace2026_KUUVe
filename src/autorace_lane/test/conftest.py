import os
import sys

# colcon 빌드 없이 `pytest src/autorace_lane/test`로 돌릴 때 패키지를 찾게 한다
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
