# 레포 루트에서 pytest 를 돌려도 이 패키지를 찾도록 패키지 폴더를 경로에 추가
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
