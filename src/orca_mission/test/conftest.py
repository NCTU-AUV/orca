# 讓 test/ 裡的 harness 和原始碼的 orca_mission 不用安裝就能 import
import os
import sys

HERE = os.path.dirname(__file__)
sys.path[:0] = [HERE, os.path.join(HERE, '..')]
