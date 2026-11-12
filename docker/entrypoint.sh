#!/bin/bash
# 載入 ROS 與 orca workspace 的環境，再執行 compose 指定的指令
source /opt/ros/humble/setup.bash
source /opt/orca_ws/install/setup.bash
exec "$@"
