#!/usr/bin/env bash
# 一起跑 mavros 與 auv_adapter，給容器當 command 用：bridge.sh <rpi_relay|sitl|serial>
#
# 停止時（docker compose stop/down 送 SIGTERM）先 SIGINT 給 auv_adapter，讓它走 Ctrl-C 的收尾
# （送中位、上鎖），等它結束再停 mavros（上鎖要經過 mavros）。任何一個自己結束，整個就結束。
set -u
LINK=${1:?usage: bridge.sh <rpi_relay|sitl|serial>}
SHARE=$(ros2 pkg prefix orca_bridge)/share/orca_bridge
# 直接執行 auv_adapter，不經 ros2 run：訊號才會直接送到它
ADAPTER_BIN=$(ros2 pkg prefix orca_bridge)/lib/orca_bridge/auv_adapter

ros2 run mavros mavros_node --ros-args --params-file "$SHARE/config/mavros_$LINK.yaml" &
MAVROS=$!
"$ADAPTER_BIN" &
ADAPTER=$!
trap 'kill -INT $ADAPTER; wait $ADAPTER; kill -TERM $MAVROS; wait $MAVROS; exit 0' TERM INT
wait -n
kill -INT $ADAPTER 2>/dev/null; wait $ADAPTER
kill -TERM $MAVROS 2>/dev/null; wait $MAVROS
