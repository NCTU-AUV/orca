#!/usr/bin/env bash
# 在筆電上跑：經 ssh 把 Jetson 上 foxglove_bridge 的 8765 埠轉到筆電，Foxglove 開 ws://localhost:8765 看即時畫面。
#
#   deploy/jetson/view.sh          # Ctrl-C 結束
#
# - foxglove_bridge 只聽 Jetson 的 127.0.0.1，不對外開埠；連線走 ssh（跟 sync.sh 一樣的 Host orin）。
# - 影像請開壓縮過的 topic（/detections/image/compressed、/camera/camera/color/image_raw/compressed），
#   沒壓縮的 640x480@30 每秒約 27 MB，網路線撐得住但 Foxglove 會卡。
# - layout：tools/foxglove/orca_mission.json。
set -euo pipefail
HOST=${JETSON:-orin}
PORT=${PORT:-8765}
echo "Foxglove 開 ws://localhost:$PORT（Ctrl-C 結束）"
exec ssh -N -o ExitOnForwardFailure=yes -L "$PORT:127.0.0.1:8765" "$HOST"
