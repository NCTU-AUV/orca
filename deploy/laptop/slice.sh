#!/usr/bin/env bash
# 從場次的全程 rosbag 切出一段（例如一次任務），另存成獨立的 .mcap：
#
#   ./slice.sh --match gate_pass_20261120_101500                   # 目前場次；收工後是最近的場次
#   ./slice.sh 20261120_093000 --match 'keyboard: #3' --duration 60
#   ./slice.sh --start 10:15 --end 10:18:30
#
# - --match：markers.csv 裡含這段文字的標記，從第一筆切到最後一筆。行為樹、水池測試開始和結束都有標記，
#   給 run 的名稱就是整次任務（./mission.sh 結束時會印出這行指令）。
# - 輸出到 logs/session/<場次>/slices/<名稱>/，Foxglove 直接開裡面的 .mcap。
# - 其他參數（--pad、--name）見 src/orca_session/orca_session/bag_slice.py。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

if [[ ${1:-} =~ ^[0-9]{8}_[0-9]{6}$ ]]; then
  SESSION=$1
  shift
else
  SESSION=$(session_current)
  SESSION=${SESSION:-$(session_latest)}
fi
if [[ -z $SESSION || ! -d $SESSION_ROOT/$SESSION ]]; then
  echo "找不到場次 ${SESSION:-（logs/session/ 是空的）}" >&2
  exit 2
fi
exec docker compose --profile tools run --rm --no-deps -T gui_tools \
  ros2 run orca_session bag_slice "/sessions/$SESSION" "$@"
