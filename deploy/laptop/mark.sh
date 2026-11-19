#!/usr/bin/env bash
# 在目前場次插一筆標記：錄進 rosbag（/session/marker），也寫進 <場次>/markers.csv。
# 事後用來找段落，或用 ./slice.sh --match 切出來。
#
#   ./mark.sh surge 0.4 第 2 趟
#
# 鍵盤操控時按 M 也會插標記（內容是編號）。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

if [[ $# -eq 0 ]]; then
  echo "用法：$0 <內容>" >&2
  exit 2
fi
if ! service_running marker_log; then
  echo 'marker_log 沒在跑，先 ./up.sh' >&2
  exit 1
fi
session_mark mark.sh "$*"
