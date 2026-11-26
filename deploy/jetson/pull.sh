#!/usr/bin/env bash
# 在筆電上跑：把 Jetson 上的場次紀錄拉回筆電的 deploy/jetson/logs/session/（同樣的結構，slice、Foxglove 直接用）。
#
#   deploy/jetson/pull.sh                    # 全部場次（已經拉過的檔案不重傳）
#   deploy/jetson/pull.sh 20261203_101500    # 只拉一個場次
#
# 只複製、不刪：Jetson 上的紀錄要清時自己在 Jetson 上刪。進行中的場次也可以拉（拉到當下為止）。
set -euo pipefail
cd "$(dirname "$0")"
HOST=${JETSON:-orin}
SRC=workspace/orca/deploy/jetson/logs/session
mkdir -p logs/session
if [[ -n ${1:-} ]]; then
  rsync -a --info=stats1 "$HOST:$SRC/$1" logs/session/
else
  rsync -a --info=stats1 --exclude current "$HOST:$SRC/" logs/session/
fi
echo "→ $PWD/logs/session/"
