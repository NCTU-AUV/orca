#!/usr/bin/env bash
# 中止背景的任務（./mission.sh 啟動的 orca-mission）：送 SIGTERM，mission 收尾（MANUAL、全 0、上鎖），船浮上來。
#
#   ./stop.sh
set -euo pipefail
NAME=orca-mission
if [[ $(docker inspect -f '{{.State.Running}}' "$NAME" 2> /dev/null) != true ]]; then
  echo '沒有任務在跑'
  exit 0
fi
docker stop -t 15 "$NAME" > /dev/null
docker logs --tail 15 "$NAME"
echo "任務已中止（完整輸出：docker logs $NAME）"
