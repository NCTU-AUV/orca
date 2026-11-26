#!/usr/bin/env bash
# Jetson 收工：把目前場次收尾，再關掉所有容器。代替 docker compose down。在 Jetson 上跑。
#
#   ./down.sh
#
# 依序：
#   1. 插「場次結束」標記，停 recorder（SIGINT），確認每個 bag 都有正常收尾
#   2. 各容器（含最後一次任務 orca-mission）的終端輸出存成 <場次>/containers.log
#   3. 拍結束時的飛控參數快照 <場次>/info/fc_params_end.parm（飛控要連著）
#   4. docker compose down（所有 profile 的容器），記下收尾方式，刪掉 logs/session/current
#
# 任務還在跑（船可能在水裡）時不做，先 ./stop.sh。
# 飛控的 .BIN 還不會自動拉（deploy/laptop 是經 RPi 拉的），之後做。
# 開機自動啟動（./autostart.sh enable）的話，下次開機服務會再起來；收工後不想要它們跑就關機。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

MISSION=orca-mission
if [[ $(docker inspect -f '{{.State.Running}}' "$MISSION" 2> /dev/null) == true ]]; then
  echo '任務還在跑（船可能在水裡），先 ./stop.sh' >&2
  exit 1
fi

SESSION=$(session_current)
if [[ -z $SESSION ]]; then
  echo '沒有進行中的場次，直接關掉所有容器'
  docker compose --profile '*' down
  exit 0
fi
DIR=$SESSION_ROOT/$SESSION
echo "場次 $SESSION 收尾（$PWD/$DIR/）"

echo '1. 停止錄影'
if service_running recorder && service_running marker_log; then
  session_mark down.sh "場次 $SESSION 結束" > /dev/null || echo '  「場次結束」的標記沒發出去' >&2
fi
docker compose stop recorder
if session_check_bags "$SESSION"; then
  echo '  每個 bag 都有正常收尾'
fi

echo '2. 存各容器的終端輸出'
{
  docker compose --profile '*' logs --no-color --timestamps 2>&1 || true
  if docker inspect "$MISSION" > /dev/null 2>&1; then
    echo "===== $MISSION（最後一次任務）"
    docker logs --timestamps "$MISSION" 2>&1 || true
  fi
} > "$DIR/containers.log"
echo "  → $DIR/containers.log"

echo '3. 結束時的飛控參數快照'
if service_running fc_adapter && fcu_connected; then
  session_fc_params "$SESSION" fc_params_end.parm || echo '  沒拍到' >&2
else
  echo '  跳過：fc_adapter 沒在跑或飛控沒連上' >&2
fi

echo '4. 關掉所有容器'
docker compose --profile '*' down
docker rm "$MISSION" > /dev/null 2>&1 || true
session_close "$SESSION" '用 down.sh 收尾'
echo "場次 $SESSION 結束。紀錄拉回筆電：在筆電上跑 deploy/jetson/pull.sh"
