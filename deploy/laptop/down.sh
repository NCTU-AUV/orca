#!/usr/bin/env bash
# 實機收工：把目前場次收尾，再關掉所有容器。代替 docker compose down。
#
#   ./down.sh
#
# 依序：
#   1. 插「場次結束」標記，停 recorder（SIGINT），確認每個 bag 都有正常收尾
#   2. 各容器的終端輸出存成 <場次>/containers.log（docker compose down 之後就沒了）
#   3. 拍結束時的飛控參數快照 <場次>/info/fc_params_end.parm（飛控要連著）
#   4. 拉飛控的 DataFlash（.BIN）到 <場次>/fc_logs/（經 RPi，用 MAVFTP，約 400 KiB/s：每解鎖 1 小時約 7 分鐘），
#      再從 bag 算飛控開機時間 → ROS 時間的對照（info/fc_time.csv）。飛控要上鎖
#   5. docker compose down（所有 profile 的容器），記下收尾方式，刪掉 logs/session/current
#
# 直接 docker compose down、斷電的話，下一次 ./up.sh 會發現上一個場次沒收尾，補做還能補的部分。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

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
docker compose --profile '*' logs --no-color --timestamps > "$DIR/containers.log" 2>&1 || true
echo "  → $DIR/containers.log"

echo '3. 結束時的飛控參數快照'
if service_running fc_adapter && fcu_connected; then
  session_fc_params "$SESSION" fc_params_end.parm || echo '  沒拍到' >&2
else
  echo '  跳過：fc_adapter 沒在跑或飛控沒連上' >&2
fi

echo '4. 飛控的 .BIN'
if ! session_fc_logs_fetch "$SESSION"; then
  echo "  沒拉完。之後（飛控上鎖、RPi 連得上時）用 ./fc_logs.sh fetch $SESSION 再拉" >&2
fi
echo '  飛控時間對照'
session_fc_time "$SESSION" || echo "  沒算成功，之後用 ./fc_logs.sh time $SESSION 再算" >&2

echo '5. 關掉所有容器'
docker compose --profile '*' down
session_close "$SESSION" '用 down.sh 收尾'
echo "場次 $SESSION 結束。切出某一段：./slice.sh $SESSION --match <標記的文字>"
