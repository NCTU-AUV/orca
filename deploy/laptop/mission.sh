#!/usr/bin/env bash
# 實機跑一棵行為樹（src/orca_mission/trees/<樹>.yaml），mission 服務：
#   ./up.sh（映像不存在或過期才 build；起容器、等飛控／相機／偵測就緒；已經在跑的不動）→ 開始錄 rosbag → 行為樹
#
#   ./up.sh                               # 可以先起好容器、確認就緒（不跑也行，這裡會代跑）
#   ./mission.sh                          # gate_pass
#   ./mission.sh gate_pass -p pass_s=6    # -p 覆寫樹的 vars
#   ./mission.sh smoke_test               # 動作測試（不需要門）
#   RECORD=0 ./mission.sh                 # 不錄 rosbag
#
# - 改了 msg、package.xml、setup.py、Dockerfile 或新增檔案，up.sh 會自動重 build；改 .py、樹的 yaml 不會。
# - RPi 端要先設好（cam-relay、mavproxy 的 udpin 14560），見 docs/laptop-rpi-relay.md。
# - 行為樹會列出檢查清單等你按 Enter：那是解鎖前最後一次確認，按了才會歸零、解鎖、下潛。
# - 任何時候 Ctrl-C：上鎖，船浮上來。
# - 紀錄：logs/mission/<樹>_<時間>.csv；rosbag（MCAP，影像只有畫好框的 JPEG）在同名的 logs/bag/<樹>_<時間>/，
#   從樹開始（列檢查清單時）錄到樹結束、船浮上水面：mission 上鎖後會等深度回到水面（最多 30 秒，
#   再按一次 Ctrl-C 不等）才結束，接著這裡停止錄影、把 .mcap 收尾（不到 1 秒），之後就能用 Foxglove 開
#  （還在寫的檔 Foxglove 打不開）。
set -euo pipefail
cd "$(dirname "$0")"
TREE=gate_pass
if [[ $# -gt 0 && $1 != -* ]]; then TREE=$1; shift; fi
RECORD=${RECORD:-1}
RUN=$(basename "$TREE" .yaml)_$(date +%Y%m%d_%H%M%S)    # CSV 和 rosbag 的名稱

./up.sh

if [[ $RECORD == 1 ]]; then
  stop_recorder() {
    docker compose --profile record stop recorder > /dev/null 2>&1 || true     # SIGINT：把 .mcap 收尾（< 1 秒）
    echo "rosbag 在 $PWD/logs/bag/$RUN/（用 Foxglove 開裡面的 .mcap）"
  }
  trap stop_recorder EXIT
  # --force-recreate：每次換新的資料夾名稱；之前沒停的 recorder 也會先收尾再重開
  BAG_NAME=$RUN docker compose --profile record up -d --force-recreate recorder
  printf '等開始錄 rosbag'
  start=$SECONDS
  until docker compose --profile record logs recorder 2>/dev/null | grep -q 'Subscribed to topic'; do
    if (( SECONDS - start >= 30 )); then
      printf '\n30 秒內沒開始錄 rosbag。看 docker compose --profile record logs recorder；不錄就用 RECORD=0 ./mission.sh\n' >&2
      exit 1
    fi
    printf '.'
    sleep 2
  done
  printf ' 好了\n'
fi

docker compose --profile mission run --rm mission mission "$TREE" --run-name "$RUN" "$@"
