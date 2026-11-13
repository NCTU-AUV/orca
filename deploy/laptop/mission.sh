#!/usr/bin/env bash
# 實機跑一棵行為樹（src/orca_mission/trees/<樹>.yaml），mission 服務：
#   起 front_camera、object_detector、fc_adapter、recorder → 等飛控、偵測就緒 → 行為樹
#
#   ./mission.sh                          # gate_pass
#   ./mission.sh gate_pass -p pass_s=6    # -p 覆寫樹的 vars
#   ./mission.sh smoke_test               # 動作測試（不需要門）
#
# - RPi 端要先設好（cam-relay、mavproxy 的 udpin 14560），見 docs/laptop-rpi-relay.md。
# - 行為樹會列出檢查清單等你按 Enter：那是解鎖前最後一次確認，按了才會歸零、解鎖、下潛。
# - 任何時候 Ctrl-C：上鎖，船浮上來。紀錄在 logs/mission/。
set -euo pipefail
cd "$(dirname "$0")"
TREE=gate_pass
if [[ $# -gt 0 && $1 != -* ]]; then TREE=$1; shift; fi

if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ../../ml/rfdetr/fetch_weights.sh
fi

docker compose build fc_adapter
docker compose --profile mission build mission
docker compose up -d front_camera object_detector fc_adapter recorder

in_adapter() { docker compose exec -T fc_adapter /entrypoint.sh "$@" 2>/dev/null; }

# wait_for <說明> <最多等幾秒> <沒等到時的提示> <成功條件（指令）...>
wait_for() {
  local what=$1 limit=$2 hint=$3 start=$SECONDS
  shift 3
  printf '等%s' "$what"
  until "$@"; do
    if (( SECONDS - start >= limit )); then
      printf '\n%s 秒內沒等到%s。%s\n' "$limit" "$what" "$hint" >&2
      exit 1
    fi
    printf '.'
    sleep 2
  done
  printf ' 好了\n'
}

fcu_connected() {
  local state
  state=$(in_adapter timeout 5 ros2 topic echo --once /mavros/state) || return 1
  [[ $state == *'connected: true'* ]]
}
has_topic() { in_adapter timeout 5 ros2 topic echo --once "$1" > /dev/null; }

wait_for '飛控連線' 60 'RPi 的 mavproxy 有加 udpin 14560 嗎？看 docker compose logs fc_adapter' fcu_connected
wait_for '偵測' 300 '模型載入失敗？看 docker compose logs object_detector' has_topic /detections

exec docker compose --profile mission run --rm mission mission "$TREE" "$@"
