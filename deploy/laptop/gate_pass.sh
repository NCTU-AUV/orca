#!/usr/bin/env bash
# 一個指令跑過門：起 cam_bridge、detector、bridge → 等飛控、相機、偵測都就緒 → 開始 mission/gate_pass.py。
#
#   ./gate_pass.sh [gate_pass.py 的參數，例如 --pass-s 6 --depth 0.3]
#
# - 第一次會下載模型權重、build image，要等比較久。
# - RPi 端要先設好（cam-relay、mavproxy 的 udpin 14560），見 docs/laptop-rpi-relay.md。
# - 流程開始前仍會列出檢查清單等你按 Enter：那是解鎖前最後一次確認，按了才會歸零、解鎖、下潛。
# - 任何時候 Ctrl-C：上鎖，船浮上來。流程結束後容器繼續跑，要再跑一次直接重新執行這個腳本；
#   全部關掉用 docker compose down。
set -euo pipefail
cd "$(dirname "$0")"

if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ../../ml/rfdetr/fetch_weights.sh
fi

docker compose up -d cam_bridge detector bridge

# 在 bridge 容器裡執行指令（/entrypoint.sh 會載入 ROS 環境）
in_bridge() { docker compose exec -T bridge /entrypoint.sh "$@" 2>/dev/null; }

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
  state=$(in_bridge timeout 5 ros2 topic echo --once /mavros/state) || return 1
  [[ $state == *'connected: true'* ]]
}
has_topic() { in_bridge timeout 5 ros2 topic echo --once "$1" > /dev/null; }

wait_for '飛控連線' 60 'RPi 的 mavproxy 有加 udpin 14560 嗎？看 docker compose logs bridge' fcu_connected
wait_for '相機' 60 'RPi 的 cam-relay 在跑嗎？看 docker compose logs cam_bridge' \
  has_topic /camera/camera/color/camera_info
wait_for '偵測' 300 '模型載入失敗？看 docker compose logs detector' has_topic /detections

exec docker compose exec bridge /entrypoint.sh python3 /orca/mission/gate_pass.py "$@"
