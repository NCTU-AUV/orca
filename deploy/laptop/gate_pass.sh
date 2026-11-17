#!/usr/bin/env bash
# 一個指令跑過門（舊版 mission/gate_pass.py；行為樹版見 mission.sh）：
#   ./up.sh（映像不存在或過期才 build；起容器、等飛控／相機／偵測就緒；已經在跑的不動）→ 開始 mission/gate_pass.py
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

./up.sh

exec docker compose exec fc_adapter /entrypoint.sh python3 /orca/mission/gate_pass.py "$@"
