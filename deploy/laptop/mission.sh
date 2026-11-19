#!/usr/bin/env bash
# 實機跑一棵行為樹（src/orca_mission/trees/<樹>.yaml），mission 服務：
#   ./up.sh（開場次或沿用；映像不存在或過期才 build；起容器、等飛控／相機／偵測／錄影就緒；已經在跑的不動）→ 行為樹
#
#   ./up.sh                               # 可以先起好容器、確認就緒（不跑也行，這裡會代跑）
#   ./mission.sh                          # gate_pass
#   ./mission.sh gate_pass -p pass_s=6    # -p 覆寫樹的 vars
#   ./mission.sh gate_pass_yaw            # 過門，下潛後先轉向對準門再鎖航向
#   ./mission.sh smoke_test               # 動作測試（不需要門）
#
# - 改了 msg、package.xml、setup.py、Dockerfile 或新增檔案，up.sh 會自動重 build；改 .py、樹的 yaml 不會。
# - RPi 端要先設好（cam-relay、mavproxy 的 udpin 14560），見 docs/laptop-rpi-relay.md。
# - 行為樹會列出檢查清單等你按 Enter：那是解鎖前最後一次確認，按了才會歸零、解鎖、下潛。
# - 任何時候 Ctrl-C：上鎖，船浮上來。
# - 紀錄：CSV 在 logs/session/<場次>/mission/<樹>_<時間>.csv。rosbag 是場次的全程錄影，
#   樹開始、結束（上鎖後等船浮上水面，最多 30 秒，再按一次 Ctrl-C 不等）都會插標記，
#   開始的標記帶程式版本。單獨要這次任務的 .mcap：./slice.sh --match <樹>_<時間>（結束時會印出來）。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh
TREE=gate_pass
if [[ $# -gt 0 && $1 != -* ]]; then TREE=$1; shift; fi
RUN=$(basename "$TREE" .yaml)_$(date +%Y%m%d_%H%M%S)    # CSV 的檔名，也是標記裡的名稱

./up.sh

status=0
docker compose --profile mission run --rm -e ORCA_GIT="$(session_git_version)" \
  mission mission "$TREE" --run-name "$RUN" "$@" || status=$?
echo "切出這次任務的 rosbag：./slice.sh --match $RUN"
exit "$status"
