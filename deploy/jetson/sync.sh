#!/usr/bin/env bash
# 在筆電上跑：把整個 repo（含 .git、模型權重）同步到 Jetson 的 ~/workspace/orca。
#
#   deploy/jetson/sync.sh            # 預設主機 orin（~/.ssh/config 的 Host）
#   JETSON=orin2 deploy/jetson/sync.sh
#
# - 筆電是開發機、Jetson 只跑：Jetson 上的程式一律從這裡同步過去，不要直接在 Jetson 上改
#  （--delete 會把 Jetson 上多出來的檔案刪掉）。
# - 連同 .git 一起同步，場次紀錄的程式版本（info/git.txt）才跟筆電上的一致，包含還沒 commit 的修改。
# - 不同步、也不會刪的：各跑法的 logs/（Jetson 上的場次紀錄）、TensorRT 引擎（*.engine，在 Jetson 上轉的，
#   只能在同型號、同 TensorRT 版本的機器上用）和它的驗證紀錄（*.engine.checked，刪了 up.sh 會再比對一次）、
#   筆電的模擬與編輯器暫存。
# - 改了 .py、樹的 yaml：同步後重跑任務（或重開那個容器）就生效。要不要重 build 由 Jetson 上的 up.sh 判斷。
set -euo pipefail
cd "$(dirname "$0")/../.."
HOST=${JETSON:-orin}
DEST=workspace/orca

rsync -a --delete --info=stats1 \
  --exclude '/deploy/*/logs/' \
  --exclude '*.engine' --exclude '*.engine.checked' \
  --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  --exclude '/.vscode/' \
  --exclude '/sim/worlds/generated/' \
  ./ "$HOST:$DEST/"
echo "已同步到 $HOST:~/$DEST（$(git rev-parse --short HEAD)$(git diff --quiet HEAD || echo '+未提交的修改')）"
