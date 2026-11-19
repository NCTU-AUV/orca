#!/usr/bin/env bash
# 跑水池測試腳本（tools/pool_tests/water_*.py），在 fc_adapter 容器裡執行：
#
#   ./pool_test.sh water_depth            # 定深，預設 0.15 → 0.20 → 0.10 m
#   ./pool_test.sh water_depth 0.1 0.2    # 後面的參數直接傳給腳本
#   ./pool_test.sh water_heading 45 0
#   ./pool_test.sh water_mission
#
# - 要先 ./up.sh（有場次、fc_adapter 在跑）。CSV 寫到 logs/session/<場次>/pool_tests/。
# - 開始、結束都會插標記，開始的標記帶程式版本（commit，有未提交的修改加 +dirty）。
#   直接用 docker compose exec 跑的話，標記裡的版本是「未知」。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

name=${1:-}
if [[ -z $name || $name == water_common || ! -f ../../tools/pool_tests/$name.py ]]; then
  echo "用法：$0 <腳本> [參數...]，腳本：$(cd ../../tools/pool_tests && ls water_*.py | grep -v water_common \
    | sed 's/\.py$//' | tr '\n' ' ')" >&2
  exit 2
fi
shift
if ! service_running fc_adapter || [[ -z $(session_current) ]]; then
  echo '沒有進行中的場次或 fc_adapter 沒在跑，先 ./up.sh' >&2
  exit 1
fi
exec docker compose exec -e ORCA_GIT="$(session_git_version)" fc_adapter \
  /entrypoint.sh python3 "/orca/tools/pool_tests/$name.py" "$@"
