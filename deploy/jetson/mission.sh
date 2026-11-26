#!/usr/bin/env bash
# AUV 獨自跑一棵行為樹（src/orca_mission/trees/<樹>.yaml）。在 Jetson 上跑（筆電：ssh -t orin）。
#
#   ./mission.sh                              # gate_pass，倒數 60 秒
#   ./mission.sh gate_pass_yaw --delay 90     # 倒數 90 秒
#   ./mission.sh gate_pass -p pass_s=6        # -p 覆寫樹的 vars
#
# 流程：
#   1. ./up.sh（開場次或沿用、起服務、等就緒）
#   2. 在這個終端機列出樹的檢查清單，按 Enter 確認（Ctrl-C 取消，什麼都不會動）
#   3. 任務在背景啟動（容器 orca-mission），倒數 --delay 秒：這段時間拔掉網路線、把船放下水。
#      ssh 斷線不影響任務。倒數完才歸零、解鎖、下潛
#   4. 這裡接著顯示任務的輸出，斷線或任務結束就停；重新連上後 docker logs -f orca-mission 接著看
#
# - 中止：./stop.sh（上鎖，船浮上來）。網路線拔掉之後只能靠樹的逾時、kill switch。
# - 任務結束（完成、逾時、出錯）都會 MANUAL、上鎖，船浮上來。mission 被強制結束（沒走收尾）的話，
#   auv_adapter 5 秒收不到指令會上鎖；adapter 也掛掉的話飛控 3 秒收不到搖桿指令會上鎖（FS_PILOT_INPUT）。
# - 紀錄：CSV 在 logs/session/<場次>/mission/<樹>_<時間>.csv；rosbag 是場次的全程錄影，
#   單獨要這次任務的 .mcap：./slice.sh --match <樹>_<時間>（啟動時會印出來）。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

NAME=orca-mission
TREE=gate_pass
DELAY=60
if [[ $# -gt 0 && $1 != -* ]]; then TREE=$1; shift; fi
ARGS=()
while (( $# )); do
  case $1 in
    --delay) DELAY=${2:?--delay 要給秒數}; shift 2 ;;
    *) ARGS+=("$1"); shift ;;
  esac
done
if ! [[ $DELAY =~ ^[0-9]+$ ]]; then
  echo "--delay 要是整數秒：$DELAY" >&2
  exit 2
fi

if [[ $(docker inspect -f '{{.State.Running}}' "$NAME" 2> /dev/null) == true ]]; then
  echo "已經有任務在跑（docker logs -f $NAME 看輸出，./stop.sh 中止）" >&2
  exit 1
fi

./up.sh < /dev/null          # 不讓 up.sh 裡的 docker compose exec 吃掉下面等 Enter 要讀的輸入

RUN=$(basename "$TREE" .yaml)_$(date +%Y%m%d_%H%M%S)    # CSV 的檔名，也是標記裡的名稱
mission_run() { docker compose --profile mission run --no-deps "$@"; }

# 檢查清單（順便確認樹載得起來、參數沒寫錯）。< /dev/null：docker compose run 預設會讀標準輸入，
# 不擋掉的話會吃掉下面等 Enter 要讀的輸入
CHECKLIST=$(mission_run --rm -T mission mission "$TREE" --print-checklist "${ARGS[@]}" < /dev/null)
echo
echo "任務 $TREE（$RUN），啟動後倒數 $DELAY 秒才歸零、解鎖、下潛。開始前確認："
while IFS= read -r item; do
  [[ -n $item ]] && echo "  - $item"
done <<< "$CHECKLIST"
echo "  - 倒數 $DELAY 秒內來得及拔網路線、把船放到起點"
if ! read -r -p '  >>> 都確認了按 Enter 啟動（Ctrl-C 取消）'; then
  echo '沒有讀到確認（輸入已結束），不啟動' >&2
  exit 1
fi

docker rm "$NAME" > /dev/null 2>&1 || true            # 上一次任務留下的（已結束）
mission_run -d --name "$NAME" -e ORCA_GIT="$(session_git_version)" \
  mission mission "$TREE" --run-name "$RUN" --start-delay "$DELAY" "${ARGS[@]}" > /dev/null < /dev/null
cat << EOF

任務已在背景啟動，$DELAY 秒後開始。現在可以拔網路線、放船下水。
  中止：./stop.sh　　看輸出：docker logs -f $NAME　　切出這次的 rosbag：./slice.sh --match $RUN
以下是任務的輸出（Ctrl-C 只停止顯示，不影響任務）：

EOF
docker logs -f "$NAME" || true
