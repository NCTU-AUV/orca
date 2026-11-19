#!/usr/bin/env bash
# 實機：開場次（或沿用）、起服務、開始全程錄影，等飛控、相機、偵測、錄影就緒。
# 起的服務：front_camera、object_detector、fc_adapter、recorder（全程 rosbag）、marker_log（標記）。
# 映像不存在或過期才 build；沒重 build 的話，已經在跑的容器不動（不重開，mavros 的連線不會斷）。
#
#   ./up.sh            # 自動判斷要不要 build
#   ./up.sh --build    # 不管指紋，全部重 build（有 Docker 快取，沒改的部分很快）
#
# 場次（一次下水的全部紀錄，logs/session/<場次>/，結構見 deploy/common.sh）：
# - 沒有場次 → 開新場次：記開始時間、程式版本（git），起服務後拍飛控參數快照、記下飛控 SD 卡上
#   已有的 .BIN（收工時只拉這個場次新增的）、插「場次開始」標記。
# - 有場次而且 recorder 還在錄 → 沿用（mission.sh 每次都會先跑這個腳本）。
# - 有場次但 recorder 沒在跑、或中間重開過機 → 上一個場次沒用 ./down.sh 收尾：
#   補做還能補的（停止錄影、確認 bag 收尾、飛控連上後拍結束時的參數快照），再開新場次。
#   上一個場次的 .BIN 不在這裡拉（要好幾分鐘），印出指令：./fc_logs.sh fetch <上一個場次>
#
# 過期的判斷：映像的 orca.build-hash label 跟 docker/build-hash.sh 現在算的不同。指紋只包含
# Dockerfile、package.xml、setup.py、msg 這類「改了就要重 build」的檔案，以及 src/ 的檔案清單
# （新增、刪除檔案）。改 .py、行為樹的 yaml 不會觸發 build：compose 把 src/ 掛進容器
# （colcon --symlink-install），重跑任務（或重開那個容器）就生效。
# 重 build 之後，用到那個映像的容器會重開。
# 改了 compose.yaml 要套用到已經在跑的容器：docker compose up -d <服務>
#
# - RPi 端要先設好（cam-relay、mavproxy 的 udpin 14560），見 docs/laptop-rpi-relay.md。
# - 跑完接著開始任務：./mission.sh（行為樹），它會先跑這個腳本。
# - 收工：./down.sh（場次收尾再全部關掉），不要直接 docker compose down。
set -euo pipefail
cd "$(dirname "$0")"
source ../../docker/build-hash.sh
source ../common.sh

SERVICES=(front_camera object_detector fc_adapter recorder marker_log)

FORCE=0
case "${1:-}" in
  '') ;;
  --build) FORCE=1 ;;
  *) echo "用法：$0 [--build]" >&2; exit 2 ;;
esac

if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ../../ml/rfdetr/fetch_weights.sh
fi

# need_build <映像> <指紋>：要 build 就印原因並回傳 0
need_build() {
  local image=$1 want=$2 have
  if (( FORCE )); then
    echo "$image：--build，重 build"
    return 0
  fi
  if ! have=$(docker image inspect -f '{{index .Config.Labels "orca.build-hash"}}' "$image" 2> /dev/null); then
    echo "$image：映像不存在，build"
    return 0
  fi
  if [[ $have != "$want" ]]; then
    echo "$image：build 設定檔或 src/ 的檔案清單有變（指紋 ${have:-無} → $want），重 build"
    return 0
  fi
  return 1
}

RECREATE=0    # orca-base、orca-perception 換了：用到它們的容器要重開
if need_build orca-base "$ORCA_BASE_HASH"; then
  docker compose build fc_adapter                   # front_camera、fc_adapter、recorder、marker_log、gui_tools 共用
  RECREATE=1
fi
# orca-mission 疊在 orca-base 上，要在它之後。只有 mission 換了的話不用重開任何容器（mission 每次都是新的）
if need_build orca-mission "$ORCA_MISSION_HASH"; then
  docker compose --profile mission build mission
fi
if need_build orca-perception "$ORCA_PERCEPTION_HASH"; then
  docker compose build object_detector
  RECREATE=1
fi

# ---------- 場次 ----------

session_prepare
SESSION=$(session_current)
PREV=''       # 沒收尾的上一個場次
NEW=0
if [[ -n $SESSION ]]; then
  if service_running recorder && [[ $(session_boot "$SESSION") == "$(boot_id)" ]]; then
    echo "沿用場次 $SESSION"
  else
    PREV=$SESSION
    SESSION=''
  fi
fi
if [[ -n $PREV ]]; then
  cat >&2 << EOF
上一個場次 $PREV 沒有用 ./down.sh 收尾（直接 docker compose down、重開機或斷電）。
補做還能補的：停止錄影、確認 bag 收尾，飛控連上後拍結束時的參數快照。容器的終端輸出已經沒辦法補存。
飛控的 .BIN 之後用 ./fc_logs.sh fetch $PREV 拉（飛控上鎖時）。
EOF
  docker compose stop recorder marker_log > /dev/null 2>&1 || true
  session_check_bags "$PREV" || true
  session_close "$PREV" '沒有用 down.sh 收尾，由下一次 up.sh 補救：停止錄影；容器的終端輸出遺失'
fi
if [[ -z $SESSION ]]; then
  SESSION=$(session_new)
  NEW=1
  echo "新場次 $SESSION，紀錄在 $PWD/$SESSION_ROOT/$SESSION/"
fi

# 這個版本之前建立的 fc_adapter 沒掛 logs/session/，要重開一次
if service_running fc_adapter && ! docker compose exec -T fc_adapter test -d /sessions; then
  RECREATE=1
fi

if (( RECREATE )); then
  docker compose up -d "${SERVICES[@]}"             # 映像或設定換了的容器重開，其他不動
else
  # --no-recreate：容器已存在就不重開，停著的直接啟動
  docker compose up -d --no-recreate "${SERVICES[@]}"
fi

wait_for '飛控連線' 60 'RPi 的 mavproxy 有加 udpin 14560 嗎？看 docker compose logs fc_adapter' fcu_connected
wait_for '相機' 60 'RPi 的 cam-relay 在跑嗎？看 docker compose logs front_camera' \
  has_topic /camera/camera/color/camera_info
wait_for '偵測' 300 '模型載入失敗？看 docker compose logs object_detector' has_topic /detections
wait_for '開始錄影' 30 '看 docker compose logs recorder' session_recording "$SESSION"

if [[ ! -f $SESSION_ROOT/$SESSION/info/fc_params_start.parm ]]; then
  session_fc_params "$SESSION" fc_params_start.parm \
    || echo '開始時的飛控參數快照沒拍到，下次 ./up.sh 會再試' >&2
fi
if [[ -n $PREV ]]; then
  if session_fc_params "$PREV" fc_params_end.parm \
       '（上一個場次沒收尾，由下一個場次的 up.sh 補拍；兩個場次之間改過參數的話不準）'; then
    session_close "$PREV" '補拍結束時的飛控參數快照（info/fc_params_end.parm）'
  fi
fi
if (( NEW )); then
  if session_fc_logs_start "$SESSION"; then
    echo "飛控 SD 卡上已有 $(wc -l < "$SESSION_ROOT/$SESSION/info/fc_logs_start.txt") 個 .BIN，收工時只拉這個場次新增的"
  else
    echo '列不出飛控 SD 卡上的 .BIN，收工時不會自動拉。RPi 連得上嗎？fc_logs.py 部署了嗎' \
      '（deploy/rpi-relay/deploy.sh fc-logs）' >&2
  fi
  session_mark up.sh "場次 $SESSION 開始（$(session_git_version)）" > /dev/null \
    || echo '「場次開始」的標記沒發出去，看 docker compose logs marker_log' >&2
fi
echo "場次 $SESSION 錄影中。收工用 ./down.sh"
