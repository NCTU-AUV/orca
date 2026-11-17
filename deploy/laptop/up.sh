#!/usr/bin/env bash
# 實機：起任務需要的容器（front_camera、object_detector、fc_adapter），等飛控、相機、偵測就緒。
# 不起 recorder：rosbag 只在 ./mission.sh 跑行為樹時錄。
# 映像不存在或過期才 build；沒重 build 的話，已經在跑的容器不動（不重開，mavros 的連線不會斷）。
#
#   ./up.sh            # 自動判斷要不要 build
#   ./up.sh --build    # 不管指紋，全部重 build（有 Docker 快取，沒改的部分很快）
#
# 過期的判斷：映像的 orca.build-hash label 跟 docker/build-hash.sh 現在算的不同。指紋只包含
# Dockerfile、package.xml、setup.py、msg 這類「改了就要重 build」的檔案，以及 src/ 的檔案清單
# （新增、刪除檔案）。改 .py、行為樹的 yaml 不會觸發 build：compose 把 src/ 掛進容器
# （colcon --symlink-install），重跑任務（或重開那個容器）就生效。
# 重 build 之後，用到那個映像的容器會重開。
# 改了 compose.yaml 要套用到已經在跑的容器：docker compose up -d <服務>
#
# - RPi 端要先設好（cam-relay、mavproxy 的 udpin 14560），見 docs/laptop-rpi-relay.md。
# - 跑完接著開始任務：./mission.sh（行為樹）或 ./gate_pass.sh（舊版），兩個都會先跑這個腳本。
# - 全部關掉：docker compose down。
set -euo pipefail
cd "$(dirname "$0")"
source ../../docker/build-hash.sh

SERVICES=(front_camera object_detector fc_adapter)

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
  docker compose build fc_adapter                   # front_camera、fc_adapter、recorder、gui_tools 共用
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

if (( RECREATE )); then
  docker compose up -d "${SERVICES[@]}"             # 映像換了的容器重開，其他不動
else
  # --no-recreate：容器已存在就不重開，停著的直接啟動
  docker compose up -d --no-recreate "${SERVICES[@]}"
fi

# 在 fc_adapter 容器裡執行指令（/entrypoint.sh 會載入 ROS 環境）
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
wait_for '相機' 60 'RPi 的 cam-relay 在跑嗎？看 docker compose logs front_camera' \
  has_topic /camera/camera/color/camera_info
wait_for '偵測' 300 '模型載入失敗？看 docker compose logs object_detector' has_topic /detections
