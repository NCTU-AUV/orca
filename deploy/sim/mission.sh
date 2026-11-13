#!/usr/bin/env bash
# 在模擬裡跑一棵行為樹（src/orca_mission/trees/<樹>.yaml），mission 服務：
#   Gazebo（有門和相機的世界）+ ArduSub SITL + fc_adapter + 相機 + object_detector → 等就緒 → 行為樹
#
#   ./mission.sh                                      # gate_pass，資格賽門（orca_qgate.sdf）
#   ./mission.sh gate_pass -p pass_s=6                # -p 覆寫樹的 vars
#   WORLD=orca_gate.sdf ./mission.sh gate_pass -p depth=0.9   # 導航門
#   WORLD=orca_pool.sdf ./mission.sh smoke_test       # 動作測試（不需要門）
#   GUI=0 ./mission.sh                                # 不開 Gazebo 視窗
#   RECORD=0 ./mission.sh                             # 不錄 rosbag
#
# - 每次執行都會重開 simulator，船從水面、原點開始；其他服務沿用已經在跑的。
# - 第一次會下載模型權重、build image，要等比較久。
# - 不列檢查清單（--no-checklist）。任何時候 Ctrl-C：上鎖，船浮上來。
# - 紀錄：logs/mission/<樹>_<時間>.csv，rosbag（MCAP，影像只有畫好框的 JPEG）在同名的 logs/bag/<樹>_<時間>/，
#   從樹開始錄到樹結束（Ctrl-C 也會收尾），用 Foxglove 直接開 .mcap。
# - 全部關掉：docker compose --profile vision down
# - 跟實機的 deploy/laptop 都綁 UDP 14551，不能同時跑。
set -euo pipefail
cd "$(dirname "$0")"
TREE=gate_pass
if [[ $# -gt 0 && $1 != -* ]]; then TREE=$1; shift; fi
export WORLD=${WORLD:-orca_qgate.sdf} GUI=${GUI:-1}
RECORD=${RECORD:-1}
RUN=$(basename "$TREE" .yaml)_$(date +%Y%m%d_%H%M%S)    # CSV 和 rosbag 的名稱

if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ../../ml/rfdetr/fetch_weights.sh
fi
if [[ $GUI == 1 ]]; then
  xhost +SI:localuser:root > /dev/null
fi

# 世界檔裡有相機（model://orca_cam）才起相機和偵測
VISION=0
if grep -q 'model://orca_cam' "../../sim/worlds/$WORLD"; then VISION=1; fi

# mission 的 image 疊在 orca-base 上，要先 build orca-base（recorder 也用 orca-base）
docker compose build fc_adapter
docker compose --profile mission build mission
if [[ $VISION == 1 ]]; then
  docker compose --profile vision up -d fc_adapter sim_cameras object_detector
else
  docker compose up -d fc_adapter
fi
docker compose up -d --force-recreate simulator

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

ekf_ready() { docker compose logs simulator 2>/dev/null | grep -q 'MAG0 initial yaw alignment complete'; }
fcu_connected() {
  local state
  state=$(in_adapter timeout 5 ros2 topic echo --once /mavros/state) || return 1
  [[ $state == *'connected: true'* ]]
}
has_topic() { in_adapter timeout 5 ros2 topic echo --once "$1" > /dev/null; }
recording() { docker compose --profile record logs recorder 2>/dev/null | grep -q 'Subscribed to topic'; }

wait_for '飛控 EKF 對準' 90 '看 docker compose logs simulator' ekf_ready
wait_for '飛控連線' 60 '看 docker compose logs fc_adapter' fcu_connected
if [[ $VISION == 1 ]]; then
  wait_for '偵測' 300 '模型載入失敗？看 docker compose logs object_detector' has_topic /detections
fi

if [[ $RECORD == 1 ]]; then
  stop_recorder() {
    docker compose --profile record stop recorder > /dev/null 2>&1 || true     # SIGINT：把 .mcap 收尾
    echo "rosbag 在 logs/bag/$RUN/（用 Foxglove 開裡面的 .mcap）"
  }
  trap stop_recorder EXIT
  BAG_NAME=$RUN docker compose --profile record up -d --force-recreate recorder
  wait_for '開始錄 rosbag' 30 '看 docker compose logs recorder' recording
fi

# 相機、偵測的就緒由行為樹的 WaitReady 再確認一次
docker compose --profile mission run --rm mission mission "$TREE" --no-checklist --run-name "$RUN" "$@"
