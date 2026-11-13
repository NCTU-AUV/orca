#!/usr/bin/env bash
# 一個指令在模擬裡跑過門（舊版 mission/gate_pass.py；行為樹版見 mission.sh）：Gazebo（有門和相機的世界）+ ArduSub SITL + fc_adapter + 相機 + object_detector
# → 等就緒 → 開始 mission/gate_pass.py。
#
#   ./gate_pass.sh [gate_pass.py 的參數，例如 --pass-s 6]
#   GUI=0 ./gate_pass.sh                                   # 不開 Gazebo 視窗
#   WORLD=orca_gate.sdf ./gate_pass.sh --depth 0.9         # 導航門（立在池底，要潛深一點）
#
# - 預設世界 orca_qgate.sdf：資格賽門，在船頭前方 6 m、偏右 1 m。
# - 每次執行都會重開 simulator，船從水面、原點開始；fc_adapter、sim_cameras、object_detector 沿用已經在跑的。
# - 第一次會下載模型權重、build image，要等比較久。
# - 不會等你按 Enter（模擬不需要實機的檢查清單）。任何時候 Ctrl-C：上鎖，船浮上來。
# - 跑完容器繼續跑，要再跑一次直接重新執行；全部關掉：docker compose --profile vision down
# - 跟實機的 deploy/laptop 都綁 UDP 14551，不能同時跑。
set -euo pipefail
cd "$(dirname "$0")"
export WORLD=${WORLD:-orca_qgate.sdf} GUI=${GUI:-1}

if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ../../ml/rfdetr/fetch_weights.sh
fi
if [[ $GUI == 1 ]]; then
  xhost +SI:localuser:root > /dev/null
fi

docker compose --profile vision up -d fc_adapter sim_cameras object_detector
docker compose up -d --force-recreate simulator

in_bridge() { docker compose exec -T fc_adapter /entrypoint.sh "$@" 2>/dev/null; }

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
  state=$(in_bridge timeout 5 ros2 topic echo --once /mavros/state) || return 1
  [[ $state == *'connected: true'* ]]
}
has_topic() { in_bridge timeout 5 ros2 topic echo --once "$1" > /dev/null; }

wait_for '飛控 EKF 對準' 90 '看 docker compose logs simulator' ekf_ready
wait_for '飛控連線' 60 '看 docker compose logs fc_adapter' fcu_connected
wait_for '相機' 60 "世界檔（$WORLD）有相機嗎？看 docker compose logs sim_cameras" \
  has_topic /camera/camera/color/camera_info
wait_for '偵測' 300 '模型載入失敗？看 docker compose logs object_detector' has_topic /detections

exec docker compose exec fc_adapter /entrypoint.sh python3 /orca/mission/gate_pass.py --no-checklist "$@"
