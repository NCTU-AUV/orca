#!/usr/bin/env bash
# 起整個模擬：授權容器連 X → 檢查 NVIDIA runtime → docker compose up
# 環境變數：GUI=0 不開 Gazebo 視窗；WORLD=<檔名> 換世界檔（repo 的 sim/worlds 底下）；UNDERWATER=0 關掉水下效果
# 世界檔有相機（model://orca_cam）時，連相機和偵測（vision profile）一起起。其他參數交給 docker compose up，
# 例如 ./run.sh -d（背景執行）、./run.sh -d --force-recreate（重開，船回到起點）
set -euo pipefail
cd "$(dirname "$0")"
export WORLD=${WORLD:-orca_pool.sdf}
# 映像的 build 指紋（orca.build-hash label）：跟 deploy/laptop 共用映像，laptop 的 up.sh 靠它判斷要不要重 build
source ../../docker/build-hash.sh

if [[ "${GUI:-1}" == "1" ]]; then
  xhost +SI:localuser:root >/dev/null
fi

# compose 的 gpus: all 靠 NVIDIA Container Toolkit：註冊成 Docker 的 nvidia runtime，或只裝了它的 hook 都可以
if ! docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"' && ! command -v nvidia-container-runtime-hook >/dev/null; then
  cat >&2 <<'EOF'
找不到 Docker 的 nvidia runtime。請安裝 NVIDIA Container Toolkit 並設定 Docker：
  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
  sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
EOF
  exit 1
fi

PROFILES=()
if grep -q 'model://orca_cam' "../../sim/worlds/$WORLD"; then
  PROFILES=(--profile vision)
  if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
    ../../ml/rfdetr/fetch_weights.sh
  fi
fi

exec docker compose "${PROFILES[@]}" up --build "$@"
