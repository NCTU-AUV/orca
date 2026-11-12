#!/usr/bin/env bash
# 起整個模擬：授權容器連 X → 檢查 NVIDIA runtime → docker compose up
# 環境變數：GUI=0 不開 Gazebo 視窗；WORLD=<檔名> 換世界檔（repo 的 sim/worlds 底下）
set -euo pipefail
cd "$(dirname "$0")"

if [[ "${GUI:-1}" == "1" ]]; then
  xhost +SI:localuser:root >/dev/null
fi

if ! docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"'; then
  cat >&2 <<'EOF'
找不到 Docker 的 nvidia runtime。請安裝 NVIDIA Container Toolkit 並設定 Docker：
  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
  sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
EOF
  exit 1
fi

exec docker compose up --build "$@"
