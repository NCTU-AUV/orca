#!/usr/bin/env bash
# Jetson 的一次性設定（要 sudo，在 Jetson 上跑）：裝 Docker、讓容器用得到 GPU、讓使用者不用 sudo 就能跑 docker、
# 功耗模式 MAXN 並在開機時固定最高時脈。
#
#   ssh -t orin workspace/orca/deploy/jetson/setup.sh      # 從筆電跑；-t 才能輸入 sudo 密碼
#
# 可以重複跑：已經做過的步驟會略過或覆蓋成一樣的結果。
#
# - Docker 固定在 27.5.1，並用 apt-mark hold 擋住升級：JetPack 6 的 kernel（5.15-tegra）沒有 iptable_raw 模組，
#   Docker 28 起的版本會用到它，在這個 kernel 上可能起不來。要升級先在 Jetson 上試過。
# - GPU：JetPack 已經裝了 nvidia-container-toolkit，這裡只把 nvidia runtime 登記到 Docker
#  （/etc/docker/daemon.json）。compose 的服務用 runtime: nvidia 指定。
# - 把使用者加進 docker 群組（等於 root 權限）：deploy/jetson 的腳本要從筆電經 ssh 直接跑 docker。
#   這台本來就是 sudo 群組，不會多給權限。重新登入後生效。
# - 功耗模式 MAXN、開機跑 jetson_clocks（systemd 服務 jetson-clocks.service）：25W 模式把 GPU 鎖在 408 MHz，
#   偵測每幀約 60 ms；MAXN＋jetson_clocks（918 MHz）約 28～40 ms（ml/rfdetr/README.md、docs/jetson.md）。
#   nvpmodel 的模式重開機會保留，jetson_clocks 不會，所以做成開機服務。
set -euo pipefail

DOCKER_VERSION='5:27.5.1-1~ubuntu.22.04~jammy'

if [[ ! -e /etc/nv_tegra_release ]]; then
  echo '這不是 Jetson（沒有 /etc/nv_tegra_release），不做' >&2
  exit 1
fi

echo '== 1. 安裝 Docker（固定 27.5.1）'
if [[ ! -f /etc/apt/sources.list.d/docker.sources && ! -f /etc/apt/sources.list.d/docker.list ]]; then
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null << 'EOF'
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: jammy
Components: stable
Architectures: arm64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
fi
sudo apt-get update
sudo apt-mark unhold docker-ce docker-ce-cli > /dev/null 2>&1 || true
sudo apt-get install -y --allow-downgrades \
  "docker-ce=$DOCKER_VERSION" "docker-ce-cli=$DOCKER_VERSION" \
  containerd.io docker-buildx-plugin docker-compose-plugin
sudo apt-mark hold docker-ce docker-ce-cli

echo '== 2. 登記 nvidia runtime'
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl enable docker containerd
sudo systemctl restart docker

echo "== 3. $USER 加進 docker 群組"
sudo usermod -aG docker "$USER"

echo '== 4. 功耗模式 MAXN、開機跑 jetson_clocks'
if [[ $(nvpmodel -q 2> /dev/null | sed -n 2p) != 0 ]]; then
  # 換模式時 nvpmodel 可能問要不要重開機：回 NO，最後再提醒
  echo NO | sudo nvpmodel -m 0 || true
  NEED_REBOOT=1
fi
sudo tee /etc/systemd/system/jetson-clocks.service > /dev/null << 'EOF'
# 由 orca 的 deploy/jetson/setup.sh 產生：CPU、GPU、記憶體時脈固定在目前功耗模式的最高值（偵測要用）
[Unit]
Description=jetson_clocks（orca）
After=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now jetson-clocks.service

echo '== 5. 檢查'
sudo docker version --format 'Docker {{.Server.Version}}'
sudo docker info --format '{{json .Runtimes}}' | grep -q nvidia && echo 'nvidia runtime：有'
sudo docker run --rm hello-world > /dev/null && echo '跑容器：可以'
echo "功耗模式：$(nvpmodel -q 2> /dev/null | head -n 1)；GPU $(( $(cat /sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq) / 1000000 )) MHz"
if [[ ${NEED_REBOOT:-0} == 1 ]]; then
  echo '功耗模式換過了，重開機後才完全生效：sudo reboot'
fi

cat << EOF

完成。重新登入（或重開 ssh）之後，$USER 不用 sudo 就能跑 docker。
接著在 Jetson 上：cd ~/workspace/orca/deploy/jetson && ./up.sh
EOF
