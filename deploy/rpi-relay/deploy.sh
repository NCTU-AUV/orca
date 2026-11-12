#!/usr/bin/env bash
# 在筆電執行：把 cam_relay 部署到 RPi，並改成開機自動跑 cam-relay（取代 realsense-web）。
# 不需要 sudo。venv 已存在就不重建。
#
#   ./deploy.sh           部署並啟用 cam-relay
#   ./deploy.sh revert    改回開機跑 realsense-web
set -euo pipefail
HOST=${RPI_HOST:-rpi}
DIR="$(cd "$(dirname "$0")" && pwd)"

if [ "${1:-}" = revert ]; then
  ssh "$HOST" 'systemctl --user disable --now cam-relay.service; systemctl --user enable --now realsense-web.service'
  exit 0
fi

ssh "$HOST" 'mkdir -p ~/workspace/cam-relay ~/.config/systemd/user'
scp -q "$DIR/cam_relay.py" "$HOST:workspace/cam-relay/"
scp -q "$DIR/cam-relay.service" "$HOST:.config/systemd/user/"
ssh "$HOST" bash -s <<'EOF'
set -e
cd ~/workspace/cam-relay
if [ ! -x .venv/bin/python ]; then
  python3 -m venv .venv
  .venv/bin/pip install -q pyrealsense2==2.59.1.11053 numpy opencv-python-headless
fi
systemctl --user daemon-reload
systemctl --user disable realsense-web.service 2>/dev/null || true
systemctl --user enable --now cam-relay.service
sleep 4
systemctl --user --no-pager status cam-relay.service | head -15
EOF
