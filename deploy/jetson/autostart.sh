#!/usr/bin/env bash
# Jetson 開機自動起服務（相機、偵測、飛控、標記、foxglove），用 systemd 的使用者服務 orca.service。在 Jetson 上跑。
#
#   ./autostart.sh enable     # 安裝並啟用（不用 sudo）
#   ./autostart.sh disable    # 停用（不關掉已經在跑的容器）
#   ./autostart.sh status
#
# - 使用者服務在沒登入時也會跑，前提是這個帳號開了 linger（loginctl show-user $USER -p Linger 要是 yes）。
# - 開機只起服務、不開場次、不錄影：場次從 ./up.sh（./mission.sh 會先跑它）開始。開機就起好，
#   ssh 進來時相機、偵測（TensorRT 引擎已載入）、飛控連線都已經就緒，./up.sh 不用再等。
# - 映像、TensorRT 引擎要先用 ./up.sh 準備好；開機時不 build、不轉引擎。
set -euo pipefail
cd "$(dirname "$0")"
UNIT=orca.service
UNIT_FILE=$HOME/.config/systemd/user/$UNIT
SERVICES='camera object_detector fc_adapter marker_log foxglove'

case "${1:-}" in
  enable)
    if [[ $(loginctl show-user "$USER" -p Linger --value 2> /dev/null) != yes ]]; then
      echo "這個帳號沒開 linger，沒登入時使用者服務不會跑。要先：sudo loginctl enable-linger $USER" >&2
      exit 1
    fi
    mkdir -p "$(dirname "$UNIT_FILE")"
    cat > "$UNIT_FILE" << EOF
# 由 $PWD/autostart.sh 產生
[Unit]
Description=Orca：開機起相機、偵測、飛控（deploy/jetson）

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=$PWD
# 使用者服務不能直接排在 docker.service 後面，等 docker 能用。
# sg docker：使用者的 systemd 是開機時啟動的，setup.sh 把帳號加進 docker 群組之後、重開機之前，
# 它沒有 docker 群組，直接跑 docker 會沒有權限
ExecStartPre=/usr/bin/sg docker -c 'until docker info > /dev/null 2>&1; do sleep 2; done'
ExecStart=/usr/bin/sg docker -c 'docker compose up -d --no-build $SERVICES'
TimeoutStartSec=300

[Install]
WantedBy=default.target
EOF
    systemctl --user daemon-reload
    systemctl --user enable "$UNIT"
    echo "已啟用：下次開機自動起 $SERVICES"
    ;;
  disable)
    systemctl --user disable "$UNIT" 2> /dev/null || true
    echo '已停用（已經在跑的容器不動）'
    ;;
  status)
    systemctl --user status "$UNIT" --no-pager || true
    ;;
  *)
    echo "用法：$0 enable|disable|status" >&2
    exit 2
    ;;
esac
