#!/usr/bin/env bash
# 在 RPi 上用 sudo 執行：mavproxy 多開一個 udpin:0.0.0.0:14560 給筆電的 mavros。
# udpin = mavproxy 被動等待，誰先送封包來（筆電的 mavros）就回送給誰。
# 原本的兩個輸出（14570 給 bcast-relay/QGC、14551 給 RPi 本機 mavros）保留。
#
#   scp mavproxy-udpin.sh rpi: && ssh -t rpi sudo bash mavproxy-udpin.sh
set -euo pipefail
UNIT=/etc/systemd/system/mavproxy.service

if grep -q 'udpin:0.0.0.0:14560' "$UNIT"; then
  echo "already configured"; exit 0
fi
cp "$UNIT" "$UNIT.bak-$(date +%Y%m%d%H%M%S)"
# 在 --out=udp:127.0.0.1:14551 那行後面插一行
sed -i 's|^\(\s*--out=udp:127.0.0.1:14551\) \\$|\1 \\\n  --out=udpin:0.0.0.0:14560 \\|' "$UNIT"
grep -q 'udpin:0.0.0.0:14560' "$UNIT" || { echo "edit failed, unit unchanged pattern"; exit 1; }
systemctl daemon-reload
systemctl restart mavproxy.service
sleep 2
systemctl --no-pager status mavproxy.service | head -12
