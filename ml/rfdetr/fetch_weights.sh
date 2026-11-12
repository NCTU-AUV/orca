#!/usr/bin/env bash
# 下載 RF-DETR 權重到 ml/rfdetr/weights/，並檢查 sha256。detector 容器把這個目錄掛成 /model。
set -euo pipefail
cd "$(dirname "$0")"
URL=https://github.com/NCTU-AUV/orca/releases/download/rfdetr-v2/checkpoint_best_total.pth
SHA256=0c8eccf88c103515d8616087738252aef2e6c69e9a1ff06bb184a28adba9c2c8
OUT=weights/checkpoint_best_total.pth

mkdir -p weights
if [ -f "$OUT" ] && echo "$SHA256  $OUT" | sha256sum -c --quiet 2>/dev/null; then
  echo "already up to date: $OUT"; exit 0
fi
curl -fL --progress-bar -o "$OUT.part" "$URL"
echo "$SHA256  $OUT.part" | sha256sum -c --quiet
mv "$OUT.part" "$OUT"
echo "ok: $OUT"
