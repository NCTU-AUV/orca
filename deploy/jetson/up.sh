#!/usr/bin/env bash
# Jetson：開場次（或沿用）、起服務、開始全程錄影，等飛控、相機、偵測、錄影就緒。在 Jetson 上跑。
# 起的服務：camera、object_detector、fc_adapter、recorder（全程 rosbag）、marker_log（標記）、foxglove。
# 映像不存在或過期才 build；沒重 build 的話，已經在跑的容器不動（不重開，mavros 的連線不會斷）。
#
#   ./up.sh            # 自動判斷要不要 build
#   ./up.sh --build    # 不管指紋，全部重 build
#
# - 場次的規則（沿用、補救沒收尾的上一個場次）同 deploy/laptop/up.sh，寫在 ../common.sh 的 session_begin。
# - 偵測模型：要有 ml/rfdetr/weights/rfdetr-large.onnx（筆電上 ml/rfdetr/export_onnx.sh 匯出，sync.sh 帶過來）。
#   這台還沒有對應的 TensorRT 引擎就先轉（幾分鐘），轉完用 docs/sim/reference 的影像跟筆電上 PyTorch 版的
#   結果比對（ml/rfdetr/check_export.py），對不上就停下來，不會用一個錯的引擎下水。
# - 第一次 build 要下載 NVIDIA 的 l4t-tensorrt（約 1.5 GB）和 ROS 套件，Jetson 要能上網。
# - 飛控的 .BIN 還不會自動拉（deploy/laptop 是經 RPi 拉的），之後做。
set -euo pipefail
cd "$(dirname "$0")"
source ../../docker/build-hash.sh
source ../common.sh

SERVICES=(camera object_detector fc_adapter recorder marker_log foxglove)
WEIGHTS=../../ml/rfdetr/weights

FORCE=0
case "${1:-}" in
  '') ;;
  --build) FORCE=1 ;;
  *) echo "用法：$0 [--build]" >&2; exit 2 ;;
esac

for f in rfdetr-large.onnx rfdetr-large.json check_reference.json; do
  if [[ ! -f $WEIGHTS/$f ]]; then
    echo "沒有 ml/rfdetr/weights/$f：在筆電上跑 ml/rfdetr/export_onnx.sh，再 deploy/jetson/sync.sh" >&2
    exit 1
  fi
done

# ---------- 映像 ----------

RECREATE=0
if need_build orca-base "$ORCA_BASE_HASH"; then
  docker compose build fc_adapter                   # camera、fc_adapter、recorder、marker_log、foxglove、gui_tools 共用
  RECREATE=1
fi
if need_build orca-mission "$ORCA_MISSION_HASH"; then
  docker compose --profile mission build mission
fi
if need_build orca-perception-jetson "$ORCA_PERCEPTION_JETSON_HASH"; then
  docker compose build object_detector
  RECREATE=1
fi

# ---------- TensorRT 引擎 ----------

# in_detector_image <指令...>：在偵測的映像裡跑一次性的指令（不影響在跑的 object_detector）
in_detector_image() { docker compose run --rm --no-deps -T object_detector "$@" < /dev/null; }

# build_engine 的進度照樣印出來（tee 到 stderr），最後一行是引擎路徑
ENGINE=$(in_detector_image ros2 run orca_perception build_engine /model/rfdetr-large.onnx | tee /dev/stderr | tail -n 1)
CHECKED=$WEIGHTS/$(basename "$ENGINE").checked
if [[ ! -f $CHECKED ]]; then
  echo "驗證 TensorRT 引擎（跟筆電上 PyTorch 版的結果比對）"
  if ! in_detector_image python3 /rfdetr/check_export.py trt "$ENGINE" --ref /model/check_reference.json; then
    echo "TensorRT 引擎的結果跟 PyTorch 版對不上（見上面）。不要用這個引擎下水；" \
      "刪掉 ml/rfdetr/weights/$(basename "$ENGINE") 重轉，或回報問題" >&2
    exit 1
  fi
  date '+%Y-%m-%d %H:%M:%S 驗證通過' > "$CHECKED"
fi

# ---------- 場次 ----------

session_begin

if (( RECREATE )); then
  docker compose up -d "${SERVICES[@]}"             # 映像或設定換了的容器重開，其他不動
else
  docker compose up -d --no-recreate "${SERVICES[@]}"
fi

wait_for '飛控連線' 60 '飛控的 USB 插好了嗎（ls /dev/serial/by-id/）？看 docker compose logs fc_adapter' fcu_connected
wait_for '相機' 60 'D435i 插好了嗎？有沒有別的程式佔著（hw_check 的面板、~/rfdetr_jetson）？看 docker compose logs camera' \
  has_topic /camera/camera/color/camera_info
wait_for '偵測' 120 '看 docker compose logs object_detector' has_topic /detections
wait_for '開始錄影' 30 '看 docker compose logs recorder' session_recording "$SESSION"

session_after_ready
if (( NEW )); then
  session_mark up.sh "場次 $SESSION 開始（$(session_git_version)）" > /dev/null \
    || echo '「場次開始」的標記沒發出去，看 docker compose logs marker_log' >&2
fi
echo "場次 $SESSION 錄影中。任務用 ./mission.sh，收工用 ./down.sh"
