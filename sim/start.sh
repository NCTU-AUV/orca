#!/usr/bin/env bash
# 依序啟動 gz sim（與選用的 GUI）→ ardusub → mavproxy。
# server、ardusub、mavproxy 任何一個結束，整個容器就結束；關掉 GUI 視窗不影響模擬。
set -euo pipefail

# ---- 可由 compose 環境變數覆寫 ----
WORLD="${WORLD:-orca_pool.sdf}"   # /sim/worlds 底下的世界檔（隨機佈局是 generated/<世界>_seed<N>.sdf）
GUI="${GUI:-1}"                   # 1 = 開 Gazebo 視窗，0 = 只跑 server
UNDERWATER="${UNDERWATER:-1}"     # 0 = 不要水下的樣子（水面、水體霧），回到沒有水的乾淨畫面
QGC_PORT="${QGC_PORT:-14550}"     # mavproxy → 主機上的 QGroundControl
MAVROS_PORT="${MAVROS_PORT:-14551}" # mavproxy → mavros（adapter 服務）
LOG_DIR="${LOG_DIR:-/logs}"

core=()
all=()
cleanup() { kill "${all[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT INT TERM

# 1. Gazebo server（-r：載入後立刻開始跑）。
#    不開 GUI 時相機用 EGL 算圖（--headless-rendering），不需要 X；世界裡沒有相機時沒有影響
server_args=()
if [[ "${GUI}" != "1" ]]; then
  server_args+=(--headless-rendering)
fi
# UNDERWATER=0：把世界檔裡 UNDERWATER:BEGIN 到 UNDERWATER:END 之間的那一段（model://water 與
# particle 的 plugin）拿掉。世界檔是唯讀掛進來的，改過的那份放在 /tmp。沒有這兩個標記的世界檔不受影響。
# 相機影像的水下後處理是另一回事，在 sim_cameras 服務（gz_camera_bridge.py），吃同一個 UNDERWATER。
world_file="/sim/worlds/${WORLD}"
if [[ "${UNDERWATER}" == "0" ]]; then
  sed '/UNDERWATER:BEGIN/,/UNDERWATER:END/d' "${world_file}" > "/tmp/$(basename "${WORLD}")"
  world_file="/tmp/$(basename "${WORLD}")"
fi
gz sim -s -r -v 3 "${server_args[@]}" "${world_file}" &
core+=($!); all+=($!)

if [[ "${GUI}" == "1" ]]; then
  gz sim -g -v 2 &
  all+=($!)
fi

# 2. ArduSub SITL：物理來自 Gazebo（JSON over UDP 9002），MAVLink 在 TCP 5760。
#    -w 每次開機清掉 eeprom，參數一律從 firmware/ardusub/sitl.parm 重新載入。
mkdir -p "${LOG_DIR}/ardusub"
(cd "${LOG_DIR}/ardusub" && exec ardusub -S -w --model JSON --speedup 1 \
    --defaults "${PARAM_FILE:-/params/sitl.parm}" -I0 > ardusub.out 2>&1) &
core+=($!); all+=($!)

# 3. mavproxy：角色同實機 RPi 上的 mavproxy.service，把 MAVLink 分送給 QGC 與 mavros
mkdir -p "${LOG_DIR}/mavproxy"
(cd "${LOG_DIR}/mavproxy" && exec mavproxy.py --master=tcp:127.0.0.1:5760 \
    --out="udp:127.0.0.1:${QGC_PORT}" --out="udp:127.0.0.1:${MAVROS_PORT}" \
    --daemon --non-interactive) &
core+=($!); all+=($!)

wait -n "${core[@]}"
