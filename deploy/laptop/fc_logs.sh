#!/usr/bin/env bash
# 飛控 SD 卡上的 DataFlash（.BIN）：列出、拉回場次資料夾、算時間對照、轉成帶 ROS 時間的 CSV。
# 平常 ./down.sh 會做 fetch 和 time，這支用在 down.sh 沒拉成功、補救沒收尾的場次，或要轉 CSV 時。
#
#   ./fc_logs.sh list                          列出飛控上的 .BIN（編號 大小）
#   ./fc_logs.sh fetch [場次] [編號...]          拉到 <場次>/fc_logs/，接著重算時間對照。沒給編號就拉
#                                              這個場次新增或變大的檔（down.sh 第 4 步做的事）
#   ./fc_logs.sh time [場次]                    從 bag 重算飛控開機時間 → ROS 時間的對照（info/fc_time.csv）
#   ./fc_logs.sh csv [場次] [bin_time.py 的參數]  把 fc_logs/*.BIN 轉成 fc_logs/<編號>/<類型>.csv（加上 ros_time）
#                                              例如 --types ATT,RCOU、--types all、--bin 00000014.BIN
#
# - 沒給場次就用目前的場次，沒有的話用最近的一個。
# - fetch 要飛控上鎖、RPi 連得上，RPi 上要有 fc_logs.py（deploy/rpi-relay/deploy.sh fc-logs）。
# - csv 在 orca-sim 映像裡跑（要 pymavlink），沒有的話先在 deploy/sim build。
set -euo pipefail
cd "$(dirname "$0")"
source ../common.sh

usage() { sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//' >&2; exit 2; }

cmd=${1:-}
[[ -n $cmd ]] || usage
shift
if [[ $cmd != list ]]; then
  if [[ ${1:-} =~ ^[0-9]{8}_[0-9]{6}$ ]]; then
    SESSION=$1
    shift
  else
    SESSION=$(session_current)
    SESSION=${SESSION:-$(session_latest)}
  fi
  if [[ -z $SESSION || ! -d $SESSION_ROOT/$SESSION ]]; then
    echo "找不到場次 ${SESSION:-（沒有任何場次）}" >&2
    exit 1
  fi
  echo "場次 $SESSION"
fi

case $cmd in
  list)
    rpi_fc_logs list
    ;;
  fetch)
    status=0
    session_fc_logs_fetch "$SESSION" "$@" || status=1
    session_fc_time "$SESSION" || status=1
    exit "$status"
    ;;
  time)
    session_fc_time "$SESSION"
    ;;
  csv)
    if ! docker image inspect orca-sim > /dev/null 2>&1; then
      echo '沒有 orca-sim 映像（要它的 pymavlink），先在 deploy/sim 跑一次 ./run.sh 把它 build 出來' >&2
      exit 1
    fi
    docker run --rm --network none -u "$(id -u):$(id -g)" \
      -v "$PWD/$SESSION_ROOT:/sessions" -v "$PWD/../../tools:/orca/tools:ro" \
      --entrypoint python3 orca-sim -I /orca/tools/fc_logs/bin_time.py "/sessions/$SESSION" "$@"
    ;;
  *)
    usage
    ;;
esac
