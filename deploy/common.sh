# deploy/<跑法>/ 的腳本共用的函式，用 source 載入（目前目錄要是 deploy/<跑法>/）：
#   source ../common.sh
#
# 場次（session）= 一次下水的全部紀錄，從 ./up.sh 開始到 ./down.sh 結束。目前只有 deploy/laptop 用
#（模擬還是一次任務錄一個 bag，見 docs/sim/README.md），寫成跟跑法無關，之後模擬也能直接用。
# 用到這些函式的 compose 要有 fc_adapter、recorder、marker_log 服務，並把 ./logs/session 掛在 /sessions。
#
#   logs/session/current             目前場次的名稱；down.sh 收尾後刪掉，還在就表示上一個場次沒收尾
#   logs/session/<場次>/             名稱是開始時間 YYYYmmdd_HHMMSS
#     info/start.txt                 開始時間、主機、開機 ID（判斷中間有沒有重開機）
#     info/git.txt、info/git.diff    開始時的 commit、git status、未提交的修改
#     info/fc_params_start.parm      開始時的飛控參數；fc_params_end.parm 是結束時的
#     info/closed.txt                怎麼收尾的（down.sh，或下一次 up.sh 補救）
#     bag_<時間>/                    全程 rosbag：recorder 每次啟動一個資料夾，每 5 分鐘切一個 .mcap
#     markers.csv                    標記（時間、來源、內容），同樣的標記也錄在 bag 的 /session/marker
#     mission/、pool_tests/          行為樹、水池測試腳本的 CSV
#     containers.log                 各容器的終端輸出（down.sh 存的）
#     slices/                        ./slice.sh 切出來的片段

SESSION_ROOT=logs/session

# ---------- 容器 ----------

# service_running <服務>：這個 compose 服務有沒有在跑
service_running() {
  docker compose ps --status running --services 2> /dev/null | grep -qx "$1"
}

# in_adapter <指令...>：在 fc_adapter 容器裡執行（/entrypoint.sh 會載入 ROS 環境）
in_adapter() { docker compose exec -T fc_adapter /entrypoint.sh "$@" 2> /dev/null; }

# wait_for <說明> <最多等幾秒> <沒等到時的提示> <成功條件（指令）...>
wait_for() {
  local what=$1 limit=$2 hint=$3 start=$SECONDS
  shift 3
  printf '等%s' "$what"
  until "$@"; do
    if (( SECONDS - start >= limit )); then
      printf '\n%s 秒內沒等到%s。%s\n' "$limit" "$what" "$hint" >&2
      exit 1
    fi
    printf '.'
    sleep 2
  done
  printf ' 好了\n'
}

fcu_connected() {
  local state
  state=$(in_adapter timeout 5 ros2 topic echo --once /mavros/state) || return 1
  [[ $state == *'connected: true'* ]]
}
has_topic() { in_adapter timeout 5 ros2 topic echo --once "$1" > /dev/null; }

# ---------- 場次 ----------

# logs/ 是容器（root）建的，主機上的使用者寫不進去。第一次用時借一個容器，把 logs/、logs/session/ 改成自己的
#（裡面已有的檔案不動）
session_prepare() {
  [[ -d $SESSION_ROOT && -w $SESSION_ROOT ]] && return 0
  docker run --rm --network none --entrypoint sh -v "$PWD/logs:/logs" orca-base \
    -c "mkdir -p /logs/session && chown $(id -u):$(id -g) /logs /logs/session"
}

session_current() { cat "$SESSION_ROOT/current" 2> /dev/null || true; }

# 最近的一個場次（不管有沒有收尾）
session_latest() {
  find "$SESSION_ROOT" -mindepth 1 -maxdepth 1 -type d -regextype egrep -regex '.*/[0-9]{8}_[0-9]{6}' \
    -printf '%f\n' 2> /dev/null | sort | tail -n 1
}

boot_id() { cat /proc/sys/kernel/random/boot_id; }

# session_boot <場次>：這個場次開始時的開機 ID
session_boot() { sed -n 's/^boot_id=//p' "$SESSION_ROOT/$1/info/start.txt" 2> /dev/null; }

# 程式版本，給標記用（容器裡沒有 .git，由主機算好用 ORCA_GIT 傳進去）：
# commit 前 7 碼；追蹤中的檔案有修改，或 src/、tools/ 有還沒加入 git 的檔案，就加上 +dirty
session_git_version() {
  local root=../.. v
  v=$(git -C "$root" rev-parse --short=7 HEAD 2> /dev/null) || { echo 'git 版本未知'; return; }
  if ! git -C "$root" diff --quiet HEAD -- \
     || [[ -n $(git -C "$root" ls-files --others --exclude-standard -- src tools) ]]; then
    v+='+dirty'
  fi
  echo "$v"
}

# 開新場次：建資料夾、記開始資訊與程式版本、寫 current。印出場次名稱
session_new() {
  local name dir root=../..
  name=$(date +%Y%m%d_%H%M%S)
  dir=$SESSION_ROOT/$name
  mkdir -p "$dir/info"
  {
    echo "start=$(date '+%Y-%m-%d %H:%M:%S')"
    echo "host=$(hostname)"
    echo "boot_id=$(boot_id)"
  } > "$dir/info/start.txt"
  {
    echo "commit: $(git -C "$root" rev-parse HEAD 2> /dev/null || echo 未知)"
    echo "branch: $(git -C "$root" rev-parse --abbrev-ref HEAD 2> /dev/null || echo 未知)"
    echo "version: $(session_git_version)"
    echo
    echo '# git status --porcelain'
    git -C "$root" status --porcelain 2> /dev/null || true
  } > "$dir/info/git.txt"
  git -C "$root" diff HEAD > "$dir/info/git.diff" 2> /dev/null || true
  echo "$name" > "$SESSION_ROOT/current.tmp"
  mv "$SESSION_ROOT/current.tmp" "$SESSION_ROOT/current"
  echo "$name"
}

# session_close <場次> <怎麼收尾的>：記下收尾方式；它是目前場次的話刪掉 current
session_close() {
  echo "$(date '+%Y-%m-%d %H:%M:%S') $2" >> "$SESSION_ROOT/$1/info/closed.txt"
  if [[ $(session_current) == "$1" ]]; then
    rm -f "$SESSION_ROOT/current"
  fi
}

# session_mark <來源> <內容>：發一筆標記（要有 marker_log 在跑）
session_mark() {
  docker compose exec -T marker_log /entrypoint.sh ros2 run orca_session mark --source "$1" -- "$2"
}

# session_fc_params <場次> <檔名> [說明]：飛控參數快照存到 <場次>/info/<檔名>（要有 fc_adapter、飛控連上）
session_fc_params() {
  docker compose exec -T fc_adapter /entrypoint.sh \
    ros2 run orca_session fc_params "/sessions/$1/info/$2" --note "${3:-}"
}

# session_recording <場次>：recorder 在跑，而且這個場次已經有 .mcap
session_recording() {
  service_running recorder && compgen -G "$SESSION_ROOT/$1/bag_*/*.mcap" > /dev/null
}

# session_check_bags <場次>：每個 bag_* 都要有 metadata.yaml（ros2 bag record 正常結束時才寫）。
# 回傳沒收尾的數量
session_check_bags() {
  local d bad=0
  for d in "$SESSION_ROOT/$1"/bag_*/; do
    [[ -d $d ]] || continue
    if [[ ! -f $d/metadata.yaml ]]; then
      echo "  沒有正常收尾：$d（最後一個 .mcap 可能少了結尾，Foxglove 可能打不開）" >&2
      bad=$((bad + 1))
    fi
  done
  return "$bad"
}
