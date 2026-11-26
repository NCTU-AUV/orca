# deploy/<跑法>/ 的腳本共用的函式，用 source 載入（目前目錄要是 deploy/<跑法>/）：
#   source ../common.sh
#
# 場次（session）= 一次下水的全部紀錄，從 ./up.sh 開始到 ./down.sh 結束。deploy/laptop、deploy/jetson 用
#（模擬還是一次任務錄一個 bag，見 docs/sim/README.md），寫成跟跑法無關，之後模擬也能直接用。
# 用到這些函式的 compose 要有 fc_adapter、recorder、marker_log 服務，並把 ./logs/session 掛在 /sessions。
#
#   logs/session/current             目前場次的名稱；down.sh 收尾後刪掉，還在就表示上一個場次沒收尾
#   logs/session/<場次>/             名稱是開始時間 YYYYmmdd_HHMMSS
#     info/start.txt                 開始時間、主機、開機 ID（判斷中間有沒有重開機）
#     info/git.txt、info/git.diff    開始時的 commit、git status、未提交的修改
#     info/fc_params_start.parm      開始時的飛控參數；fc_params_end.parm 是結束時的
#     info/fc_logs_start.txt         開始時飛控 SD 卡上的 .BIN（每行「編號 大小」）；fc_logs_end.txt 是拉檔時的
#     info/fc_time.csv               飛控開機時間 → ROS 時間的對照（orca_session fc_time，從 bag 算）
#     info/closed.txt                怎麼收尾的（down.sh，或下一次 up.sh 補救）
#     bag_<時間>/                    全程 rosbag：recorder 每次啟動一個資料夾，每 5 分鐘切一個 .mcap
#     markers.csv                    標記（時間、來源、內容），同樣的標記也錄在 bag 的 /session/marker
#     mission/、pool_tests/          行為樹、水池測試腳本的 CSV
#     fc_logs/<編號>.BIN             這個場次的飛控 DataFlash（deploy/laptop 的 down.sh 經 RPi 拉回來的；Jetson 還沒做）
#     fc_logs/<編號>/<類型>.csv      .BIN 轉成的 CSV，加上 ros_time（./fc_logs.sh csv）
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
  state=$(in_adapter timeout 5 ros2 topic echo --once --qos-durability volatile /mavros/state) || return 1
  [[ $state == *'connected: true'* ]]
}
has_topic() { in_adapter timeout 5 ros2 topic echo --once "$1" > /dev/null; }

# ---------- build ----------

# need_build <映像> <指紋>：要 build 就印原因並回傳 0。FORCE=1（up.sh --build）時一律 build。
# 過期的判斷：映像的 orca.build-hash label 跟 docker/build-hash.sh 現在算的不同
need_build() {
  local image=$1 want=$2 have
  if (( ${FORCE:-0} )); then
    echo "$image：--build，重 build"
    return 0
  fi
  if ! have=$(docker image inspect -f '{{index .Config.Labels "orca.build-hash"}}' "$image" 2> /dev/null); then
    echo "$image：映像不存在，build"
    return 0
  fi
  if [[ $have != "$want" ]]; then
    echo "$image：build 設定檔或 src/ 的檔案清單有變（指紋 ${have:-無} → $want），重 build"
    return 0
  fi
  return 1
}

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

# session_begin：up.sh 用，決定這次的場次，設定全域變數 SESSION、PREV、NEW：
# - 有場次而且 recorder 還在錄、沒重開過機 → 沿用（NEW=0）
# - 有場次但 recorder 沒在跑、或中間重開過機 → 上一個場次（PREV）沒收尾：停錄影、確認 bag，開新場次
# - 沒有場次 → 開新場次（NEW=1）
# 上一個場次結束時的參數快照要等飛控連上才拍（session_after_ready）
session_begin() {
  session_prepare
  SESSION=$(session_current)
  PREV=''
  NEW=0
  if [[ -n $SESSION ]]; then
    if service_running recorder && [[ $(session_boot "$SESSION") == "$(boot_id)" ]]; then
      echo "沿用場次 $SESSION"
    else
      PREV=$SESSION
      SESSION=''
    fi
  fi
  if [[ -n $PREV ]]; then
    cat >&2 << EOF
上一個場次 $PREV 沒有用 ./down.sh 收尾（直接 docker compose down、重開機或斷電）。
補做還能補的：停止錄影、確認 bag 收尾，飛控連上後拍結束時的參數快照。容器的終端輸出已經沒辦法補存。
EOF
    docker compose stop recorder marker_log > /dev/null 2>&1 || true
    session_check_bags "$PREV" || true
    session_close "$PREV" '沒有用 down.sh 收尾，由下一次 up.sh 補救：停止錄影；容器的終端輸出遺失'
  fi
  if [[ -z $SESSION ]]; then
    SESSION=$(session_new)
    NEW=1
    echo "新場次 $SESSION，紀錄在 $PWD/$SESSION_ROOT/$SESSION/"
  fi
}

# session_after_ready：up.sh 在服務就緒（飛控連上）之後呼叫：拍開始時的參數快照，
# 補拍上一個場次（PREV）結束時的參數快照
session_after_ready() {
  if [[ ! -f $SESSION_ROOT/$SESSION/info/fc_params_start.parm ]]; then
    session_fc_params "$SESSION" fc_params_start.parm \
      || echo '開始時的飛控參數快照沒拍到，下次 ./up.sh 會再試' >&2
  fi
  if [[ -n $PREV ]]; then
    if session_fc_params "$PREV" fc_params_end.parm \
         '（上一個場次沒收尾，由下一個場次的 up.sh 補拍；兩個場次之間改過參數的話不準）'; then
      session_close "$PREV" '補拍結束時的飛控參數快照（info/fc_params_end.parm）'
    fi
  fi
}

# ---------- 飛控的 DataFlash（.BIN）----------
# .BIN 在飛控的 SD 卡上。由 RPi 下載（deploy/rpi-relay/fc_logs.py，經 mavproxy 的 14551 用 MAVFTP），
# 再複製回 <場次>/fc_logs/。做法與實測數字見 docs/laptop-rpi-relay.md 的「紀錄」。

RPI_HOST=${RPI_HOST:-rpi}
RPI_FC_LOGS='~/mavproxy-venv/bin/python ~/workspace/fc-logs/fc_logs.py'
FC_LOG_RATE=400     # KiB/s，2026-11-19 實測約 420，估時間用

# rpi_ssh <指令>：在 RPi 上執行。-n：不讀標準輸入（不然在 while read 迴圈裡會吃掉迴圈的輸入）
rpi_ssh() { ssh -n -o BatchMode=yes -o ConnectTimeout=5 "$RPI_HOST" "$@"; }

# rpi_fc_logs <參數...>：在 RPi 上跑 fc_logs.py（list、get）
rpi_fc_logs() { rpi_ssh "$RPI_FC_LOGS $*"; }

# session_fc_logs_start <場次>：記下開始時飛控 SD 卡上有哪些 .BIN（info/fc_logs_start.txt）
session_fc_logs_start() {
  local list
  list=$(rpi_fc_logs list) || return 1
  printf '%s\n' "$list" > "$SESSION_ROOT/$1/info/fc_logs_start.txt"
}

# session_fc_logs_new <場次>：印出屬於這個場次的 .BIN（每行「編號 大小」），同時存成 info/fc_logs_end.txt。
# 屬於這個場次 = 開始時沒有的，或開始時就有但變大了：LOG_FILE_DSRMROT 是 0，飛控沒重開機的話，
# 再次解鎖會接著寫同一個檔（上鎖後檔案不關）
session_fc_logs_new() {
  local info=$SESSION_ROOT/$1/info now
  if [[ ! -f $info/fc_logs_start.txt ]]; then
    echo "  沒有 $info/fc_logs_start.txt（開場次時沒記到飛控的 log 清單），不知道哪些是這個場次的。" \
      "用 ./fc_logs.sh list 看編號，再 ./fc_logs.sh fetch $1 <編號>..." >&2
    return 1
  fi
  now=$(rpi_fc_logs list) || return 1
  printf '%s\n' "$now" > "$info/fc_logs_end.txt"
  awk 'NR == FNR { size[$1] = $2; next } !($1 in size) || $2 > size[$1]' \
    "$info/fc_logs_start.txt" "$info/fc_logs_end.txt"
}

# session_fc_logs_fetch <場次> [編號...]：把 .BIN 拉到 <場次>/fc_logs/。沒給編號就拉 session_fc_logs_new 的。
# 先在 RPi 上下載到 ~/workspace/fc-logs/tmp/<場次>/，複製回來、大小對得上再刪掉 RPi 上的
session_fc_logs_fetch() {
  local session=$1 dir=$SESSION_ROOT/$1/fc_logs tmp=workspace/fc-logs/tmp/$1 list nums=() total=0
  local num size secs f bad=0
  shift
  if (( $# )); then
    nums=("$@")
  else
    list=$(session_fc_logs_new "$session") || return 1
    while read -r num size; do
      [[ -n $num ]] || continue
      nums+=("$num")
      total=$((total + size))
    done <<< "$list"
    if (( ${#nums[@]} == 0 )); then
      echo '  這個場次沒有新的 .BIN（沒有解鎖過）'
      return 0
    fi
    echo "  ${#nums[@]} 個檔（${nums[*]}），共 $((total / 1048576)) MiB，估計 $((total / 1024 / FC_LOG_RATE / 60 + 1)) 分鐘內"
  fi
  mkdir -p "$dir"
  # RPi 每下載完一個檔印一行，這邊接著複製回來
  while read -r num size secs; do
    f=$(printf '%08d.BIN' "$num")
    if scp -q "$RPI_HOST:$tmp/$f" "$dir/$f" < /dev/null && [[ $(stat -c %s "$dir/$f") == "$size" ]]; then
      rpi_ssh "rm -f $tmp/$f"
      echo "  $f  $((size / 1024)) KiB，下載 $secs 秒 → $dir/$f"
    else
      echo "  $f 複製回筆電失敗，RPi 上還留著 $tmp/$f" >&2
      bad=$((bad + 1))
    fi
  done < <(rpi_fc_logs get "$tmp" "${nums[@]}")
  rpi_ssh "rmdir $tmp 2> /dev/null" || true
  for num in "${nums[@]}"; do
    f=$(printf '%08d.BIN' "$num")
    [[ -f $dir/$f ]] || { echo "  $f 沒拉到" >&2; bad=$((bad + 1)); }
  done
  return "$bad"
}

# session_fc_time <場次>：從 bag 算飛控開機時間 → ROS 時間的對照（info/fc_time.csv）。
# fc_adapter 在跑就在裡面跑，不然借 gui_tools（同一個映像）
session_fc_time() {
  if service_running fc_adapter; then
    in_adapter ros2 run orca_session fc_time "/sessions/$1"
  else
    docker compose run --rm --no-deps -T gui_tools ros2 run orca_session fc_time "/sessions/$1" 2> /dev/null
  fi
}
