# 隨機佈局：用 source 載入（run.sh、mission.sh、referee_check.sh、detect_sweep.sh 都用它）。
#
#   random_layout <世界檔>    SEED 沒設：原樣印出 <世界檔>（固定佈局）
#                             SEED=<數字> 或 SEED=random：用 tools/sim/layout.py 照 SAUVC 2027 規則抽一個佈局，
#                             產生 sim/worlds/generated/<世界>_seed<N>.sdf，印出它（相對 sim/worlds 的路徑）
#
# 用法：WORLD=$(random_layout "$WORLD")（失敗時 set -e 會讓腳本停下來）。
# 目前只有決賽主場地（sauvc_final.sdf）有隨機佈局，其他世界設了 SEED 會報錯。
# 佈局的說明印在標準錯誤；真值在旁邊的 .layout.json，裁判會把它寫進 logs/referee/ 的紀錄。
random_layout() {
  if [[ -z ${SEED:-} ]]; then
    echo "$1"
    return 0
  fi
  python3 "$(dirname "${BASH_SOURCE[0]}")/../../tools/sim/layout.py" "$1" --seed "$SEED"
}
