#!/usr/bin/env bash
# 偵測驗證：用現有的 RF-DETR 權重跑模擬畫面，量「哪個道具在多遠、什麼角度、什麼深度偵測得到」，
# 水下效果開與關各一輪。位姿是程式擺的（不用開船），每次跑結果一樣。
#
#   ./detect_sweep.sh                         # 全部道具，約 1500 張影像，拍攝約 4 分鐘 + 推論約 1 分鐘（RTX 3060 Laptop）
#   QUICK=1 ./detect_sweep.sh                 # 試跑：每個道具 3 個距離
#   PROPS=gate,q_gate ./detect_sweep.sh       # 只跑這些道具（gate q_gate orange_flare flare_red flare_yellow
#                                             #   flare_blue drum_red drum_blue surfacing_square）
#   UNDERWATER_STRENGTH=1.5 ./detect_sweep.sh # 水下效果的強度（「開」那一輪）
#   RUN=名稱 ./detect_sweep.sh                # 結果資料夾的名稱（預設 sweep_<時間>）
#
# 結果在 logs/detect/<RUN>/：summary.md（報告）、summary.json、results.csv、samples/、frames/（影像，約 100 MB）。
# 比較兩次：python3 ../../tools/sim_detect/compare.py logs/detect/<A> logs/detect/<B>
# 只重算（例如換了權重，不重拍）：docker compose --profile sweep run --rm detect_eval \
#   python3 -u /orca/tools/sim_detect/evaluate.py /out/<RUN>
# 不需要先起模擬；跟 ./run.sh、./mission.sh 可以同時跑（另一個 gz partition），只是會搶 GPU。
set -euo pipefail
cd "$(dirname "$0")"
export RUN=${RUN:-sweep_$(date +%Y%m%d_%H%M%S)}
export GIT_REV=${GIT_REV:-$(git -C ../.. describe --always --dirty 2>/dev/null || true)}

if [[ ! -f ../../ml/rfdetr/weights/checkpoint_best_total.pth ]]; then
  ../../ml/rfdetr/fetch_weights.sh
fi
docker compose --profile sweep build detect_eval > /dev/null
docker compose --profile sweep run --rm sim_sweep
docker compose --profile sweep run --rm detect_eval python3 -u /orca/tools/sim_detect/evaluate.py "/out/$RUN"
echo
cat "logs/detect/$RUN/summary.md"
