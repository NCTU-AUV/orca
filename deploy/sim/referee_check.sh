#!/usr/bin/env bash
# 驗收：規則事件（碰撞、過門、掉球、進 drum、上浮、abort…）都做得出來、也讀得到。
# 兩個世界各起一次模擬（不開視窗），用外力把船推去撞道具，檢查裁判（sim_referee）發出的事件與分數，
# 以及該擋住的有擋住、該撞得動的撞得動。細節見 sim/tests/referee_check.py。約 6 分鐘。
#
#   ./referee_check.sh                    # sauvc_final.sdf、sauvc_qual.sdf 都跑
#   ./referee_check.sh sauvc_qual.sdf     # 只跑一個
#
# 結果印在畫面上（每一項 PASS／FAIL），也存到 logs/referee/check_<世界>.json；有 FAIL 時結束碼非 0。
# 跑完會把 simulator、sim_referee、sim_dropper 停掉。跟 ./run.sh、./mission.sh 不能同時跑（同一套容器）。
set -euo pipefail
cd "$(dirname "$0")"
export GUI=0
worlds=("$@")
if [[ ${#worlds[@]} -eq 0 ]]; then worlds=(sauvc_final.sdf sauvc_qual.sdf); fi

docker compose build simulator     # Dockerfile 改過才會真的重 build，平常幾秒
status=0
for world in "${worlds[@]}"; do
  export WORLD=$world
  docker compose up -d --force-recreate simulator sim_referee sim_dropper
  printf '等模擬與裁判就緒'
  for _ in $(seq 60); do
    if docker compose logs sim_referee 2>/dev/null | grep -q '模型：'; then break; fi
    printf '.'; sleep 2
  done
  printf '\n'
  sleep 5       # 讓船在水面靜止下來
  docker compose exec -T -e CHECK_OUT="/logs/check_${world%.sdf}.json" sim_referee \
    bash -c 'source /opt/ros/humble/setup.bash && python3 /sim/tests/referee_check.py' || status=1
done
docker compose stop simulator sim_referee sim_dropper > /dev/null 2>&1 || true
exit $status
