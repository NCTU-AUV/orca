# 算各映像的 build 指紋，給 compose 的 build.labels（orca.build-hash）用。用 source 載入：
#   source docker/build-hash.sh   # export ORCA_BASE_HASH、ORCA_MISSION_HASH、ORCA_PERCEPTION_HASH
#
# 映像上的 orca.build-hash 跟現在算出來的不同，就表示要重 build（deploy/laptop/up.sh 這樣判斷）。
# 指紋只包含「改了就要重 build」的東西：
#   - Dockerfile、docker/entrypoint.sh、.dockerignore
#   - src/ 裡的 package.xml、setup.py、setup.cfg、CMakeLists.txt、msg／srv／action、resource/
#   - src/ 的檔案清單（只看路徑）：新增、刪除、改名檔案要重 build，setup.py 用 glob 收 data_files
# .py、yaml 的內容不算：compose 把 src/ 掛進容器（colcon --symlink-install），改了直接生效。
# 隱藏檔（.pytest_cache、編輯器暫存檔）和 __pycache__ 不算。

_orca_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

# 標準輸入 → 16 字元的 sha256
_orca_sum() { sha256sum | cut -c1-16; }

# src/ 的部分：build 設定檔的內容 + 所有檔案的路徑
_orca_src=$(
  cd "$_orca_root" || exit 1
  {
    find src -type f -not -path '*/.*' -not -path '*/__pycache__/*' \
      \( -name package.xml -o -name setup.py -o -name setup.cfg -o -name CMakeLists.txt \
         -o -name '*.msg' -o -name '*.srv' -o -name '*.action' -o -path '*/resource/*' \) -print0 \
      | LC_ALL=C sort -z | xargs -0 -r sha256sum
    find src -type f -not -path '*/.*' -not -path '*/__pycache__/*' -not -name '*.pyc' | LC_ALL=C sort
  } | _orca_sum
)

# _orca_image_hash <其他輸入（字串）> <檔案...>：src 指紋 + 字串 + 檔案內容
_orca_image_hash() {
  local extra=$1
  shift
  { echo "$_orca_src $extra"; (cd "$_orca_root" && sha256sum "$@"); } | _orca_sum
}

ORCA_BASE_HASH=$(_orca_image_hash '' docker/base.Dockerfile docker/entrypoint.sh .dockerignore)
# orca-mission 疊在 orca-base 上：orca-base 的輸入變了，它也要重 build
ORCA_MISSION_HASH=$(_orca_image_hash "$ORCA_BASE_HASH" docker/mission.Dockerfile)
ORCA_PERCEPTION_HASH=$(_orca_image_hash '' docker/perception-x86.Dockerfile docker/entrypoint.sh .dockerignore)
export ORCA_BASE_HASH ORCA_MISSION_HASH ORCA_PERCEPTION_HASH

unset -f _orca_sum _orca_image_hash
unset _orca_root _orca_src
