# orca-mission：行為樹決策層（src/orca_mission）。
# 疊在 orca-base 上，只多 py_trees 與畫流程圖的工具；base.Dockerfile（fc_adapter 也用）不動。
# build context 是 repo 根目錄（見 .dockerignore）。先有 orca-base（deploy/*/compose.yaml 會一起 build）。
FROM orca-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-py-trees \
        ros-humble-py-trees-ros \
        python3-yaml \
        python3-pytest \
        python3-pydot \
        graphviz \
 && rm -rf /var/lib/apt/lists/*

# orca-base 的 workspace 是它 build 當時的 src；這裡用目前的 src 重 build 一次，orca_mission 才會在裡面
COPY src /opt/orca_ws/src
RUN source /opt/ros/humble/setup.bash && cd /opt/orca_ws && rm -rf build install \
 && colcon build --symlink-install && rm -rf log

# mission、render_tree 直接執行（不經 ros2 run：docker stop 的 SIGTERM 才會直接送到 mission，
# 讓它先送 MANUAL＋DISARM 再結束）
ENV PATH=/opt/orca_ws/install/orca_mission/lib/orca_mission:$PATH
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
