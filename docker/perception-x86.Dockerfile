# orca-perception（x86_64 + NVIDIA）：RF-DETR 推論。Jetson 要另寫一份（arm64 + JetPack 的 PyTorch）。
# build context 是 repo 根目錄。

# workspace 在另一個 stage 用 ROS 原本的 setuptools build：rfdetr 會把 setuptools 升到新版，
# 新版在 colcon 的 ament_python build 會失敗
FROM ros:humble AS ws
COPY src /opt/orca_ws/src
RUN . /opt/ros/humble/setup.sh && cd /opt/orca_ws && colcon build --symlink-install && rm -rf log

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-opencv \
        ros-humble-vision-msgs \
 && rm -rf /var/lib/apt/lists/*

# rfdetr 版本跟訓練時一致。PyPI 的 torch 自帶 CUDA runtime，主機只要有 NVIDIA 驅動。
# numpy 鎖在 1.x：ROS Humble 的訊息 C 擴充是對 numpy 1 編譯的，裝到 numpy 2 會 import 失敗
RUN pip3 install --no-cache-dir "rfdetr==1.12.0" "numpy<2"

# 整個 workspace（src、build、install）一起複製，--symlink-install 的連結才不會斷
COPY --from=ws /opt/orca_ws /opt/orca_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash && source /opt/orca_ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
