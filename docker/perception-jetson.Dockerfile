# orca-perception-jetson（Jetson Orin、JetPack 6）：RF-DETR 的 TensorRT 推論。build context 是 repo 根目錄。
#
# - JetPack 6 的 nvidia-container-runtime 只把驅動掛進容器，CUDA、TensorRT 要在映像裡。用 NVIDIA 的
#   l4t-tensorrt（Ubuntu 22.04、CUDA 12.6、TensorRT 10.3，跟 JetPack 6.2 主機同版），再裝 ROS 2 Humble。
#   l4t-jetpack（5 GB，含整套開發工具）用不到。
# - 不裝 torch、rfdetr：前後處理用 numpy 寫在 orca_perception/rfdetr_trt.py，模型是筆電匯出的 ONNX，
#   在這個容器裡轉成 TensorRT 引擎（build_engine）。
# - workspace 在 ros:humble 裡 build（跟 perception-x86 一樣），整個複製過來。

FROM ros:humble AS ws
COPY src /opt/orca_ws/src
RUN . /opt/ros/humble/setup.sh && cd /opt/orca_ws && colcon build --symlink-install && rm -rf log

FROM nvcr.io/nvidia/l4t-tensorrt:r10.3.0-runtime

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8

# ROS 的 apt 來源（含金鑰）直接沿用 ros:humble 映像的；只要二進位套件，不要 deb-src
COPY --from=ws /etc/apt/sources.list.d/ros2.sources /etc/apt/sources.list.d/ros2.sources
RUN sed -i 's/^Types: deb deb-src$/Types: deb/' /etc/apt/sources.list.d/ros2.sources \
 && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-vision-msgs \
        python3-opencv \
        python3-numpy \
        python3-pip \
 && rm -rf /var/lib/apt/lists/*

# cuda-python：配置 GPU 記憶體、在 CPU 與 GPU 之間複製資料（TensorRT 的 Python API 本身不管這些）。版本跟 CUDA 12.6 對齊
RUN pip3 install --no-cache-dir "cuda-python==12.6.*"

# 確認 TensorRT 的 Python 綁定和 ONNX 解析器（build_engine 要用）在映像裡。只查檔案、不 import：
# import tensorrt 要主機的驅動函式庫（libnvdla_compiler.so 等），nvidia runtime 只在容器跑起來時掛進來，
# docker build 時沒有。真的 import 由 up.sh 轉引擎時（runtime: nvidia）做
RUN test -f /usr/lib/python3.10/dist-packages/tensorrt/__init__.py \
 && ls /usr/lib/aarch64-linux-gnu/libnvonnxparser.so.10

COPY --from=ws /opt/orca_ws /opt/orca_ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash && source /opt/orca_ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
