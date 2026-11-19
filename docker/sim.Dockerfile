# orca-sim：ROS 2 Humble + Gazebo Harmonic + ArduSub SITL（與實機同版本）+ ardupilot_gazebo + MAVProxy。
# build context 是 docker/（不需要任何檔案）；另外由 compose 的 additional_contexts 送進 sim_plugins（= sim/plugins，
# 自寫的 gz 外掛原始碼）。世界、模型、start.sh、參數由 compose 掛載。
# ---- base：ROS 2 Humble + Gazebo Harmonic（OSRF apt）----
FROM ros:humble AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

# ---- build：編 ArduSub SITL 與 ArduPilotPlugin，建置依賴不進最終 image ----
FROM base AS build

ARG DEBIAN_FRONTEND=noninteractive
ARG JOBS=8

# rapidjson / OpenCV / GStreamer：ardupilot_gazebo 的 CMake 在 configure 階段就要求，
# 雖然這裡只編 ArduPilotPlugin 一個 target
RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-pip python3-dev \
      rapidjson-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
 && rm -rf /var/lib/apt/lists/*

# ArduPilot 的 waf 需要 empy 3.x（4.x 會失敗）
RUN pip3 install --no-cache-dir empy==3.3.4 pexpect future

# ArduSub，與實機飛控相同版本
ARG ARDUPILOT_TAG=Sub-4.7.2
RUN git clone --depth 1 --branch ${ARDUPILOT_TAG} https://github.com/ArduPilot/ardupilot /src/ardupilot \
 && cd /src/ardupilot \
 && git submodule update --init --recursive --depth 1 --jobs 8 \
 && ./waf configure --board sitl \
 && ./waf sub -j${JOBS}

# Gazebo 端的 ArduPilot 外掛
ARG ARDUPILOT_GAZEBO_REF=main
RUN git clone --depth 1 --branch ${ARDUPILOT_GAZEBO_REF} https://github.com/ArduPilot/ardupilot_gazebo /src/ardupilot_gazebo \
 && cd /src/ardupilot_gazebo \
 && GZ_VERSION=harmonic cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 && cmake --build build --target ArduPilotPlugin -j${JOBS}

# 自寫的 gz 外掛（sim/plugins）：放在最後，改外掛不會讓上面的 ArduSub 重編
COPY --from=sim_plugins thrust_curve /src/orca_plugins/thrust_curve
RUN cmake -S /src/orca_plugins/thrust_curve -B /src/orca_plugins/build/thrust_curve -DCMAKE_BUILD_TYPE=RelWithDebInfo \
 && cmake --build /src/orca_plugins/build/thrust_curve -j${JOBS}

# ---- 最終 image ----
FROM base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
 && rm -rf /var/lib/apt/lists/*
# 只做非互動的 MAVLink 轉送，不需要 MAVProxy 的地圖、繪圖等相依（opencv、vtk、matplotlib）
RUN pip3 install --no-cache-dir --no-deps MAVProxy==1.8.75 pymavlink==2.4.50 \
 && pip3 install --no-cache-dir pyserial
# 相機影像的水下後處理（sim/underwater.py）用 OpenCV 做模糊與縮放。headless 版不帶 GUI 相依。
# --no-deps：numpy 用 ROS 帶的那一版（apt 的 1.21），不要讓 pip 另外裝一份。
# 版本鎖在 4.6：4.7 之後的 cv2.typing 需要 numpy >= 1.22，配 1.21 會 import 失敗
RUN pip3 install --no-cache-dir --no-deps opencv-python-headless==4.6.0.66

COPY --from=build /src/ardupilot/build/sitl/bin/ardusub /usr/local/bin/ardusub
COPY --from=build /src/ardupilot_gazebo/build/libArduPilotPlugin.so /usr/local/lib/ardupilot_gazebo/
COPY --from=build /src/orca_plugins/build/thrust_curve/libOrcaThrustCurve.so /usr/local/lib/orca_gz/

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/local/lib/ardupilot_gazebo:/usr/local/lib/orca_gz
# 世界、模型、start.sh 掛載到 /sim，飛控參數掛載到 /params，改了不必重建 image
ENV GZ_SIM_RESOURCE_PATH=/sim/models
CMD ["/sim/start.sh"]
