# orca-base：ROS 2 Humble + mavros + OpenCV + orca workspace。
# bridge、cam_bridge、recorder、tools 共用。build context 是 repo 根目錄（見 .dockerignore）。
FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-mavros \
        ros-humble-vision-msgs \
        ros-humble-rqt-image-view \
        ros-humble-compressed-image-transport \
        ros-humble-rosbag2-storage-mcap \
        python3-opencv \
        python3-pygame \
        wget \
 && rm -rf /var/lib/apt/lists/*

# mavros 啟動時需要 GeographicLib 的大地水準面資料，沒有會直接報錯退出
RUN source /opt/ros/humble/setup.bash && ros2 run mavros install_geographiclib_datasets.sh

# --symlink-install：Python 檔以符號連結裝進 install/，compose 把 repo 的 src/ 掛到
# /opt/orca_ws/src 之後，改 .py 重開容器就生效。改了 msg、package.xml、setup.py 才要重 build
COPY src /opt/orca_ws/src
RUN source /opt/ros/humble/setup.bash && cd /opt/orca_ws && colcon build --symlink-install && rm -rf log

COPY docker/entrypoint.sh /entrypoint.sh
RUN echo 'source /opt/ros/humble/setup.bash && source /opt/orca_ws/install/setup.bash' >> /root/.bashrc
ENTRYPOINT ["/entrypoint.sh"]
