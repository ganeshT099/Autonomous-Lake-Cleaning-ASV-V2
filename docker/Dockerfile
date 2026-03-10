# ============================================================
# ASV Docker Image — Jetson (ARM64) + ROS2 Humble
# Base: Jetson compatible ROS2 Humble
# ============================================================

FROM dustynv/ros:humble-ros-base-l4t-r35.3.1

# ── Environment ──────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/asv_ws

# ── System dependencies ───────────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-serial \
    python3-numpy \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-joy \
    v4l-utils \
    git \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ───────────────────────────────────────
RUN pip3 install \
    pyserial \
    pynmea2 \
    pyproj \
    opencv-python \
    "numpy<2" \
    ultralytics

# ── Create workspace ──────────────────────────────────────────
WORKDIR $WORKSPACE

# ── Copy source ───────────────────────────────────────────────
COPY src/ src/

# ── Build ─────────────────────────────────────────────────────
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# ── Source setup ──────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# ── Entrypoint ────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "asv_bringup", "asv_bringup.launch.py"]
