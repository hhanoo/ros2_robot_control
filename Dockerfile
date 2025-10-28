# ROS 2 Humble Desktop base -------------------------------------------------
# https://hub.docker.com/_/ros/tags
FROM docker.io/osrf/ros:humble-desktop

# 비대화 모드  ------------------------------------------------------------
ENV DEBIAN_FRONTEND=noninteractive
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Set timezone ------------------------------------------------------------
ENV TZ=Asia/Seoul
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install essentials ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    curl \
    wget \
    nano \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# GUI/OpenGL Dependencies (for RViz, rqt, OpenCV display) ------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dev \
    mesa-common-dev \
    libglu1-mesa-dev \
    libx11-dev \
    libxext-dev \
    libxrender-dev \
    libx11-xcb-dev \
    libxcb1-dev \
    libxcb-render0-dev \
    libxcb-shm0-dev \
    libfontconfig1-dev \
    libfreetype6-dev \
    libglib2.0-dev \
    && rm -rf /var/lib/apt/lists/*

# USB Communication ------------------------------------------------------------  
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Optional Dependencies ----------------------------------------------------------
# Poco Library
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpoco-dev \
    && rm -rf /var/lib/apt/lists/*

# Python Package Manager (pip, uv) ------------------------------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel uv

# Python Dependencies ------------------------------------------------------------
RUN uv pip install --system \
    numpy \
    setuptools==69.5.1

# Set locale ------------------------------------------------------------
RUN apt-get update && apt-get install -y locales && \
    echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen && \
    update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    LANGUAGE=en_US:en

# Setup workspace ------------------------------------------------------------
WORKDIR /ros2_ws

# Default command ------------------------------------------------------------
CMD ["bash"]