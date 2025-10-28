#!/bin/bash

# ===============================================================
# Docker Run Script for ROS 2 Humble Development Environment
# ---------------------------------------------------------------
# - Launches container with X11 GUI support
# - Mounts current project directory
# - Enables host networking (for ROS 2 DDS)
# ===============================================================

IMAGE_NAME="ros2-humble-develop"
CONTAINER_NAME="ros2_robot_control"
WORKSPACE_DIR="$(pwd)"

# 1. Check if image exists
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "[Error] Docker image '$IMAGE_NAME' not found."
    echo "→ Please build it first with: ./build.sh"
    exit 1
fi

# 2. Enable X11 access for Docker
echo "[Info] Enabling X11 access for Docker..."
xhost +local:docker

# 3. Run the container
echo "[Info] Starting Docker container... (Image Name: $IMAGE_NAME, Container Name: $CONTAINER_NAME)"
docker run -it --rm \
    --name $CONTAINER_NAME \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev:rw \
    -v $WORKSPACE_DIR:/ros2_ws \
    $IMAGE_NAME

# 4. Disable X11 access after exit
echo "[Info] Disabling X11 access after exit..."
xhost -local:docker

# 5. Exit script with success status
echo "[Info] Container exited cleanly."