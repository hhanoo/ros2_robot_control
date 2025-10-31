#!/bin/bash

# Set image name
IMAGE_NAME="ros2-humble-develop"

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME..."
docker build -t $IMAGE_NAME -f Dockerfile .

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Docker image $IMAGE_NAME built successfully!"
else
    echo "Failed to build Docker image. Check the logs above for errors."
    exit 1
fi