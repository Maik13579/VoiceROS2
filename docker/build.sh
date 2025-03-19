#!/bin/bash
set -e

# License confirmation prompt
echo "> You must confirm the following:"
echo "| > \"I have purchased a commercial license from Coqui: licensing@coqui.ai\""
echo "| > \"Otherwise, I agree to the terms of the non-commercial CPML: https://coqui.ai/cpml\" - [y/n]"
read -r CONFIRMATION
if [ "$CONFIRMATION" != "y" ]; then
    echo "License not confirmed, exiting."
    exit 1
fi

# Get the directory of the script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

# Local variables
UBUNTU_MAJOR=22
UBUNTU_MINOR=04
CUDA_MAJOR=12
CUDA_MINOR=6
CUDA_PATCH=3
ROS_DISTRO=humble
IMAGE_TAG="voice_ros2"

# Build Base image
docker build -t ${IMAGE_TAG} --target base \
  --build-arg UBUNTU_MAJOR=${UBUNTU_MAJOR} \
  --build-arg UBUNTU_MINOR=${UBUNTU_MINOR} \
  --build-arg CUDA_MAJOR=${CUDA_MAJOR} \
  --build-arg CUDA_MINOR=${CUDA_MINOR} \
  --build-arg CUDA_PATCH=${CUDA_PATCH} \
  --build-arg ROS_DISTRO=${ROS_DISTRO} \
  -f $PARENT_DIR/coqui_tts_ros2/docker/Dockerfile \
  $PARENT_DIR/coqui_tts_ros2

# Download coquiTTS
docker build -t ${IMAGE_TAG} --target build_tts \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/coqui_tts_ros2/docker/Dockerfile \
  $PARENT_DIR/coqui_tts_ros2

# Download coqui models
docker build -t ${IMAGE_TAG} --target download_models \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/coqui_tts_ros2/docker/Dockerfile \
  $PARENT_DIR/coqui_tts_ros2

# Download vosk models
docker build -t ${IMAGE_TAG} --target download_models \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/vosk_ros2/docker/Dockerfile \
  $PARENT_DIR/vosk_ros2

# Build custom vosk models (like gpsr)
docker build -t ${IMAGE_TAG} --target build_custom_models \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/vosk_ros2/docker/Dockerfile \
  $PARENT_DIR/vosk_ros2

# Build the coqui_tts_ros2
docker build -t ${IMAGE_TAG} --target build_ros \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/coqui_tts_ros2/docker/Dockerfile \
  $PARENT_DIR/coqui_tts_ros2

# Build the vosk_ros2
docker build -t ${IMAGE_TAG} --target build \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/vosk_ros2/docker/Dockerfile \
  $PARENT_DIR/vosk_ros2

# Finally build this Dockerfile ontop
docker build -t ${IMAGE_TAG} \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f $PARENT_DIR/docker/Dockerfile \
  $PARENT_DIR