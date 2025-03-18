#!/bin/bash
set -e

# Local variables
BASE_IMAGE="ros:humble"
IMAGE_TAG="voice_ros2"

# Build the vosk_ros2 image with the specified base and tag.
docker build -t ${IMAGE_TAG} \
  --build-arg BASE_IMAGE=${BASE_IMAGE} \
  -f vosk_ros2/docker/Dockerfile \
  vosk_ros2

# Build the coqui_ros2 image using the previously built vosk_ros2 image as base.
docker build -t ${IMAGE_TAG} \
  --build-arg BASE_IMAGE=${IMAGE_TAG} \
  -f coqui_ros2/docker/Dockerfile \
  coqui_ros2
