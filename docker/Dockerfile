ARG BASE_IMAGE=text_to_suppress_warning
FROM ${BASE_IMAGE}

COPY voice_ros2 /root/ros2_ws/src/voice_ros2
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --packages-select voice_ros2