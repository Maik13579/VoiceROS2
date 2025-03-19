ARG BASE_IMAGE=text_to_suppress_warning
FROM ${BASE_IMAGE}

COPY scripts /scripts
RUN echo "alias gpsr='python3 /scripts/gpsr.py'" >> ~/.bashrc
RUN echo "alias voice_clone='python3 /scripts/voice_cloning.py'" >> ~/.bashrc
COPY voice_ros2 /root/ros2_ws/src/voice_ros2
RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --packages-select voice_ros2