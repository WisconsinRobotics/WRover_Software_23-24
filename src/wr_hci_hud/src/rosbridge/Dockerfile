FROM osrf/ros:jazzy-desktop-full

# Install ROSbridge and necessary tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-rosbridge-suite &&\
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Set environment variables for ROS
# ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

# Create launch directory in container
# RUN mkdir -p /root/rosbridge_launch

# Expose ROSbridge WebSocket port
EXPOSE 9090

# Default command
CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
# ros2 run rosbridge_server rosbridge_websocket