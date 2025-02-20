FROM osrf/ros:jazzy-desktop-full

# Install ROSbridge and necessary tools
RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-rtsp \
    gstreamer1.0-libav \
    gstreamer1.0-alsa \
    curl \
    cargo \
    clang \
    cmake \
    pkg-config \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    ros-jazzy-rosbridge-suite &&\
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Install the latest Rust using rustup
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y
ENV PATH="/root/.cargo/bin:$PATH"

ENV PKG_CONFIG_PATH="/usr/lib/x86_64-linux-gnu/pkgconfig:$PKG_CONFIG_PATH"

RUN rustc --version

# Set environment variables for ROS
# ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

# Create launch directory in container
# RUN mkdir -p /root/rosbridge_launch

# Expose ROSbridge WebSocket port
EXPOSE 9090 8443

# Default command
# CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
# ros2 run rosbridge_server rosbridge_websocket

# FROM osrf/ros:jazzy-desktop-full

# # Install dependencies for ROSbridge and GStreamer WebRTC
# RUN apt-get update && apt-get install -y \
#     ros-jazzy-rosbridge-suite \
    # gstreamer1.0-tools \
    # gstreamer1.0-plugins-bad \
    # gstreamer1.0-plugins-good \
    # gstreamer1.0-plugins-ugly \
    # gstreamer1.0-rtsp \
    # gstreamer1.0-libav \
    # gstreamer1.0-alsa \
    # cargo \
    # clang \
    # cmake \
    # pkg-config \
    # libgstreamer1.0-dev \
    # libgstreamer-plugins-base1.0-dev \
    # && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /root

# Create a new Rust project
RUN cargo new gst-webrtc-server
WORKDIR /root/gst-webrtc-server

# Add gst-plugin-webrtc dependency

RUN echo 'gst-plugin = "0.3.2"' >> Cargo.toml

RUN git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git
# Build the WebRTC signaling server
RUN cargo build

# Expose ROSbridge and WebRTC signaling server ports
# EXPOSE 9090 8443 8080

# Copy start script
COPY start.sh /root/start.sh
RUN chmod +x /root/start.sh

CMD ["/root/start.sh"]
