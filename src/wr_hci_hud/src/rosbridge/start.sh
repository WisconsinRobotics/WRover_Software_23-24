#!/bin/bash

# Source ROS environment
source /opt/ros/jazzy/setup.bash

# Start ROSbridge WebSocket server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

# Start WebRTC signaling server
cd /root/gst-webrtc-server/gst-plugins-rs/net/webrtc/signalling
export WEBRTCSINK_SIGNALLING_SERVER_LOG=debug
cargo run --bin gst-webrtc-signalling-server



# Wait for both processes
wait