#!/bin/bash
# filepath: rosbridge/start.sh

set -e

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS setup.bash not found!"
  exit 1
fi

source /opt/ros/humble/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml &

if [ ! -d /root/gst-plugins-rs/net/webrtc/signalling ]; then
  echo "WebRTC signaling server directory not found!"
  exit 1
fi

cd /root/gst-plugins-rs/net/webrtc/signalling

if ! command -v cargo &> /dev/null; then
  echo "Cargo (Rust) not installed!"
  exit 1
fi

export WEBRTCSINK_SIGNALLING_SERVER_LOG=debug
cargo run --bin gst-webrtc-signalling-server

wait