FROM ubuntu:24.04

# Install GStreamer, Rust, and build tools
RUN apt update && apt install -y \
    build-essential \
    cmake \
    pkg-config \
    curl \
    git \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-nice \
    sudo \
    vim \
    ca-certificates && apt clean

# Install Rust
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Prepare workspace
WORKDIR /workspace

# Create folder for mounted gst-plugins-rs
RUN mkdir -p /workspace/gst-plugins-rs

# Default command: run an interactive shell
CMD ["/bin/bash"]
