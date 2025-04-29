# Use an official Ubuntu 20.04 LTS as a parent image
FROM osrf/ros:humble-desktop

# Set noninteractive to avoid prompts during the build
ARG DEBIAN_FRONTEND=noninteractive

# Update apt package list and install general packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-rosbridge-server\
    curl\
    python3-pip\
    wget


# Install python packages

# Set the default command to execute
# When creating a container, this will simulate `docker run -it`
CMD ["bash"]
