FROM osrf/ros:noetic-desktop-full AS base

# install pip and virtualenv for python dependencies
RUN apt-get update && apt-get install -y python3-pip
RUN pip install virtualenv

# install dependencies from project.json
RUN apt-get install -y can-utils ros-noetic-moveit ros-noetic-moveit-visual-tools 

# install g++10
RUN apt-get install -y g++-10
RUN unlink /usr/bin/g++
RUN ln -s g++-10 /usr/bin/g++

# install development tools
RUN apt-get install -y git clangd clang-tidy

# create devcontainer user
RUN useradd -m devcontainer
RUN usermod -aG sudo devcontainer
RUN echo "devcontainer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers

USER devcontainer

ENV SHELL /bin/bash
