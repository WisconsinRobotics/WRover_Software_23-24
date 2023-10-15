FROM osrf/ros:noetic-desktop-full AS base

RUN apt-get update

# install pip and virtualenv for python dependencies
RUN apt-get install -y python-is-python3 python3-pip
RUN pip install virtualenv

# install g++10
RUN apt-get install -y g++-10
RUN unlink /usr/bin/g++
RUN ln -s g++-10 /usr/bin/g++

# install development tools
RUN apt-get install -y bash-completion curl git wget unzip

# Add Microsoft repo for dotnet
RUN wget https://packages.microsoft.com/config/ubuntu/20.04/packages-microsoft-prod.deb -O packages-microsoft-prod.deb
RUN sudo dpkg -i packages-microsoft-prod.deb
RUN rm packages-microsoft-prod.deb

RUN apt-get update
RUN apt-get install -y dotnet-sdk-6.0 

# Install clangd for code completion
RUN wget https://github.com/clangd/clangd/releases/download/16.0.2/clangd-linux-16.0.2.zip -O clangd.zip
RUN unzip clangd.zip -d /opt/clangd

# install dependencies from project.json
RUN apt-get install -y can-utils ros-noetic-moveit ros-noetic-moveit-visual-tools

# create devcontainer user
RUN useradd -m devcontainer
RUN usermod -aG sudo devcontainer
RUN echo "devcontainer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> home/devcontainer/.bashrc
RUN echo "source /workspaces/WRover_Software/setup.sh" >> home/devcontainer/.bashrc
RUN echo "export PATH=$PATH:/opt/clangd/clangd_16.0.2/bin" >> /home/devcontainer/.profile

USER devcontainer

ENV SHELL /bin/bash
