#!/bin/bash

cd /workspaces/WRover_Software

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

./assemble.py init

source setup.sh
./assemble.py build
