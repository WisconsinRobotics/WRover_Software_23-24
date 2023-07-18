#!/bin/bash

# Setup ROS environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

./assemble.py init

source setup.sh
./assemble.py build

echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> ~/.bashrc
echo "source \"$(pwd)/setup.sh\"" >> ~/.bashrc
