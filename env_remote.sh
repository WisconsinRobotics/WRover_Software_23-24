#!/usr/bin/env bash

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  exit 1
fi

CATKIN_SHELL=sh
_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"

export ROS_MASTER_URI='http://wrover-pi.local:11311'
export ROS_IP=`ip addr show eth0 | grep -Po 'inet [\d.]+' | awk '{print $2}'`
export ROS_HOSTNAME='wrover-pi.local'

exec "$@"
