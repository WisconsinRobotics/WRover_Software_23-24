#!/bin/bash
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $ROOT_DIR
./assemble.py build
gnome-terminal -- roscore
rostopic list &>./out.temp;
COUNT=$(grep -Ec "Unable to communicate with master" out.temp)
while [ $COUNT -ne 0 ]; do
	rostopic list &>./out.temp;
	COUNT=$(grep -Ec "Unable to communicate with master" out.temp)
done
rm ./out.temp;
export WROVER_LOCAL=true
gnome-terminal -- roslaunch wr_entry_point test_arm.launch
gnome-terminal -- roslaunch wr_control_drive_arm std.launch
gnome-terminal -- roslaunch wroboarm_21 demo_test.launch
cd -
