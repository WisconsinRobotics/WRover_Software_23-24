#!/bin/bash
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $ROOT_DIR
./assemble.py build
pgrep roscore || gnome-terminal -- roscore
echo "launched roscore"
rostopic list &>./out.temp;
COUNT=$(grep -Ec "Unable to communicate with master" out.temp)
while [ $COUNT -ne 0 ]; do
	rostopic list &>./out.temp;
	COUNT=$(grep -Ec "Unable to communicate with master" out.temp)
done
rm ./out.temp;
export WROVER_LOCAL=true
pgrep wr_entry_point || gnome-terminal -- roslaunch wr_entry_point test_arm.launch
echo "launched entry point"
pgrep wr_control_drive_arm || gnome-terminal -- roslaunch wr_control_drive_arm std.launch
echo "launched control drive"
pgrep wroboarm_21 || gnome-terminal -- roslaunch wroboarm_21 demo_test.launch
echo "launching roboarm"
gnome-terminal -- echo running rqt_plot; rqt_plot /control/arm/30/setpoint /control/arm/30/feedback /control/arm/21/setpoint /control/arm/21/feedback

# until pgrep wroboarm_21
# do
sleep 10
echo "waiting for roboarm"
# done

pgrep wr_logic_teleop_arm || gnome-terminal -- /bin/sh -c "rosrun wr_logic_teleop_arm ArmTeleopLogic; while true; do sleep 1; done"
pgrep wreadinput || gnome-terminal -- roslaunch wreadinput test_xbox360.launch
cd -
