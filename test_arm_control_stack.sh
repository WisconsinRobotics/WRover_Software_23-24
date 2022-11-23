#!/bin/bash
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd $ROOT_DIR
./assemble.py build
#gnome-terminal -- roscore
#rostopic list &>./out.temp;
#COUNT=$(grep -Ec "Unable to communicate with master" out.temp)
#while [ $COUNT -ne 0 ]; do
#	rostopic list &>./out.temp;
#	COUNT=$(grep -Ec "Unable to communicate with master" out.temp)
#done
#rm ./out.temp;
export WROVER_LOCAL=true
export WROVER_HW=MOCK
MOTOR_TO_INSPECT=11
# (sleep 2 && rqt_plot /control/arm/$MOTOR_TO_INSPECT/output /control/arm/$MOTOR_TO_INSPECT/setpoint /control/arm/$MOTOR_TO_INSPECT/feedback) & #/hsi/roboclaw/aux1/cmd/left 
roslaunch wr_entry_point mock_arm.launch

