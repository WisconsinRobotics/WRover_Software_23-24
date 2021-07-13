# kill existing roscore
ROSCORE_PID=`ps aux | grep roscore | grep /opt/ros | awk '{print $2}'`
if [ ! -z "$ROSCORE_PID" ]; then
  kill $ROSCORE_PID
fi

