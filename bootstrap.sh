# kill existing roscore
ROSCORE_PID=`ps aux | grep roscore | grep /opt/ros | awk '{print $2}'`
if [ ! -z "$ROSCORE_PID" ]; then
  kill $ROSCORE_PID
fi

# launch new roscore
nohup roscore >/dev/null 2>&1 &

# wait until the roscore is finished launching
RET=0
while [ "$RET" -ne 1 ]; do
  rostopic list 2>&1 >/dev/null | grep "Unable to communicate with master" >/dev/null
  RET=$?
done

# job's done!
echo ok
