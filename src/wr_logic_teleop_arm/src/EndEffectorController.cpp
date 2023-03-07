#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"
#include <ros/ros.h>

auto main(int argc, char** argv) -> int {
  ros::init(argc, argv, "EndEffectorController");
  ros::NodeHandle n;
  
  ros::Publisher openAPub = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", 1000);
  ros::Publisher closeB = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", 1000);
  
  openA.publish(Int16(32767));
  closeB.publish(Int16(-32768));

  return 0;
}