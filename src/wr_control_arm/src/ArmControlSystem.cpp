#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

void executeAction(const control_msgs::FollowJointTrajectoryActionGoalConstPtr& goal, Server* s){
    s->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ArmControlSystem");
  ros::NodeHandle n;
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&executeAction, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}