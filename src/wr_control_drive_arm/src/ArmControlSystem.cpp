#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include "ArmMotor.hpp"

ArmMotor motors[6];
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
    // Do lots of awesome groundbreaking robot stuff here
    //TODO: Execute arm actions in goal->trajectory.points[].position
    for(int i = 0; i < goal->trajectory.points.size(); i++){
      trajectory_msgs::JointTrajectoryPoint currTargetPosition = goal->trajectory.points[i];
      for(int j = 0; j < currTargetPosition.positions.size(); j++){
        motors[j].runToTarget(currTargetPosition.positions[j], 0.1);
      }
      bool hasPositionFinished = false;
      //TODO: Implement checking each motor for completed position actions.
    }
    as->setSucceeded();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ArmControlSystem");
  ros::NodeHandle n;
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}