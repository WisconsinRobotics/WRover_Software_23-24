#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include "ArmMotor.hpp"

ArmMotor motors[6];
ros::Publisher jointStatePublisher;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as) {
    for(int i = 0; i < goal->trajectory.points.size(); i++){
      trajectory_msgs::JointTrajectoryPoint currTargetPosition = goal->trajectory.points[i];
      for(int j = 0; j < currTargetPosition.positions.size(); j++){
        motors[j].runToTarget(currTargetPosition.positions[j], 0.1);
      }

      bool hasPositionFinished = false;
      std::vector<std::string> names();
      std::vector<double> positions();
      while(!hasPositionFinished){
        bool temp = true;
        sensor_msgs::JointState* js_msg;
        names.clear();
        positions.clear();
        for(int j = 0; j < currTargetPosition.positions.size(); j++){
          motors[j].runToTarget(currTargetPosition.positions[j], 0.1);
          temp &= motors[j].getMotorState() == MotorState::STOP;
        }
        //TODO: Set js_msg
        jointStatePublisher.publish(js_msg);
        hasPositionFinished = temp;
      }
    }
    as->setSucceeded();
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "ArmControlSystem");
  ros::NodeHandle n;
  jointStatePublisher = n.advertise<sensor_msgs::JointState>("", 1000);
  Server server(n, "/arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}