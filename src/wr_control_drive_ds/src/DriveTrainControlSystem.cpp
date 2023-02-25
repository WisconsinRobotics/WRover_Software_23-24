#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wroboclaw/Int16Pair.h"

#include <array>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
// Store the input abstract power values
std::array<float, 2> inp_vals{0.0, 0.0};

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

// Main program
int main(int argc, char **argv) {

  // Initialize the node in ROS
  ros::init(argc, argv, "DriveTrainControlSystem");

  // Create a node handle for the current node to subscribe to/advertise topics
  ros::NodeHandle n;

  // Create a publisher to the output topic for WRoboClaw duty cycle numbers for
  // command 34
  ros::Publisher outLeft =
      n.advertise<std_msgs::Float64>("/left/power", MESSAGE_CACHE_SIZE);

  ros::Publisher outRight =
      n.advertise<std_msgs::Float64>("/right/power", MESSAGE_CACHE_SIZE);

  // Create a subscriber to the input topic for power values
  ros::Subscriber inTopic =
      n.subscribe("/control/drive_system/cmd", MESSAGE_CACHE_SIZE,
                  static_cast<boost::function<void(
                      const wr_drive_msgs::DriveTrainCmd::ConstPtr &)>>(
                      [&outLeft, &outRight](
                          const wr_drive_msgs::DriveTrainCmd::ConstPtr &msg) {
                        outLeft.publish(msg->left_value);
                        outRight.publish(-msg->right_value);
                      }));

  ros::spin();
}
