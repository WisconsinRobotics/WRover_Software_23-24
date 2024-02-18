#include "BNO085_hal.hpp"

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "sh2.h"
#include "std_msgs/Float64.h"
#include <cstdlib>

static constexpr uint32_t QUEUE_SIZE = 1;
static BNO085 sensor;

auto main(int argc, char **argv) -> int {

    // Initialize the node in ROS
    ros::init(argc, argv, "BNO085");

    // Create a node handle for the current node to subscribe to/advertise topics
    ros::NodeHandle nHandle;

    // Create publishers
    auto pub_x{nHandle.advertise<std_msgs::Float64>("/mag_x", QUEUE_SIZE)};
    auto pub_y{nHandle.advertise<std_msgs::Float64>("/mag_y", QUEUE_SIZE)};
    auto pub_z{nHandle.advertise<std_msgs::Float64>("/mag_z", QUEUE_SIZE)};

    // Initalize IMU
    if (!sensor.begin()) {
        ROS_FATAL("FAILED TO INITIALIZE IMU");
    }

    sensor.set_sensor_config(SH2_ACCELEROMETER);

    return EXIT_SUCCESS;
}
