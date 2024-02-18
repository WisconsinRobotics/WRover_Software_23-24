#include "BNO085_hal.hpp"

#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "sh2.h"
#include "sh2_err.h"
#include "std_msgs/Float64.h"
#include <cstdlib>
#include <iostream>

static constexpr uint32_t QUEUE_SIZE = 1;

auto main(int argc, char **argv) -> int {

    // Initialize the node in ROS
    ros::init(argc, argv, "BNO085");

    // Create a node handle for the current node to subscribe to/advertise topics
    ros::NodeHandle nHandle;

    ros::Rate rate(10);

    // Create publishers
    auto pub_x{nHandle.advertise<std_msgs::Float64>("/mag_x", QUEUE_SIZE)};
    auto pub_y{nHandle.advertise<std_msgs::Float64>("/mag_y", QUEUE_SIZE)};
    auto pub_z{nHandle.advertise<std_msgs::Float64>("/mag_z", QUEUE_SIZE)};

    // Initalize IMU
    BNO085 sensor;
    if (!sensor.begin()) {
        ROS_FATAL("FAILED TO INITIALIZE IMU");
    }

    if (sensor.set_sensor_config(SH2_RAW_MAGNETOMETER) != SH2_OK) {
        ROS_FATAL("FAILED TO ENABLE MAGNETOMETER");
    }

    while (ros::ok()) {
        if (sensor.get_sensor_event()) {
            std::cout << "Mag X: " << sensor.get_mag_x() << "Mag Y: " << sensor.get_mag_y() << "Mag Z: " << sensor.get_mag_z() << std::endl;
        }

        rate.sleep();
    }

    return EXIT_SUCCESS;
}
