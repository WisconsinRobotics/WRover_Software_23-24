#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "ros/timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;
constexpr int SAMPLES_PER_READING = 3;

ros::Publisher moistureSensor1;
int baudRate;
std::string file;
int fd;

void moistureCallback(const ros::TimerEvent &) {
    float buf[SAMPLES_PER_READING];
    /* Flush anything already in the serial buffer */
    tcflush(fd, TCIFLUSH);
    /* read up to 128 bytes from the fd */
    int n = read(fd, buf, SAMPLES_PER_READING*sizeof(float));
    std_msgs::Float32 temp;
    for(int i = 0; i < SAMPLES_PER_READING; i++) {
        temp.data = buf[i];
        moistureSensor1.publish(temp);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Science Teleop Control");

    ros::NodeHandle n;
    ros::NodeHandle nh{"~"};
    nh.getParam("fileLoc", file);
    nh.getParam("baudRate", baudRate);
    fd = open(file.c_str(), O_RDWR | O_NOCTTY);
    ROS_ASSERT_MSG(fd != -1, "File %s does not exist", file.c_str());
    /* Set up the control structure */
    struct termios toptions;
 
    /* Get currently set options for the tty */
    tcgetattr(fd, &toptions);


 
    /* Set custom options */
    cfsetispeed(&toptions, baudRate);
    cfsetospeed(&toptions, baudRate);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    /* no hardware flow control */
    toptions.c_cflag &= ~CRTSCTS;
    /* enable receiver, ignore status lines */
    toptions.c_cflag |= CREAD | CLOCAL;
    /* disable input/output flow control, disable restart chars */
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    /* disable canonical input, disable echo,
    disable visually erase chars,
    disable terminal-generated signals */
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /* disable output processing */
    toptions.c_oflag &= ~OPOST;
    /* wait for 12 characters to come in before read returns */
    /* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
    /* CHARACTERS HAVE COME IN! */
    toptions.c_cc[VMIN] = 12;
    /* no minimum time to wait before read returns */
    toptions.c_cc[VTIME] = 0;
    /* avoid hangup */
    toptions.c_cflag &= ~HUPCL;
    /* commit the options */
    tcsetattr(fd, TCSANOW, &toptions);
    moistureSensor1 = n.advertise<std_msgs::Float32>("control/science/moisture1", MESSAGE_CACHE_SIZE);
    
    ros::Timer timer = n.createTimer(ros::Duration(5), moistureCallback);
 
    ros::Publisher screwLiftPow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTablePow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher linearActuatorPow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux1/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher clawPow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux1/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTableEncoder = n.advertise<std_msgs::Float64>("pid/turnTable", MESSAGE_CACHE_SIZE);

    ros::Subscriber encoderSub = n.subscribe("/hsi/roboclaw/aux0/enc/right", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::UInt32::ConstPtr&)>>(
                [&turnTableEncoder](const std_msgs::UInt32::ConstPtr& msg) {
                    std_msgs::Float64 out;
                    out.data = msg->data;
                    turnTableEncoder.publish(out);
                }
    ));
    ros::Subscriber screwLiftSub = n.subscribe("logic/science/screwLift", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftPow](const std_msgs::Float32::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    screwLiftPow.publish(out);
                }
    ));
    ros::Subscriber turnTableSub = n.subscribe("pid/turnTablePow", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float64::ConstPtr&)>>(
                [&turnTablePow](const std_msgs::Float64::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    turnTablePow.publish(out);
                }
            )); 
            
    ros::Subscriber linearActuatorSub = n.subscribe("logic/science/linearActuator", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&linearActuatorPow](const std_msgs::Float32::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    linearActuatorPow.publish(out);
                }
    ));
    ros::Subscriber clawSub = n.subscribe("logic/science/claw", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&clawPow](const std_msgs::Float32::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    clawPow.publish(out);
                }
    ));

    ros::spin();
}
    