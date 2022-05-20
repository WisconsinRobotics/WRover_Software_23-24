#include "ros/assert.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "ros/timer.h"
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <array>

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;
constexpr int SAMPLES_PER_READING = 3;
constexpr float TIMER_CALLBACK_DURATION = 5;
constexpr int NUM_CHARS_READ = 12;

int baudRate;
std::string file;
int fd;
std::array<ros::Publisher, SAMPLES_PER_READING> sensorPublishers;

void moistureCallback(const ros::TimerEvent &timerEvent) {
    std::array<float, SAMPLES_PER_READING> buf;
    /* Flush anything already in the serial buffer */
    tcflush(fd, TCIFLUSH);
    /* read up to 128 bytes from the fd */
    int readStatus = read(fd, buf.data(), SAMPLES_PER_READING*sizeof(float));
    //NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg, hicpp-no-array-decay, hicpp-no-assembler, hicpp-vararg)
    ROS_ASSERT_MSG(readStatus != -1, "Read failed");
    std_msgs::Float32 temp;
    for(int i = 0; i < SAMPLES_PER_READING; i++) {
        temp.data = buf[i];
        sensorPublishers[i].publish(temp);
    }
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "Science Teleop Control");

    ros::NodeHandle n;
    ros::NodeHandle nh{"~"};
    nh.getParam("fileLoc", file);
    nh.getParam("baudRate", baudRate);
    //NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg, hicpp-vararg, hicpp-signed-bitwise)
    fd = open(file.c_str(), O_RDWR | O_NOCTTY);
    //NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay, cppcoreguidelines-pro-type-vararg, hicpp-no-array-decay, hicpp-no-assembler, hicpp-vararg)
    ROS_ASSERT_MSG(fd != -1, "File %s does not exist", file.c_str());
    /* Set up the control structure */
    struct termios toptions;
 
    /* Get currently set options for the tty */
    tcgetattr(fd, &toptions);


 
    /* Set custom options */
    cfsetispeed(&toptions, baudRate);
    cfsetospeed(&toptions, baudRate);
    /* 8 bits, no parity, no stop bits */
    toptions.c_cflag &= ~PARENB; //NOLINT(hicpp-signed-bitwise)
    toptions.c_cflag &= ~CSTOPB; //NOLINT(hicpp-signed-bitwise)
    toptions.c_cflag &= ~CSIZE; //NOLINT(hicpp-signed-bitwise)
    toptions.c_cflag |= CS8; //NOLINT(hicpp-signed-bitwise)

    /* no hardware flow control */
    toptions.c_cflag &= ~CRTSCTS;

    /* enable receiver, ignore status lines */
    toptions.c_cflag |= CREAD | CLOCAL; //NOLINT(hicpp-signed-bitwise)

    /* disable input/output flow control, disable restart chars */
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); //NOLINT(hicpp-signed-bitwise)

    /* disable canonical input, disable echo,
    disable visually erase chars,
    disable terminal-generated signals */
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //NOLINT(hicpp-signed-bitwise)

    /* disable output processing */
    toptions.c_oflag &= ~OPOST; //NOLINT(hicpp-signed-bitwise)
    
    /* wait for 12 characters to come in before read returns */
    /* WARNING! THIS CAUSES THE read() TO BLOCK UNTIL ALL */
    /* CHARACTERS HAVE COME IN! */
    toptions.c_cc[VMIN] = NUM_CHARS_READ; 
    /* no minimum time to wait before read returns */
    toptions.c_cc[VTIME] = 0;
    /* avoid hangup */
    toptions.c_cflag &= ~HUPCL;
    /* commit the options */
    tcsetattr(fd, TCSANOW, &toptions);
    for(int i = 0; i < SAMPLES_PER_READING; i++) {
        sensorPublishers[i] = n.advertise<std_msgs::Float32>(&"control/science/moisture" [ std::to_string(i)], MESSAGE_CACHE_SIZE);
    }
    
    ros::Timer timer = n.createTimer(ros::Duration(TIMER_CALLBACK_DURATION), moistureCallback);
 
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
    