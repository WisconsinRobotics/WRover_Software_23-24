#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

using namespace std;

int serial_port;
struct termios tty;

int setupSerial(int baudrate) {
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    tty.c_cflag &= ~PARENB;  // Disable parity
    tty.c_cflag &= ~CSTOPB;  // One stop bit
    tty.c_cflag |= CS8;      // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Turn off hardware RTS control
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;  // Allows us to read raw data
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off softwre flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 0;   // Return any data recieved immediately, we must handle buffering
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, baudrate); // Set baud rate
    cfsetospeed(&tty, baudrate);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

string getColor() {
    string input;
    cout<<"Enter a hex color: ";
    cin>>input;

    if (input.length() != 6 || (input.find_first_not_of("0123456789abcdefABCDEF") != std::string::npos)) {
        cout<<"Invalid, maybe try without # or 0x";
        return "";
    }

    return input;
}

int main() {
    setupSerial(9600);

    while (true) {
        string color = getColor();
        if (color != "") {
            char const *msg = color.c_str();
            write(serial_port, msg, sizeof(msg));
        }
    }

    return 0;
}

