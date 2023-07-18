/** @file serial_interface.c
*
*
* @brief This is a file that defines the object that will represent the serial connection with the EVK
*
*
*
*/

#include <ros/ros.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

#include "serial_interface.h"


#define DEBUG 0
#define MAX_READ_NUM 1000
#define MAX_BUF_LEN 128

serial_interface::serial_interface ()
{
    this->usb_fd = -1;
}

serial_interface::serial_interface (const char *portname)
{
    this->usb_fd = open(portname, O_RDWR);   
    if (this->usb_fd < 0){
        ROS_INFO("file open error");
        exit(1);
    }

    struct termios options; 
    memset(&options, 0, sizeof(options));
    if (tcgetattr(this->usb_fd, &options) != 0) {
        ROS_INFO("Error %i from tcgetattr: %s\n",errno, strerror(errno));
        exit(1);
    }

    options.c_cflag &= ~PARENB;         // disable parity bit
    options.c_cflag &= ~CSTOPB;         // use one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;             // 8-bit data
    options.c_cflag &= ~CRTSCTS;        // disable hardware flow control
    options.c_lflag = 0;                // no signaling characters, no echo
    options.c_oflag = 0;                // no remapping, no delays
    options.c_cc[VMIN] = 0;             // read doesn't block
    options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    if (tcsetattr(this->usb_fd, TCSANOW, &options) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }
}

size_t serial_interface::get_data (char *buf, size_t buf_len)
{
    if (this->usb_fd < 0)
    {
        ROS_ERROR("Anello ros driver serial port file escriptor not defined");
        exit(1);
    }
    size_t bytes_read = read(this->usb_fd, buf, (buf_len * sizeof(char)) - 1);
    buf[bytes_read] = '\0';
    return bytes_read;
}


serial_interface::~serial_interface ()
{
    if (this->usb_fd > 0)
    {
        close(this->usb_fd);
    }
}