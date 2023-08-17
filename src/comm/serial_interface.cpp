/********************************************************************************
 * File Name:   serial_interface.cpp
 * Description: Defines the serial_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        set the baud rate in the constructor to default EVK baud of 921600.
 *              This can be changed in the constructor.
 ********************************************************************************/

#include "main_anello_ros_driver.h"

#if COMPILE_WITH_ROS
#include <ros/ros.h>
#endif

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>

#include "serial_interface.h"

#define DEBUG 0
#define MAX_READ_NUM 1000
#define SER_PORT_FLUSH_COUNT 20

serial_interface::serial_interface()
{
    this->usb_fd = -1;
}

serial_interface::serial_interface(const char *portname)
{
    this->usb_fd = open(portname, O_RDWR);
    if (this->usb_fd < 0)
    {
#if COMPILE_WITH_ROS
        ROS_INFO("file open error");
#else
        printf("File open error\n");
#endif
        exit(1);
    }

    struct termios options;
    memset(&options, 0, sizeof(options));
    if (tcgetattr(this->usb_fd, &options) != 0)
    {
#if COMPILE_WITH_ROS
        ROS_INFO("Error %i from tcgetattr: %s\n", errno, strerror(errno));
#else
        printf("Error %i from tcgetattr: %s\n", errnom strerror(errno));
#endif
        exit(1);
    }

    options.c_cflag &= ~PARENB; // disable parity bit
    options.c_cflag &= ~CSTOPB; // use one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8-bit data
    options.c_cflag &= ~CRTSCTS; // disable hardware flow control
    options.c_lflag = 0;         // no signaling characters, no echo
    options.c_oflag = 0;         // no remapping, no delays
    options.c_cc[VMIN] = 0;      // read doesn't block
    options.c_cc[VTIME] = 5;     // 0.5 seconds read timeout

    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);

    // set the options
    if (tcsetattr(this->usb_fd, TCSANOW, &options) != 0)
    {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        exit(1);
    }

    // Flush Port
    for (int i = 0; i < SER_PORT_FLUSH_COUNT; i++)
    {
        usleep(1000);
        ioctl(this->usb_fd, TCFLSH, 2);
    }
}

size_t serial_interface::get_data(char *buf, size_t buf_len)
{
    if (this->usb_fd < 0)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("Anello ros driver serial port file descriptor not defined");
#else
        printf("Anello ros driver serial port file descriptor not defined")
#endif
        exit(1);
    }
    size_t bytes_read = read(this->usb_fd, buf, (buf_len * sizeof(char)) - 1);
    buf[bytes_read] = '\0';
    return bytes_read;
}

serial_interface::~serial_interface()
{
    if (this->usb_fd > 0)
    {
        tcflush(this->usb_fd, TCIOFLUSH);
        close(this->usb_fd);
    }
}