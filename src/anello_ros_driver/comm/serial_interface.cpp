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
#include "bit_tools.h"

#if COMPILE_WITH_ROS
#include <ros/ros.h>
#endif

#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>
#include <dirent.h>
#include <vector>
// #include <libusb1.0/libusb.h>

#include "serial_interface.h"

#define DEBUG 0
#define MAX_READ_NUM 1000
#define SER_PORT_FLUSH_COUNT 20

serial_interface::serial_interface()
{
    this->portname = "";
    this->usb_fd = -1;
    this->port_enabled = false;
}

void serial_interface::init(std::string portname)
{
    this->portname = portname;


    //Check for disabled port
    if (strcmp(this->portname.c_str(), "OFF") == 0)
    {
#if DEBUG_SERIAL
#if COMPILE_WITH_ROS
        ROS_INFO("Serial port disabled");
#else
        printf("Serial port disabled\n");
#endif
#endif
        this->port_enabled = false;
        return;
    }

    this->usb_fd = open(this->portname.c_str(), O_RDWR);
    if (this->usb_fd < 0)
    {
#if COMPILE_WITH_ROS
        ROS_INFO("file open error with port %s", this->portname.c_str());
#else
        printf("File open error with port %s\n", this->portname.c_str());
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
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
#endif
        exit(1);
    }

    options.c_cflag &= ~PARENB; // disable parity bit
    options.c_cflag &= ~CSTOPB; // use one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;      // 8-bit data
    options.c_cflag &= ~CRTSCTS; // disable hardware flow control
    options.c_iflag = 0; // disable software flow control
    options.c_lflag = 0;         // no signaling characters, no echo
    options.c_oflag = 0;         // no remapping, no delays
    options.c_cc[VMIN] = 0;      // read doesn't block
    options.c_cc[VTIME] = 5;     // 0.5 seconds read timeout

    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    
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
#if DEBUG_SERIAL
#if COMPILE_WITH_ROS
    ROS_INFO("Serial port %s initialized", this->portname.c_str());
#else
    printf("Serial port %s initialized\n", this->portname.c_str());
#endif
#endif
    this->port_enabled = true;
}

size_t serial_interface::get_data(char *buf, size_t buf_len)
{
    if (!this->port_enabled)
    {
        return 0;
    }
    if (this->usb_fd < 0)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("ANELLO ROS driver serial port file descriptor not defined");
#else
        printf("ANELLO ROS driver serial port file descriptor not defined");
#endif
        exit(1);
    }

    size_t bytes_read = read(this->usb_fd, buf, (buf_len * sizeof(char)) - 1);
    buf[bytes_read] = '\0';
    return bytes_read;
}

size_t serial_interface::get_data(char *buf, size_t buf_len, int timeout)
{
    if (!this->port_enabled)
    {
        return 0;
    }
    if (this->usb_fd < 0)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("ANELLO ROS driver serial port file descriptor not defined");
#else
        printf("ANELLO ROS driver serial port file descriptor not defined");
#endif
        exit(1);
    }

    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(this->usb_fd, &readSet);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = timeout * 1000;

    int ready = select(this->usb_fd + 1, &readSet, NULL, NULL, &tv);
    if (ready < 0)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("Error from select: %s\n", strerror(errno));
#else
        printf("Error from select: %s\n", strerror(errno));
#endif
        exit(1);
    }
    else if (ready == 0)
    {
        return 0;
    }

    return serial_interface::get_data(buf, buf_len);
}

void serial_interface::write_data(const char *buf, size_t buf_len) 
{ 
    if (!this->port_enabled)
    {
        return;
    }
    write(usb_fd, buf, buf_len);
}

const std::string serial_interface::get_portname()
{
    return this->portname;
}

bool serial_interface::get_port_enabled()
{
    return this->port_enabled;
}

void serial_interface::close_port()
{
    if (this->usb_fd > 0)
    {
        tcflush(this->usb_fd, TCIOFLUSH);
        close(this->usb_fd);
        this->usb_fd = -1;
    }
}

serial_interface::~serial_interface()
{
    this->close_port();
}