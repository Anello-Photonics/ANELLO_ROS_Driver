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
}

serial_interface::serial_interface(const char *portname)
{
    this->portname = portname;
    this->usb_fd = -1;
}

void serial_interface::init()
{
    this->usb_fd = open(this->portname.c_str(), O_RDWR);
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
        printf("Anello ros driver serial port file descriptor not defined");
#endif
        exit(1);
    }

    size_t bytes_read = read(this->usb_fd, buf, (buf_len * sizeof(char)) - 1);
    buf[bytes_read] = '\0';
    return bytes_read;
}

void serial_interface::write_data(const char *buf, size_t buf_len) 
{ 
    write(usb_fd, buf, buf_len);
}

serial_interface::~serial_interface()
{
    if (this->usb_fd > 0)
    {
        tcflush(this->usb_fd, TCIOFLUSH);
        close(this->usb_fd);
        this->usb_fd = -1;
    }
}

void anello_config_port::init()
{
    if (strcmp(this->portname.c_str(), "AUTO") == 0)
    {
        // get all /dev/ttyUSB* ports
        DIR *dir = opendir(PORT_DIR);
        if (nullptr == dir)
        {
#if COMPILE_WITH_ROS
            ROS_ERROR("Failed to open port directory");
#else
            printf("Failed to open port directory");
#endif
            exit(1);
        }

        struct dirent *entry;
        std::vector<std::string> port_names;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string temp_port_name = PORT_DIR;
            if (strncmp(entry->d_name, PORT_PREFIX, strlen(PORT_PREFIX)) == 0)
            {
                // port_name = PORT_DIR;
                temp_port_name += entry->d_name;
                // port_names.push_back(temp_port_name);
                port_names.insert(port_names.begin(), temp_port_name);  
                // break;
            }
        }
        closedir(dir);

        // try to open each port and send a command to it
        std::string command = "#APPNG*48\r\n";
        for (std::string port_name : port_names)
        {
            this->portname = port_name;
            serial_interface::init();
            char buf[100];

            this->get_data(buf, 100);
            this->write_data(command.c_str(), command.length()*sizeof(char));
            this->get_data(buf, 100);

            if (strstr(buf, "#APPNG") != nullptr)
            {
                break;
            }
            else
            {
                this->~anello_config_port();
            }
        }

    }
    else
    {
        serial_interface::init();
    }
}

anello_data_port::anello_data_port(const char *ser_port_name) : serial_interface(ser_port_name)
{
    this->decode_success = false;
    this->port_index = 0;

    // get all /dev/ttyUSB* ports
    DIR *dir = opendir(PORT_DIR);
    if (nullptr == dir)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("Failed to open port directory");
#else
        printf("Failed to open port directory");
#endif
        exit(1);
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        std::string temp_port_name = PORT_DIR;
        if (strncmp(entry->d_name, PORT_PREFIX, strlen(PORT_PREFIX)) == 0)
        {
            // port_name = PORT_DIR;
            temp_port_name += entry->d_name;
            port_names.push_back(temp_port_name);
            // this->port_names.insert(this->port_names.begin(), temp_port_name);  
            // break;
        }
    }
    closedir(dir);

    //check for auto-detect
    if (strcmp(this->portname.c_str(), "AUTO") == 0)
    {
        this->auto_detect = true;
    }
    else
    {
        this->auto_detect = false;
    }
}

void anello_data_port::init()
{
    if (this->auto_detect)
    {
        /**/
        this->portname = this->port_names[this->port_index];
    }

    serial_interface::init();
}

void anello_data_port::port_parse_fail()
{
    //exit if auto detect is off or decode success is true
    if (this->decode_success || !this->auto_detect)
    {
        return;
    }

    this->fail_count++;
    
    //move on to next port
    //if max fail count is reached
    if (MAX_PORT_PARSE_FAIL < this->fail_count)
    {
        //reset fail count
        this->fail_count = 0;

        //move to next port
        this->port_index++;
        if (this->port_index >= this->port_names.size())
        {
            this->port_index = 0;
        }

        //set port name
        this->portname = this->port_names[this->port_index];

        //reset port
        // this->~anello_data_port();
        tcflush(this->usb_fd, TCIOFLUSH);
        close(this->usb_fd);
        this->usb_fd = -1;
        
        //reinitialize port
        this->init();
    }
}

void anello_data_port::port_confirm()
{
    //reset fail count
    this->fail_count = 0;

    //set decode success
    this->decode_success = true;
}

size_t anello_data_port::get_data(char *buf, size_t buf_len)
{
    if (this->usb_fd < 0)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("Anello ros driver serial port file descriptor not defined");
#else
        printf("Anello ros driver serial port file descriptor not defined");
#endif
        exit(1);
    }
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(this->usb_fd, &readSet);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10 * 1000; // 10 ms

    int ready = select(this->usb_fd + 1, &readSet, NULL, NULL, &timeout);
    if (ready < 0)
    {
#if COMPILE_WITH_ROS
        ROS_ERROR("Error from select: %s\n", strerror(errno));
#else
        printf("Error from select: %s\n", strerror(errno));
#endif
        // exit(1);
        return 0;
    }
    else if (ready == 0)
    {
        this->port_parse_fail();
        return 0;
    }

    return serial_interface::get_data(buf, buf_len);    
}