/********************************************************************************
 * File Name:   anello_data_port.cpp
 * Description: Defines the anello_data_port class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        
 ********************************************************************************/

#include "../main_anello_ros_driver.h"
#include "../bit_tools.h"

#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <unistd.h>
#include <string>
#include <sys/ioctl.h>
#include <dirent.h>
#include <vector>

#include "anello_data_port.h"

anello_data_port::anello_data_port(const interface_config_t *config) : uart_port(), ethernet_port(config->remote_ip, 1, config->local_data_port)
{
    this->config.type = config->type;
    this->config.data_port_name = config->data_port_name;
    this->config.config_port_name = config->config_port_name;
    this->config.remote_ip = config->remote_ip;
    this->config.local_data_port = config->local_data_port;
    this->config.local_config_port = config->local_config_port;
    this->config.local_odometer_port = config->local_odometer_port;

    this->decode_success = false;
    this->port_index = 0;
    this->fail_count = 0;

    // get all /dev/ttyUSB* ports
    DIR *dir = opendir(PORT_DIR);
    if (nullptr == dir)
    {
#if COMPILE_WITH_ROS2
        RCLCPP_INFO(rclcpp::get_logger("anello_ros_driver"), "Failed to open port directory");
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
        }
    }
    closedir(dir);

    //check for auto-detect
    if (strcmp(this->config.data_port_name.c_str(), "AUTO") == 0)
    {
#if DEBUG_SERIAL
        DEBUG_PRINT("Auto detect data port enabled");
#endif
        this->auto_detect = true;
    }
    else
    {
        this->auto_detect = false;
    }
}

anello_data_port::~anello_data_port()
{
    this->ethernet_port.~ethernet_interface();
    this->uart_port.~serial_interface();
}

void anello_data_port::init()
{
    if (this->config.type == ETH)
    {
        this->init_ethernet();
    }
    else
    {
        this->init_uart();
    }
}

void anello_data_port::init_uart()
{
    if (this->auto_detect)
    {
        /**/
        this->config.data_port_name = this->port_names[this->port_index];
    }

#if DEBUG_SERIAL
    DEBUG_PRINT("Data: Trying port %s", this->uart_port.get_portname().c_str());
#endif

    this->uart_port.init(this->config.data_port_name);
}

void anello_data_port::init_ethernet()
{
    this->ethernet_port.init();
}

void anello_data_port::port_parse_fail()
{
    if (this->config.type == ETH)
    {
        this->port_parse_fail_ethernet();
    }
    else
    {
        this->port_parse_fail_uart();
    }
}

void anello_data_port::port_parse_fail_uart()
{
    //exit if auto detect is off or decode success is true
    if (this->decode_success || !this->auto_detect || !this->uart_port.get_port_enabled())
    {
        return;
    }

#if DEBUG_SERIAL
    DEBUG_PRINT("Data: Failed to parse data on port %s", this->uart_port.get_portname().c_str());
#endif

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
        this->config.data_port_name = this->port_names[this->port_index];

        //reset port
        this->uart_port.close_port();
        this->uart_port.init(this->config.data_port_name);
    }
}

void anello_data_port::port_parse_fail_ethernet()
{
    /* Nothing to do */
}

void anello_data_port::port_confirm()
{
    if (this->config.type == ETH)
    {
        this->port_confirm_ethernet();
    }
    else
    {
        this->port_confirm_uart();
    }
}

void anello_data_port::port_confirm_uart()
{
    //reset fail count
    this->fail_count = 0;
    if (this->decode_success)
    {
        return;
    }

    //set decode success
    this->decode_success = true;

#if DEBUG_SERIAL
    DEBUG_PRINT("Data: Confirmed port %s", this->uart_port.get_portname().c_str());
#endif
}

void anello_data_port::port_confirm_ethernet()
{
    /* Nothing to do */
}

size_t anello_data_port::get_data(char *buf, size_t buf_len)
{
    if (this->config.type == ETH)
    {
        return this->get_data_ethernet(buf, buf_len);
    }
    else
    {
        return this->get_data_uart(buf, buf_len);
    }
}

size_t anello_data_port::get_data_uart(char *buf, size_t buf_len)
{
    size_t bytes_read = this->uart_port.get_data(buf, buf_len, 10);
    if (bytes_read == 0)
    {
        this->port_parse_fail();
        return 0;
    }

    return bytes_read;
}

size_t anello_data_port::get_data_ethernet(char *buf, size_t buf_len)
{
    return this->ethernet_port.get_data(buf, buf_len);
}

void anello_data_port::write_data(const char *buf, size_t buf_len)
{
    if (this->config.type == ETH)
    {
        this->write_data_ethernet(buf, buf_len);
    }
    else
    {
        this->write_data_uart(buf, buf_len);
    }
}

void anello_data_port::write_data_uart(const char *buf, size_t buf_len)
{
    this->uart_port.write_data(buf, buf_len);
}

void anello_data_port::write_data_ethernet(const char *buf, size_t buf_len)
{
    this->ethernet_port.write_data(buf, buf_len);
}