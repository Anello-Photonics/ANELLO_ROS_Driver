/********************************************************************************
 * File Name:   anello_config_port.cpp
 * Description: Defines the anello_config_port class.
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

#include "anello_config_port.h"

anello_config_port::anello_config_port(const interface_config_t *config) : uart_port(), ethernet_port(config->remote_ip, 2, config->local_config_port)
{
    this->config.type = config->type;
    this->config.data_port_name = config->data_port_name;
    this->config.config_port_name = config->config_port_name;
    this->config.remote_ip = config->remote_ip;
    this->config.local_data_port = config->local_data_port;
    this->config.local_config_port = config->local_config_port;
    this->config.local_odometer_port = config->local_odometer_port;
}

anello_config_port::~anello_config_port() {
    this->ethernet_port.~ethernet_interface();
    this->uart_port.~serial_interface();
}

void anello_config_port::init()
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

void anello_config_port::init_uart()
{
    if (strcmp(this->config.config_port_name.c_str(), "AUTO") == 0)
    {
#if DEBUG_SERIAL
        DEBUG_PRINT("Auto detect config port enabled");
#endif

        // get all /dev/ttyUSB* ports
        DIR *dir = opendir(PORT_DIR);
        if (nullptr == dir)
        {
#if COMPILE_WITH_ROS2
            RCLCPP_ERROR(rclcpp::get_logger("anello_ros_driver"), "Failed to open port directory");
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
                temp_port_name += entry->d_name;
                port_names.insert(port_names.begin(), temp_port_name);  
            }
        }
        closedir(dir);

        // try to open each port and send a command to it
        std::string command = "#APPNG*48\r\n";
        bool port_found = false;
        while (!port_found)
        {        
            for (std::string port_name : port_names)
            {
#if DEBUG_SERIAL
                DEBUG_PRINT("config_port: Trying port %s", port_name.c_str());
#endif
                this->config.config_port_name = port_name;
                this->uart_port.init(this->config.config_port_name);
                char buf[100];

                this->uart_port.get_data(buf, 100, 10);
                this->uart_port.write_data(command.c_str(), command.length()*sizeof(char));
                usleep(500 * 1000); // 500 ms
                this->uart_port.get_data(buf, 100, 10);

#if DEBUG_SERIAL
                DEBUG_PRINT("config_port: Received: %s", buf);
#endif
                if (strstr(buf, "#APPNG") != nullptr)
                {
                    port_found = true;
                    break;
                }
                else
                {
                    this->uart_port.close_port();
                }
            }
            if (!port_found)
            {
                DEBUG_PRINT("No port found");
            }
        }

    }
    else
    {
        this->uart_port.init(this->config.config_port_name);
    }
}

void anello_config_port::init_ethernet()
{
    this->ethernet_port.init();

    std::string command = "#APPNG*48\r\n";
    char buf[100];

    this->ethernet_port.write_data(command.c_str(), command.length()*sizeof(char));
    usleep(500 * 1000); // 500 ms
    this->ethernet_port.get_data(buf, 100); 

#if DEBUG_ETHERNET
    DEBUG_PRINT("Sent: %s", command.c_str());
    DEBUG_PRINT("Received: %s", buf);
#endif
}

size_t anello_config_port::get_data(char *buf, size_t buf_len)
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

size_t anello_config_port::get_data_uart(char *buf, size_t buf_len)
{
    return this->uart_port.get_data(buf, buf_len);
}

size_t anello_config_port::get_data_ethernet(char *buf, size_t buf_len)
{
    return this->ethernet_port.get_data(buf, buf_len);
}

void anello_config_port::write_data(const char *buf, size_t buf_len)
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

void anello_config_port::write_data_uart(const char *buf, size_t buf_len)
{
    this->uart_port.write_data(buf, buf_len);
}

void anello_config_port::write_data_ethernet(const char *buf, size_t buf_len)
{
    this->ethernet_port.write_data(buf, buf_len);
}

double anello_config_port::get_baseline()
{

    std::string command = "#APVEH,R,bsl*65\r\n";
    char *field_array[MAXFIELD];
    char buf[100];
    int num_fields;
    double baseline;

    this->write_data(command.c_str(), command.length()*sizeof(char));
    usleep(500 * 1000); // 500 ms
    this->get_data(buf, 100);

    num_fields = parse_fields(buf, field_array);


    if ((strstr(field_array[0], "APVEH") == nullptr) || (strstr(field_array[1], "bsl") == nullptr))
    {
        baseline = 0.0;
    }
    else
    {
        baseline = atof(field_array[2]);
    }


#if DEBUG_SERIAL
    DEBUG_PRINT("Sent: %s", command.c_str());
    DEBUG_PRINT("Received: %s", buf);
    DEBUG_PRINT("Baseline: %f", baseline);
#endif

    return baseline;
}