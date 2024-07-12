/********************************************************************************
 * File Name:   ethernet_interface.h
 * Description: header file for the ethernet_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        This class is used to read and write data to the ANELLO unit over ethernet.
 ********************************************************************************/

#ifndef ETHERNET_INTERFACE_H
#define ETHERNET_INTERFACE_H

#include <string>
#include <netinet/in.h> 

#include "base_interface.h"

class ethernet_interface : base_interface
{
protected:
    std::string remote_ip_address;
    int local_port;
    int remote_port;

    int sockfd;
    struct sockaddr_in servaddr, cliaddr;

public:
    ethernet_interface(std::string remote_ip_address, int remote_port, int local_port);
    ~ethernet_interface();

    void init();

    size_t get_data(char *buf, size_t buf_len);
    void write_data(const char *buf, size_t buf_len);
};

#endif