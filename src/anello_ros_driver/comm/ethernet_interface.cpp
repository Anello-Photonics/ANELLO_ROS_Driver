/********************************************************************************
 * File Name:   ethernet_interface.cpp
 * Description: definition for the ethernet_interface class.
 *
 * Author:      Austin Johnson
 * Date:        7/12/24
 *
 * License:     MIT License
 *
 * Note:        This class is used to read and write data to the ANELLO unit over ethernet.
 ********************************************************************************/


#include "main_anello_ros_driver.h"
#include "ethernet_interface.h"

#if COMPILE_WITH_ROS
#include <ros/ros.h>
#endif

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 

ethernet_interface::ethernet_interface(std::string remote_ip_address, int remote_port, int local_port)
{
    
    this->remote_ip_address = remote_ip_address;
    this->local_port = local_port;
    this->remote_port = remote_port;
    this->sockfd = -1;
}

ethernet_interface::~ethernet_interface()
{
    close(this->sockfd);
}

void ethernet_interface::init()
{
#if DEBUG_ETHERNET
    DEBUG_PRINT("Remote IP: %s\n", this->remote_ip_address.c_str());
    DEBUG_PRINT("Remote Port: %d\n", this->remote_port);
    DEBUG_PRINT("Local Port: %d\n", this->local_port);
#endif

    // Creating socket file descriptor
    if ((this->sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&(this->servaddr), 0, sizeof(this->servaddr));
    memset(&(this->cliaddr), 0, sizeof(this->cliaddr));

    // Filling server information
    this->servaddr.sin_family = AF_INET; // IPv4
    this->servaddr.sin_addr.s_addr = INADDR_ANY;
    this->servaddr.sin_port = htons(this->local_port);
    

    // Bind the socket with the server address
    if (bind(this->sockfd, (const struct sockaddr *)&(this->servaddr), sizeof(this->servaddr)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Set the remote address
    memset(&(this->cliaddr), 0, sizeof(this->cliaddr));
    this->cliaddr.sin_family = AF_INET;
    this->cliaddr.sin_port = htons(this->remote_port);
    this->cliaddr.sin_addr.s_addr = inet_addr(this->remote_ip_address.c_str());

}

void ethernet_interface::write_data(const char *buf, size_t buf_len)
{
    sendto(this->sockfd, buf, buf_len, 0, (const struct sockaddr *)&(this->cliaddr), sizeof(this->cliaddr));
}

size_t ethernet_interface::get_data(char *buf, size_t buf_len)
{
    socklen_t len;
    int n = recvfrom(this->sockfd, buf, buf_len, 0, (struct sockaddr *)&(this->cliaddr), &len);
    buf[n] = '\0';
    return n;
}