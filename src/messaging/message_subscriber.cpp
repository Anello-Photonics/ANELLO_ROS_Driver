/********************************************************************************
 * File Name:   ros_subscriber.cpp
 * Description: contains functions that subscribe to ROS topics.
 *
 * Author:      Austin Johnson
 * Date:        8/17/23
 *
 * License:     MIT License
 *
 * Note:        
 ********************************************************************************/

#include <string>
#include "main_anello_ros_driver.h"
#include "ntrip_buffer.h"
#include "bit_tools.h"

#if COMPILE_WITH_ROS
#include <ros/ros.h>
#include "mavros_msgs/RTCM.h"
#include "anello_ros_driver/APODO.h"
#include "anello_ros_driver/init_heading.h"

#ifndef APODO_HEADER
#define APODO_HEADER "APODO"
#endif

#ifndef APINI_HEADER
#define APINI_HEADER "APINI"
#endif



// subscribe to the ntrip client 
void ntrip_rtcm_callback(const mavros_msgs::RTCM::ConstPtr& msg)
{
#if DEBUG_SUBSCRIBERS
    ROS_INFO("RTCM callback");
#endif
    global_ntrip_buffer.add_data_to_buffer(msg->data.data(), msg->data.size());
}


void apodo_callback(const anello_ros_driver::APODOConstPtr& msg)
{
#if DEBUG_SUBSCRIBERS
    ROS_INFO("APODO callback");
#endif

    //format odo speed into string
    std::stringstream speed_body;
    speed_body << std::fixed << std::setprecision(2) << msg->odo_speed;    

    //fill out the body of the message
    std::stringstream message_body;
    message_body << APODO_HEADER;
    message_body << ',';
    message_body << speed_body.str();
    std::string message_body_str = message_body.str();

    // get checksum
    std::string ck_string = compute_checksum(message_body_str.c_str(), message_body_str.length());

    //put message together
    std::stringstream full_message;
    full_message << '#';
    full_message << message_body_str;
    full_message << '*';
    full_message << ck_string;
    full_message << "\r\n";
    std::string full_message_str = full_message.str();


#if DEBUG_SUBSCRIBERS
    ROS_INFO("%s", full_message_str.c_str());
#endif
    global_config_buffer.add_data_to_buffer((uint8_t *)full_message_str.c_str(), full_message_str.length());
    // global_config_buffer.add_data_to_buffer((uint8_t *)full_message_str.c_str(), full_message_str.length());
}

bool init_heading_callback(anello_ros_driver::init_heading::Request &req, anello_ros_driver::init_heading::Response &res)
{

#if DEBUG_SUBSCRIBERS
    ROS_INFO("init_heading callback");
#endif

    const int resp_max_data = 150;
    char port_response[resp_max_data];
    float heading = req.init_heading;

    std::stringstream message_body;
    message_body << APINI_HEADER;
    message_body << ',';
    message_body << "hdg";
    message_body << ',';
    message_body << heading;

    std::string ck_string = compute_checksum(message_body.str().c_str(), message_body.str().length());

    std::stringstream full_message;
    full_message << '#';
    full_message << message_body.str();
    full_message << '*';

    //transform ck_string to lowercase
    full_message << ck_string;

    std::string full_message_str = full_message.str();

#if DEBUG_SUBSCRIBERS
    ROS_INFO("Send: %s", full_message_str.c_str());
#endif
    res.is_success = true;
    // return true;

    gp_global_config_port->write_data((char *)full_message_str.c_str(), full_message_str.length());
    sleep(.1);
    int bytes_rec = gp_global_config_port->get_data(port_response, resp_max_data);

#if DEBUG_SUBSCRIBERS
    ROS_INFO("resp: %s %d", port_response, bytes_rec);
#endif
    // global_config_buffer.add_data_to_buffer((uint8_t *)full_message_str.c_str(), full_message_str.length());
    return true;
}

#endif



