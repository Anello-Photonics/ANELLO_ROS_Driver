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
#include "anello_ros_driver/set_heading_no_unc.h"
#include "std_srvs/SetBool.h"

#if APINI_UPD
#include "anello_ros_driver/set_heading_with_unc.h"
#endif

#ifndef APODO_HEADER
#define APODO_HEADER "APODO"
#endif

#ifndef APINI_HEADER
#define APINI_HEADER "APINI"
#endif

#ifndef APUPD_HEADER
#define APUPD_HEADER "APUPD"
#endif



// subscribe to the ntrip client 
void ntrip_rtcm_callback(const mavros_msgs::RTCM::ConstPtr& msg)
{
#if DEBUG_SUBSCRIBERS
    ROS_INFO("RTCM callback: %ld bytes\n", msg->data.size());
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

bool static is_success_message(const char *resp_message, const char *success_header)
{
    return (strstr(resp_message, "err") == NULL) && (strstr(resp_message, "ERR") == NULL) && (strstr(resp_message, success_header) != NULL);
}

bool ahrs_set_zupt_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    const int resp_max_data = 150;
    char port_response[resp_max_data];

    // create the zupt flag string
    if (req.data > 1)
    {
        res.success = false;
        res.message = "Invalid zupt flag";
        return false;
    }

    std::string zupt_flag = req.data ? "1" : "0";
    
    // build message body
    std::stringstream message_body;
    message_body << "APCFG";
    message_body << ',';
    message_body << "w";
    message_body << ',';
    message_body << "azupt";
    message_body << ',';
    message_body << zupt_flag;

    //get checksum
    std::string ck_string = compute_checksum(message_body.str().c_str(), message_body.str().length());

    //put message together
    std::stringstream full_message;
    full_message << '#';
    full_message << message_body.str();
    full_message << '*';
    full_message << ck_string;
    std::string full_message_str = full_message.str();

    gp_global_config_port->write_data((char *)full_message_str.c_str(), full_message_str.length());

    sleep(.005);
    int bytes_rec = gp_global_config_port->get_data(port_response, resp_max_data);

    port_response[bytes_rec] = '\0';
    res.success = is_success_message(port_response, "APCFG");
    res.message = port_response;

#if DEBUG_SUBSCRIBERS
    ROS_INFO("ahrs_set_zupt callback");
    ROS_INFO("Send: %s", full_message_str.c_str());
    ROS_INFO("resp: %s %d", port_response, bytes_rec);
    ROS_INFO("req.data: %d", req.data);
#endif

    return true;
}

bool ahrs_set_heading_callback(anello_ros_driver::set_heading_no_unc::Request &req, anello_ros_driver::set_heading_no_unc::Response &res)
{
    const int resp_max_data = 150;
    char port_response[resp_max_data];
    float heading = req.heading * 1e3f;          // Heading message wants milledegrees

    // build message body
    std::stringstream message_body;
    message_body << "APCFG";
    message_body << ',';
    message_body << "w";
    message_body << ',';
    message_body << "ahdg";
    message_body << ',';
    message_body << heading;

    //get checksum
    std::string ck_string = compute_checksum(message_body.str().c_str(), message_body.str().length());

    //put message together
    std::stringstream full_message;
    full_message << '#';
    full_message << message_body.str();
    full_message << '*';
    full_message << ck_string;
    std::string full_message_str = full_message.str();

    gp_global_config_port->write_data((char *)full_message_str.c_str(), full_message_str.length());
    sleep(.005);
    int bytes_rec = gp_global_config_port->get_data(port_response, resp_max_data);

    port_response[bytes_rec] = '\0';
    res.is_success = is_success_message(port_response, "APCFG");
    res.message = port_response;

#if DEBUG_SUBSCRIBERS
    ROS_INFO("ahrs_set_heading callback");
    ROS_INFO("Send: %s", full_message_str.c_str());
    ROS_INFO("resp: %s %d", port_response, bytes_rec);
#endif

    return true;
}


#if APINI_UPD
//TODO update for uncertainty parameter
bool init_heading_callback(anello_ros_driver::set_heading_no_unc::Request &req, anello_ros_driver::set_heading_no_unc::Response &res)
{

    const int resp_max_data = 150;
    char port_response[resp_max_data];
    float heading = req.heading;

    // build message body
    std::stringstream message_body;
    message_body << APINI_HEADER;
    message_body << ',';
    message_body << "hdg";
    message_body << ',';
    message_body << heading;

    //get checksum
    std::string ck_string = compute_checksum(message_body.str().c_str(), message_body.str().length());

    //put message together
    std::stringstream full_message;
    full_message << '#';
    full_message << message_body.str();
    full_message << '*';
    full_message << ck_string;
    std::string full_message_str = full_message.str();


    // send data to port and get response
    gp_global_config_port->write_data((char *)full_message_str.c_str(), full_message_str.length());
    sleep(.005);
    int bytes_rec = gp_global_config_port->get_data(port_response, resp_max_data);

    // process response
    port_response[bytes_rec] = '\0';
    res.is_success = is_success_message(port_response, APINI_HEADER);
    res.message = port_response;

#if DEBUG_SUBSCRIBERS
    ROS_INFO("init_heading callback");
    ROS_INFO("Send: %s", full_message_str.c_str());
    ROS_INFO("resp: %s %d", port_response, bytes_rec);
#endif

    return true;
}

bool upd_heading_callback(anello_ros_driver::set_heading_with_unc::Request &req, anello_ros_driver::set_heading_with_unc::Response &res)
{
    const int resp_max_data = 150;
    char port_response[resp_max_data];
    float heading = req.heading;
    float uncertainty = req.heading_uncertainty;

    // build message body
    std::stringstream message_body;
    message_body << APUPD_HEADER;
    message_body << ',';
    message_body << "hdg";
    message_body << ',';
    message_body << std::fixed << std::setprecision(1) << heading;
    message_body << ',';
    message_body << "hdg_unc";
    message_body << ',';
    message_body << std::fixed << std::setprecision(1) << uncertainty;

    //get checksum
    std::string ck_string = compute_checksum(message_body.str().c_str(), message_body.str().length());

    //put message together
    std::stringstream full_message;
    full_message << '#';
    full_message << message_body.str();
    full_message << '*';
    full_message << ck_string;
    std::string full_message_str = full_message.str();


    // send data to port and get response
    gp_global_config_port->write_data((char *)full_message_str.c_str(), full_message_str.length());
    sleep(.005);
    int bytes_rec = gp_global_config_port->get_data(port_response, resp_max_data);

    // process response
    port_response[bytes_rec] = '\0';
    res.is_success = is_success_message(port_response, APUPD_HEADER);
    res.message = port_response;

#if DEBUG_SUBSCRIBERS
    ROS_INFO("APUPD_HDG callback");
    ROS_INFO("Send: %s", full_message_str.c_str());
    ROS_INFO("resp: %s %d", port_response, bytes_rec);
#endif

    return true;
}
#endif

#endif



