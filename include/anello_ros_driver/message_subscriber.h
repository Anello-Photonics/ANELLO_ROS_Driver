/********************************************************************************
 * File Name:   ros_subscriber.h
 * Description: header file message_subscriber.cpp
 *
 * Author:      Austin Johnson
 * Date:        8/17/23
 *
 * License:     MIT License
 *
 * Note:        
 ********************************************************************************/
#ifndef MESSAGE_SUBSCRIBER_H
#define MESSAGE_SUBSCRIBER_H

#include "main_anello_ros_driver.h"


#if COMPILE_WITH_ROS
#include <ros/ros.h>
#include "mavros_msgs/RTCM.h"
#include "anello_ros_driver/APODO.h"
#include "anello_ros_driver/set_heading_no_unc.h"
#include "std_srvs/SetBool.h"

#if APINI_UPD
#include "anello_ros_driver/set_heading_with_unc.h"
#endif

void ntrip_rtcm_callback(const mavros_msgs::RTCM::ConstPtr& msg);

void apodo_callback(const anello_ros_driver::APODOConstPtr& msg);

bool ahrs_set_heading_callback(anello_ros_driver::set_heading_no_unc::Request &req, anello_ros_driver::set_heading_no_unc::Response &res);
bool ahrs_set_zupt_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

#if APINI_UPD
bool init_heading_callback(anello_ros_driver::set_heading_no_unc::Request &req, anello_ros_driver::set_heading_no_unc::Response &res);
bool upd_heading_callback(anello_ros_driver::set_heading_with_unc::Request &req, anello_ros_driver::set_heading_with_unc::Response &res);
#endif

#endif













#endif