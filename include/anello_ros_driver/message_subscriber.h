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

void ntrip_rtcm_callback(const mavros_msgs::RTCM::ConstPtr& msg);


#endif













#endif