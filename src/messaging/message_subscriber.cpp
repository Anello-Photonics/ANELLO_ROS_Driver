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

#include "main_anello_ros_driver.h"
#include "ntrip_buffer.h"

#if COMPILE_WITH_ROS
#include <ros/ros.h>
#include "mavros_msgs/RTCM.h"


// subscribe to the ntrip client 
void ntrip_rtcm_callback(const mavros_msgs::RTCM::ConstPtr& msg)
{
    global_ntrip_buffer.add_ntrip_data(msg->data.data(), msg->data.size());
}

#endif



