/********************************************************************************
 * File Name:   message_publisher.h
 * Description: header for message_publisher.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        All functions publish messages with out transforming the data
 ********************************************************************************/

#ifndef MESSAGE_PUBLISHER_H
#define MESSAGE_PUBLISHER_H
#include "main_anello_ros_driver.h"
#include "health_message.h"


#if COMPILE_WITH_ROS
#include <ros/ros.h>

/*
 * Parameters:
 * double *imu : Double array at least 12 items long that contains the imu msg fields
 * ros::Publisher pub : Publisher used to publish the imu message
 *
 * Notes:
 * The publisher is called here
 */
void publish_imu(double *imu, ros::Publisher pub);

/*
 * Parameters:
 * double *im1 : Double array at least 10 items long that contains the im1 msg fields
 * ros::Publisher pub : Publisher used to publish the im1 message
 *
 * Notes:
 * The publisher is called here
 */
void publish_im1(double *im1, ros::Publisher pub);

/*
 * Parameters:
 * double *ins : Double array at least 13 items long that contains the ins msg fields
 * ros::Publisher pub : Publisher used to publish the ins message
 *
 * Notes:
 * The publisher is called here
 */
void publish_ins(double *ins, ros::Publisher pub);

/*
 * Parameters:
 * double *gps : Double array at least 16 items long that contains the gps msg fields
 * ros::Publisher pub : Publisher used to publish the gps message
 *
 * Notes:
 * The publisher is called here
 */
void publish_gps(double *gps, ros::Publisher pub);

/*
 * Parameters:
 * double *gp2 : Double array at least 16 items long that contains the gps msg fields
 * ros::Publisher pub : Publisher used to publish the gps message
 *
 * Notes:
 * The publisher is called here
 */
void publish_gp2(double *gp2, ros::Publisher pub);

/*
 * Parameters:
 * double *hdg : Double array at least 10 items long that contains the hdg msg fields
 * ros::Publisher pub : Publisher used to publish the hdg message
 *
 * Notes:
 * The publisher is called here
 */
void publish_hdr(double *hdg, ros::Publisher pub);


/*
 * Parameters:
 * double *gps : Double array at least 16 items long that contains the gps msg fields
 * ros::Publisher pub : Publisher used to publish the gga message
 *
 * Notes:
 * The publisher is called here
 */
void publish_gga(double *gps, ros::Publisher pub);


/*
 * Parameters:
 * health_message *health_msg : Pointer to the health message object used to create statistics
 * ros::Publisher pub : Publisher used to publish the health message
 *
 * Notes:
 * The publisher is called here
 */
void publish_health(const health_message *health_msg, ros::Publisher pub);
#endif

#endif