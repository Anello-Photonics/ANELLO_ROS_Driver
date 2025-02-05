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
 * double *imu : Double array that has the angular rate values followed by wz values stored 1 through 7
 *                where 6 is the an wz measurement but 7 is the value that is actually used.
 *                Accel should be in G and Angular rate should be in deg/s (these are converted in this function).
 *  ros::Publisher : Publisher that will be used to publish the sensor_msgs/Imu message
 * double last_roll : The most recent recorded roll value in degrees
 * double last_pitch : The most recent recorded pitch value in degrees
 * double last_heading : The most recent recorded heading value in degrees
 * 
 * Notes:
 * This is called on every IMU message. The roll, pitch, heading are from the most recent valid ahrs or INS output.
 * The publisher is called here.
*/
void publish_standard_imu(double *imu, ros::Publisher pub, double last_roll, double last_pitch, double last_heading);

/*
 * Parameters:
 * double *ahrs : Double array at least 6 items long that contains the ahrs msg fields
 * ros::Publisher pub : Publisher used to publish the im1 message
 *
 * Notes:
 * The publisher is called here
 */
void publish_ahrs(double *ahrs, ros::Publisher pub);

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