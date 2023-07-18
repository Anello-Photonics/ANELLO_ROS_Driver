#ifndef MESSAGE_PUBLISHING_H
#define MESSAGE_PUBLISHING_H

#include <ros/ros.h>

/*
* Parameters:
* double *imu : Double array at least 12 items long that contains the imu msg fields
* ros::Publisher pub : Publisher used to publish the imu message
*
* Notes:
* The publisher is called here
*/
void process_imu(double* imu, ros::Publisher pub);

/*
* Parameters:
* double *ins : Double array at least 13 items long that contains the ins msg fields
* ros::Publisher pub : Publisher used to publish the ins message
*
* Notes:
* The publisher is called here
*/
void process_ins(double* ins, ros::Publisher pub);

/*
* Parameters:
* double *gps : Double array at least 16 items long that contains the gps msg fields
* ros::Publisher pub : Publisher used to publish the gps message
*
* Notes:
* The publisher is called here
*/
void process_gps(double* gps, ros::Publisher pub);

/*
* Parameters:
* double *hdg : Double array at least 10 items long that contains the hdg msg fields
* ros::Publisher pub : Publisher used to publish the hdg message
*
* Notes:
* The publisher is called here
*/
void process_hdr(double* hdg, ros::Publisher pub);

#endif