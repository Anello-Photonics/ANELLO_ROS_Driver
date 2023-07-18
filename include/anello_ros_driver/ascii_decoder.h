#ifndef ASCII_DECODER_H
#define ASCII_DECODER_H

#include "decoder.h"

/*
* Parameters:
* char *val[] : array of char arrays will be decoded and published
* ros::Publisher pub_gps : Publisher used to publish the decoded message to ROS
*
* Return:
* 1 if the decode succeeded
*/
int decode_ascii_gps(char *val[], ros::Publisher pub_gps);



/*
* Parameters:
* char *val[] : array of char arrays will be decoded and published
* ros::Publisher hdg_pub : Publisher used to publish the decoded message to ROS
*
* Return:
* 1 if the decode succeeded
*/
int decode_ascii_hdr(char *val[], ros::Publisher pub_hdg);



/*
* Parameters:
* char *val[] : array of char arrays will be decoded and published
* ros::Publisher pub_imu : Publisher used to publish the decoded message to ROS
*
* Return:
* 1 if the decode succeeded
*/
int decode_ascii_imu(char *val[], int field_num, ros::Publisher pub_imu);



/*
* Parameters:
* char *val[] : array of char arrays will be decoded and published
* ros::Publisher pub_ins : Publisher used to publish the decoded message to ROS
*
* Return:
* 1 if the decode succeeded
*/
int decode_ascii_ins(char *val[], ros::Publisher pub_ins);

#endif