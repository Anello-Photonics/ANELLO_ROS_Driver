#ifndef MESSAGE_PROCESSING_H
#define MESSAGE_PROCESSING_H

#include <ros/ros.h>

void process_imu(double* imu, ros::Publisher pub);
void process_ins(double* ins, ros::Publisher pub);
void process_gps(double* gps, ros::Publisher pub);
void process_hdr(double* hdg, ros::Publisher pub);

#endif