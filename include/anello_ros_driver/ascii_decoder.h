#ifndef ASCII_DECODER_H
#define ASCII_DECODER_H

#include "decoder.h"


int decode_ascii_gps(char *val[], ros::Publisher gps_pub);
int decode_ascii_hdr(char *val[], ros::Publisher pub_hdg);
int decode_ascii_imu(char *val[], int field_num, ros::Publisher pub_imu);
int decode_ascii_ins(char *val[], ros::Publisher pub_ins);

#endif