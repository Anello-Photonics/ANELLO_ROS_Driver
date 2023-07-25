#ifndef RTCM_DECODER_H
#define RTCM_DECODER_H

#include "decoder.h"

/*
* Parameters:
* a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
* ros_publishers_t pub_arr : variable containing pointers to the publishers for each of the message types
*
* return:
* 1 on success
*/
int decode_rtcm_message(a1buff_t a1buff, ros_publishers_t pub_arr);


void decode_rtcm_imu_msg(double imu[], a1buff_t a1buff);


void decode_rtcm_ins_msg(double ins[], a1buff_t a1buff);


int decode_rtcm_gps_msg(double gps[], a1buff_t a1buff);


void decode_rtcm_hdg_msg(double hdg[], a1buff_t a1buff);


#endif