/********************************************************************************
 * File Name:   rtcm_decoder.h
 * Description: header file for the rtcm_decoder.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:
 ********************************************************************************/

#ifndef RTCM_DECODER_H
#define RTCM_DECODER_H

#include "main_anello_ros_driver.h"

enum RECEIVER_ID
{
    GPS1 = 1,
    GPS2 = 2
};

/*
 * Parameters:
 * double imu[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_imu_msg(double imu[], a1buff_t a1buff);

/*
 * Parameters:
 * double im1[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_im1_msg(double im1[], a1buff_t a1buff);

/*
 * Parameters:
 * double ins[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_ins_msg(double ins[], a1buff_t a1buff);

/*
 * Parameters:
 * double gps[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 * Return:
 * 1 if antenna 1
 * 2 if antenna 2
 */
int decode_rtcm_gps_msg(double gps[], a1buff_t a1buff);

/*
 * Parameters:
 * double hdg[] : pointer to an array of size 'MAXFIELDS' which will be filled with the values of the message
 * a1buff_t a1buff : buffer variable where the rtcm message is buffered and information about it is stored
 *
 */
void decode_rtcm_hdg_msg(double hdg[], a1buff_t a1buff);

#endif
