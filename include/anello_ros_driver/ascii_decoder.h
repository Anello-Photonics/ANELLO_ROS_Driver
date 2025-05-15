/********************************************************************************
 * File Name:   ascii_decoder.h
 * Description: header file for the ascii_decoder.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        All functions decode messages with out transforming the data
 ********************************************************************************/

#ifndef ASCII_DECODER_H
#define ASCII_DECODER_H

#include "main_anello_ros_driver.h"

/*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * double *val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_gps(char *val[], double *output_val);

/*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * double *val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_hdr(char *val[], double *output_val);

/*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * int field_num : number of fields in the message
 * double *val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_imu(char *val[], int field_num, double *output_val);

/*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * int field_num : number of fields in the message
 * double *output_val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_im1(char *val[], int field_num, double *output_val);
 
 /*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * int field_num : number of fields in the message
 * double *output_val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_ahrs(char *val[], double *output_val);

/*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * double *val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_ins(char *val[], double *output_val);

/*
 * Parameters:
 * char *val[] : array of char arrays will be decoded and published
 * double *val : pointer to an array of the same size of val which is filled with the decoded values of val
 *
 * Return:
 * 1 if the decode succeeded
 */
int decode_ascii_cov(char *val[], double *output_val);

#endif
