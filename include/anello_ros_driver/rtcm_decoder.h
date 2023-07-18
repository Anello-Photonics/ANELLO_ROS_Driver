#ifndef RTCM_DECODER_H
#define RTCM_DECODER_H

#include "decoder.h"

int decode_rtcm_message(a1buff_t a1buff, ros_publishers_t pub_arr);


#endif