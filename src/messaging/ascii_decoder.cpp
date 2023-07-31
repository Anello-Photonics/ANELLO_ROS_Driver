/********************************************************************************
 * File Name:   ascii_decoder.cpp
 * Description: contains the decoder functions for the ascii messages
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        This is a good location to change units of the values.
 ********************************************************************************/

#include <ros/ros.h>

#include "ascii_decoder.h"
#include "message_publisher.h"

int decode_ascii_gps(char *val[], double *output_val)
{

    /* time [s], lat [deg], lon [deg], ht [m], speed [m/s], heading [deg], hor. accuracy [m], ver. accuracy [m], PDOP, fixType, sat num, gps second [s], pps [s] */
    /*
#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,180.0000,0*65
    */
    output_val[0] = atof(val[1]); /* time MCU */
    output_val[1] = atof(val[2]); /* GPS ns */

    output_val[2] = atof(val[3]); /* lat */
    output_val[3] = atof(val[4]); /* lon */
    output_val[4] = atof(val[5]); /* ht */
    output_val[5] = atof(val[6]); /* msl */

    output_val[6] = atof(val[7]);   /* speed */
    output_val[7] = atof(val[8]);   /* heading */
    output_val[8] = atof(val[9]);   /* acc_h */
    output_val[9] = atof(val[10]);  /* acc_v */
    output_val[10] = atof(val[11]); /* pdop */
    output_val[11] = atof(val[12]); /* fix type */
    output_val[12] = atof(val[13]); /* sat number */
    output_val[13] = atof(val[14]); /* acc speed */
    output_val[14] = atof(val[15]); /* acc heading */
    output_val[15] = atof(val[16]); /* rtk fix status */

    return 1;
}

int decode_ascii_hdr(char *val[], double *output_val)
{
    /*
#APHDG,31527.383,1362269876750000128,2.13,1.60,3.23,4.19,36.92845,0.2796,4.00156,303*59
    */
    output_val[0] = atof(val[1]); /* time MCU */
    output_val[1] = atof(val[2]); /* GPS ns */

    output_val[2] = atof(val[3]); /* n */
    output_val[3] = atof(val[4]); /* d */
    output_val[4] = atof(val[5]); /* d */
    output_val[5] = atof(val[6]); /* length */

    output_val[6] = atof(val[7]);  /* heading */
    output_val[7] = atof(val[8]);  /* acc length */
    output_val[8] = atof(val[9]);  /* acc length */
    output_val[9] = atof(val[10]); /* flag */

    return 1;
}

int decode_ascii_imu(char *val[], int field_num, double *output_val)
{
    /*
    #APIMU,318214.937,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,0.01051,0.0000,318214.548,47.0547*55
    */
    output_val[0] = atof(val[1]);
    int loc = (field_num == 15) ? 3 : 2;
    output_val[1] = atof(val[loc++]);          /* fx */
    output_val[2] = atof(val[loc++]);          /* fy */
    output_val[3] = atof(val[loc++]);          /* fz */
    output_val[4] = atof(val[loc++]);          /* wx */
    output_val[5] = atof(val[loc++]);          /* wy */
    output_val[6] = atof(val[loc++]);          /* wz */
    output_val[7] = atof(val[loc++]);          /* wz_fog */
    output_val[8] = atof(val[loc++]);          /* odr */
    output_val[9] = atof(val[loc++]) * 1.0e-3; /* odr time */
    output_val[10] = atof(val[loc++]);         /* temp */

    return 1;
}

int decode_ascii_ins(char *val[], double *output_val)
{
    /* time[s], lat[radian], lon[radian], ht[m], vn[m / s], ve[m / s], vd[m / s], roll[deg], pitch[deg], yaw[deg] */
    /*
    #APINS,318215,1343773580502990592,1,37.398875500000,-121.979132700000,-27.965002059937,,,,-0.166232,1.773182,0.250746,1*74
    */
    output_val[0] = atof(val[1]);
    output_val[1] = atof(val[2]);
    output_val[2] = atof(val[3]);

    output_val[3] = atof(val[4]);
    output_val[4] = atof(val[5]);
    output_val[5] = atof(val[6]);

    output_val[6] = atof(val[7]);
    output_val[7] = atof(val[8]);
    output_val[8] = atof(val[9]);

    output_val[9] = atof(val[10]);
    output_val[10] = atof(val[11]);
    output_val[11] = atof(val[12]);

    output_val[12] = atoi(val[13]); /* zupt */
    output_val[13] = atoi(val[2]) * 1.0e-9;

    return 1;
}