#include <ros/ros.h>

#include "ascii_decoder.h"
#include "message_publisher.h"




int decode_ascii_gps (char *val[], ros::Publisher pub_gps)
{

    /* time [s], lat [deg], lon [deg], ht [m], speed [m/s], heading [deg], hor. accuracy [m], ver. accuracy [m], PDOP, fixType, sat num, gps second [s], pps [s] */
    /*
#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,180.0000,0*65
    */
    double gps[20];
    gps[0] = atof(val[1]); /* time MCU */
    gps[1] = atof(val[2]); /* GPS ns */    

    gps[2] = atof(val[3]); /* lat */
    gps[3] = atof(val[4]); /* lon */
    gps[4] = atof(val[5]); /* ht */
    gps[5] = atof(val[6]); /* msl */

    gps[6] = atof(val[7]); /* speed */
    gps[7] = atof(val[8]); /* heading */
    gps[8] = atof(val[9]); /* acc_h */
    gps[9] = atof(val[10]); /* acc_v */
    gps[10] = atof(val[11]); /* pdop */
    gps[11] = atof(val[12]); /* fix type */
    gps[12] = atof(val[13]); /* sat number */
    gps[13] = atof(val[14]); /* acc speed */
    gps[14] = atof(val[15]); /* acc heading */
    gps[15] = atof(val[16]); /* rtk fix status */
    process_gps(gps, pub_gps);


    return 1;
}



int decode_ascii_hdr (char *val[], ros::Publisher pub_hdg)
{
    /*
#APHDG,31527.383,1362269876750000128,2.13,1.60,3.23,4.19,36.92845,0.2796,4.00156,303*59
    */
    double hdr[20];
    hdr[0] = atof(val[1]); /* time MCU */
    hdr[1] = atof(val[2]); /* GPS ns */

    hdr[2] = atof(val[3]); /* n */
    hdr[3] = atof(val[4]); /* d */
    hdr[4] = atof(val[5]); /* d */
    hdr[5] = atof(val[6]); /* length */

    hdr[6] = atof(val[7]); /* heading */
    hdr[7] = atof(val[8]); /* acc length */
    hdr[8] = atof(val[9]); /* acc length */
    hdr[9] = atof(val[10]); /* flag */

    process_hdr(hdr, pub_hdg);
    return 1;
}





int decode_ascii_imu (char *val[], int field_num, ros::Publisher pub_imu)
{
    /*
    #APIMU,318214.937,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,0.01051,0.0000,318214.548,47.0547*55
    */
    double imu[20];
    imu[0] = atof(val[1]);
    int loc = (field_num == 15) ? 3 : 2;
    imu[1] = atof(val[loc++]); /* fx */
    imu[2] = atof(val[loc++]); /* fy */
    imu[3] = atof(val[loc++]); /* fz */
    imu[4] = atof(val[loc++]); /* wx */
    imu[5] = atof(val[loc++]); /* wy */
    imu[6] = atof(val[loc++]); /* wz */
    imu[7] = atof(val[loc++]); /* wz_fog */
    imu[8] = atof(val[loc++]); /* odr */
    imu[9] = atof(val[loc++]) * 1.0e-3; /* odr time */
    imu[10] = atof(val[loc++]); /* temp */


    process_imu(imu, pub_imu);

    return 1;
}



int decode_ascii_ins (char *val[], ros::Publisher pub_ins)
{
    /* time[s], lat[radian], lon[radian], ht[m], vn[m / s], ve[m / s], vd[m / s], roll[deg], pitch[deg], yaw[deg] */
    /*
    #APINS,318215,1343773580502990592,1,37.398875500000,-121.979132700000,-27.965002059937,,,,-0.166232,1.773182,0.250746,1*74
    */
    double ins[20];
    ins[0] = atof(val[1]);
    ins[1] = atof(val[2]);
    ins[2] = atof(val[3]);

    ins[3] = atof(val[4]);
    ins[4] = atof(val[5]);
    ins[5] = atof(val[6]);

    ins[6] = atof(val[7]);
    ins[7] = atof(val[8]);
    ins[8] = atof(val[9]);

    ins[9] = atof(val[10]);
    ins[10] = atof(val[11]);
    ins[11] = atof(val[12]);

    ins[12] = atoi(val[13]); /* zupt */
    ins[13] = atoi(val[2]) * 1.0e-9;
    process_ins(ins, pub_ins);

    return 1;
}