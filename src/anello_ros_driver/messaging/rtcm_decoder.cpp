/********************************************************************************
 * File Name:   rtcm_decoder.cpp
 * Description: This file contains the functions for decoding ANELLO RTCM messages.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        This file contains the scalings for RTCM messages based on the ANELLO firmware.
 *
 *              The RTCM messages are decoded into a double array defined in rtcm_decoder.h.
 *
 ********************************************************************************/

#include <cstring>

#include "rtcm_decoder.h"
#include "message_publisher.h"

void decode_rtcm_imu_msg(double imu[], a1buff_t a1buff)
{
    rtcm_apimu_t rtcm_apimu = {0};
    rtcm_old_apimu_t rtcm_old_apimu = {0};

    if (a1buff.nlen >= 61)
    {
        memcpy((uint8_t *)&rtcm_apimu, a1buff.buf + 5, sizeof(rtcm_apimu_t));
        imu[0] = rtcm_apimu.MCU_Time * 1e-6;
        imu[1] = rtcm_apimu.AX * 1.0 / 0x08888889;    /* fx */
        imu[2] = rtcm_apimu.AY * 1.0 / 0x08888889;    /* fy */
        imu[3] = rtcm_apimu.AZ * 1.0 / 0x08888889;    /* fz */
        imu[4] = rtcm_apimu.WX * 1.0 / 0x0048D15A;    /* wx */
        imu[5] = rtcm_apimu.WY * 1.0 / 0x0048D15A;    /* wy */
        imu[6] = rtcm_apimu.WZ * 1.0 / 0x0048D15A;    /* wz */
        imu[7] = rtcm_apimu.OG_WZ * 1.0 / 0x0048D15A; /* wz_fog */
        imu[8] = rtcm_apimu.ODO * 0.01;               /* odr */
        imu[9] = rtcm_apimu.ODO_time * 1.0e-9;        /* odr time */
        imu[10] = rtcm_apimu.Temp_C * 0.01;           /* temp */
        imu[11] = rtcm_apimu.Sync_Time * 1.0e-6;       /* T_Sync ms*/
    }
    else
    {
        memcpy((uint8_t *)&rtcm_old_apimu, a1buff.buf + 5, sizeof(rtcm_old_apimu_t));
        imu[0] = rtcm_old_apimu.MCU_Time * 1e-6;
        imu[1] = rtcm_old_apimu.AX * 1.0 / 0x08888889;    /* fx */
        imu[2] = rtcm_old_apimu.AY * 1.0 / 0x08888889;    /* fy */
        imu[3] = rtcm_old_apimu.AZ * 1.0 / 0x08888889;    /* fz */
        imu[4] = rtcm_old_apimu.WX * 1.0 / 0x0048D15A;    /* wx */
        imu[5] = rtcm_old_apimu.WY * 1.0 / 0x0048D15A;    /* wy */
        imu[6] = rtcm_old_apimu.WZ * 1.0 / 0x0048D15A;    /* wz */
        imu[7] = rtcm_old_apimu.OG_WZ * 1.0 / 0x0048D15A; /* wz_fog */
        imu[8] = rtcm_old_apimu.ODO * 0.01;               /* odr */
        imu[9] = rtcm_old_apimu.ODO_time * 1.0e-9;        /* odr time */
        imu[10] = rtcm_old_apimu.Temp_C * 0.01;           /* temp */
    }
}

void decode_rtcm_im1_msg(double im1[], a1buff_t a1buff)
{
    rtcm_apim1_t rtcm_apim1 = {0};

    memcpy((uint8_t *)&rtcm_apim1, a1buff.buf + 5, sizeof(rtcm_apim1_t));
    im1[0] = rtcm_apim1.MCU_Time * 1e-6;
    im1[1] = rtcm_apim1.AX * 1.0 / 0x08888889;    /* fx */
    im1[2] = rtcm_apim1.AY * 1.0 / 0x08888889;    /* fy */
    im1[3] = rtcm_apim1.AZ * 1.0 / 0x08888889;    /* fz */
    im1[4] = rtcm_apim1.WX * 1.0 / 0x0048D15A;    /* wx */
    im1[5] = rtcm_apim1.WY * 1.0 / 0x0048D15A;    /* wy */
    im1[6] = rtcm_apim1.WZ * 1.0 / 0x0048D15A;    /* wz */
    im1[7] = rtcm_apim1.OG_WZ * 1.0 / 0x0048D15A; /* wz_fog */
    im1[8] = rtcm_apim1.Temp_C * 0.01;           /* temp */
    im1[9] = rtcm_apim1.Sync_Time * 1.0e-6;       /* T_Sync ms*/
}

void decode_rtcm_ahrs_msg(double ahrs[], a1buff_t a1buff)
{
    rtcm_apahrs_t rtcm_apahrs = {0};
    memcpy((uint8_t*)&rtcm_apahrs, a1buff.buf + 5, sizeof(rtcm_apahrs_t));
						
    ahrs[0] = rtcm_apahrs.Time * 1.0e-6;        // System time in ms
    ahrs[1] = rtcm_apahrs.Sync_Time * 1.0e-6;	// Sync time in ms
    ahrs[2] = rtcm_apahrs.Roll * 1.0e-5;	    // roll in deg
    ahrs[3] = rtcm_apahrs.Pitch * 1.0e-5; 	    // pitch in deg
    ahrs[4] = rtcm_apahrs.Yaw * 1.0e-5;	        // yaw in deg
    ahrs[5] = rtcm_apahrs.Zupt_Status;          // Zupt
}



void decode_rtcm_ins_msg(double ins[], a1buff_t a1buff)
{
    rtcm_apins_t rtcm_apins = {0};

    memcpy((uint8_t *)&rtcm_apins, a1buff.buf + 5, sizeof(rtcm_apins_t));

    ins[0] = rtcm_apins.Time * 1e-6; /* MCU Time */
    ins[1] = rtcm_apins.GPS_Time;    /* GPS Time */
    ins[2] = rtcm_apins.Status;      /* Solution Status */

    ins[3] = rtcm_apins.Latitude * 1.0e-7;      /* lat */
    ins[4] = rtcm_apins.Longitude * 1.0e-7;     /* lon */
    ins[5] = rtcm_apins.Alt_ellipsoid * 1.0e-3; /* Altitude */

    ins[6] = rtcm_apins.Vn * 1.0e-3; /* vn */
    ins[7] = rtcm_apins.Ve * 1.0e-3; /* ve */
    ins[8] = rtcm_apins.Vd * 1.0e-3; /* vd */

    ins[9] = rtcm_apins.Roll * 1.0e-5;         /* Roll */
    ins[10] = rtcm_apins.Pitch * 1.0e-5;       /* Pitch */
    ins[11] = rtcm_apins.Heading_Yaw * 1.0e-5; /* Heading */
    ins[12] = rtcm_apins.ZUPT;                 /* zupt */
}

void decode_rtcm_cov_msg(double cov[], a1buff_t a1buff)
{
    rtcm_apcov_t rtcm_apcov = {0};

    memcpy((uint8_t *)&rtcm_apcov, a1buff.buf + 5, sizeof(rtcm_apcov_t));
    cov[0] = rtcm_apcov.Time * 1e-6; /* time MCU */
    cov[1] = rtcm_apcov.covLatLat;   /* lat lat */
    cov[2] = rtcm_apcov.covLonLon;   /* lon lon */
    cov[3] = rtcm_apcov.covAltAlt;   /* alt alt */
    cov[4] = rtcm_apcov.covLatLon;   /* lat lon */
    cov[5] = rtcm_apcov.covLatAlt;   /* lat alt */
    cov[6] = rtcm_apcov.covLonAlt;   /* lon alt */
    cov[7] = rtcm_apcov.covVnVn;     /* vn vn */
    cov[8] = rtcm_apcov.covVeVe;     /* ve ve */
    cov[9] = rtcm_apcov.covVdVd;     /* vd vd */
    cov[10] = rtcm_apcov.covVnVe;    /* vn ve */
    cov[11] = rtcm_apcov.covVnVd;    /* vn vd */
    cov[12] = rtcm_apcov.covVeVd;    /* ve vd */
    cov[13] = rtcm_apcov.covRollRoll;/* roll roll */
    cov[14] = rtcm_apcov.covPitchPitch;/* pitch pitch */
    cov[15] = rtcm_apcov.covYawYaw;  /* yaw yaw */
    cov[16] = rtcm_apcov.covRollPitch;/* roll pitch */
    cov[17] = rtcm_apcov.covRollYaw;  /* roll yaw */
    cov[18] = rtcm_apcov.covPitchYaw; /* pitch yaw */
}

int decode_rtcm_gps_msg(double gps[], a1buff_t a1buff)
{
    rtcm_apgps_t rtcm_apgps = {0};

    memcpy((uint8_t *)&rtcm_apgps, a1buff.buf + 5, sizeof(rtcm_apgps_t));
    gps[0] = rtcm_apgps.Time * 1e-6;           /* time MCU */
    gps[1] = rtcm_apgps.GPS_Time;              /* GPS ns */
    gps[2] = rtcm_apgps.Latitude * 1.0e-7;     /* lat */
    gps[3] = rtcm_apgps.Longitude * 1.0e-7;    /* lon */
    gps[4] = rtcm_apgps.Alt_ellipsoid * 0.001; /* ht */
    gps[5] = rtcm_apgps.Alt_msl * 0.001;       /* msl */

    gps[6] = rtcm_apgps.Speed * 0.001;     /* speed */
    gps[7] = rtcm_apgps.Heading * 1.0e-3;  /* heading (use 1e-3 same as the firmware code) */
    gps[8] = rtcm_apgps.Hor_Acc * 0.001;   /* acc_h */
    gps[9] = rtcm_apgps.Ver_Acc * 0.001;   /* acc_v */
    gps[10] = rtcm_apgps.PDOP * 0.01;      /* pdop */
    gps[11] = rtcm_apgps.FixType;          /* fixtype */
    gps[12] = rtcm_apgps.SatNum;           /* sat number */
    gps[13] = rtcm_apgps.Spd_Acc * 1.0e-3; /* acc speed */
    gps[14] = rtcm_apgps.Hdg_Acc * 1.0e-5; /* acc heading */
    gps[15] = rtcm_apgps.RTK_Status;       /* rtk fix status */

    if (rtcm_apgps.Antenna_ID == 0)
    {
        return GPS1;
    }
    else
    {
        return GPS2;
    }
}

void decode_rtcm_hdg_msg(double hdr[], a1buff_t a1buff)
{
    rtcm_aphdr_t rtcm_aphdr = {0};

    memcpy((uint8_t *)&rtcm_aphdr, a1buff.buf + 5, sizeof(rtcm_aphdr_t));
    hdr[0] = rtcm_aphdr.MCU_Time * 1e-6; /* time MCU */
    hdr[1] = rtcm_aphdr.GPS_Time;        /* GPS ns */

    hdr[2] = rtcm_aphdr.relPosN * 1.0e-2;      /* n */
    hdr[3] = rtcm_aphdr.relPosE * 1.0e-2;      /* d */
    hdr[4] = rtcm_aphdr.resPosD * 1.0e-2;      /* d */
    hdr[5] = rtcm_aphdr.relPosLength * 1.0e-2; /* length */

    hdr[6] = rtcm_aphdr.relPosHeading * 1.0e-5;          /* heading */
    hdr[7] = rtcm_aphdr.relPosLength_Accuracy * 1.0e-4;  /* acc length */
    hdr[8] = rtcm_aphdr.relPosHeading_Accuracy * 1.0e-5; /* heading length */
    hdr[9] = rtcm_aphdr.statusFlags;                     /* flag */
}
