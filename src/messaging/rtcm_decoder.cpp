
#include <cstring>

#include "rtcm_decoder.h"
#include "message_publisher.h"


void decode_rtcm_imu_msg (double imu[], a1buff_t a1buff)
{
	rtcm_apimu_t rtcm_apimu = { 0 };
	rtcm_old_apimu_t rtcm_old_apimu = { 0 };

    if (a1buff.nlen >= 61)
    {
        memcpy((uint8_t*)&rtcm_apimu, a1buff.buf + 5, sizeof(rtcm_apimu_t));
        imu[0] = rtcm_apimu.MCU_Time * 1e-6;
        imu[1] = rtcm_apimu.AX * 1.0 / 0x08888889; /* fx */
        imu[2] = rtcm_apimu.AY * 1.0 / 0x08888889; /* fy */
        imu[3] = rtcm_apimu.AZ * 1.0 / 0x08888889; /* fz */
        imu[4] = rtcm_apimu.WX * 1.0 / 0x0048D15A; /* wx */
        imu[5] = rtcm_apimu.WY * 1.0 / 0x0048D15A; /* wy */
        imu[6] = rtcm_apimu.WZ * 1.0 / 0x0048D15A; /* wz */
        imu[7] = rtcm_apimu.OG_WZ * 1.0 / 0x0048D15A; /* wz_fog */
        imu[8] = rtcm_apimu.ODO * 0.01; /* odr */
        imu[9] = rtcm_apimu.ODO_time * 1.0e-9; /* odr time */
        imu[10] = rtcm_apimu.Temp_C * 0.01; /* temp */
        imu[11] = rtcm_apimu.Sync_Time * 1.0e-9;

    }
    else
    {
        memcpy((uint8_t*)&rtcm_old_apimu, a1buff.buf + 5, sizeof(rtcm_old_apimu_t));
        imu[0] = rtcm_old_apimu.MCU_Time * 1e-6;
        imu[1] = rtcm_old_apimu.AX * 1.0 / 0x08888889; /* fx */
        imu[2] = rtcm_old_apimu.AY * 1.0 / 0x08888889; /* fy */
        imu[3] = rtcm_old_apimu.AZ * 1.0 / 0x08888889; /* fz */
        imu[4] = rtcm_old_apimu.WX * 1.0 / 0x0048D15A; /* wx */
        imu[5] = rtcm_old_apimu.WY * 1.0 / 0x0048D15A; /* wy */
        imu[6] = rtcm_old_apimu.WZ * 1.0 / 0x0048D15A; /* wz */
        imu[7] = rtcm_old_apimu.OG_WZ * 1.0 / 0x0048D15A; /* wz_fog */
        imu[8] = rtcm_old_apimu.ODO * 0.01; /* odr */
        imu[9] = rtcm_old_apimu.ODO_time * 1.0e-9; /* odr time */
        imu[10] = rtcm_old_apimu.Temp_C * 0.01; /* temp */
    }

}

void decode_rtcm_ins_msg (double ins[], a1buff_t a1buff)
{
	rtcm_apins_t rtcm_apins = { 0 };

    memcpy((uint8_t*)&rtcm_apins, a1buff.buf + 5, sizeof(rtcm_apins_t));

    ins[0] = rtcm_apins.Time * 1e-6; /* MCU Time */
    ins[1] = rtcm_apins.GPS_Time; /* GPS Time */
    ins[2] = rtcm_apins.Status; /* Solution Status */

    ins[3] = rtcm_apins.Latitude * 1.0e-7; /* lat */
    ins[4] = rtcm_apins.Longitude * 1.0e-7; /* lon */
    ins[5] = rtcm_apins.Alt_ellipsoid * 1.0e-3; /* Altitude */

    ins[6] = rtcm_apins.Vn * 1.0e-3; /* vn */
    ins[7] = rtcm_apins.Ve * 1.0e-3; /* ve */
    ins[8] = rtcm_apins.Vd * 1.0e-3; /* vd */

    ins[9] = rtcm_apins.Roll * 1.0e-5; /* Roll */
    ins[10] = rtcm_apins.Pitch * 1.0e-5; /* Pitch */
    ins[11] = rtcm_apins.Heading_Yaw * 1.0e-5; /* Heading */
    ins[12] = rtcm_apins.ZUPT; /* zupt */

}

int decode_rtcm_gps_msg (double gps[], a1buff_t a1buff)
{
	rtcm_apgps_t rtcm_apgps = { 0 };

    memcpy((uint8_t*)&rtcm_apgps, a1buff.buf + 5, sizeof(rtcm_apgps_t));
    gps[0] = rtcm_apgps.Time * 1e-6; /* time MCU */
    gps[1] = rtcm_apgps.GPS_Time; /* GPS ns */
    gps[2] = rtcm_apgps.Latitude * 1.0e-7; /* lat */
    gps[3] = rtcm_apgps.Longitude * 1.0e-7; /* lon */
    gps[4] = rtcm_apgps.Alt_ellipsoid * 0.001; /* ht */
    gps[5] = rtcm_apgps.Alt_msl * 0.001; /* msl */

    gps[6] = rtcm_apgps.Speed * 0.001; /* speed */
    gps[7] = rtcm_apgps.Heading * 1.0e-3; /* heading (use 1e-3 same as the firmware code) */
    gps[8] = rtcm_apgps.Hor_Acc * 0.001; /* acc_h */
    gps[9] = rtcm_apgps.Ver_Acc * 0.001; /* acc_v */
    gps[10] = rtcm_apgps.PDOP * 0.01; /* pdop */
    gps[11] = rtcm_apgps.FixType; /* fixtype */
    gps[12] = rtcm_apgps.SatNum; /* sat number */
    gps[13] = rtcm_apgps.Spd_Acc * 1.0e-3; /* acc speed */
    gps[14] = rtcm_apgps.Hdg_Acc * 1.0e-5; /* acc heading */
    gps[15] = rtcm_apgps.RTK_Status; /* rtk fix status */

    if (rtcm_apgps.Antenna_ID == 0)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

void decode_rtcm_hdg_msg (double hdr[], a1buff_t a1buff)
{
	rtcm_aphdr_t rtcm_aphdr = { 0 };

    memcpy((uint8_t*)&rtcm_aphdr, a1buff.buf + 5, sizeof(rtcm_aphdr_t));
    hdr[0] = rtcm_aphdr.MCU_Time * 1e-6; /* time MCU */
    hdr[1] = rtcm_aphdr.GPS_Time; /* GPS ns */

    hdr[2] = rtcm_aphdr.relPosN*1.0e-2; /* n */
    hdr[3] = rtcm_aphdr.relPosE * 1.0e-2; /* d */
    hdr[4] = rtcm_aphdr.resPosD * 1.0e-2; /* d */
    hdr[5] = rtcm_aphdr.relPosLength * 1.0e-2; /* length */

    hdr[6] = rtcm_aphdr.relPosHeading * 1.0e-5; /* heading */
    hdr[7] = rtcm_aphdr.relPosLength_Accuracy * 1.0e-4; /* acc length */
    hdr[8] = rtcm_aphdr.relPosHeading_Accuracy * 1.0e-5; /* heading length */
    hdr[9] = rtcm_aphdr.statusFlags; /* flag */

}