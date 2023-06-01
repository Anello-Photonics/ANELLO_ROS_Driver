#include "ppTask.h"
// #include "GlobalConstants.h"
// #include "MiscConsts.h"

// //
// #include "WGS84_EarthModel.h"

// #include "EKF_Math.h"

// //
// #include "Algorithm.h"
// #include "algorithm_parameters.h"

// #include "EKF_Algorithm.h"
// #include "EKF_Propagation.h"
// #include "EKF_Innovation.h"
// #include "EKF_Update.h"
// #include "GPS.h"

#include "data_buff.h"

// #include "globals_ref.h"

#ifndef NO_GGA
#define NO_GGA
#endif

// ----- Variables used for IMU data collection -----
//  Inertial sensor data structure
#ifndef NO_GGA
IMU       asm330   = { 0 };
FOG       optGyro  = { 0 };
Odometer  odometer = { 0 };
// GPS data structures
GPS       ublox_f9p = { 0 };
GPS       ublox_f9h = { 0 };

/* GNSS data */
gnss_t gnss = { 0 };

//
EKF_InputStruct       EKF_Input = { 0 };
EKF_StateStruct       EKF_States = { 0 };
EKF_VarStruct         EKF_Vars = { 0 };

EKF_SensorCharStruct  EKF_SensorChars = { 0 };

EarthModel earthModel = { 0 };

EKF_InputStruct* pEKF_Input = &EKF_Input;

AnelloModule_t      gAnelloModule;
#endif

#define  DEG_TO_RAD  (0.017453292519943295)
#define  MSEC_TO_SEC  (0.001f)

#ifndef NO_GGA
static void PopulateDataStruct_GPS(double* gpsdata, GPS* gps)
{

    // This is the routine called based on the input file
    // Extract data from the GPS message and placed in KF input structure
    gps->imu_time.decimal_sec = (float)gpsdata[0] * MSEC_TO_SEC;
    gps->itow_msec = (uint32_t)gpsdata[1];

    // Position (lat/lon/alt)
    gps->lat.decimalVal_deg = gpsdata[2];   // gps data is in [deg]
    gps->lon.decimalVal_deg = gpsdata[3];   // gps data is in [deg]

    gps->lat.decimalVal_rad = gps->lat.decimalVal_deg * DEG_TO_RAD;   // [rad]
    gps->lon.decimalVal_rad = gps->lon.decimalVal_deg * DEG_TO_RAD;   // [rad]

    gps->h_ell_m = (float)gpsdata[4];   // Elem. 4 is h_ell; elem 5 is h_msl (unused)
    gps->h_msl_m = (float)gpsdata[5];

    // Speed and heading
    gps->speed_mps = (float)gpsdata[6];
    gps->heading_motion_deg = -(float)gpsdata[7];
    gps->heading_motion_rad = gps->heading_motion_deg * (float)DEG_TO_RAD;

    gps->velocity_mps[0] = gps->speed_mps * sinf(-gps->heading_motion_rad);
    gps->velocity_mps[1] = gps->speed_mps * cosf(gps->heading_motion_rad);

    gps->posAccurEst_m[0] = (float)gpsdata[8];
    gps->posAccurEst_m[1] = (float)gpsdata[9];
    gps->p_dop = (float)gpsdata[10];

    gps->speedAccur_mps = (float)gpsdata[13];
    gps->headingAccur_deg = (float)gpsdata[14];
    gps->headingAccur_rad = gps->headingAccur_deg * (float)DEG_TO_RAD;

    gps->fixType = (int)(gpsdata[11]);  /* fixType */
    gps->nSats = (int)gpsdata[12];    /* sat number */
    //gps->rtk = (int)gpsdata[15];
}
#endif

#ifndef NO_GGA
static void PopulateDataStruct_IMU(double* imuData, IMU* imu, FOG* fog, Odometer* odom)
{
    // Place in IMU data structure
    imu->time.secCntr = (unsigned int)(imuData[0]);
    imu->time.msecCntr = (unsigned int)((imuData[0] - (double)(imu->time.secCntr)) * 1000.0);
    imu->time.usecCntr = (unsigned int)(((imuData[0] - (double)(imu->time.secCntr)) * 1000.0 - (double)(imu->time.msecCntr)) * 1000.0 + 0.5);

    // ASM330 puts out data in [g]
    imu->accel.scaledValues[0] = (float)(imuData[1]);
    imu->accel.scaledValues[1] = (float)(imuData[2]);
    imu->accel.scaledValues[2] = (float)(imuData[3]);

    // ASM330 puts out data in [dps]
    imu->angRate.scaledValues[0] = (float)(imuData[4]);
    imu->angRate.scaledValues[1] = (float)(imuData[5]);
    imu->angRate.scaledValues[2] = (float)(imuData[6]);

    // If the FOG is used, overwrite the ASM angular-rate with the FOG rate
    fog->angRate_combined[2] = (float)(imuData[7]);

    // Odometer input (from the file) is in [mps]
    /* remove the abs for odometer */
    odom->speed_mps[0] = (float)(imuData[8]);  /* ODOM counter */
    odom->tOdom = (float)(imuData[9]); /* ODOM timer */
}
#endif

#ifndef NO_GGA
extern int add_gps_ant1(double *gps)
{
    PopulateDataStruct_GPS(gps, &ublox_f9p);
    gnss.status |= OK_NAV_PVT_ANT1;
    if (gnss.status & OK_NAV_PVT_ANT1)
    {
        Algo_GPSUpdate((void*)&gAnelloModule, 1);
    }
    gnss.status = 0;
    return 0;
}
#endif
#ifndef NO_GGA
extern int add_gps_ant2(double *gps)
{
    PopulateDataStruct_GPS(gps, &ublox_f9h);
    gnss.status |= OK_NAV_PVT_ANT2;
    if (gnss.status & OK_NAV_PVT_ANT2)
    {
        /* 2nd antenna */
        Algo_GPSUpdate((void*)&gAnelloModule, 2);
    }
    gnss.status = 0;
    return 0;
}
#endif
#ifndef NO_GGA
extern int add_hdr_ant2(ubx_nav_relposned_t* nav_relposned)
{
    gnss.nav_relposned = *nav_relposned;
    gnss.status |= OK_NAV_RELPOSNED_ANT2;
    if (gnss.status & OK_NAV_RELPOSNED_ANT1 || gnss.status & OK_NAV_RELPOSNED_ANT2)
    {
        Algo_HDRUpdate((void*)&gAnelloModule, 2);
    }
    gnss.status = 0;
    return 0;
}
#endif
#ifndef NO_GGA
extern int add_imu_data(double *imu)
{
    PopulateDataStruct_IMU(imu, &asm330, &optGyro, &odometer);

    EKF_SetInput_IMU((void*)&gAnelloModule);
    EKF_SetInput_FOG((void*)&gAnelloModule);

    Algo_Calculations((void*)&gAnelloModule);
	return 0;
}
#endif
#ifndef NO_GGA
extern int get_ins_solu(double *ins)
{
    /* convert position to GPS */
    int ret = 1;
    double r_N[3] = { EKF_States.r_N[0], EKF_States.r_N[1], EKF_States.r_N[2] };
    float* lb = EKF_States.lao_out;
    float ln[3] = { 0 };
    if (EKF_GetHeadingInitializationState(gAnelloModule.pEKF_Vars) == INITIALIZED)
    {
        /* compensation level arm offset to output solution @ GPS ANT 1 */
        ln[0] = EKF_States.R_BinN[0][0] * lb[0] + EKF_States.R_BinN[0][1] * lb[1] + EKF_States.R_BinN[0][2] * lb[2];
        ln[1] = EKF_States.R_BinN[1][0] * lb[0] + EKF_States.R_BinN[1][1] * lb[1] + EKF_States.R_BinN[1][2] * lb[2];
        ln[2] = EKF_States.R_BinN[2][0] * lb[0] + EKF_States.R_BinN[2][1] * lb[1] + EKF_States.R_BinN[2][2] * lb[2];
        r_N[0] += ln[1] * earthModel.RMh_Inv; /* N meter to radian */
        r_N[1] += ln[0] * earthModel.RNh_Inv * earthModel.secLat; /* E meter to radian */
        r_N[2] += ln[2];
        ret = 2;
    }
    ins[0] = r_N[0];
    ins[1] = r_N[1];
    ins[2] = r_N[2];
    ins[3] = (double)(EKF_States.v_N[1]); /* noth */
    ins[4] = (double)(EKF_States.v_N[0]); /* east */
    ins[5] = -(double)(EKF_States.v_N[2]); /* down */
    ins[6] = (double)(EKF_States.theta_BinN[1]); /* roll */
    ins[7] = (double)(EKF_States.theta_BinN[0]); /* pitch */
    ins[8] = -(double)(EKF_States.theta_BinN[2]);/* heading */
    ins[9] = EKF_States.aBias_B[1] / GRAVITY;
    ins[10] = EKF_States.aBias_B[0] / GRAVITY;
    ins[11] =-EKF_States.aBias_B[2] / GRAVITY;
    ins[12] = EKF_States.wBias_B[1] * R2D;
    ins[13] = EKF_States.wBias_B[0] * R2D;
    ins[14] =-EKF_States.wBias_B[2] * R2D;
    ins[15] = EKF_States.aSF_B[1];
    ins[16] = EKF_States.aSF_B[0];
    ins[17] = -EKF_States.aSF_B[2];
    ins[18] = EKF_States.wSF_B[1];
    ins[19] = EKF_States.wSF_B[0];
    ins[20] = -EKF_States.wSF_B[2];
    ins[21] = EKF_States.sfo;
    /* correct v_N due to lever arm offset */
    /* change to use uncorrected measurement */
    float M[3][3] = { { 0 } };
    double C[3][3] = {0}; /* b -> n */
    double R = ins[6];
    double P = ins[7];
    double H = ins[8];
    C[0][0] = cos(H) * cos(P);
    C[1][0] = sin(H) * cos(P);
    C[2][0] = -sin(P);
    C[0][1] = -sin(H) * cos(R) + cos(H) * sin(P) * sin(R);
    C[1][1] = cos(H) * cos(R) + sin(H) * sin(P) * sin(R);
    C[2][1] = cos(P) * sin(R);
    C[0][2] = sin(H) * sin(R) + cos(H) * sin(P) * cos(R);
    C[1][2] = -cos(H) * sin(R) + sin(H) * sin(P) * cos(R);
    C[2][2] = cos(P) * cos(R);

    /* $\omega_^b_{ib} \times l^b $ */
    double lao_rwc[3] = { EKF_States.lao_rwc[1], EKF_States.lao_rwc[0], -EKF_States.lao_rwc[2] };
    double wx = pEKF_Input->wMeas_S[0];
    double wy = pEKF_Input->wMeas_S[1];
    double wz = pEKF_Input->wMeas_S[2];
    M[0][0] = 0;    M[0][1] = -wz;  M[0][2] = +wy;
    M[1][0] = +wz;  M[1][1] = 0;    M[1][2] = -wx;
    M[2][0] = -wy;  M[2][1] = +wx;  M[2][2] = 0;
    double vb_rwc[3] = { 0 };
    vb_rwc[0] = M[0][0] * lao_rwc[0] + M[0][1] * lao_rwc[1] + M[0][2] * lao_rwc[2];
    vb_rwc[1] = M[1][0] * lao_rwc[0] + M[1][1] * lao_rwc[1] + M[1][2] * lao_rwc[2];
    vb_rwc[2] = M[2][0] * lao_rwc[0] + M[2][1] * lao_rwc[1] + M[2][2] * lao_rwc[2];
    double vn_rwc[3] = { 0 };
    vn_rwc[0] = C[0][0] * vb_rwc[0] + C[0][1] * vb_rwc[1] + C[0][2] * vb_rwc[2];
    vn_rwc[1] = C[1][0] * vb_rwc[0] + C[1][1] * vb_rwc[1] + C[1][2] * vb_rwc[2];
    vn_rwc[2] = C[2][0] * vb_rwc[0] + C[2][1] * vb_rwc[1] + C[2][2] * vb_rwc[2];

    ins[3] += vn_rwc[0];
    ins[4] += vn_rwc[1];
    ins[5] += vn_rwc[2];
    ins[22] = wz;
    return ret;
}
#endif

#ifndef NO_GGA
extern int set_sys_err(double* bf, double* bw, double* sff, double* sfw, double sfo)
{
    EKF_States.aBias_B[0] = bf[1] * GRAVITY;
    EKF_States.aBias_B[1] = bf[0] * GRAVITY;
    EKF_States.aBias_B[2] =-bf[2] * GRAVITY;
    EKF_States.wBias_B[0] = bw[1] * D2R;
    EKF_States.wBias_B[1] = bw[0] * D2R;
    EKF_States.wBias_B[2] =-bw[2] * D2R;
    //EKF_States.aSF_B[0] = sff[1];
    //EKF_States.aSF_B[1] = sff[0];
    //EKF_States.aSF_B[2] =-sff[2];
    //EKF_States.wSF_B[0] = sfw[1];
    //EKF_States.wSF_B[1] = sfw[0];
    //EKF_States.wSF_B[2] =-sfw[2];
    EKF_States.sfo = sfo;
    return 0;
}
#endif

#ifndef NO_GGA
extern int set_sys_lao(double* ant1, double* ant2, double* out, double* rmc)
{
    EKF_States.lao_ant1[0] = (float)ant1[1];
    EKF_States.lao_ant1[1] = (float)ant1[0];
    EKF_States.lao_ant1[2] = (float)(- ant1[2]);
    EKF_States.lao_ant2[0] = (float)ant2[1];
    EKF_States.lao_ant2[1] = (float)ant2[0];
    EKF_States.lao_ant2[2] = (float)(- ant2[2]);
    EKF_States.lao_out[0] = (float)out[1];
    EKF_States.lao_out[1] = (float)out[0];
    EKF_States.lao_out[2] = (float)(- out[2]);
    EKF_States.lao_rwc[0] = (float)rmc[1];
    EKF_States.lao_rwc[1] = (float)rmc[0];
    EKF_States.lao_rwc[2] = (float)( - rmc[2]);
    return 0;
}
#endif

#ifndef NO_GGA
extern int reset_system()
{
    memset(&asm330, 0, sizeof(IMU));
    memset(&optGyro, 0, sizeof(FOG));
    memset(&odometer, 0, sizeof(Odometer));
    memset(&ublox_f9p, 0, sizeof(GPS));
    memset(&ublox_f9h, 0, sizeof(GPS));
    memset(&EKF_Input, 0, sizeof(EKF_InputStruct));
    memset(&EKF_States, 0, sizeof(EKF_StateStruct));
    memset(&EKF_Vars, 0, sizeof(EKF_VarStruct));
    memset(&EKF_SensorChars, 0, sizeof(EKF_SensorCharStruct));
    memset(&earthModel, 0, sizeof(EarthModel));
    memset(&gnss, 0, sizeof(gnss_t));

    //Initialize the data strcuture gAnelloModule
    memset(&gAnelloModule, 0, sizeof(gAnelloModule));
    // FIXME: does this work for the simulation?
    gAnelloModule.pAsm330_Arr[0] = &asm330;
    gAnelloModule.pOptGyro = &optGyro;
    gAnelloModule.pOdometer = &odometer;
    gAnelloModule.pUblox_f9p = &ublox_f9p;
    gAnelloModule.pUblox_f9h = &ublox_f9h;
    gAnelloModule.pEKF_Input = &EKF_Input;
    gAnelloModule.pEKF_States = &EKF_States;
    gAnelloModule.pEKF_Vars = &EKF_Vars;
    gAnelloModule.pEKF_SensorChars = &EKF_SensorChars;
    gAnelloModule.pEarthModel = &earthModel;

    gAnelloModule.pGNSS = &gnss;
    return 0;
}
#endif

#ifndef NO_GGA
extern int add_buf_ant(uint8_t* buf, int len, int ant)
{
    int ret = 0, i = 0;
    ubxbuff_t* ubx = ant == 1 ? &gnss.ant1_buff : &gnss.ant2_buff;
    ubx_nav_pvt_t *nav_pvt = ant == 1 ? &gnss.nav_pvt_ant1 : &gnss.nav_pvt_ant2;
    ubx_nav_hpposllh_t* nav_hpposllh = ant == 1 ? &gnss.nav_hpposllh_ant1 : &gnss.nav_hpposllh_ant2;
    ubx_nav_relposned_t* nav_relposned = &gnss.nav_relposned;
    for (i = 0; i < len; ++i)
    {
        ret = input_ubx_data(ubx, buf[i]);
        if (ret)
        {
            if (ubx->type1 == 0x01 && ubx->type2 == 0x07)
            {
                if (decode_ubx_01_07(ubx->buf, ubx->nlen, nav_pvt))
                {
                    /* process nav pvt */
                }
            }
            if (ubx->type1 == 0x01 && ubx->type2 == 0x14)
            {
                if (decode_ubx_01_14(ubx->buf, ubx->nlen, nav_hpposllh))
                {
                    /* process nav hpposllh */
                }
            }
            if (ubx->type1 == 0x01 && ubx->type2 == 0x3c)
            {
                if (decode_ubx_01_3c(ubx->buf, ubx->nlen, nav_relposned))
                {
                    /* process nav relposned */
                    //Algo_HDRUpdate((void*)&gAnelloModule);
                }
            }
        }
    }
    return ret;
}
#endif


#ifndef NO_GGA
extern int set_nhc_sigma(uint8_t side, uint8_t vert, uint8_t forward)
{
    gAnelloModule.algo_setting.nhc_sigma_side = side;
    gAnelloModule.algo_setting.nhc_sigma_vert = vert;
    gAnelloModule.algo_setting.speed_sigma = forward;
    return 0;
}
#endif

#ifndef NO_GGA
extern int set_err_sigma(double bf_sigma_hori, double bf_sigma_vert, double bw_sigma_hori, double bw_sigma_vert)
{
    if (bw_sigma_hori > 0) gAnelloModule.algo_setting.bw_sigma_hori = bw_sigma_hori;
    if (bw_sigma_vert > 0) gAnelloModule.algo_setting.bw_sigma_vert = bw_sigma_vert;

    if (bf_sigma_hori > 0) gAnelloModule.algo_setting.bf_sigma_hori = bf_sigma_hori;
    if (bf_sigma_vert > 0) gAnelloModule.algo_setting.bf_sigma_vert = bf_sigma_vert;
    return 0;
}
#endif

#ifndef NO_GGA
extern int set_ini_sol(double* blh, double *vel, double *att, double pos_sigma_hori, double pos_sigma_vert, double vel_sigma_hori, double vel_sigma_vert, double att_sigma_hori, double att_sigma_vert)
{
    // -------- Initialize the Kalman filter variables --------
    if (pos_sigma_hori > 0)
    {
        gAnelloModule.algo_setting.pos_sigma_hori = pos_sigma_hori * 100;
        if (gAnelloModule.algo_setting.pos_sigma_hori < 1) 
            gAnelloModule.algo_setting.pos_sigma_hori = 1;
    }
    if (pos_sigma_vert > 0)
    {
        gAnelloModule.algo_setting.pos_sigma_vert = pos_sigma_vert * 100;
        if (gAnelloModule.algo_setting.pos_sigma_vert < 1)
            gAnelloModule.algo_setting.pos_sigma_vert = 1;
    }
    if (vel_sigma_hori > 0)
    {
        gAnelloModule.algo_setting.vel_sigma_hori = vel_sigma_hori * 100;
        if (gAnelloModule.algo_setting.vel_sigma_hori < 1)
            gAnelloModule.algo_setting.vel_sigma_hori = 1;
    }
    if (vel_sigma_vert > 0)
    {
        gAnelloModule.algo_setting.vel_sigma_vert = vel_sigma_vert * 100;
        if (gAnelloModule.algo_setting.vel_sigma_vert < 1)
            gAnelloModule.algo_setting.vel_sigma_vert = 1;
    }
    if (att_sigma_hori > 0)
    {
        if (att_sigma_hori < 25.5)
            gAnelloModule.algo_setting.att_sigma_roll = gAnelloModule.algo_setting.att_sigma_pitch = att_sigma_hori * 10;
        else
            gAnelloModule.algo_setting.att_sigma_roll = gAnelloModule.algo_setting.att_sigma_pitch = 255;
    }
    if (att_sigma_vert > 0)
    {
        if (att_sigma_vert < 25.5)
            gAnelloModule.algo_setting.att_sigma_heading = att_sigma_vert * 10;
        else
            gAnelloModule.algo_setting.att_sigma_heading = 255;
    }

    EKF_InitializeAlgorithmVariables(&gAnelloModule);
    gAnelloModule.pEKF_Vars->attInitCntr = 0;
    gAnelloModule.pEKF_Vars->headingInitCntr = 0;

    // Set the initialization state flags false

    /* init position */
    gAnelloModule.pEKF_Vars->positionInitCompletionFlag = 1;

    // Place GPS LLA into position variable
    gAnelloModule.pEKF_States->r_N[0] = blh[0];
    gAnelloModule.pEKF_States->r_N[1] = blh[1];
    gAnelloModule.pEKF_States->r_N[2] = blh[2];

    WGS84_EarthModel(&gAnelloModule);

    /* init velocity */
    gAnelloModule.pEKF_States->v_N[0] = (float)vel[1];
    gAnelloModule.pEKF_States->v_N[1] = (float)vel[0];
    gAnelloModule.pEKF_States->v_N[2] =(float)(-vel[2]);

    /* init attitude (roll, pitch) */
    gAnelloModule.pEKF_Vars->attInitCompletionFlag = 1;
    /* init heading */
    gAnelloModule.pEKF_Vars->headingInitCompletionFlag = 1;

    gAnelloModule.pEKF_States->theta_BinN[0] = (float)att[1];
    gAnelloModule.pEKF_States->theta_BinN[1] = (float)att[0];
    gAnelloModule.pEKF_States->theta_BinN[2] = (float)(-att[2]);

    if (gAnelloModule.pEKF_States->theta_BinN[2] > PI) gAnelloModule.pEKF_States->theta_BinN[2] -= (float)(PI + PI);
    if (gAnelloModule.pEKF_States->theta_BinN[2] < -PI) gAnelloModule.pEKF_States->theta_BinN[2] += (float)(PI + PI);

    // compute the attitude quaternion
    EAngsToQuat_float(&gAnelloModule.pEKF_States->theta_BinN[0], &gAnelloModule.pEKF_States->q_BinN[0]);
    Q_BinNToR_BinN_float(&gAnelloModule.pEKF_States->q_BinN[0], &gAnelloModule.pEKF_States->R_BinN[0][0]);
    EKF_Update_R_NinB(gAnelloModule.pEKF_States);

    //gAnelloModule.pEKF_States->aBias_B[0] = 0.0f;
    //gAnelloModule.pEKF_States->aBias_B[1] = 0.0f;
    //gAnelloModule.pEKF_States->aBias_B[2] = 0.0f;

    // Use the accumulated angular-rate readings to find the angular-rate bias
    // estimates.
    //gAnelloModule.pEKF_States->wBias_B[0] = 0;
    //gAnelloModule.pEKF_States->wBias_B[1] = 0;
    //gAnelloModule.pEKF_States->wBias_B[2] = 0;

    // Reset the counter variables in case the system is reset.  To recompute the bias
    // estimates, pEKF_Vars->attInitCompletionFlag must be set false.
    gAnelloModule.pEKF_Vars->attInitCntr = 0;

    gAnelloModule.pEKF_Vars->insStatus = 2;
    return 1;
}
#endif