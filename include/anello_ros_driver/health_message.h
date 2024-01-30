#ifndef HEALTH_MESSAGE_H
#define HEALTH_MESSAGE_H
/********************************************************************************
 * File Name:   health_message.h
 * Description: Declaration of the health_message class
 *
 * Author:      Austin Johnson
 * Date:        1/29/24
 *
 * License:     MIT License
 *
 * Note: This class is used to store the health status of the system. It is
 *       updated by the health_message class and read by the main thread. Finally,
 *       it is published by the main thread.       
 ********************************************************************************/

#include <cstdint>

#ifndef IMU_MOVING_AVERAGE_SIZE
#define IMU_MOVING_AVERAGE_SIZE 10
#endif



enum HEALTH_STATUS_FLAGS
{
    CM_LEVEL_ACCURACY,
    SUB_METER_LEVEL_ACCURACY,
    GPS_ACC_POOR,
    HEADING_UNSTABLE,
    GYRO_DISCREPANCY
};

class health_message {
private:
    // imu message information
    bool buffer_full;
    double wz_mems_circular_buffer[IMU_MOVING_AVERAGE_SIZE];
    double wz_fog_circular_buffer[IMU_MOVING_AVERAGE_SIZE];
    double wz_mems_current_sum;
    double wz_fog_current_sum;
    double wz_mems_moving_average;
    double wz_fog_moving_average;
    int circular_buffer_index;

    // ins message information
    double ins_heading;

    // gps message information
    double gps_heading;
    double gps_accuracy;
    double rtk_status;

    // hdg message information
    double hdg_baseline;
    double hdg_heading;

    // helper functions
    bool has_rtk_fix();
    bool has_gyro_discrepancy();
    bool has_stable_heading();
    bool has_good_gps_accuracy();

    bool is_baseline_correct();
public:
    void add_imu_message(double *data);
    void add_ins_message(double *data);
    void add_gps_message(double *data);
    void add_hdg_message(double *data);
    uint8_t get_health_status();
};

















#endif // HEALTH_MESSAGE_H