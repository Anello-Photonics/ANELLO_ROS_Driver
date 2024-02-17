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



enum POSITION_STATUS_FLAGS
{
    CM_LEVEL_ACCURACY,
    SUB_METER_LEVEL_ACCURACY,
    GPS_ACC_POOR
};

enum HEADING_STATUS_FLAGS
{
    HEADING_STABLE,
    HEADING_UNSTABLE
};

enum GYRO_STATUS_FLAGS
{
    GYRO_GOOD,
    GYRO_BAD
};

class health_message {
private:
    // imu message information
    double cur_imu_time;
    bool buffer_full;
    double wz_mems_circular_buffer[IMU_MOVING_AVERAGE_SIZE];
    double wz_fog_circular_buffer[IMU_MOVING_AVERAGE_SIZE];
    double wz_mems_current_sum;
    double wz_fog_current_sum;
    double wz_mems_moving_average;
    double wz_fog_moving_average;

    double wz_fog_std_dev;
    double wz_mems_std_dev;
    
    int circular_buffer_index;

    // ins message information
    double ins_heading;

    // gps message information
    double gps_heading;
    double gps_heading_acc;
    double gps_hacc;
    double rtk_status;
    
    bool gps_read_flag;
    bool hdg_read_flag;
    int gps_ins_mismatch_streak;
    int hdg_ins_mismatch_streak;


    // hdg message information
    double hdg_heading;
    double hdg_heading_acc;
    double hdg_baseline;

    double configured_baseline;

    // helper functions
    bool has_rtk_fix() const;
    bool has_gyro_discrepancy() const;
    bool has_good_gps_accuracy() const;

    void get_current_diff(double *gps_diff, double *hdg_diff);

    bool is_single_antenna_heading_valid();
    bool is_baseline_correct();
public:
    health_message();

    void add_imu_message(double *data);
    void add_ins_message(double *data);
    void add_gps_message(double *data);
    void add_hdg_message(double *data);
    void set_baseline(double baseline);

    uint8_t get_position_status() const;
    uint8_t get_heading_status() const;
    uint8_t get_gyro_status() const;

    // debug functions
    void get_csv_line(double *llh, char *buffer, int len);
    const char *get_csv_header();
};

















#endif // HEALTH_MESSAGE_H