/********************************************************************************
 * File Name:   health_message.h
 * Description: Definition of the health_message class
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

#include <stdio.h>
#include <cstdlib>
#include "health_message.h"

#ifndef GYRO_DISCREPANCY_THRESHOLD
#define GYRO_DISCREPANCY_THRESHOLD .1   //g
#endif

#ifndef HEADING_STABILITY_THRESHOLD
#define HEADING_STABILITY_THRESHOLD 3  //deg
#endif

#ifndef GOOD_GPS_ACC_TRESHOLD
#define GOOD_GPS_ACC_TRESHOLD 1         //m
#endif

health_message::health_message()
{
    this->buffer_full = false;
    this->wz_mems_current_sum = 0;
    this->wz_fog_current_sum = 0;
    this->wz_mems_moving_average = 0;
    this->wz_fog_moving_average = 0;
    this->circular_buffer_index = 0;

    for (int i = 0; i < IMU_MOVING_AVERAGE_SIZE; i++)
    {
        this->wz_mems_circular_buffer[i] = 0;
        this->wz_fog_circular_buffer[i] = 0;
    }

    this->ins_heading = 0;
    this->gps_heading = 0;
    this->hdg_heading = 0;

    this->gps_accuracy = 0;
    this->rtk_status = 0;
}

void health_message::add_imu_message(double *imu_msg)
{
    double wz = imu_msg[6];
    double wz_fog = imu_msg[7];

    // add new data
    this->wz_mems_current_sum += wz;
    this->wz_fog_current_sum += wz_fog;

    // subtract old data
    this->wz_mems_current_sum -= this->wz_mems_circular_buffer[this->circular_buffer_index];
    this->wz_fog_current_sum -= this->wz_fog_circular_buffer[this->circular_buffer_index];

    // add to circular buffer
    this->wz_mems_circular_buffer[this->circular_buffer_index] = wz;
    this->wz_fog_circular_buffer[this->circular_buffer_index] = wz_fog;

    // increment circular buffer index
    this->circular_buffer_index++;
    if (this->circular_buffer_index >= IMU_MOVING_AVERAGE_SIZE)
    {
        this->circular_buffer_index = 0;
        this->buffer_full = true;
    }

    // update moving average when buffer is full
    if (this->buffer_full)
    {
        this->wz_mems_moving_average = this->wz_mems_current_sum / IMU_MOVING_AVERAGE_SIZE;
        this->wz_fog_moving_average = this->wz_fog_current_sum / IMU_MOVING_AVERAGE_SIZE;
    }
}

void health_message::add_ins_message(double *ins_msg)
{
    this->ins_heading = ins_msg[11];
}

void health_message::add_gps_message(double *gps_msg)
{
    this->gps_heading = gps_msg[7];
    this->gps_accuracy = gps_msg[8];
    this->rtk_status = gps_msg[15];
}

void health_message::add_hdg_message(double *hdg_msg)
{
    this->hdg_baseline = hdg_msg[5];
    this->hdg_heading = hdg_msg[6];
}

//TODO: Implement baseline check
bool health_message::is_baseline_correct()
{
    return true;
}

bool health_message::has_rtk_fix()
{
    return (this->rtk_status >= 2);
}

bool health_message::has_gyro_discrepancy()
{
    bool ret_val = false;

    // if moving average not ready yet, return false
    if (this->buffer_full)
    {
        double diff = this->wz_mems_moving_average - this->wz_fog_moving_average;
        if (diff > GYRO_DISCREPANCY_THRESHOLD)
        {
            ret_val = true;
        }
    }

    return ret_val;
}

bool health_message::has_stable_heading()
{
    bool ret_val = false;

    double hdg_ins_diff = abs(this->ins_heading - this->hdg_heading);
    double gps_ins_diff = abs(this->ins_heading - this->gps_heading);

    if (gps_ins_diff < HEADING_STABILITY_THRESHOLD)
    {
        ret_val = true;
    }

    if (this->is_baseline_correct())
    {
        ret_val = (ret_val) && (hdg_ins_diff < HEADING_STABILITY_THRESHOLD);
    }

    return ret_val;
}

bool health_message::has_good_gps_accuracy()
{
    return (this->gps_accuracy < GOOD_GPS_ACC_TRESHOLD);
}

uint8_t health_message::get_health_status()
{
    uint8_t ret_val = GPS_ACC_POOR;

    if (this->has_rtk_fix())
    {
        ret_val = CM_LEVEL_ACCURACY;
    }
    else if (this->has_gyro_discrepancy())
    {
        ret_val = GYRO_DISCREPANCY;
    }
    else if (!this->has_stable_heading())
    {
        ret_val = HEADING_UNSTABLE;
    } 
    else if (this->has_good_gps_accuracy())
    {
        ret_val = SUB_METER_LEVEL_ACCURACY;
    }
    else
    {
        ret_val = GPS_ACC_POOR;
    }

    return ret_val;
}