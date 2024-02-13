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
#define GYRO_DISCREPANCY_THRESHOLD .2   // deg/s
#endif

#ifndef HEADING_STABILITY_THRESHOLD
#define HEADING_STABILITY_THRESHOLD 3  // deg
#endif

#ifndef GOOD_GPS_ACC_TRESHOLD
#define GOOD_GPS_ACC_TRESHOLD 1         // m
#endif

#ifndef BASELINE_ACC_THRESHOLD
#define BASELINE_ACC_THRESHOLD .03
#endif

#ifndef GPS_HEADING_ACC_GOOD_THRESHOLD
#define GPS_HEADING_ACC_GOOD_THRESHOLD 1
#endif

#ifndef HEADING_MISMATCH_COUNT_TH
#define HEADING_MISMATCH_COUNT_TH 4
#endif

health_message::health_message()
{
    this->cur_imu_time = 0.0;
    this->buffer_full = false;
    this->wz_mems_current_sum = 0.0;
    this->wz_fog_current_sum = 0.0;
    this->wz_mems_moving_average = 0.0;
    this->wz_fog_moving_average = 0.0;
    this->circular_buffer_index = 0.0;

    for (int i = 0; i < IMU_MOVING_AVERAGE_SIZE; i++)
    {
        this->wz_mems_circular_buffer[i] = 0.0;
        this->wz_fog_circular_buffer[i] = 0.0;
    }

    this->ins_heading = 0.0;
    this->gps_heading = 0.0;
    this->hdg_heading = 0.0;

    this->hdg_baseline = 0;
    this->configured_baseline = 2.01;

    this->gps_hacc = 0.0;
    this->gps_heading_acc = 0.0;
    this->rtk_status = 0;

    this->gps_read_flag = false;
    this->hdg_read_flag = false;
    this->gps_ins_mismatch_streak = 0;
    this->hdg_ins_mismatch_streak = 0;
}

void health_message::add_imu_message(double *imu_msg)
{
    this->cur_imu_time = imu_msg[0];
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

void health_message::add_ins_message(double* ins_msg)
{
    this->ins_heading = ins_msg[11];
    double ins_status = ins_msg[2];

    double gps_ins_diff, hdg_ins_diff;
    this->get_current_diff(&gps_ins_diff, &hdg_ins_diff);

    // if gps has been read recently update mismatch streak accordingly
    if (this->gps_read_flag)
    {
        this->gps_read_flag = false;

        if (this->is_single_antenna_heading_valid())
        {
            this->gps_read_flag = false;
            //check backward direction as well
            bool match = ((HEADING_STABILITY_THRESHOLD > gps_ins_diff) || ((180 - HEADING_STABILITY_THRESHOLD) < gps_ins_diff));
            if (!match)
            {
                this->gps_ins_mismatch_streak++;
            }
            else
            {
                this->gps_ins_mismatch_streak = 0;
            }
        }
        else
        {
            // what to do in this case?
            this->gps_ins_mismatch_streak = 0;
        }
    }

    // If hdg has been read recently update mismatch streak accordingly
    if (this->hdg_read_flag)
    {
        this->hdg_read_flag = false;
        if (this->is_baseline_correct())
        {

            bool match = (HEADING_STABILITY_THRESHOLD > hdg_ins_diff);
            if (!match)
            {
                this->hdg_ins_mismatch_streak++;
            }
            else
            {
                this->hdg_ins_mismatch_streak = 0;
            }
        }
        else
        {
            // what to do in this case?
            this->hdg_ins_mismatch_streak = 0;
        }
    }

    //reset streak to 0 when INS is not initialized
    if (ins_status < 2)
    {
        this->gps_ins_mismatch_streak = 0;
        this->hdg_ins_mismatch_streak = 0;
    }
}

void health_message::add_gps_message(double *gps_msg)
{
    this->gps_heading = gps_msg[7];
    if (this->gps_heading > 180)
        this->gps_heading -= 360;
    this->gps_hacc = gps_msg[8];
    this->gps_heading_acc = gps_msg[14];
    this->rtk_status = gps_msg[15];

    // tell the system to check gps vs ins at next ins message
    this->gps_read_flag = true;
}

void health_message::add_hdg_message(double *hdg_msg)
{
    this->hdg_baseline = hdg_msg[5];
    this->hdg_heading = hdg_msg[6];
    if (this->hdg_heading > 180)
        this->hdg_heading -= 360;

    // Tell the system to check hdg vs ins at next ins message
    this->hdg_read_flag = true;
}

void health_message::set_baseline(double baseline)
{
    this->configured_baseline = baseline;
}

void health_message::get_current_diff(double *gps_diff_out, double *hdg_diff_out)
{
    double hdg_ins_diff = abs(this->ins_heading - this->hdg_heading);
    double gps_ins_diff = abs(this->ins_heading - this->gps_heading);

    if (hdg_ins_diff > 180)
        hdg_ins_diff = 360 - hdg_ins_diff;
    if (gps_ins_diff > 180)
        gps_ins_diff = 360 - gps_ins_diff;

    *gps_diff_out = gps_ins_diff;
    *hdg_diff_out = hdg_ins_diff;
    return;
}

//TODO: Implement baseline check
bool health_message::is_baseline_correct()
{
    return (abs(this->hdg_baseline - this->configured_baseline) < BASELINE_ACC_THRESHOLD) && (abs(this->wz_fog_moving_average) < 5);
}

bool health_message::is_single_antenna_heading_valid()
{
    return (this->gps_heading_acc < GPS_HEADING_ACC_GOOD_THRESHOLD) && (abs(this->wz_fog_moving_average) < 5);
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

bool health_message::has_good_gps_accuracy()
{
    return (this->gps_hacc < GOOD_GPS_ACC_TRESHOLD);
}

uint8_t health_message::get_position_status()
{
    uint8_t ret_val = GPS_ACC_POOR;

    if (this->has_rtk_fix())
    {
        ret_val = CM_LEVEL_ACCURACY;
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

uint8_t health_message::get_heading_status()
{
    uint8_t ret_val = HEADING_STABLE;

    if (HEADING_MISMATCH_COUNT_TH <= this->gps_ins_mismatch_streak)
    {
        ret_val = HEADING_UNSTABLE;
    }

    if (HEADING_MISMATCH_COUNT_TH <= this->hdg_ins_mismatch_streak)
    {
        ret_val = HEADING_UNSTABLE;
    }

    return ret_val;
}

uint8_t health_message::get_gyro_status()
{
    uint8_t ret_val = GYRO_BAD;

    if (!this->has_gyro_discrepancy())
    {
        ret_val = GYRO_GOOD;
    }

    return ret_val;
}

const char* health_message::get_csv_header()
{
    return "cur_imu_time,lat,lon,alt,mems_avg,fog_avg,ins_heading,gps_heading,hdg_heading,ins_gps_heading_diff,ins_hdg_heading_diff,gps_streak,hdg_streak,hdg_baseline,gps_accuracy,gps_heading_acc,rtk,has_fix,gyro_disc,good_gps_acc,position_status,heading_status,gyro_status\n";
}

void health_message::get_csv_line(double *llh, char *buffer, int len)
{
    double ins_gps_heading_diff, ins_hdg_heading_diff;
    this->get_current_diff(&ins_gps_heading_diff, &ins_hdg_heading_diff);

  //sprintf(buffer, "%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i,%i,%i,%i,%i,%i\n",
    sprintf(buffer, "%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i,%i,%i,%i,\n",
                                                                                                    this->cur_imu_time,
                                                                                                    llh[0], llh[1], llh[2],
                                                                                                    this->wz_mems_moving_average, this->wz_fog_moving_average, 
                                                                                                    this->ins_heading, this->gps_heading, this->hdg_heading, 
                                                                                                    ins_gps_heading_diff, ins_hdg_heading_diff,
                                                                                                    this->gps_ins_mismatch_streak, this->hdg_ins_mismatch_streak,
                                                                                                    this->hdg_baseline, this->gps_hacc, this->gps_heading_acc, this->rtk_status,
                                                                                                    this->has_rtk_fix(), this->has_gyro_discrepancy(),this->has_good_gps_accuracy(),
                                                                                                    this->get_position_status(), this->get_heading_status(), this->get_gyro_status());

}