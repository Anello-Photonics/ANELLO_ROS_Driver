#ifndef _PP_TASK_H_
#define _PP_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "data_buff.h"

int add_gps_ant1(double *gps);
int add_gps_ant2(double *gps);
int add_hdr_ant2(ubx_nav_relposned_t *nav_relposned);
int add_imu_data(double *imu);
int get_ins_solu(double *ins);
int set_sys_err(double* bf, double* bw, double* sff, double* sfw, double sfo);
int set_sys_lao(double* ant1, double *ant2, double *out, double *rmc);
int reset_system();

int add_buf_ant(uint8_t* buf, int len, int ant);

int set_nhc_sigma(uint8_t side, uint8_t vert, uint8_t forward);
int set_err_sigma(double bf_sigma_hori, double bf_sigma_vert, double bw_sigma_hori, double bw_sigma_vert);

int set_ini_sol(double* blh, double *vel, double *att, double pos_sigma_hori, double pos_sigma_vert, double vel_sigma_hori, double vel_sigma_vert, double att_sigma_hori, double att_sigma_vert);

#ifdef __cplusplus
}
#endif

#endif