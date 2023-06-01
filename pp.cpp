// #include "ppTask.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <cstdlib>
#include <vector>

#include "data_buff.h"

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

#ifndef MAXFIELD
#define MAXFIELD 100
#endif

#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef D2R
#define D2R (PI/180.0)
#endif

#ifndef R2D
#define R2D (180.0/PI)
#endif

#ifndef ae_WGS84
#define ae_WGS84 6378137.0
#endif

#ifndef finv_WGS84
#define finv_WGS84 298.257223563
#endif

#ifndef NO_GGA
#define NO_GGA
#endif

typedef struct
{
	uint8_t buf[MAX_BUF_LEN];	/* buffer where the raw data is held */
	int nseg;					/* number of segments in the message */
	int nbyte;					/* number of bytes in the message */
	int nlen; 					/* length of binary message */
	int type;
	int subtype;
	int crc;
	int loc[MAXFIELD];
}a1buff_t;

static int input_a1_data(a1buff_t* a1, uint8_t data)
{
	int ret = 0, i = 0;
	if (a1->nbyte >= MAX_BUF_LEN) a1->nbyte = 0;
	/* $G , #AP, 0xD3 */
#ifndef NO_GGA
	if (a1->nbyte == 0 && !(data == '#' || data == '$' || data == 0xD3)) { a1->nbyte = 0; return 0; }
	if (a1->nbyte == 1 && !((data == 'A' && a1->buf[0] == '#') || (data == 'G' && a1->buf[0] == '$') || a1->buf[0] == 0xD3)) { a1->nbyte = 0; return 0; }
	if (a1->nbyte == 2 && !((data == 'P' && a1->buf[1] == 'A' && a1->buf[0] == '#') || (a1->buf[1] == '$' && a1->buf[0] == 'G') || a1->buf[0] == 0xD3)) { a1->nbyte = 0; return 0; }
#else
	if (a1->nbyte == 0 && !(data == '#' || data == 0xD3)) { a1->nbyte = 0; return 0; }
	if (a1->nbyte == 1 && !((data == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3)) { a1->nbyte = 0; return 0; }
	if (a1->nbyte == 2 && !((data == 'P' && a1->buf[1] == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3)) { a1->nbyte = 0; return 0; }
#endif
	if (a1->nbyte == 0) memset(a1, 0, sizeof(a1buff_t));
	if (a1->nbyte < 3) { a1->buf[a1->nbyte++] = data; return 0; }
	if (a1->buf[0]!= 0xD3)
	{
		if (data == ',')
		{
			a1->loc[a1->nseg++] = a1->nbyte;
			if (a1->nseg == 2)
			{
#ifndef NO_GGA
				if (strstr((char*)a1->buf, "APANT") != NULL || strstr((char*)a1->buf, "APRTK") != NULL)
				{
					uint8_t* temp = a1->buf + (a1->loc[0]) + 1;
					a1->nlen = (int)atof((char*)temp);
				}
				else
#endif
				{
					a1->nlen = 0;
				}
			}
		}
		a1->buf[a1->nbyte++] = data;
		if (a1->nlen == 0)
		{
			/* check message end for normal asc message */
			if (data == '\r' || data == '\n')
			{
				/* 1*74 */
				if (a1->nbyte > 3 && a1->buf[a1->nbyte - 4] == '*')
				{
					a1->loc[a1->nseg++] = a1->nbyte - 4;
					ret = 1;
				}
			}
		}
#ifndef NO_GGA
		else
		{
			/* check message end for binary message ,binary msg\r\n */
			if (a1->nbyte >= (a1->nlen + a1->loc[1] + 3))
			{
				if (a1->buf[6] == '1') /* APANT1 */
					ret = 2;
				else if (a1->buf[6] == '2') /* APANT2 */
					ret = 3;
				else /* APRTK */
					ret = 4;
			}
		}
#endif
	}
	else
	{
		/* rtcm data */
		a1->buf[a1->nbyte++] = data;
		a1->nlen = getbitu(a1->buf, 14, 10) + 3; /* length without parity */
		if (a1->nbyte >= a1->nlen + 3)
		{
			i = 24;
			a1->type = getbitu(a1->buf, i, 12);
			i += 12;
			/* check parity */
			if (crc24q(a1->buf, a1->nlen) != getbitu(a1->buf, a1->nlen * 8, 24))
			{
				a1->crc = 1;
			}
			else
			{
				a1->crc = 0;
				if (a1->type == 4058)
				{
					/* decode subtype */
					a1->subtype = getbitu(a1->buf, i, 4);
				}
			}
			ret = 5; /* rtcm valid message */
		}
	}
	return ret;
}

#ifndef NO_GGA
static void set_output_file_name(const char* fname, const char* key, char* outfilename)
{
	char filename[255] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
}
#endif

static FILE* set_output_file(const char* fname, const char* key)
{
	char filename[255] = { 0 }, outfilename[255] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
	return fopen(outfilename, "w");
}

static int parse_fields(char* const buffer, char** val)
{
	char* p, * q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if (p == NULL) break;
		if ((q = strchr(p, ',')) || (q = strchr(p, '*')) || (q = strchr(p, '\n')) || (q = strchr(p, '\r'))) {
			val[n++] = p; *q = '\0';
		}
		else break;
	}
	if (p != NULL)
	{
		val[n++] = p;
	}
	return n;
}

static void deg2dms(double deg, double* dms)
{
	double sign = deg < 0.0 ? (-1.0) : (1.0), a = fabs(deg);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = a; dms[0] *= sign;
}
extern int outnmea_gga(unsigned char* buff, float time, int type, double* blh, int ns, float dop, float age)
{
	double h, ep[6], dms1[3], dms2[3];
	char* p = (char*)buff, * q, sum;

	if (type != 1 && type != 4 && type != 5) {
		p += sprintf(p, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
		p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
		return (int)(p - (char*)buff);
	}
	time -= 18.0;
	ep[2] = floor(time / (24 * 3600));
	time -= (float)(ep[2] * 24 * 3600.0);
	ep[3] = floor(time / 3600);
	time -= (float)(ep[3] * 3600);
	ep[4] = floor(time / 60);
	time -= (float)(ep[4] * 60);
	ep[5] = time;
	h = 0.0;
	deg2dms(fabs(blh[0]) * 180 / PI, dms1);
	deg2dms(fabs(blh[1]) * 180 / PI, dms2);
	p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, blh[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, blh[1] >= 0 ? "E" : "W", type,
		ns, dop, blh[2] - h, h, age);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return (int)(p - (char*)buff);
}

#ifndef NO_GGA
static int set_imu_orientation(const char* pAxis, double Cbs[3][3])
{
	int isBoresightSet = 0;
	const char* p = pAxis;
	memset(&Cbs[0][0], 0, sizeof(double) * 9);

	for (int i = 0; i < 3; ++i)
	{
		if (*p == 'x' || *p == 'X')
		{
			Cbs[i][0] = 1.0;
			++p;
		}
		else if (*p == '-' && (*(p + 1) == 'x' || *(p + 1) == 'X')) {
			Cbs[i][0] = -1.0;
			p = p + 2;
		}
		else if (*p == 'y' || *p == 'Y') {
			Cbs[i][1] = 1.0;
			++p;
		}
		else if (*p == '-' && (*(p + 1) == 'y' || *(p + 1) == 'Y')) {
			Cbs[i][1] = -1.0;
			p = p + 2;
		}
		else if (*p == 'z' || *p == 'Z') {
			Cbs[i][2] = 1.0;
			++p;
		}
		else if (*p == '-' && (*(p + 1) == 'z' || *(p + 1) == 'Z')) {
			Cbs[i][2] = -1.0;
			p = p + 2;
		}
		else {
			isBoresightSet = 0;
			memset(Cbs, 0, sizeof(Cbs));
			Cbs[0][0] = 1.0;
			Cbs[1][1] = 1.0;
			Cbs[2][2] = 1.0;
			break;
		}
		++isBoresightSet;
	}
	return isBoresightSet;
}
#endif

void rotate_vector_(double C[3][3], double* vec, double* vel_, int isTranspose)
{
	if (!isTranspose)
	{
		vel_[0] = C[0][0] * vec[0] + C[0][1] * vec[1] + C[0][2] * vec[2];
		vel_[1] = C[1][0] * vec[0] + C[1][1] * vec[1] + C[1][2] * vec[2];
		vel_[2] = C[2][0] * vec[0] + C[2][1] * vec[1] + C[2][2] * vec[2];
	}
	else
	{
		vel_[0] = C[0][0] * vec[0] + C[1][0] * vec[1] + C[2][0] * vec[2];
		vel_[1] = C[0][1] * vec[0] + C[1][1] * vec[1] + C[2][1] * vec[2];
		vel_[2] = C[0][2] * vec[0] + C[1][2] * vec[1] + C[2][2] * vec[2];
	}
}
void rotate_vector(double C[3][3], double* vec, int isTranspose)
{
	double temp[3] = { vec[0], vec[1], vec[2] };
	rotate_vector_(C, temp, vec, isTranspose);
}

#ifndef NO_GGA
static void att2C_nb(double roll, double pitch, double yaw, double C_nb[3][3])
{
	// attitude: roll, pitch and heading
	// attitude => C_nb
	double R = roll, P = pitch, H = yaw;
	C_nb[0][0] = cos(H) * cos(P);
	C_nb[1][0] = sin(H) * cos(P);
	C_nb[2][0] = -sin(P);
	C_nb[0][1] = -sin(H) * cos(R) + cos(H) * sin(P) * sin(R);
	C_nb[1][1] = cos(H) * cos(R) + sin(H) * sin(P) * sin(R);
	C_nb[2][1] = cos(P) * sin(R);
	C_nb[0][2] = sin(H) * sin(R) + cos(H) * sin(P) * cos(R);
	C_nb[1][2] = -cos(H) * sin(R) + sin(H) * sin(P) * cos(R);
	C_nb[2][2] = cos(P) * cos(R);
}
#endif

double lat2local(double lat, double* lat2north)
{
	double f_WGS84 = (1.0 / finv_WGS84);
	double e2WGS84 = (2.0 * f_WGS84 - f_WGS84 * f_WGS84);
	double slat = sin(lat);
	double clat = cos(lat);
	double one_e2_slat2 = 1.0 - e2WGS84 * slat * slat;
	double Rn = ae_WGS84 / sqrt(one_e2_slat2);
	double Rm = Rn * (1.0 - e2WGS84) / (one_e2_slat2);
	*lat2north = Rm;
	return Rn * clat;
}


typedef struct
{
	double time;
	double fxyz[3];
	double wxyz[4];
	double odr;
	double timeodr;
	double temp;
}imu_t;

typedef struct
{
	double time;  /* IMU time */
	double timegps; /* GPS nsec */
	double lat; /* latitude -- deg*/
	double lon;/* longitude -- deg*/
	double ht; /* ht */
	double msl; /* MSL */
	double speed; /* speed [m/s] */
	double heading; /* heading [deg] */
	double acc_h; /* hor. accuracy [m] */
	double acc_v; /* ver. accuracy [m] */
	double pdop; /* PDOP */
	int fixtype; /* fixType */
	int nsat; /* sat number */
	double acc_speed;/* speed accur */
	double acc_heading; /* heading accur */
	int rtk_status; /* rtk fix status */
}pvt_t;

/* dual antenna message */
typedef struct
{
	double time; /* IMU time (use last IMU message */
	ubx_nav_relposned_t nav_relposned;
}hdr_t;

typedef struct
{
	double time;
	double timegps;
	double pos[3];
	double vel[3];
	double att[3];
	int status;
}pva_t;

/* use for gap analysis */
typedef struct
{
	double time;
	double length;
	double dist;
	double time_odr;
}gap_t;

typedef struct
{
	double time;
	double length;
}zupt_t;

/* use for constant speed constraint */
typedef struct
{
	double time;
	double length;
	double value;
}speed_t;

typedef struct
{
	char key[20];
	char axis[10]; /* axis definition, default is XYZ */
	double ant1[3];/* lever arm offset of antenna 1 */
	double ant2[3];/* lever arm offset of antenna 2 */
	double out[3]; /* lever arm offset of output center */
	double rwc[3]; /* lever arm offset of Rear-Wheel-Center */
	double mia[3]; /* mis-alignment angle */
	double timeoffset; /* imu time offset */
	double timeoffset_gps; /* gps time offset */
	double dt;
	bool is_compensate_gps_time;
	bool is_pp; /* run pp or not, default is off */
	bool is_old_imu_data; /* support the old and new IMU RTCM data */
	bool is_rts_enu; /* support the RTCM INS message output bug */
	bool is_ant1;
	bool is_ant2;
	bool is_mems;
	bool is_gps_speed;
	bool csv_header;
	double start_time;
	double end_time;
	std::vector<zupt_t> zupts; /* zupts */
	std::vector<gap_t> gaps; /* gap */
	std::vector<speed_t> speeds; /* constant speed constraint */
	double init_pos[3]; /* initial position (latitude, longitude, height) */
	double init_vel[3]; /* initial velocity (north, east, down) */
	double init_att[3]; /* initial attitude (roll, pitch, heading) */
	double init_pos_sigma_hori; /* latitude, longitude */
	double init_pos_sigma_vert; /* height/up */
	double init_vel_sigma_hori; /* velocity north, velocity east */
	double init_vel_sigma_vert; /* velocity down */
	double init_att_sigma_hori; /* roll & pitch */
	double init_att_sigma_vert; /* heading */
	double init_bf[3]; /* initial acceleration bias */
	double init_bw[3]; /* initial gyro bias */
	double init_bf_sigma_hori;
	double init_bf_sigma_vert;
	double init_bw_sigma_hori;
	double init_bw_sigma_vert;
	double init_sff[3];
	double init_sfw[3];
	double init_sfo;
	uint8_t nhc_sigma_side; /* NHC sigma in side */
	uint8_t nhc_sigma_vert; /* NHC sigma in vert */
	uint8_t speed_sigma; /* odometer sigma (speed in forward direction) */
}setting_t;


#ifndef NO_GGA
static int process_log_data(const char *fname, std::vector<imu_t>& imus, std::vector<pvt_t>& pvts, std::vector<pvt_t>& pv2s, std::vector< hdr_t>&hdrs, std::vector<pva_t>& pvas, setting_t* setting)
{
	FILE* fCSV_PP = NULL;
	FILE* fGGA_PP = NULL;

	std::vector<imu_t>::iterator pimu = imus.begin();
	std::vector<pvt_t>::iterator pgps = pvts.begin();
	std::vector<pvt_t>::iterator pgp2 = pv2s.begin();
	std::vector<hdr_t>::iterator phdr = hdrs.begin();
	std::vector<pva_t>::iterator ppva = pvas.begin();

	double gps[20] = { 0 };
	double gp2[20] = { 0 };
	double imu[20] = { 0 };
	double ins[20] = { 0 };
	double pva[25] = { 0 };


	double Cbs[3][3] = { 0 };
	double Cma[3][3] = { { 1, 0, 0}, { 0, 1, 0 }, { 0, 0, 1 } };
	double pp_speed_heading = 0;
	double dt = 0;
	char outbuff[512] = { 0 };

	printf("ant1=%7.3f,%7.3f,%7.3f\nant2=%7.3f,%7.3f,%7.3f\nout =%7.3f,%7.3f,%7.3f\nrwc =%7.3f,%7.3f,%7.3f\nmia =%7.3f,%7.3f,%7.3f\nimu time offset=%10.4f\ngps time offset=%10.4f\nis_old_imu_data= %i\nis_rts_enu     = %i\n"
		, setting->ant1[0], setting->ant1[1], setting->ant1[2]
		, setting->ant2[0], setting->ant2[1], setting->ant2[2]
		, setting->out[0], setting->out[1], setting->out[2]
		, setting->rwc[0], setting->rwc[1], setting->rwc[2]
		, setting->mia[0], setting->mia[1], setting->mia[2]
		, setting->timeoffset
		, setting->timeoffset_gps
		, setting->is_old_imu_data
		, setting->is_rts_enu
	);

	printf("key            =%s\n", setting->key);
	printf("is_mems        =%i\n", setting->is_mems);
	printf("is_ant1        =%i\n", setting->is_ant1);
	printf("is_ant2        =%i\n", setting->is_ant2);
	printf("is_gps_speed   =%i\n", setting->is_gps_speed);
	printf("csv_header     =%i\n", setting->csv_header);
	printf("nhc_sigma_side =%i\n", setting->nhc_sigma_side);
	printf("nhc_sigma_vert =%i\n", setting->nhc_sigma_vert);
	printf("speed_sigma    =%i\n", setting->speed_sigma);

	printf("start_time     =%.4f\n", setting->start_time);
	printf("end_time       =%.4f\n", setting->end_time  );

	printf("init_att       =%.4f,%.4f,%.4f\n", setting->init_att[0] * R2D, setting->init_att[1] * R2D, setting->init_att[2] * R2D);
	printf("att_sigma_hori =%.4f\n", setting->init_att_sigma_hori);
	printf("att_sigma_vert =%.4f\n", setting->init_att_sigma_vert);

	printf("init_pos       =%.9f,%.9f,%.4f\n", setting->init_pos[0] * R2D, setting->init_pos[1] * R2D, setting->init_pos[2]);
	printf("pos_sigma_hori =%.4f\n", setting->init_pos_sigma_hori);
	printf("pos_sigma_vert =%.4f\n", setting->init_pos_sigma_vert);

	printf("init_vel       =%.4f,%.4f,%.4f\n", setting->init_vel[0], setting->init_vel[1], setting->init_vel[2]);
	printf("vel_sigma_hori =%.4f\n", setting->init_vel_sigma_hori);
	printf("vel_sigma_vert =%.4f\n", setting->init_vel_sigma_vert);

	printf("init_bw        =%.6f,%.6f,%.6f\n", setting->init_bw[0], setting->init_bw[1], setting->init_bw[2]);
	printf("bw_sigma_hori  =%.6f\n", setting->init_bw_sigma_hori);
	printf("bw_sigma_vert  =%.6f\n", setting->init_bw_sigma_vert);

	printf("init_sfw       =%.6f,%.6f,%.6f\n", setting->init_sfw[0], setting->init_sfw[1], setting->init_sfw[2]);

	printf("init_bf        =%.6f,%.6f,%.6f\n", setting->init_bf[0], setting->init_bf[1], setting->init_bf[2]);
	printf("bf_sigma_hori  =%.6f\n", setting->init_bf_sigma_hori);
	printf("bf_sigma_vert  =%.6f\n", setting->init_bf_sigma_vert);

	printf("init_sfo       =%.6f\n", setting->init_sfo);

	for (std::vector<gap_t>::iterator pgap = setting->gaps.begin(); pgap != setting->gaps.end(); ++pgap)
	{
		printf("gap            =%.4f,%.4f\n", pgap->time, pgap->length);
	}
	for (std::vector<zupt_t>::iterator pzupt = setting->zupts.begin(); pzupt != setting->zupts.end(); ++pzupt)
	{
		printf("zupt           =%.4f,%.4f\n", pzupt->time, pzupt->length);
	}
	for (std::vector<speed_t>::iterator pspeed = setting->speeds.begin(); pspeed != setting->speeds.end(); ++pspeed)
	{
		printf("fix_speed      =%.4f,%.4f,%.4f\n", pspeed->time, pspeed->length, pspeed->value);
	}


#ifndef NO_GGA
	set_imu_orientation(setting->axis, Cbs);

	att2C_nb(setting->mia[0] * D2R, setting->mia[1] * D2R, setting->mia[2] * D2R, Cma);

	reset_system();

	set_sys_lao(setting->ant1, setting->ant2, setting->out, setting->rwc);
	set_sys_err(setting->init_bf, setting->init_bw, setting->init_sff, setting->init_sfw, setting->init_sfo);
	set_err_sigma(setting->init_bf_sigma_hori, setting->init_bf_sigma_vert, setting->init_bw_sigma_hori, setting->init_bw_sigma_vert);

	set_nhc_sigma(setting->nhc_sigma_side, setting->nhc_sigma_vert, setting->speed_sigma);
#endif

	if (fabs(setting->init_pos[0]) < 0.001 && fabs(setting->init_pos[1]) < 0.001)
	{
	}
	else
	{
#ifndef NO_GGA
		set_ini_sol(setting->init_pos, setting->init_vel, setting->init_att, setting->init_pos_sigma_hori, setting->init_pos_sigma_vert, setting->init_vel_sigma_hori, setting->init_vel_sigma_vert, setting->init_att_sigma_hori, setting->init_att_sigma_vert);
#endif
	}

	for (pimu = imus.begin(), pgps = pvts.begin(); pimu != imus.end(); ++pimu)
	{
		/* gps data at antenna 1 */
		while (pgps != pvts.end())
		{
			if ((pgps->time+setting->dt) > pimu->time) break;

			gps[0] = pgps->time + setting->dt;  /* IMU time */
			gps[1] = pgps->timegps * 1.0e9;  /* GPS nsec */
			gps[2] = pgps->lat * R2D;  /* latitude -- deg*/
			gps[3] = pgps->lon * R2D;  /* longitude -- deg*/
			gps[4] = pgps->ht;  /* ht */
			gps[5] = pgps->msl;  /* MSL */
			gps[6] = pgps->speed;  /* speed [m/s] */
			gps[7] = pgps->heading;  /* heading [deg] */
			gps[8] = pgps->acc_h;  /* hor. accuracy [m] */
			gps[9] = pgps->acc_v; /* ver. accuracy [m] */
			gps[10] = pgps->pdop; /* PDOP */
			gps[11] = pgps->fixtype; /* fixType */
			gps[12] = pgps->nsat; /* sat number */
			gps[13] = pgps->acc_speed; /* speed accur */
			gps[14] = pgps->acc_heading; /* heading accur */
			gps[15] = pgps->rtk_status; /* rtk fix status */
			/* project GPS data to IMU time */
			dt = (pgps->time + setting->dt)-pgps->timegps;

			double l2n = 0;
			double l2e = lat2local(pgps->lat, &l2n);
			double dist = pgps->speed * dt;
			double dn = dist * cos(pgps->heading * D2R);
			double de = dist * sin(pgps->heading * D2R);
			if (setting->is_compensate_gps_time)
			{
				gps[2] += dn / l2n * R2D;
				gps[3] += de / l2e * R2D;
			}
			/* send to engine */
			bool is_gap = 0;
			for (std::vector<gap_t>::iterator pgap = setting->gaps.begin(); pgap != setting->gaps.end(); ++pgap)
			{
				if (pgap->length > 0 && pgps->time >= pgap->time && pgps->time <= (pgap->time + pgap->length))
				{
					is_gap = 1;
					break;
				}
			}
			if (is_gap)
			{
			}
			else if (setting->is_ant1)
			{
#ifndef NO_GGA
				add_gps_ant1(gps);
#endif
			}

			++pgps;

		}
		/* gps data at antenna 2 */
		while (pgp2 != pv2s.end())
		{
			if ((pgp2->time + setting->dt) > pimu->time) break;

			gp2[0] = pgp2->time + setting->dt;  /* IMU time */
			gp2[1] = pgp2->timegps * 1.0e9;  /* GPS nsec */
			gp2[2] = pgp2->lat * R2D;  /* latitude -- deg*/
			gp2[3] = pgp2->lon * R2D;  /* longitude -- deg*/
			gp2[4] = pgp2->ht;  /* ht */
			gp2[5] = pgp2->msl;  /* MSL */
			gp2[6] = pgp2->speed;  /* speed [m/s] */
			gp2[7] = pgp2->heading;  /* heading [deg] */
			gp2[8] = pgp2->acc_h;  /* hor. accuracy [m] */
			gp2[9] = pgp2->acc_v; /* ver. accuracy [m] */
			gp2[10] = pgp2->pdop; /* PDOP */
			gp2[11] = pgp2->fixtype; /* fixType */
			gp2[12] = pgp2->nsat; /* sat number */
			gp2[13] = pgp2->acc_speed; /* speed accur */
			gp2[14] = pgp2->acc_heading; /* heading accur */
			gp2[15] = pgp2->rtk_status; /* rtk fix status */
			/* project GPS data to IMU time */
			dt = (pgp2->time + setting->dt) - pgp2->timegps;
			double l2n = 0;
			double l2e = lat2local(pgp2->lat, &l2n);
			double dist = pgp2->speed * dt;
			double dn = dist * cos(pgp2->heading * D2R);
			double de = dist * sin(pgp2->heading * D2R);
			if (setting->is_compensate_gps_time)
			{
				gp2[2] += dn / l2n * R2D;
				gp2[3] += de / l2e * R2D;
			}
			/* send to engine */
			bool is_gap = 0;
			for (std::vector<gap_t>::iterator pgap = setting->gaps.begin(); pgap != setting->gaps.end(); ++pgap)
			{
				if (pgap->length > 0 && pgp2->time >= pgap->time && pgp2->time <= (pgap->time + pgap->length))
				{
					is_gap = 1;
					break;
				}
			}
			if (is_gap)
			{
			}
			else if (setting->is_ant2)
			{
#ifndef NO_GGA
				add_gps_ant2(gp2);
#endif
			}

			++pgp2;
		}
		while (phdr != hdrs.end())
		{
			if ((phdr->time + setting->dt) > pimu->time) break;
			/* send to engine */
#ifndef NO_GGA
			add_hdr_ant2(&phdr->nav_relposned);
#endif
			++phdr;
		}
		/* pva */
		while (ppva != pvas.end())
		{
			if ((ppva->time + setting->dt) > pimu->time) break;
			ins[0] = ppva->time;
			ins[1] = ppva->pos[0];
			ins[2] = ppva->pos[1];
			ins[3] = ppva->pos[2];
			ins[4] = ppva->vel[0];
			ins[5] = ppva->vel[1];
			ins[6] = ppva->vel[2];
			ins[7] = ppva->att[0];
			ins[8] = ppva->att[1];
			ins[9] = ppva->att[2];
			ins[10] = ppva->status;
			ins[11] = ppva->timegps;
			++ppva;
		}
		/* imu data */
		if (setting->start_time > 0 && pimu->time < setting->start_time) continue;
		if (setting->end_time > 0 && pimu->time > setting->end_time)
			break;

		imu[0] = pimu->time;
		imu[1] = pimu->fxyz[0]; /* fx */
		imu[2] = pimu->fxyz[1]; /* fy */
		imu[3] = pimu->fxyz[2]; /* fz */
		imu[4] = pimu->wxyz[0]; /* wx */
		imu[5] = pimu->wxyz[1]; /* wy */
		imu[6] = pimu->wxyz[2]; /* wz */
		if (setting->is_mems)
			imu[7] = pimu->wxyz[2]; /* wz */
		else
			imu[7] = pimu->wxyz[3]; /* wz_fog */
		imu[8] = pimu->odr; /* odr */
		imu[9] = pimu->timeodr; /* odr time */
		imu[10] = pimu->temp; /* temp */

		/* use GPS speed as odometer */
		if (setting->is_gps_speed && (pimu->timeodr < 0.01) && ((gps[12] > 15 && gps[13] < 0.20) || (gp2[12] > 15 && gp2[13] < 0.20)))
		{
			/* use GPS speed */
			double dt1 = gps[0] - imu[0];
			double dt2 = gp2[0] - imu[0];
			if (fabs(dt1) < fabs(dt2))
			{
				if (fabs(dt1) < 0.01)
				{
					imu[8] = gps[6];
					imu[9] = gps[0];
					memset(gps, 0, sizeof(gps));
				}
			}
			else
			{
				if (fabs(dt2) < 0.01)
				{
					imu[8] = gp2[6];
					imu[9] = gp2[0];
					memset(gp2, 0, sizeof(gp2));
				}
			}
		}

		/* use constant speed constraint */
		for (std::vector<speed_t>::iterator pspeed = setting->speeds.begin(); pspeed != setting->speeds.end(); ++pspeed)
		{
			if (pspeed->length > 0 && imu[0] >= pspeed->time && imu[0] <= (pspeed->time + pspeed->length))
			{
				imu[8] = pspeed->value;
				imu[9] = imu[0];
				break;
			}
		}

		/* check zupt */
		int is_zupt = 0;
		for (std::vector<zupt_t>::iterator pzupt = setting->zupts.begin(); pzupt != setting->zupts.end(); ++pzupt)
		{
			if (pzupt->length > 0 && imu[0] >= pzupt->time && imu[0] <= (pzupt->time + pzupt->length))
			{
				is_zupt = 1;
				break;
			}
		}

		if (is_zupt)
		{
			imu[8] = 0.005; /* > 1e-3 */
			imu[9] = imu[0];
#ifndef NO_GGA
			set_nhc_sigma(1, 1, 1);
#endif
		}
		else
		{
#ifndef NO_GGA
			set_nhc_sigma(setting->nhc_sigma_side, setting->nhc_sigma_vert, setting->speed_sigma);
#endif
		}

		/* rotate data to match the vehicle definition */
		rotate_vector(Cbs, imu + 1, 0);
		double wxyz1[3] = { imu[4], imu[5], imu[6] };
		double wxyz2[3] = { imu[4], imu[5], imu[7] };
		rotate_vector(Cbs, wxyz1, 0);
		rotate_vector(Cbs, wxyz2, 0);
		imu[4] = wxyz1[0];
		imu[5] = wxyz1[1];
		imu[6] = wxyz1[2];
		imu[7] = wxyz2[2];
#if 1
		/* rotate data to accomodate the install angle */
		rotate_vector(Cma, imu + 1, 0);
		double wxyz3[3] = { imu[4], imu[5], imu[6] };
		double wxyz4[3] = { imu[4], imu[5], imu[7] };
		rotate_vector(Cma, wxyz3, 0);
		rotate_vector(Cma, wxyz4, 0);
		imu[4] = wxyz3[0];
		imu[5] = wxyz3[1];
		imu[6] = wxyz3[2];
		imu[7] = wxyz4[2];
#endif
		/* send to engine */

		add_imu_data(imu);
		/* PVA contains the EKF solution.  type is always 1 (-1 => no solution) */
		int type = get_ins_solu(pva);
		if (type > 0)
		{
			char buffer[255] = { 0 };
			if (strlen(setting->key)>0)
				sprintf(buffer, "-pp-%s.csv", setting->key);
			else
				sprintf(buffer, "-pp.csv");
			if (!fCSV_PP) fCSV_PP = set_output_file(fname, buffer);
			if (fabs(pva[0]) > 1.0e-10 && fabs(pva[1]) > 1.0e-10 && fabs(pva[2]) > 1.0e-10)
			{
				if (strlen(setting->key) > 0)
					sprintf(buffer, "-pp-%s.nmea", setting->key);
				else
					sprintf(buffer, "-pp.nmea");
				if (!fGGA_PP) fGGA_PP = set_output_file(fname, buffer);
				if (fGGA_PP)
				{
					outnmea_gga((unsigned char*)buffer, (float)imu[0], 1, pva, 10, (float)1.0, (float)0);
					fprintf(fGGA_PP, "%s", buffer);
				}
			}
			double speed = sqrt(pva[3] * pva[3] + pva[4] * pva[4]);
			if (speed > 0.5)
				pp_speed_heading = atan2(pva[4], pva[3]) * R2D;
			double C_nb[3][3] = { 0 };
			att2C_nb(pva[6], pva[7], pva[8], C_nb);
			rotate_vector(C_nb, pva + 3, 1);

			if (fCSV_PP)
			{
				fprintf(fCSV_PP, "%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%10.4f,%10.4f", imu[0], pva[0] * R2D, pva[1] * R2D, pva[2], pva[3], pva[4], pva[5], pva[6] * R2D, pva[7] * R2D, pva[8] * R2D, type, gps[7] > 180 ? gps[7] - 360 : gps[7], pp_speed_heading);
				fprintf(fCSV_PP, ",%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f", pva[9], pva[10], pva[11], pva[12], pva[13], pva[14]); /* bfxyz, bwxyz */
				fprintf(fCSV_PP, ",%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f", pva[15], pva[16], pva[17], pva[18], pva[19], pva[20], pva[21]); /* sff, sfwxyz*/
				fprintf(fCSV_PP, ",%10.4f,%10.4f,%10.4f\n", pva[22], pva[23], pva[24]);
			}
		}
	}
	if (fCSV_PP) fclose(fCSV_PP);
	if (fGGA_PP) fclose(fGGA_PP);
	return 0;
}
#endif

typedef struct {
	uint64_t    MCU_Time;//	UInt64	ns	    Time since power on
	uint64_t    Sync_Time; // UInt64   ns   Timestamp of external sync pulse
	uint64_t    ODO_time;//	Int64	ns	    Timestamp of ODometer reading
	int32_t     AX;	     //Int32	15 g	X-axis accel
	int32_t     AY;	     //Int32	15 g	Y-axis accel
	int32_t     AZ;	     //Int32	15 g	Z-axis accel
	int32_t     WX;	     //Int32	450 dps	X-axis angular rate (MEMS)
	int32_t     WY;	     //Int32	450 dps	Y-axis angular rate (MEMS)
	int32_t     WZ;	     //Int32	450 dps	Z-axis angular rate (MEMS)
	int32_t     OG_WZ;	 //Int32	450 dps	High precision z-axis angular rate 
	int16_t     ODO;	 //Int16	m/s	    Scaled composite odometer value
	int16_t     Temp_C;	 //Int16	�C
}rtcm_apimu_t;

/* old format do not have Sync_Time */
typedef struct {
	uint64_t    MCU_Time;//	UInt64	ns	    Time since power on
	uint64_t    ODO_time;//	Int64	ns	    Timestamp of ODometer reading
	int32_t     AX;	     //Int32	15 g	X-axis accel
	int32_t     AY;	     //Int32	15 g	Y-axis accel
	int32_t     AZ;	     //Int32	15 g	Z-axis accel
	int32_t     WX;	     //Int32	450 dps	X-axis angular rate (MEMS)
	int32_t     WY;	     //Int32	450 dps	Y-axis angular rate (MEMS)
	int32_t     WZ;	     //Int32	450 dps	Z-axis angular rate (MEMS)
	int32_t     OG_WZ;	 //Int32	450 dps	High precision z-axis angular rate 
	int16_t     ODO;	 //Int16	m/s	    Scaled composite odometer value
	int16_t     Temp_C;	 //Int16	�C
}rtcm_old_apimu_t;

typedef struct {
	uint64_t    Time;	        //UInt64	ns	    Time since power on
	uint64_t    GPS_Time;	    //UInt64	ns	    GPS time (GTOW)
	int32_t     Latitude;	    //Int32	    1e-7 deg	
	int32_t     Longitude;	    //Int32	    1e-7 deg	
	int32_t     Alt_ellipsoid;  //Int32	    0.001 m	
	int32_t     Alt_msl;	    //Int32	    0.001 m	
	int32_t     Speed;	        //Int32	    0.001 mps	
	int32_t     Heading;	    //Int32	    0.001 deg	
	uint32_t    Hor_Acc;	    //UInt32	0.001 m	
	uint32_t    Ver_Acc;	    //UInt32	0.001 m	
	uint32_t    Hdg_Acc;	    //UInt32	1e-5 deg	
	uint32_t    Spd_Acc;	    //UInt32	0.001 mps	
	uint16_t    PDOP;	        //UInt16	0.01	
	uint8_t     FixType;	    //UInt8	 	
	uint8_t     SatNum;	        //UInt8	 	
	uint8_t     RTK_Status;	    //UInt8	 	
	uint8_t     Antenna_ID;	    //Uint8	    Primary antenna or 2nd antenna	
}rtcm_apgps_t;


typedef struct {
	uint64_t    MCU_Time;                 //UInt64	ns
	uint64_t    GPS_Time;                 //UInt64	    ns

	int32_t     relPosN;                  //Int32	    0.001 m
	int32_t     relPosE;                  //Int32	    0.001 m
	int32_t     resPosD;                  //Int32	    0.001 m

	int32_t     relPosLength;             //Int32	    0.001 m
	int32_t     relPosHeading;            //Int32	    1e-5 deg
    
	uint32_t    relPosLength_Accuracy;    //UInt32	1e-5 deg
	uint32_t    relPosHeading_Accuracy;   //UInt32	1e-5 deg
    
    uint16_t statusFlags;
}rtcm_aphdr_t;


typedef struct {
	uint64_t    Time;	        //UInt64	ns	
	uint64_t    GPS_Time;	    //UInt64	ns	
	int32_t     Latitude;	    //Int32	    1.0e-7 deg	
	int32_t     Longitude;	    //Int32	    1.0e-7 deg	
	int32_t     Alt_ellipsoid;  //Int32	    0.001 m	
	int32_t     Vn;	            //Int32	    0.001 mps	
	int32_t     Ve;	            //Int32	    0.001 mps	
	int32_t     Vd;	            //Int32	    0.001 mps	
	int32_t     Roll;	        //Int32	    1e-5 deg	
	int32_t     Pitch;	        //Int32	    1e-5 deg	
	int32_t     Heading_Yaw;	//Int32	    1e-5 deg	
	uint8_t     ZUPT;	        //UInt8	    1 � stationary, 0 - moving
	uint8_t     Status;	        //UInt8	    See ASCII packet
}rtcm_apins_t;

#ifndef NO_GGA
static void process_gps(double *gps, std::vector<pvt_t>& pvts, FILE *fGPS_CSV, FILE *fGPS_GGA, int is_pp)
#else
static void process_gps(double *gps, FILE *fGPS_CSV)
#endif
{
	if (fGPS_CSV)
	{
#ifndef NO_GGA
		fprintf(fGPS_CSV, "%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%14.7f,%10.3f\n", gps[0], gps[2], gps[3], gps[4], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15], gps[1] - gps[0], gps[16],gps[5]);
#else
		fprintf(fGPS_CSV, "%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", gps[0], gps[1], gps[2], gps[3], gps[4], gps[5], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15]);
#endif
	}
#ifndef NO_GGA
	if (fGPS_GGA)
	{
		char gga_buffer[255] = { 0 };
		double blh[3] = { gps[2] * D2R, gps[3] * D2R, gps[4] };
		outnmea_gga((unsigned char*)gga_buffer, (float)gps[0], 1, blh, (int)gps[12], (float)gps[10], 0);
		fprintf(fGPS_GGA, "%s", gga_buffer);
	}
	/* send to engine */
	add_gps_ant1(gps);
	pvt_t cur_pvt = { 0 };
	cur_pvt.time = gps[0];
	cur_pvt.timegps = gps[1];
	cur_pvt.lat = gps[2] * D2R;
	cur_pvt.lon = gps[3] * D2R;
	cur_pvt.ht = gps[4];
	cur_pvt.msl = gps[5];
	cur_pvt.speed = gps[6];
	cur_pvt.heading = gps[7];
	cur_pvt.acc_h = gps[8];
	cur_pvt.acc_v = gps[9];
	cur_pvt.pdop = gps[10];	
	cur_pvt.fixtype = (int)gps[11];
	cur_pvt.nsat = (int)gps[12];
	cur_pvt.acc_speed = gps[13];
	cur_pvt.acc_heading = gps[14];
	cur_pvt.rtk_status = (int)gps[15];
	if (is_pp)
		pvts.push_back(cur_pvt);
#endif
}

#ifndef NO_GGA
static void process_hdr(double* hdr, std::vector<hdr_t>& hdrs, FILE* fCSV, int is_pp)
#else
static void process_hdr(double* hdr, FILE* fCSV)
#endif
{
	if (fCSV)
	{
		int status = (int)hdr[9];
#ifndef NO_GGA
		fprintf(fCSV,
			/* h[x] = hdr[x] */
			/* h0      h2    h3     h4      h5    h6     h7     h8      h1  h9 s0 s1 s2 34 s5 s6 s7 s8 s9  h10*/
			"%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.5f,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%14.7f\n",
			hdr[0],
			hdr[2],
			hdr[3],
			hdr[4],
			hdr[5],
			hdr[6],
			hdr[7],
			hdr[8],
			hdr[1],
			(int)hdr[9],
			(status & (1 << 0)) > 0,
			(status & (1 << 1)) > 0,
			(status & (1 << 2)) > 0,
			(status & (3 << 3)) >> 3,
			(status & (1 << 5)) > 0,
			(status & (1 << 6)) > 0,
			(status & (1 << 7)) > 0,
			(status & (1 << 8)) > 0,
			(status & (1 << 9)) > 0,
			hdr[10]
		);
#else
		fprintf(fCSV,
			/* h[x] = hdr[x] */
			/* h0     h1     h2    h3      h4     h5     h6     h7     h8   h9 s0 s1 s2 34 s5 s6 s7 s8 s9*/
			"%10.4f,%10.5f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\n",
			hdr[0],
			hdr[1],
			hdr[2],
			hdr[3],
			hdr[4],
			hdr[5],
			hdr[6],
			hdr[7],
			hdr[8],
			(int)hdr[9],
			(status & (1 << 0)) > 0,
			(status & (1 << 1)) > 0,
			(status & (1 << 2)) > 0,
			(status & (3 << 3)) >> 3,
			(status & (1 << 5)) > 0,
			(status & (1 << 6)) > 0,
			(status & (1 << 7)) > 0,
			(status & (1 << 8)) > 0,
			(status & (1 << 9)) > 0
		);
#endif

		fflush(fCSV);
	}
#ifndef NO_GGA
	hdr_t cur_hdr = { 0 };
	cur_hdr.time = hdr[0];
	cur_hdr.nav_relposned.relPosN = hdr[2] * 100;
	cur_hdr.nav_relposned.relPosE = hdr[3] * 100;
	cur_hdr.nav_relposned.relPosD = hdr[4] * 100;
	cur_hdr.nav_relposned.relPosLength = hdr[5] * 100;
	cur_hdr.nav_relposned.relPosHeading = hdr[6] * 1e5;
	cur_hdr.nav_relposned.accLength = hdr[7] * 1e4;
	cur_hdr.nav_relposned.accHeading = hdr[8] * 1e5;
	cur_hdr.nav_relposned.iTOW = hdr[1];
	cur_hdr.nav_relposned.flags = hdr[9];
	if (is_pp)
		hdrs.push_back(cur_hdr);
#endif
}

#ifndef NO_GGA
static void process_imu(double* imu, std::vector<imu_t>& imus, FILE* fIMU, int is_pp)
#else
static void process_imu(double* imu, FILE* fIMU)
#endif
{
	if (fIMU)
	{
		//                0      1      2      3      4     5       6      7     8      9       10
		fprintf(fIMU, "%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6], imu[7], imu[8], imu[9], imu[10], imu[11]);
	}
#ifndef NO_GGA
	imu_t cur_imu = { 0 };
	cur_imu.time = imu[0];
	cur_imu.fxyz[0] = imu[1]; /* fx */
	cur_imu.fxyz[1] = imu[2]; /* fy */
	cur_imu.fxyz[2] = imu[3]; /* fz */
	cur_imu.wxyz[0] = imu[4]; /* wx */
	cur_imu.wxyz[1] = imu[5]; /* wy */
	cur_imu.wxyz[2] = imu[6]; /* wz */
	cur_imu.wxyz[3] = imu[7]; /* wz_fog */
	cur_imu.odr = imu[8]; /* odr */
	cur_imu.timeodr = imu[9]; /* odr time */
	cur_imu.temp = imu[10]; /* temp */
#endif
}

#ifndef NO_GGA
static void process_ins(double* ins, std::vector<pva_t>& pvas, FILE *fCSV, FILE* fGGA, int is_pp)
#else
static void process_ins(double* ins, FILE *fCSV)
#endif
{
	/* rotate velocity to FRD */
#ifndef NO_GGA
	double C_nb[3][3] = { 0 };
	att2C_nb(ins[9] * D2R, ins[10] * D2R, ins[11] * D2R, C_nb);
	rotate_vector(C_nb, ins + 6, 1);

	pva_t cur_pva = { 0 };
	cur_pva.time = ins[0];
	cur_pva.timegps = ins[1];
	cur_pva.status = (int)ins[2];
	cur_pva.pos[0] = ins[3];
	cur_pva.pos[1] = ins[4];
	cur_pva.pos[2] = ins[5];

	cur_pva.vel[0] = ins[6];
	cur_pva.vel[1] = ins[7];
	cur_pva.vel[2] = ins[8];

	cur_pva.att[0] = ins[9];
	cur_pva.att[1] = ins[10];
	cur_pva.att[2] = ins[11];
	if (is_pp)
		pvas.push_back(cur_pva);
#endif

	if (fCSV)
	{
#ifndef NO_GGA
		fprintf(fCSV, "%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%14.7f\n", ins[0], ins[3], ins[4], ins[5], ins[6], ins[7], ins[8], ins[9], ins[10], ins[11], ins[2], ins[12], ins[1] - ins[0], ins[13]);
#else	
		fprintf(fCSV, "%10.3f,%14.7f,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", ins[0], ins[1], ins[2], ins[3], ins[4], ins[5], ins[6], ins[7], ins[8], ins[9], ins[10], ins[11], ins[12]);
#endif
	}
#ifndef NO_GGA
	if (fGGA)
	{
		char gga_buffer[255] = { 0 };
		double blh[3] = { ins[3] * D2R, ins[4] * D2R, ins[5] };
		outnmea_gga((unsigned char*)gga_buffer, (float)ins[0], 1, blh, 10, (float)1, (float)0);
		fprintf(fGGA, "%s", gga_buffer);
	}
#endif
	return;
}

typedef struct
{
	double integrated_angle_mems;
	double integrated_angle_fog;
	double integrated_angle_time;
	double integrated_start_time;
	double sfw[3];
	unsigned long numofsfw;
}gyro_sfw_t;

#define SFW_SEGMENT (1) /* second */
#define SFW_ROTATE  (5.0)

#ifndef NO_GGA
static int add_gyro_data(gyro_sfw_t *gyro_sfw, double time, double mems_wz, double fog_wz, double temp)
{
	double dt = time - gyro_sfw->integrated_start_time;
	if (gyro_sfw->integrated_angle_time > 0 && dt < SFW_SEGMENT)
	{
		gyro_sfw->integrated_angle_mems += mems_wz * (time - gyro_sfw->integrated_angle_time);
		gyro_sfw->integrated_angle_fog += fog_wz * (time - gyro_sfw->integrated_angle_time);
		gyro_sfw->integrated_angle_time = time;
	}
	else
	{
		if (gyro_sfw->integrated_angle_time > 0)
		{
			double dsfw[4] = { 0 };
			int is_sfw = 0;
			if (gyro_sfw->numofsfw > 0)
			{
				dsfw[0] = time - gyro_sfw->sfw[0];
				dsfw[1] = gyro_sfw->integrated_angle_mems - gyro_sfw->sfw[1];
				dsfw[2] = gyro_sfw->integrated_angle_fog - gyro_sfw->sfw[2];
				if (fabs(dsfw[1]) > SFW_ROTATE)
				{
					dsfw[3] = (dsfw[2] - dsfw[1]) / dsfw[1];
					is_sfw = 1;
				}
			}
			else
			{

			}
			++gyro_sfw->numofsfw;
			gyro_sfw->sfw[0] = time;
			gyro_sfw->sfw[1] = gyro_sfw->integrated_angle_mems;
			gyro_sfw->sfw[2] = gyro_sfw->integrated_angle_fog;
			static FILE* fSFW = NULL;
			if (!fSFW) fSFW = fopen("sfw.csv", "w");
			if (fSFW)
			{
				fprintf(fSFW, "%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.6f,%i,%10.6f\n", time, gyro_sfw->integrated_angle_mems, gyro_sfw->integrated_angle_fog, dt, dsfw[0], dsfw[1], dsfw[2], dsfw[3], is_sfw, temp);
				fflush(fSFW);
			}

		}
		gyro_sfw->integrated_angle_mems = 0;
		gyro_sfw->integrated_angle_fog = 0;
		gyro_sfw->integrated_angle_time = time;
		gyro_sfw->integrated_start_time = time;
	}
	return gyro_sfw->numofsfw;
}
#endif

#ifndef NO_GGA
static int process_log(const char* fname, setting_t* setting)
#else
static int process_log(const char* fname)
#endif
{
	FILE* fLOG = fopen(fname, "rb"); if (!fLOG) return 0;
	int data = 0;
	char* val[MAXFIELD];

	FILE* fCSV = NULL;
	FILE* fGGA = NULL;
	FILE* fIMU = NULL;
	FILE* fGPS_CSV = NULL;
	FILE* fGP2_CSV = NULL;
	FILE* fGPS_GGA = NULL;
	FILE* fGP2_GGA = NULL;
	FILE* fANT1 = NULL;
	FILE* fANT2 = NULL;
	FILE* fBASE = NULL;
	FILE* fHDR = NULL;

	a1buff_t a1buff = { 0 };
#ifndef NO_GGA
	ubxbuff_t ubx1 = { 0 };
	ubxbuff_t ubx2 = { 0 };
#endif

	double gps[20] = { 0 };
	double gp2[20] = { 0 };
	double hdr[20] = { 0 };
	double imu[20] = { 0 };
	double ins[20] = { 0 };

#ifndef NO_GGA
	std::vector<imu_t> imus;
	std::vector<pvt_t> pvts;
	std::vector<pvt_t> pv2s;
	std::vector<hdr_t> hdrs;
	std::vector<pva_t> pvas;
#endif

	rtcm_apimu_t rtcm_apimu = { 0 };
	rtcm_old_apimu_t rtcm_old_apimu = { 0 };
	rtcm_apgps_t rtcm_apgps = { 0 };
	rtcm_aphdr_t rtcm_aphdr = { 0 };
	rtcm_apins_t rtcm_apins = { 0 };

#ifndef NO_GGA
	double start_time = 0;
	double end_time = 0;
	unsigned long numofepoch = 0;
	unsigned long numofsec_IMU = 0;
	unsigned long numofsec_GPS = 0;
	int sec_gps = 0;
	int sec_imu = 0;

	double distance_traveled = 0;
	double distance_time = 0;

	std::vector<gap_t> vGAP;
	gap_t gap = { 0 };

	gyro_sfw_t gyro_sfw = { 0 };
#endif

	while (fLOG != NULL && !feof(fLOG) && (data = fgetc(fLOG)) != EOF)
	{
		int ret = input_a1_data(&a1buff, data);
		if (ret)
		{
			int isOK = 0;
			int num = 0;
			if (ret == 1)
			{

#ifndef NO_GGA
				if (strstr((char*)a1buff.buf, "APCFG"))
					printf("%s", a1buff.buf);
#endif
				num = parse_fields((char*)a1buff.buf, val);
			}
#ifndef NO_GGA
			else if (ret == 2) /* APANT1 */
			{
				if (!fANT1) fANT1 = set_output_file(fname, "-ant1.log");
				if (fANT1)
				{
					fwrite(a1buff.buf + a1buff.loc[1] + 1, sizeof(char), a1buff.nlen, fANT1);
				}
				isOK = 1;
			}
			else if (ret == 3) /* APANT2 */
			{
				if (!fANT2) fANT2 = set_output_file(fname, "-ant2.log");
				if (fANT2)
				{
					fwrite(a1buff.buf + a1buff.loc[1] + 1, sizeof(char), a1buff.nlen, fANT2);
				}
				isOK = 1;
			}
			else if (ret == 4) /* APRTK */
			{
				if (!fBASE) fBASE = set_output_file(fname, "-base.log");
				if (fBASE)
				{
					fwrite(a1buff.buf + a1buff.loc[1] + 1, sizeof(char), a1buff.nlen, fBASE);
				}
				isOK = 1;
			}
#endif
			else if (ret == 5) /* rtcm */
			{
				//printf("%4i,%4i,%4i,%4i,%i\n", a1buff.type, a1buff.subtype, a1buff.nbyte, a1buff.nlen, a1buff.crc);
				if (a1buff.type == 4058 && !a1buff.crc)
				{
					if (a1buff.subtype == 1) /* IMU */
					{
						if (!fIMU)
						{
							fIMU = set_output_file(fname, "-imu.csv");
#ifndef NO_GGA
							if (fIMU && setting->csv_header) fprintf(fIMU, "%s", IMU_HEADER);
#endif
						}
						if (a1buff.nlen >= 61)
						{
							memcpy((uint8_t*)&rtcm_apimu, a1buff.buf + 5, sizeof(rtcm_apimu_t));
#ifndef NO_GGA
							imu[0] = rtcm_apimu.MCU_Time * 1.0e-9 + setting->timeoffset;
#else
							imu[0] = rtcm_apimu.MCU_Time;
#endif	
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

							/* calibrate the FOG gyro scale factor */
#ifndef NO_GGA
							add_gyro_data(&gyro_sfw, imu[0], imu[6], imu[7], imu[10]);
#endif

							/* remove gyro scale factor */
#ifndef NO_GGA
							imu[4] *= 1.0 - setting->init_sfw[0];
							imu[5] *= 1.0 - setting->init_sfw[1];
							if (setting->is_mems)
								imu[6] *= 1.0 - setting->init_sfw[2];
							else
								imu[7] *= 1.0 - setting->init_sfw[2];

							setting->is_old_imu_data = 0;
#endif
						}
						else
						{
							memcpy((uint8_t*)&rtcm_old_apimu, a1buff.buf + 5, sizeof(rtcm_old_apimu_t));
#ifndef NO_GGA
							imu[0] = rtcm_old_apimu.MCU_Time * 1.0e-9 + setting->timeoffset;
#else
							imu[0] = rtcm_old_apimu.MCU_Time;
#endif
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

							/* calibrate the FOG gyro scale factor */
#ifndef NO_GGA
							add_gyro_data(&gyro_sfw, imu[0], imu[6], imu[7], imu[10]);

							setting->is_old_imu_data = 1;
#endif
						}

						if (!fIMU)
						{
							fIMU = set_output_file(fname, "-imu.csv");
#ifndef NO_GGA
							if (fIMU && setting->csv_header) fprintf(fIMU, "%s", IMU_HEADER);
#endif
						}
#ifndef NO_GGA
						if (numofsec_IMU == 0)
						{
							sec_imu = floor(imu[0] + 0.5);
							++numofsec_IMU;
						}
						else if (fabs(floor(imu[0] + 0.5)-sec_imu)>0.1)
						{
							sec_imu = floor(imu[0] + 0.5);
							numofsec_IMU++;
						}

						if (imu[9] > 0.0 && gap.time>0.0)
						{
							gap.dist += imu[8] * (imu[0] - gap.time_odr);
							gap.time_odr = imu[0];
						}

						if (imu[9] > 0.0)
						{
							if (distance_time == 0)
							{
								distance_traveled = 0;
							}
							else
							{
								distance_traveled += imu[8] * (imu[0] - distance_time);
							}
							distance_time = imu[0];
						}
#endif

#ifndef NO_GGA
						process_imu(imu, imus, fIMU, setting->is_pp);
#else
						process_imu(imu, fIMU);
#endif
					}
					else if (a1buff.subtype == 2) /* GPS PVT */
					{
						memcpy((uint8_t*)&rtcm_apgps, a1buff.buf + 5, sizeof(rtcm_apgps_t));
#ifndef NO_GGA
						gps[0] = (rtcm_apgps.Time * 1.0e-9) + setting->timeoffset; /* time MCU */
						gps[1] = (rtcm_apgps.GPS_Time * 1.0e-9) + setting->timeoffset_gps; /* GPS ns */
						gps[1] -= floor(gps[1] / (7 * 24 * 3600)) * 7 * 24 * 3600;
#else
						gps[0] = rtcm_apgps.Time; /* time MCU */
						gps[1] = rtcm_apgps.GPS_Time; /* GPS ns */
#endif
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
#ifndef NO_GGA
						gps[16] = (rtcm_apgps.GPS_Time * 1.0e-9) + setting->timeoffset_gps;	/* Full GPS Time */
#endif

						if (rtcm_apgps.Antenna_ID == 0)
						{
							if (!fGPS_CSV)
							{
								fGPS_CSV = set_output_file(fname, "-gps.csv");
#ifndef NO_GGA
								if (fGPS_CSV && setting->csv_header) fprintf(fGPS_CSV, "%s", GPS_HEADER);
#endif
							}
#ifndef NO_GGA
							if (!fGPS_GGA) fGPS_GGA = set_output_file(fname, "-gps.nmea");

							if (rtcm_apgps.SatNum > 10 && gps[8] < 10)
							{
								if (numofsec_GPS == 0)
								{
									sec_gps = floor(gps[0] + 0.5);
									++numofsec_GPS;
								}
								else if (fabs(floor(gps[0] + 0.5) - sec_gps) > 0.1)
								{
									sec_gps = floor(gps[0] + 0.5);
									++numofsec_GPS;
								}
								if (gap.time > 0)
								{
									gap.length = gps[0] - gap.time;
									if (gap.length > 1.5)
									{
										vGAP.push_back(gap);
									}
								}
								gap.time = gps[0];
								gap.time_odr = gps[0];
								gap.length = 0;
								gap.dist = 0;
							}

							process_gps(gps, pvts, fGPS_CSV, fGPS_GGA, setting->is_pp);
#else
							process_gps(gps, fGPS_CSV);
#endif
						}
						else
						{
							
							if (!fGP2_CSV)
							{
								fGP2_CSV = set_output_file(fname, "-gp2.csv");
#ifndef NO_GGA
								if (fGP2_CSV && setting->csv_header) fprintf(fGP2_CSV, "%s", GPS_HEADER);
#endif
							}
#ifndef NO_GGA
							if (!fGP2_GGA) fGP2_GGA = set_output_file(fname, "-gp2.nmea");
#endif

#ifndef NO_GGA
							process_gps(gps, pv2s, fGP2_CSV, fGP2_GGA, setting->is_pp);
#else
							process_gps(gps, fGP2_CSV);
#endif
						}
					}
					else if (a1buff.subtype == 3) /* DUAL ANTENNA */
					{
						memcpy((uint8_t*)&rtcm_aphdr, a1buff.buf + 5, sizeof(rtcm_aphdr_t));
#ifndef NO_GGA
						hdr[0] = rtcm_aphdr.MCU_Time * 1.0e-9 + setting->timeoffset; /* time MCU */
						hdr[1] = rtcm_aphdr.GPS_Time * 1.0e-9 + setting->timeoffset_gps; /* GPS ns */
						hdr[1] -= floor(hdr[1] / (7 * 24 * 3600)) * 7 * 24 * 3600;
#else
						hdr[0] = rtcm_aphdr.MCU_Time; /* time MCU */
						hdr[1] = rtcm_aphdr.GPS_Time; /* GPS ns */
#endif

						hdr[2] = rtcm_aphdr.relPosN*1.0e-2; /* n */
						hdr[3] = rtcm_aphdr.relPosE * 1.0e-2; /* d */
						hdr[4] = rtcm_aphdr.resPosD * 1.0e-2; /* d */
						hdr[5] = rtcm_aphdr.relPosLength * 1.0e-2; /* length */

						hdr[6] = rtcm_aphdr.relPosHeading * 1.0e-5; /* heading */
						hdr[7] = rtcm_aphdr.relPosLength_Accuracy * 1.0e-4; /* acc length */
						hdr[8] = rtcm_aphdr.relPosHeading_Accuracy * 1.0e-5; /* heading length */
						hdr[9] = rtcm_aphdr.statusFlags; /* flag */
#ifndef NO_GGA
						hdr[10] = rtcm_aphdr.GPS_Time * 1.0e-9 + setting->csv_header;
#endif

						if (!fHDR)
						{
							fHDR = set_output_file(fname, "-hdr.csv");
#ifndef NO_GGA
							if (fHDR && setting->csv_header) fprintf(fHDR, "%s", HDG_HEADER);
#endif
						}
#ifndef NO_GGA
						process_hdr(hdr, hdrs, fHDR, setting->is_pp);
#else
						process_hdr(hdr, fHDR);
#endif

					}
					else if (a1buff.subtype == 4) /* INS */
					{
						memcpy((uint8_t*)&rtcm_apins, a1buff.buf + 5, sizeof(rtcm_apins_t));

#ifndef NO_GGA
						ins[0] = rtcm_apins.Time * 1.0e-9 + setting->timeoffset;
						ins[1] = rtcm_apins.GPS_Time * 1.0e-9;
						ins[1] -= floor(ins[1] / (7 * 24 * 3600)) * 7 * 24 * 3600;
#else
						ins[0] = rtcm_apins.Time;
						ins[1] = rtcm_apins.GPS_Time;
#endif
						ins[2] = rtcm_apins.Status;

						ins[3] = rtcm_apins.Latitude * 1.0e-7;
						ins[4] = rtcm_apins.Longitude * 1.0e-7;
						ins[5] = rtcm_apins.Alt_ellipsoid * 1.0e-3;

						ins[6] = rtcm_apins.Vn * 1.0e-3;
						ins[7] = rtcm_apins.Ve * 1.0e-3;
						ins[8] = rtcm_apins.Vd * 1.0e-3;

						ins[9] = rtcm_apins.Roll * 1.0e-5;
						ins[10] = rtcm_apins.Pitch * 1.0e-5;
						ins[11] = rtcm_apins.Heading_Yaw * 1.0e-5;
#ifndef NO_GGA
						if (setting->is_rts_enu)
						{
							ins[7] = rtcm_apins.Vn * 1.0e-3;
							ins[6] = rtcm_apins.Ve * 1.0e-3;
							ins[8] =-rtcm_apins.Vd * 1.0e-3;

							ins[10]= rtcm_apins.Roll * 1.0e-5;
							ins[ 9] = rtcm_apins.Pitch * 1.0e-5;
							ins[11] =-rtcm_apins.Heading_Yaw * 1.0e-5;
						}
#endif
						ins[12] = rtcm_apins.ZUPT; /* zupt */
#ifndef NO_GGA
						ins[13] = rtcm_apins.GPS_Time * 1.0e-9;	/* Full GPS Time */
#endif
						if (!fCSV)
						{
							fCSV = set_output_file(fname, "-rts.csv");
#ifndef NO_GGA
							if (fCSV && setting->csv_header) fprintf(fCSV, "%s", INS_HEADER);
#endif
						}
#ifndef NO_GGA
						if (!fGGA) fGGA = set_output_file(fname, "-rts.nmea");
						process_ins(ins, pvas, fCSV, fGGA, setting->is_pp);
#else
						process_ins(ins, fCSV);
#endif
#ifndef NO_GGA
						ins[1] -= floor(ins[1] / (7 * 24 * 3600.0)) * 7 * 24 * 3600;

						if (rtcm_apins.Latitude == 0 || rtcm_apins.Longitude == 0|| rtcm_apins.Status!=2)
						{

						}
						else
						{
							if (numofepoch == 0)
								start_time = ins[1];
							else
								end_time = ins[1];
							++numofepoch;
						}
#endif
					}
#ifndef NO_GGA
					else if (a1buff.subtype == 14 || a1buff.subtype == 15)
					{
						uint8_t msg[1024] = { 0 };
						int type = 0;
						int subtype = a1buff.subtype;
						int sync = 0;
						int i = 0;
						int mlen = decode_data(a1buff.buf, a1buff.nbyte, &type, &subtype, &sync, msg);
						printf("%4i,%4i,%4i,%4i\n", type, subtype, sync, mlen);
						ubxbuff_t* ubx = subtype == 14 ? &ubx1 : &ubx2;
						/* push data to engine */
						//add_buf_ant(msg, mlen, subtype - 14);
						/* decode here */
						for (i = 0; i < mlen; ++i)
						{
							ret = input_ubx_data(ubx, msg[i]);
							if (ret)
							{
								ubx_nav_pvt_t nav_pvt = { 0 };
								ubx_nav_hpposllh_t nav_hpposllh = { 0 };
								ubx_nav_relposned_t nav_relposned = { 0 };
								if (ubx->type1 == 0x01 && ubx->type2 == 0x07 && decode_ubx_01_07(ubx->buf, ubx->nlen, &nav_pvt))
								{
								}
								else if (ubx->type1 == 0x01 && ubx->type2 == 0x14 && decode_ubx_01_14(ubx->buf, ubx->nlen, &nav_hpposllh))
								{
								}
								else if (ubx->type1 == 0x01 && ubx->type2 == 0x3c && decode_ubx_01_3c(ubx->buf, ubx->nlen, &nav_relposned))
								{
									hdr_t hdr = { 0 };
									hdr.time = imu[0];
									hdr.nav_relposned = nav_relposned;
									if (setting->is_pp)
										hdrs.push_back(hdr);
									if (!fHDR)
									{
										fHDR = set_output_file(fname, "-hdr.csv");
										if (fHDR && setting->csv_header) fprintf(fHDR, "%s", HDG_HEADER);
									}
									if (fHDR)
									{
										fprintf(fHDR,
											"%10.4f,%4i,%4i,%10.4f,%10.4f,%10.4f,%10.4f,%10.5f,%10.4f,%10.4f,%10.4f,%10.4f,%10.5f\n",
											nav_relposned.iTOW * 1.0e-3,
											ubx->type1,
											ubx->type2,
											(nav_relposned.relPosN + nav_relposned.relPosHPN * 1.0e-2) * 1.0e-2,
											(nav_relposned.relPosE + nav_relposned.relPosHPE * 1.0e-2) * 1.0e-2,
											(nav_relposned.relPosD + nav_relposned.relPosHPD * 1.0e-2) * 1.0e-2,
											(nav_relposned.relPosLength + nav_relposned.relPosHPLength * 1.0e-2) * 1.0e-2,
											nav_relposned.relPosHeading * 1e-5,
											nav_relposned.accN * 1.0e-4,
											nav_relposned.accE * 1.0e-4,
											nav_relposned.accD * 1.0e-4,
											nav_relposned.accLength * 1.0e-4,
											nav_relposned.accHeading * 1.0e-5);
										fflush(fHDR);
									}
								}
								sync = 0;
							}
						}
					}
#endif
				}
				isOK = 1;
			}

			if (!isOK && num >= 17 && strstr(val[0], "APGPS") != NULL)
			{
				/* time [s], lat [deg], lon [deg], ht [m], speed [m/s], heading [deg], hor. accuracy [m], ver. accuracy [m], PDOP, fixType, sat num, gps second [s], pps [s] */
				/*
#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,180.0000,0*65
				*/
#ifndef NO_GGA
				gps[0] = (atof(val[1]) * 1.0e-3) + setting->timeoffset; /* time MCU */
				gps[1] = (atof(val[2]) * 1.0e-9) + setting->timeoffset_gps; /* GPS ns */
				gps[1] -= floor(gps[1] / (7 * 24 * 3600)) * 7 * 24 * 3600;
#else
				gps[0] = atof(val[1]); /* time MCU */
				gps[1] = atof(val[2]); /* GPS ns */
#endif

				gps[2] = atof(val[3]); /* lat */
				gps[3] = atof(val[4]); /* lon */
				gps[4] = atof(val[5]); /* ht */
				gps[5] = atof(val[6]); /* msl */

				gps[6] = atof(val[7]); /* speed */
				gps[7] = atof(val[8]); /* heading */
				gps[8] = atof(val[9]); /* acc_h */
				gps[9] = atof(val[10]); /* acc_v */
				gps[10] = atof(val[11]); /* pdop */
				gps[11] = atof(val[12]); /* fixtype */
				gps[12] = atof(val[13]); /* sat number */
				gps[13] = atof(val[14]); /* acc speed */
				gps[14] = atof(val[15]); /* acc heading */
				gps[15] = atof(val[16]); /* rtk fix status */
#ifndef NO_GGA
				gps[16] = (atof(val[2]) * 1.0e-9) + setting->timeoffset_gps;
#endif
				if (!fGPS_CSV)
				{
					fGPS_CSV = set_output_file(fname, "-gps.csv");
#ifndef NO_GGA
					if (fGPS_CSV && setting->csv_header) fprintf(fGPS_CSV, "%s", GPS_HEADER);
#endif
				}
#ifndef NO_GGA
				if (!fGPS_GGA) fGPS_GGA = set_output_file(fname, "-gps.nmea");
#endif

#ifndef NO_GGA
				process_gps(gps, pvts, fGPS_CSV, fGPS_GGA, setting->is_pp);
#else
				process_gps(gps, fGPS_CSV);
#endif

				isOK = 1;
			}
			if (!isOK && num >= 12 && strstr(val[0], "APGP2") != NULL)
			{
				/*
#APGP2,318213.258,1343773580499803648,37.3989018,-121.9791254,-27.2050,2.6840,0.0090,0.0000,0.2730,0.4510,1.1400,3,26,0.0600,180.0000,0*07
				*/
#ifndef NO_GGA
				gp2[0] = (atof(val[1]) * 1.0e-3) + setting->timeoffset; /* time MCU */
				gp2[1] = (atof(val[2]) * 1.0e-9) + setting->timeoffset_gps; /* GPS ns */
				gp2[1] -= floor(gp2[1] / (7 * 24 * 3600)) * 7 * 24 * 3600;
#else
				gp2[0] = atof(val[1]); /* time MCU */
				gp2[1] = atof(val[2]); /* GPS ns */
#endif

				gp2[2] = atof(val[3]); /* lat */
				gp2[3] = atof(val[4]); /* lon */
				gp2[4] = atof(val[5]); /* ht */
				gp2[5] = atof(val[6]); /* msl */

				gp2[6] = atof(val[7]); /* speed */
				gp2[7] = atof(val[8]); /* heading */
				gp2[8] = atof(val[9]); /* acc_h */
				gp2[9] = atof(val[10]); /* acc_v */
				gp2[10] = atof(val[11]); /* pdop */
				gp2[11] = atof(val[12]); /* fixtype */
				gp2[12] = atof(val[13]); /* sat number */
				gp2[13] = atof(val[14]); /* acc speed */
				gp2[14] = atof(val[15]); /* acc heading */
				gp2[15] = atof(val[16]); /* rtk fix status */
#ifndef NO_GGA
				gp2[16] = (atof(val[2]) * 1.0e-9) + setting->timeoffset_gps; /* GPS ns */
#endif
				if (!fGP2_CSV)
				{
					fGP2_CSV = set_output_file(fname, "-gp2.csv");
					
#ifndef NO_GGA
					if (fGP2_CSV && setting->csv_header) fprintf(fGP2_CSV, "%s", GPS_HEADER);
#endif
				}
#ifndef NO_GGA
				if (!fGP2_GGA) fGP2_GGA = set_output_file(fname, "-gp2.nmea");

				process_gps(gp2, pv2s, fGP2_CSV, fGP2_GGA, setting->is_pp);
#else
				process_gps(gp2, fGP2_CSV);
#endif
				isOK = 1;
			}
			if (!isOK && num >= 12 && strstr(val[0], "APHDG") != NULL)
			{
				/*
#APHDG,31527.383,1362269876750000128,2.13,1.60,3.23,4.19,36.92845,0.2796,4.00156,303*59
				*/
#ifndef NO_GGA
				hdr[0] = atof(val[1]) * 1.0e-3 + setting->timeoffset; /* time MCU */
				hdr[1] = atof(val[2]) * 1.0e-9 + setting->timeoffset_gps; /* GPS ns */
				hdr[1] -= floor(hdr[1] / (7 * 24 * 3600)) * 7 * 24 * 3600;
#else
				hdr[0] = atof(val[1]); /* time MCU */
				hdr[1] = atof(val[2]); /* GPS ns */
#endif

				hdr[2] = atof(val[3]); /* n */
				hdr[3] = atof(val[4]); /* d */
				hdr[4] = atof(val[5]); /* d */
				hdr[5] = atof(val[6]); /* length */

				hdr[6] = atof(val[7]); /* heading */
				hdr[7] = atof(val[8]); /* acc length */
				hdr[8] = atof(val[9]); /* acc length */
				hdr[9] = atof(val[10]); /* flag */

				if (!fHDR)
				{
					fHDR = set_output_file(fname, "-hdr.csv");
#ifndef NO_GGA
					if (fHDR && setting->csv_header) fprintf(fHDR, "%s", HDG_HEADER);
#else
				}
#ifndef NO_GGA
				process_hdr(hdr, hdrs, fHDR, setting->is_pp);
#endif
				process_hdr(hdr, fHDR);
#endif
				isOK = 1;
			}
			if (!isOK && num >= 12 && strstr(val[0], "APIMU") != NULL)
			{
				/*
				#APIMU,318214.937,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,0.01051,0.0000,318214.548,47.0547*55
				*/
#ifndef NO_GGA
				imu[0] = atof(val[1]) * 1.0e-3 + setting->timeoffset;
#else
				imu[0] = atof(val[1]);
#endif
				int loc = (num == 15) ? 3 : 2;
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

				/* calibrate the FOG gyro scale factor */
#ifndef NO_GGA
				add_gyro_data(&gyro_sfw, imu[0], imu[6], imu[7], imu[10]);
#endif

				if (!fIMU)
				{
					fIMU = set_output_file(fname, "-imu.csv");
#ifndef NO_GGA
					if (fIMU && setting->csv_header) fprintf(fIMU, "%s", IMU_HEADER);
#endif
				}
#ifndef NO_GGA
				process_imu(imu, imus, fIMU, setting->is_pp);
#else
				process_imu(imu, fIMU);
#endif

				/* send to engine */
				isOK = 1;
			}
			if (!isOK && num >= 14 && strstr(val[0], "APINS") != NULL)
			{
				/* time[s], lat[radian], lon[radian], ht[m], vn[m / s], ve[m / s], vd[m / s], roll[deg], pitch[deg], yaw[deg] */
				/*
				#APINS,318215,1343773580502990592,1,37.398875500000,-121.979132700000,-27.965002059937,,,,-0.166232,1.773182,0.250746,1*74
				*/
#ifndef NO_GGA
				ins[0] = atof(val[1]) * 1.0e-3 + setting->timeoffset;
				ins[1] = atof(val[2]) * 1.0e-9;
#else
				ins[0] = atof(val[1]);
				ins[1] = atof(val[2]);
#endif
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
				if (!fCSV)
				{
					fCSV = set_output_file(fname, "-rts.csv");
#ifndef NO_GGA
					if (fCSV && setting->csv_header) fprintf(fCSV, "%s", INS_HEADER);
#endif
				}
#ifndef NO_GGA
				if (!fGGA) fGGA = set_output_file(fname, "-rts.nmea");

				process_ins(ins, pvas, fCSV, fGGA, setting->is_pp);
#else
				process_ins(ins, fCSV);
#endif

#ifndef NO_GGA
				ins[1] -= floor(ins[1] / (7 * 24 * 3600.0)) * 7 * 24 * 3600;

				if (fabs(ins[3]) < 1.0e-10 || fabs(ins[4]) < 1.0e-10)
				{

				}
				else
				{
					if (numofepoch == 0)
						start_time = ins[1];
					else
						end_time = ins[1];
					++numofepoch;
				}
#endif
				isOK = 1;
			}
			if (!isOK)
			{
				printf("%s\n", a1buff.buf);
			}
			a1buff.nbyte = 0;
		}
	}
#ifndef NO_GGA
	printf("%s,%s\n", fname, setting->axis);
#else
	printf("%s\n", fname);
#endif

#ifndef NO_GGA
	process_log_data(fname, imus, pvts, pv2s, hdrs, pvas, setting);
#endif
	if (fLOG) fclose(fLOG);
	if (fCSV) fclose(fCSV);
	if (fGGA) fclose(fGGA);
	if (fIMU) fclose(fIMU);
	if (fHDR) fclose(fHDR);
	if (fGPS_CSV) fclose(fGPS_CSV);
	if (fGP2_CSV) fclose(fGP2_CSV);
	if (fGPS_GGA) fclose(fGPS_GGA);
	if (fGP2_GGA) fclose(fGP2_GGA);
	if (fANT1) fclose(fANT1);
	if (fANT2) fclose(fANT2);
	if (fBASE) fclose(fBASE);

#ifndef NO_GGA
	if (gap.time > 0)
	{
		gap.length = imu[0] - gap.time;
		if (gap.length > 1.5)
		{
			vGAP.push_back(gap);
		}
	}

	for (int i = 0; i < (int)vGAP.size(); ++i)
	{
		for (int j = i + 1; j < (int)vGAP.size(); ++j)
		{
			if (vGAP[i].length < vGAP[j].length)
			{
				gap = vGAP[i];
				vGAP[i] = vGAP[j];
				vGAP[j] = gap;
			}
		}
	}

	if (numofepoch > 0)
	{
		FILE* fSTATUS = fopen("status.csv", "a");
		if (fSTATUS)
		{
			fprintf(fSTATUS, "%10.3f,%10.3f,%s,%10i,%10.3f,%10i,%10i,%7.2f,%10.3f", start_time, end_time, fname, numofepoch, end_time - start_time, numofsec_GPS, numofsec_IMU, numofsec_IMU > 0 ? (numofsec_GPS * 100.0 / numofsec_IMU) : (0), distance_traveled/1000.0);
			for (int i = 0; i < (int)vGAP.size(); ++i)
			{
				fprintf(fSTATUS, ",%10.3f,(%6.0f,%7.3f)", vGAP[i].time, vGAP[i].length, vGAP[i].dist / 1000.0);
			}
			fprintf(fSTATUS, "\n");
			fclose(fSTATUS);
		}
	}
#endif
	return 0;
}


#ifndef NO_GGA
static int parse_data_buff(const char* buff, int n, double* data)
{
	char tmp[255] = { 0 };
	char* val[MAXFIELD];
	strcpy(tmp, buff);
	int num = parse_fields(tmp, val);
	if (num >= n)
	{
		for (int i=0;i<n;++i)
			data[i] = atof(val[i]);
		return 1;
	}
	else
		return 0;
}
#endif

#ifndef NO_GGA
static int parse_fname_buff(const char* buff, char *imufname, char* gpsfname, char *insfname)
{
	char tmp[255] = { 0 };
	char* val[MAXFIELD];
	strcpy(tmp, buff);
	int num = parse_fields(tmp, val);
	if (num >= 3)
	{
		strcpy(imufname, val[0]);
		strcpy(gpsfname, val[1]);
		strcpy(insfname, val[2]);
		return 1;
	}
	else
		return 0;
}
#endif

#ifndef NO_GGA
static int convert_isee_data_to_nmea(const char* fname)
{
	FILE* fLOG = fopen(fname, "r");
	FILE* fGGA = NULL;
	char buffer[255] = { 0 };
	unsigned long numofline = 0;
	char* val[MAXFIELD];
	while (fLOG && !feof(fLOG) && fgets(buffer, sizeof(buffer), fLOG))
	{
		++numofline;
		if (numofline < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 5) continue;
		double time = atof(val[0]) / 1000.0;
		double blh[3] = { 0 };
		blh[1] = atof(val[1]) * D2R;
		blh[0] = atof(val[2]) * D2R;
		blh[2] = atof(val[3]);
		int code = atoi(val[4]);
		if (!fGGA) fGGA = set_output_file(fname, "-isee.nmea");
		if (fGGA)
		{
			char buffer[255] = { 0 };
			outnmea_gga((unsigned char*)buffer, (float)time, 1, blh, 10, (float)1.0, (float)0);
			fprintf(fGGA, "%s", buffer);
		}

	}
	if (fLOG) fclose(fLOG);
	if (fGGA) fclose(fGGA);
	return numofline;
}

static int convert_isee_trimble_to_nmea(const char* fname)
{
	FILE* fLOG = fopen(fname, "r");
	FILE* fGGA = NULL;
	char buffer[255] = { 0 };
	unsigned long numofline = 0;
	char* val[MAXFIELD];
	while (fLOG && !feof(fLOG) && fgets(buffer, sizeof(buffer), fLOG))
	{
		//++numofline;
		//if (numofline < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 5) continue;
		double time = atof(val[1]) / 1000.0;
		double blh[3] = { 0 };
		blh[0] = atof(val[2]) * D2R;
		blh[1] = atof(val[3]) * D2R;
		blh[2] = atof(val[4]);
		if (!fGGA) fGGA = set_output_file(fname, ".nmea");
		if (fGGA)
		{
			char buffer[255] = { 0 };
			outnmea_gga((unsigned char*)buffer, (float)time, 1, blh, 10, (float)1.0, (float)0);
			fprintf(fGGA, "%s", buffer);
		}

	}
	if (fLOG) fclose(fLOG);
	if (fGGA) fclose(fGGA);
	return numofline;
}

/* convert OXTS export CSV file to NMEA GGA 
*  time (Ws), latitude (deg), longitude (deg), height (m), vn (m/s), ve (m/s), vd (m/s), roll (deg), pitch (deg), heading(deg), acc_x (m/s^2) acc_y (m/s^2), acc_z (m/s^2), gyro_x (rad/s), gyro_y(rad/s), gyro_z(rad/s)
*/
static int convert_oxts_csv_to_nmea(const char* fname)
{
	FILE* fLOG = fopen(fname, "r");
	FILE* fGGA = NULL;
	char buffer[255] = { 0 };
	unsigned long numofline = 0;
	char* val[MAXFIELD];
	double start_time = 0;
	double end_time = 0;
	unsigned long numofepoch = 0;
	while (fLOG && !feof(fLOG) && fgets(buffer, sizeof(buffer), fLOG))
	{
		//++numofline;
		//if (numofline < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 5) continue;
		if (buffer[0] == -17 && buffer[1] == -69 && buffer[2] == -65)
		{
			buffer[0] = buffer[1] = buffer[2] = ' ';
		}
		double time = atof(val[0]);
		double blh[3] = { 0 };
		blh[0] = atof(val[1]) * D2R;
		blh[1] = atof(val[2]) * D2R;
		blh[2] = atof(val[3]);
		if (!fGGA) fGGA = set_output_file(fname, ".nmea");
		if (fGGA)
		{
			char buffer[255] = { 0 };
			outnmea_gga((unsigned char*)buffer, (float)time, 1, blh, 10, (float)1.0, (float)0);
			fprintf(fGGA, "%s", buffer);
		}
		if (numofepoch == 0)
			start_time = time;
		else
			end_time = time;
		++numofepoch;
	}
	if (fLOG) fclose(fLOG);
	if (fGGA) fclose(fGGA);
	if (numofepoch > 0)
	{
		FILE* fSTATUS = fopen("status.csv", "a");
		if (fSTATUS)
		{
			fprintf(fSTATUS, "%10.3f,%10.3f,%s,%i,%10.3f\n", start_time, end_time, fname, numofepoch, end_time-start_time);
			fclose(fSTATUS);
		}
	}
	return numofline;
}

/* convert NovAtel asc file to NMEA GGA */
static int convert_novatel_asc_to_nmea(const char* fname)
{
	FILE* fLOG = fopen(fname, "r");
	FILE* fGGA = NULL;
	char buffer[255] = { 0 };
	unsigned long numofline = 0;
	char* val[MAXFIELD];
	double start_time = 0;
	double end_time = 0;
	unsigned long numofepoch = 0;
	while (fLOG && !feof(fLOG) && fgets(buffer, sizeof(buffer), fLOG))
	{
		//++numofline;
		//if (numofline < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 24) continue;
		if (strstr(val[0], "#INSPVAXA") && !strstr(val[4], "UNKNOWN"))
		{
			double time = atof(val[6]);
			double blh[3] = { 0 };
			blh[0] = atof(val[11]) * D2R;
			blh[1] = atof(val[12]) * D2R;
			blh[2] = atof(val[13])+ atof(val[14]);
			double vel[3] = { 0 };
			vel[0] = atof(val[15]);
			vel[1] = atof(val[16]);
			vel[2] = atof(val[17]);
			double att[3] = { 0 };
			att[0] = atof(val[18]);
			att[1] = atof(val[19]);
			att[2] = atof(val[20]);
			double rmsP[3] = { 0 };
			rmsP[0] = atof(val[21]);
			rmsP[1] = atof(val[22]);
			rmsP[2] = atof(val[23]);
			//double rmsV[3] = { 0 };
			//rmsV[0] = atof(val[24]);
			//rmsV[1] = atof(val[25]);
			//rmsV[2] = atof(val[26]);
			//double rmsA[3] = { 0 };
			//rmsA[0] = atof(val[27]);
			//rmsA[1] = atof(val[28]);
			//rmsA[2] = atof(val[29]);
			if (numofepoch == 0)
				start_time = time;
			else
				end_time = time;
			++numofepoch;
			if (!fGGA) fGGA = set_output_file(fname, ".nmea");
			if (fGGA)
			{
				char buffer[255] = { 0 };
				outnmea_gga((unsigned char*)buffer, (float)time, 1, blh, 10, (float)1.0, (float)0);
				fprintf(fGGA, "%s", buffer);
			}
		}
	}
	if (fLOG) fclose(fLOG);
	if (fGGA) fclose(fGGA);
	if (numofepoch > 0)
	{
		FILE* fSTATUS = fopen("status.csv", "a");
		if (fSTATUS)
		{
			fprintf(fSTATUS, "%10.3f,%10.3f,%s,%i,%10.3f\n", start_time, end_time, fname, numofepoch, end_time-start_time);
			fclose(fSTATUS);
		}
	}
	return numofline;
}
#endif

#ifndef NO_GGA
int read_ini_file(const char* fname, setting_t* setting)
{
	FILE* fINI = fopen(fname, "r");
	char buffer[255] = { 0 };
	char* temp = nullptr;
	double data[4] = { 0 };
	while (fINI && !feof(fINI) && fgets(buffer, sizeof(buffer), fINI))
	{
		temp = strchr(buffer, '\n'); if (temp) temp[0] = '\0';
		temp = strchr(buffer, '#'); if (temp) temp[0] = '\0';
		temp = strchr(buffer, ';'); if (temp) temp[0] = '\0';
		temp = strchr(buffer, '!'); if (temp) temp[0] = '\0';
		temp = strchr(buffer, '='); if (!temp) continue;

		if (strstr(buffer, "axis"))
			strcpy(setting->axis, temp + 1); /* axis */
		else if (strstr(buffer, "key"))
			strcpy(setting->key, temp + 1); /* axis */
		else if (strstr(buffer, "lao_ant1"))
			parse_data_buff(temp + 1, 3, setting->ant1); /* lao ant1 */
		else if (strstr(buffer, "lao_ant2"))
			parse_data_buff(temp + 1, 3, setting->ant2); /* lao ant2 */
		else if (strstr(buffer, "lao_out"))
			parse_data_buff(temp + 1, 3, setting->out); /* lao out */
		else if (strstr(buffer, "lao_rwc"))
			parse_data_buff(temp + 1, 3, setting->rwc); /* lao rwc */
		else if (strstr(buffer, "imu_time_offset"))
			setting->timeoffset = atof(temp + 1); /* imu time offset */
		else if (strstr(buffer, "gps_time_offset"))
			setting->timeoffset_gps = atof(temp + 1); /* gps time offset */
		else if (strstr(buffer, "install_angle") || strstr(buffer, "lao_mia"))
		{
			parse_data_buff(temp + 1, 3, setting->mia); /* install angle in deg */
		}
		else if (strstr(buffer, "is_rts_enu")) /* rtcm RTS in ENU frame */
			setting->is_rts_enu = 1;
		else if (strstr(buffer, "gps_speed")) /* turn on/off GPS speed as odometer */
			setting->is_gps_speed = atoi(temp + 1);
		else if (strstr(buffer, "is_mems")) /* use MEMS Gyro in Z */
			setting->is_mems = atoi(temp + 1);
		else if (strstr(buffer, "is_ant1")) /* use antenna 1   */
			setting->is_ant1 = atoi(temp + 1);
		else if (strstr(buffer, "is_ant2")) /* use antenna 2 */
			setting->is_ant2 = atoi(temp + 1);
		else if (strstr(buffer, "csv_header"))
			setting->csv_header = atoi(temp + 1);
		else if (strstr(buffer, "start_time"))  /* start time */
			setting->start_time = atof(temp + 1);
		else if (strstr(buffer, "end_time")) /* end time */
			setting->end_time = atof(temp + 1);
		else if (strstr(buffer, "nhc_sigma_side"))
			setting->nhc_sigma_side = atoi(temp + 1);
		else if (strstr(buffer, "nhc_sigma_vert"))
			setting->nhc_sigma_vert = atoi(temp + 1);
		else if (strstr(buffer, "speed_sigma"))
			setting->speed_sigma = atoi(temp + 1);
		else if (strstr(buffer, "init_pos")) /* initial position */
		{
			if (parse_data_buff(temp + 1, 3, setting->init_pos))
			{
				setting->init_pos[0] *= D2R;
				setting->init_pos[1] *= D2R;
			}
		}
		else if (strstr(buffer, "init_vel")) /* initial velocity */
			parse_data_buff(temp + 1, 3, setting->init_vel);
		else if (strstr(buffer, "init_att")) /* initial attitude */
		{
			if (parse_data_buff(temp + 1, 3, setting->init_att))
			{
				setting->init_att[0] *= D2R;
				setting->init_att[1] *= D2R;
				setting->init_att[2] *= D2R;
			}
		}
		else if (strstr(buffer, "init_bf")) /* initial acc bias */
			parse_data_buff(temp + 1, 3, setting->init_bf);
		else if (strstr(buffer, "init_bw")) /* initial gyro bias */
			parse_data_buff(temp + 1, 3, setting->init_bw);
		else if (strstr(buffer, "init_sff")) /* initial acc bias scale factor */
			parse_data_buff(temp + 1, 3, setting->init_sff);
		else if (strstr(buffer, "init_sfw")) /* initial gyro bias scale factor */
			parse_data_buff(temp + 1, 3, setting->init_sfw);
		else if (strstr(buffer, "init_sfo"))
			setting->init_sfo = atof(temp + 1);
		else if (strstr(buffer, "pos_sigma_vert"))
			setting->init_pos_sigma_vert = atof(temp + 1);
		else if (strstr(buffer, "pos_sigma_hori"))
			setting->init_pos_sigma_hori = atof(temp + 1);
		else if (strstr(buffer, "vel_sigma_vert"))
			setting->init_vel_sigma_vert = atof(temp + 1);
		else if (strstr(buffer, "vel_sigma_hori"))
			setting->init_vel_sigma_hori = atof(temp + 1);
		else if (strstr(buffer, "att_sigma_vert"))
			setting->init_att_sigma_vert = atof(temp + 1);
		else if (strstr(buffer, "att_sigma_hori"))
			setting->init_att_sigma_hori = atof(temp + 1);
		else if (strstr(buffer, "bf_sigma_hori"))
			setting->init_bf_sigma_hori = atof(temp + 1);
		else if (strstr(buffer, "bf_sigma_vert"))
			setting->init_bf_sigma_vert = atof(temp + 1);
		else if (strstr(buffer, "bw_sigma_hori"))
			setting->init_bw_sigma_hori = atof(temp + 1);
		else if (strstr(buffer, "bw_sigma_vert"))
			setting->init_bw_sigma_vert = atof(temp + 1);
		else if (strstr(buffer, "gap")) /* GPS GAP */
		{
			if (parse_data_buff(temp + 1, 2, data) && data[0] >= 0 && data[1] > 0) /* gap (time, length) */
			{
				gap_t new_gap = { 0 };
				new_gap.time = data[0];
				new_gap.length = data[1];
				setting->gaps.push_back(new_gap);
			}
		}
		else if (strstr(buffer, "zupt")) /* ZUPT */
		{
			if (parse_data_buff(temp + 1, 2, data) && data[0] >= 0 && data[1] > 0) /* zupt (time, length) */
			{
				zupt_t new_zupt = { 0 };
				new_zupt.time = data[0];
				new_zupt.length = data[1];
				setting->zupts.push_back(new_zupt);
			}
		}
		else if (strstr(buffer, "fix_speed")) /* fix speed constraint */
		{
			if (parse_data_buff(temp + 1, 3, data) && data[0] >= 0 && data[1] > 0) /* fix speed segment (time, length, value) */
			{
				speed_t new_speed = { 0 };
				new_speed.time = data[0];
				new_speed.length = data[1];
				new_speed.value = data[2];
				setting->speeds.push_back(new_speed);
			}
		}
	}
	if (fINI) fclose(fINI);
	return 0;
}
#endif


int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("help info\n");
		printf("%s filename format setting\n", argv[0]);
		printf("if only the filename exists, the default format will be a1-log, axis is XYZ, all lever arm offsets (lao) are 0 \n");
		printf("format   =>  a1-log, a1-csv\n");
		printf("filename =>  a1-log    => the asc log file with APIMU, APGPS, APINS\n");
		printf("             a1-csv    => imu.csv,gps.csv,ins.csv\n");
		printf("             a1-log-pp => the asc log file with APIMU, APGPS, APINS with PP engine on (do not recommend for big file)\n");
		printf("             a1-csv-pp => imu.csv,gps.csv,ins.csv with PP engine on (do not recommend for big file)\n");
		printf("The setting can be the following (axis, lao_ant1, lao_ant2, lao_out, lao_rwc, imu_time_offset, gps_time_offset, install_angle) with the format key=value\n");
		printf("axis           =>  how the unit align in vehicle forward-right-down frame, default is XYZ, => axis=XYZ \n");
		printf("lao_ant1       =>  GNSS antenna 1 phase center w.r.t IMU body center in forward-right-down frame (do not put space in it, unit meter) => for example lao_ant1=+0.3048,-0.7620,-0.4572 \n");
		printf("lao_ant2       =>  GNSS antenna 2 phase center w.r.t IMU body center in forward-right-down frame\n");
		printf("lao_out        =>  INS solution output center  w.r.t IMU body center in forward-right-down frame\n");
		printf("lao_rwc        =>  vehicle real-wheel-center   w.r.t IMU body center in forward-right-down frame\n");
		printf("lao_mia        =>  installation angle          w.r.t IMU body center in forward-right-down frame [unit deg] => install_angle=0.01,0.2,0.5 \n");
		printf("imu_time_offset=>  imu time offset w.r.t GPS time (second)\n");
		printf("gps_time_offset=>  gps time offset w.r.t GPS time (second) due to time delay\n");
		printf("is_rts_enu     =>  is_rts_enu=0\n");
		printf("is_gps_speed   =>  use GPS speed as odometer, for example, is_gps_speed=1\n");
		printf("is_mems        =>  use MEMS Gyro, for example, is_mems=1\n");
		printf("is_ant1        =>  is_ant1=1 =>  enable GPS Antenna 1 (default); is_ant1=0 => disable GNSS Antenna 1 \n");
		printf("is_ant2        =>  is_ant2=0 => disable GPS Antenna 2 (default); is_ant2=1 =>  enable GNSS Antenna 2, if set both is_ant1 and is_ant2 => need to set the correct lever arm between antenna 1 and antenna 2, otherwise, the solution will be bad\n");
		printf("csv_header     =>  csv_header=0 => disable header at the top CSV files (default); csv_header=1 ouutput header with column labels and unit in the first row of the csv file\n");
		printf("nhc_sigma_side =>  nhc constraint side sigma [cm/s] 0=> default value (20), 255 => off \n");
		printf("nhc_sigma_vert =>  nhc constraint vert sigma [cm/s] 0=> default value (20), 255 => off \n");
		printf("start_time     =>  start time in IMU time frame, for example, start_time=400.0\n");
		printf("end_time       =>  end   time in IMU time frame, for example, end_time  =1200.0\n");
		printf("gap            =>  GNSS GAP (start time and length), for example, set a GNSS gap start at IMU time 500.0 and length 60 (1minute), gap=500,60\n");
		printf("zupt           =>  ZUPT (start time and length), for example, set a zupt start at IMU time 500.0 and length 60 (1minute), zupt=500,60\n");
		printf("fix_speed      =>  constant speed constraint (start time, length, value), for example, fix_speed=4350,430,1.52\n");
		printf("init_bf        =>  initial acce bias [  g], for example, init_bf=-0.002377,0.027404,-0.001315\n");

		printf("init_bw        =>  initial gyro bias [dps], for example, init_bw=-0.003177,0.020409,0.004837\n");
		printf("init_pos       =>  init latitude [deg], longitude [deg], and height [m]\n");
		printf("init_att       =>  init roll [deg], pitch [deg], and heading/yaw [deg]\n");

		printf("ini_filename   =>  ini setting file\n");
	}
	else if (argc < 3)
	{
		/* exefname(0) filename(1) */
#ifndef NO_GGA
		process_log(argv[1], &setting);
#else
		process_log(argv[1]);
#endif
	}
	else
	{
#ifndef NO_GGA
		if (strstr(argv[2], "-pp")) /* turn on PP engine */
			setting.is_pp = 1;
#endif
		/* exefname(0) filename(1), format(2)  */
#ifndef NO_GGA
		for (int i = 3; i < argc; ++i)
		{
			const char* temp = strstr(argv[i], "=");
			if (temp)
			{
				double data[4] = { 0 };
				if (strstr(argv[i], "ini_filename")|| strstr(argv[i], "inifilename"))
				{
					read_ini_file(temp + 1, &setting);
				}
				else if (strstr(argv[i], "axis"))
					strcpy(setting.axis, temp + 1); /* axis */
				else if (strstr(argv[i], "key"))
					strcpy(setting.key, temp + 1); /* axis */
				else if (strstr(argv[i], "lao_ant1"))
					parse_data_buff(temp + 1, 3, setting.ant1); /* lao ant1 */
				else if (strstr(argv[i], "lao_ant2"))
					parse_data_buff(temp + 1, 3, setting.ant2); /* lao ant2 */
				else if (strstr(argv[i], "lao_out"))
					parse_data_buff(temp + 1, 3, setting.out); /* lao out */
				else if (strstr(argv[i], "lao_rwc"))
					parse_data_buff(temp + 1, 3, setting.rwc); /* lao rwc */
				else if (strstr(argv[i], "imu_time_offset"))
					setting.timeoffset = atof(temp + 1); /* imu time offset */
				else if (strstr(argv[i], "gps_time_offset"))
					setting.timeoffset_gps = atof(temp + 1); /* gps time offset */
				else if (strstr(argv[i], "install_angle"))
					parse_data_buff(temp + 1, 3, setting.mia); /* install angle */
				else if (strstr(argv[i], "lao_mia"))
					parse_data_buff(temp + 1, 3, setting.mia); /* install angle */
				else if (strstr(argv[i], "is_rts_enu")) /* rtcm RTS in ENU frame */
					setting.is_rts_enu = 1; 
				else if (strstr(argv[i], "gps_speed")) /* turn on/off GPS speed as odometer */
					setting.is_gps_speed = atoi(temp + 1);
				else if (strstr(argv[i], "is_mems")) /* use MEMS Gyro in Z */
					setting.is_mems = atoi(temp + 1);
				else if (strstr(argv[i], "is_ant1")) /* use antenna 1   */
					setting.is_ant1 = atoi(temp + 1);
				else if (strstr(argv[i], "is_ant2")) /* use antenna 2 */
					setting.is_ant2 = atoi(temp + 1);
				else if (strstr(argv[i], "csv_header"))
					setting.csv_header = atoi(temp + 1);
				else if (strstr(argv[i], "start_time"))  /* start time */
					setting.start_time = atof(temp + 1); 
				else if (strstr(argv[i], "end_time")) /* end time */
					setting.end_time = atof(temp + 1); 
				else if (strstr(argv[i], "nhc_sigma_side"))
					setting.nhc_sigma_side = atoi(temp + 1);
				else if (strstr(argv[i], "nhc_sigma_vert"))
					setting.nhc_sigma_vert = atoi(temp + 1);
				else if (strstr(argv[i], "speed_sigma"))
					setting.speed_sigma = atoi(temp + 1);
				else if (strstr(argv[i], "init_bf")) /* initial acc bias */
					parse_data_buff(temp + 1, 3, setting.init_bf); 
				else if (strstr(argv[i], "init_bw")) /* initial gyro bias */
					parse_data_buff(temp + 1, 3, setting.init_bw); 
				else if (strstr(argv[i], "init_pos")) /* initial position */
				{
					if (parse_data_buff(temp + 1, 3, setting.init_pos))
					{
						setting.init_pos[0] *= D2R;
						setting.init_pos[1] *= D2R;
					}
				}
				else if (strstr(argv[i], "init_vel")) /* initial velocity */
					parse_data_buff(temp + 1, 3, setting.init_vel); 
				else if (strstr(argv[i], "init_att")) /* initial attitude */
				{
					if (parse_data_buff(temp + 1, 3, setting.init_att))
					{
						setting.init_att[0] *= D2R;
						setting.init_att[1] *= D2R;
						setting.init_att[2] *= D2R;
					}
				}
				else if (strstr(argv[i], "pos_sigma_vert"))
					setting.init_pos_sigma_vert = atof(temp + 1);
				else if (strstr(argv[i], "pos_sigma_hori"))
					setting.init_pos_sigma_hori = atof(temp + 1);
				else if (strstr(argv[i], "vel_sigma_vert"))
					setting.init_vel_sigma_vert = atof(temp + 1);
				else if (strstr(argv[i], "vel_sigma_hori"))
					setting.init_vel_sigma_hori = atof(temp + 1);
				else if (strstr(argv[i], "att_sigma_vert"))
					setting.init_att_sigma_vert = atof(temp + 1);
				else if (strstr(argv[i], "att_sigma_hori"))
					setting.init_att_sigma_hori = atof(temp + 1);
				else if (strstr(argv[i], "bf_sigma_vert"))
					setting.init_bf_sigma_vert = atof(temp + 1);
				else if (strstr(argv[i], "bf_sigma_hori"))
					setting.init_bf_sigma_hori = atof(temp + 1);
				else if (strstr(argv[i], "bw_sigma_vert"))
					setting.init_bw_sigma_vert = atof(temp + 1);
				else if (strstr(argv[i], "bw_sigma_hori"))
					setting.init_bw_sigma_hori = atof(temp + 1);
				else if (strstr(argv[i], "gap")) /* GPS GAP */
				{
					if (parse_data_buff(temp + 1, 2, data) && data[0] >= 0 && data[1] > 0) /* gap (time, length) */
					{
						gap_t new_gap = { 0 };
						new_gap.time = data[0];
						new_gap.length = data[1];
						setting.gaps.push_back(new_gap);
					}
				}
				else if (strstr(argv[i], "zupt")) /* ZUPT */
				{
					if (parse_data_buff(temp + 1, 2, data) && data[0] >= 0 && data[1] > 0) /* zupt (time, length) */
					{
						zupt_t new_zupt = { 0 };
						new_zupt.time = data[0];
						new_zupt.length = data[1];
						setting.zupts.push_back(new_zupt);
					}
				}
				else if (strstr(argv[i], "fix_speed")) /* fix speed constraint */
				{
					if (parse_data_buff(temp + 1, 3, data) && data[0] >= 0 && data[1] > 0) /* fix speed segment (time, length, value) */
					{
						speed_t new_speed = { 0 };
						new_speed.time = data[0];
						new_speed.length = data[1];
						new_speed.value = data[2];
						setting.speeds.push_back(new_speed);
					}
				}
			}
		}
#endif
		if (strstr(argv[2], "a1-log") != NULL)
		{
#ifndef NO_GGA
			process_log(argv[1], &setting);
#else
			process_log(argv[1]);
#endif
		}
#ifndef NO_GGA
		else if (strstr(argv[2], "isee-csv") != NULL)
		{
			/* isee data lever arm setting 
			* "axis=XYZ" "lao_ant1=0.640000,-0.510000,-1.910000" "lao_ant2=1.778000,-0.510000,-1.910000" "lao_out=1.448000,-1.219000,-1.778000" "lao_rwc=-2.610000,-0.030000,0.890000"
			* "0.640000,-0.510000,-1.910000" "1.778000,-0.510000,-1.910000" "1.448000,-1.219000,-1.778000" "-2.610000,-0.030000,0.890000"
			*/
			convert_isee_data_to_nmea(argv[1]);
		}
		else if (strstr(argv[2], "isee-trimble") != NULL)
		{
			convert_isee_trimble_to_nmea(argv[1]);
		}
		else if (strstr(argv[2], "oxts-csv") != NULL)
		{
			convert_oxts_csv_to_nmea(argv[1]);
		}
		else if (strstr(argv[2], "novatel-asc") != NULL)
		{
			convert_novatel_asc_to_nmea(argv[1]);
		}
		else if (strstr(argv[2], "a1-csv") != NULL)
		{
			char imufname[255] = { 0 };
			char gpsfname[255] = { 0 };
			char insfname[255] = { 0 };
			if (parse_fname_buff(argv[1], imufname, gpsfname, insfname))
			{
				process_log_csv(imufname, gpsfname, insfname, &setting);
			}
			else
			{
				printf("failed to get the input file (imu.csv,gps.csv,ins.csv)\n");
			}
		}
#endif
		else
		{
			printf("unknown data format %s\n", argv[2]);
		}
	}
	return 0;
}