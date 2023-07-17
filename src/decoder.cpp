#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>

#include <ros/ros.h>
#include "anello_ros_driver/APIMU.h"
#include "anello_ros_driver/APINS.h"
#include "anello_ros_driver/APGPS.h"
#include "anello_ros_driver/APHDG.h"

#include "bit_tools.h"
#include "serial_interface.h"

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

#ifndef MAXFIELD
#define MAXFIELD 100
#endif

#ifndef NO_GGA
#define NO_GGA
#endif

#ifndef PRINT_VALUES
#define PRINT_VALUES 0
#endif

#ifndef DEBUG
#define debug 0
#endif

using namespace std;

typedef struct
{
	int n_used;					//how many bytes have been put through the decoded
	int nbytes;					//how many bytes are stored in the buffer
	char buff[MAX_BUF_LEN];		//buffer from file
} file_read_buf_t;


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

static int input_a1_data (a1buff_t* a1, uint8_t data)
{
	int ret = 0, i = 0;
	if (a1->nbyte >= MAX_BUF_LEN) a1->nbyte = 0;
	/* #AP, 0xD3 */
	if (a1->nbyte == 0 && !(data == '#' || data == 0xD3)) { 
		a1->nbyte = 0; 
#if DEBUG
		ROS_WARN("BUF RESET: Bad 1st byte {%c}", data); 
#endif
		return 0; 
	}
	if (a1->nbyte == 1 && !((data == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3)) { a1->nbyte = 0; ROS_WARN("BUF RESET: Bad 2nd byte"); return 0; }
	if (a1->nbyte == 2 && !((data == 'P' && a1->buf[1] == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3)) { a1->nbyte = 0; ROS_WARN("BUF RESET: Bad 3rd byte"); return 0; }
	if (a1->nbyte == 0) {
		memset(a1, 0, sizeof(a1buff_t));
	}
	if (a1->nbyte < 3) { a1->buf[a1->nbyte++] = data; return 0; }
	if (a1->buf[0] != 0xD3)
	{
		// denote beginnings of data segments
		if (data == ',')
		{
			a1->loc[a1->nseg++] = a1->nbyte;
			if (a1->nseg == 2)
			{
				a1->nlen = 0;
			}
		}
		a1->buf[a1->nbyte++] = data;

		// if statement is shorthand for 'is asc message?'
		if (a1->nlen == 0)
		{
			/* check message end for complete asc message */
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
static FILE* set_output_file (const char* fname, const char* key)
{
	char filename[255] = { 0 }, outfilename[255] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
	return fopen(outfilename, "w");
}
#endif

static int parse_fields (char* const buffer, char** val)
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

static void process_gps (double *gps, ros::Publisher pub)
{
	/*
	* gps[0] = MCU_Time [ms]
	* gps[1] = GPS_Time [ns]
	* gps[2] = Latitude [deg]
	* gps[3] = Longitude [deg]
	* gps[4] = Alt_ellipsoid [m]
	* gps[5] = Alt_msl [m]
	* gps[6] = Speed [m/s]
	* gps[7] = heading [deg]
	* gps[8] = Hacc [m]
	* gps[9] = Vacc [m]
	* gps[10] = PDOP 
	* gps[11] = FixType (0=No Fix, 2=2D Fix, 3=3D Fix, 5=Time only)
	* gps[12] = SatNum
	* gps[13] = Speed Accuracy [m/s]
	* gps[14] = Heading Accuracy [deg]
	* gps[15] = RTK Fix Status (0=SPP, 1=RTK Float, 2=RTK Fix)
	*
	*/
	
	anello_ros_driver::APGPS msg;

	msg.mcu_time = gps[0];
	msg.gps_time = gps[1];
	
	msg.lat = gps[2];
	msg.lon = gps[3];
	msg.alt_ellipsoid = gps[4];
	msg.alt_msl = gps[5];

	msg.speed = gps[6];
	msg.heading = gps[7];

	msg.hacc = gps[8];
	msg.vacc = gps[9];

	msg.pdop = gps[10];
	msg.fix_type = (uint8_t)gps[11];
	msg.sat_num = (uint8_t)gps[12];

	msg.speed_accuracy = gps[13];
	msg.heading_accuracy = gps[14];

	msg.rtk_fix_status = (uint8_t)gps[15];

	pub.publish(msg);
#if PRINT_VALUES
	ROS_INFO("APGPS,%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", gps[0], gps[1], gps[2], gps[3], gps[4], gps[5], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15]);
#endif
}

static void process_hdr (double* hdr, ros::Publisher pub)
{
	/*
	* hdr[0] = MCU_Time [ms]
	* hdr[1] = GPS Time [ns]
	*
	* hdr[2] = relPosN [m]
	* hdr[3] = relPosE [m]
	* hdr[4] = relPosD [m]
	*
	* hdr[5] = relPosLength [m]
	* hdr[6] = relPosHeading [Deg] 
	* 
	* hdr[7] = relPosLength_Accuracy [m]
	* hdr[8] = relPosHeading_Accuracy [Deg]
	*
	* hdr[9] = Flags
	*
	* Flags
	* bit 0 : gnssFixOk
	* bit 1 : diffSoln
	* bit 2 : relPosValid
	* bit 4..3 : carrSoln
	* bit 5 : isMoving
	* bit 6 : refPosMiss
	* bit 7 : refObsMiss
	* bit 8 : relPosHeadingValid
	* bit 9 : relPosNormalized
	*
	*/
	uint16_t status = (uint16_t)hdr[9];

	anello_ros_driver::APHDG msg;

	msg.mcu_time = hdr[0];	
	msg.gps_time = hdr[1];	

	msg.rel_pos_n = hdr[2];	
	msg.rel_pos_e = hdr[3];	
	msg.rel_pos_d = hdr[4];	
	
	msg.rel_pos_length = hdr[5];	
	msg.rel_pos_heading = hdr[6];	
	
	msg.rel_pos_length_accuracy = hdr[7];	
	msg.rel_pos_heading_accuracy = hdr[8];	
	
	msg.status_flags = status;

	msg.gnss_fix_ok = (status & (1 << 0)) > 0;
	msg.diff_soln = (status & (1 << 1)) > 0;
	msg.rel_pos_valid = (status & (1 << 2)) > 0;
	msg.carrier_solution = (status & (3 << 3)) >> 3;
	msg.is_moving = (status & (1 << 5)) > 0;
	msg.ref_pos_miss = (status & (1 << 6)) > 0;
	msg.ref_obs_miss = (status & (1 << 7)) > 0;
	msg.rel_pos_heading_valid = (status & (1 << 8)) > 0;
	msg.rel_pos_normalized = (status & (1 << 9)) > 0;

	pub.publish(msg);

#if PRINT_VALUES
	ROS_INFO(
		/* h[x] = hdr[x] */
		/* h0     h1     h2    h3      h4     h5     h6     h7     h8   h9 s0 s1 s2 34 s5 s6 s7 s8 s9*/
		"APHDG,%10.4f,%10.5f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i\n",
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
}

double last_imu_mcu_time = -1.0;
static void process_imu (double* imu, ros::Publisher pub)
{
	/*
	* imu[0] = MCU_Time [ms]
	* imu[1] = ax [g]
	* imu[2] = ay [g]
	* imu[3] = az [g]
	* imu[4] = wx [Deg/s]
	* imu[5] = wy [Deg/s]
	* imu[6] = wz [Deg/s]
	* imu[7] = wz_fog [Deg/s]
	* imu[8] = odr [m/s]
	* imu[9] = odr Time [ms]
	* imu[10] = Temp [C]
	*/
	double delta_imu_time;

	if (last_imu_mcu_time != -1.0)
	{
		delta_imu_time = imu[0] - last_imu_mcu_time;
		if (delta_imu_time > 6.0)
		{
			ROS_WARN("MISSING MESSAGE: dt=%5.5f", delta_imu_time);
		}
	}
	last_imu_mcu_time = imu[0];

	anello_ros_driver::APIMU msg;
	msg.mcu_time = imu[0];
	msg.ax = imu[1];
	msg.ay = imu[2];
	msg.az = imu[3];
	msg.wx = imu[4];
	msg.wy = imu[5];
	msg.wz = imu[6];
	msg.wz_fog = imu[7];
	msg.temp = imu[8];

	pub.publish(msg);

#if PRINT_VALUES
	ROS_INFO("APIMU,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6], imu[7], imu[8], imu[9], imu[10], imu[11]);
#endif
}

static void process_ins (double* ins, ros::Publisher pub)
{
	/*
	* ins[0] = MCU_Time [ms]
	* ins[1] = GPS_Time [ns]
	*
	* ins[2] = INS Status (255=uninitialized, 0=Attitude only, 1=Pos and Att, 2=Pos Hdg Att, 3= RTK Float, 4=RTK Fix)
	*
	* ins[3] = Latitude [deg]
	* ins[4] = Longitude [deg]
	* ins[5] = Alt_ellipsoid [m]
	*
	* ins[6] = Vn [m/s]
	* ins[7] = Ve [m/s]
	* ins[8] = Vd [m/s]
	*
	* ins[9] = Roll [deg]
	* ins[10] = Pitch [deg]
	* ins[11] = Heading [deg]
	*
	* ins[12] = zupt (1=stationary, 0=moving)
	*/

	anello_ros_driver::APINS msg;

	msg.mcu_time = ins[0];
	msg.gps_time = ins[1];

	msg.ins_status = (uint8_t)ins[2];
	
	msg.lat = ins[3];
	msg.lon = ins[4];
	msg.Alt_ellipsoid = ins[5];

	msg.vn = ins[6];
	msg.ve = ins[7];
	msg.vd = ins[8];

	msg.roll = ins[9];
	msg.pitch = ins[10];
	msg.heading = ins[11];

	msg.zupt = (uint8_t)ins[12];

	pub.publish(msg);

#if PRINT_VALUES
	ROS_INFO("APINS,%10.3f,%14.7f,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", ins[0], ins[1], ins[2], ins[3], ins[4], ins[5], ins[6], ins[7], ins[8], ins[9], ins[10], ins[11], ins[12]);
#endif
}

static bool is_file_buff_all_used (file_read_buf_t *buf)
{
	return buf->n_used == buf->nbytes; 
}

static char file_buff_get_c (FILE *fp, file_read_buf_t *file_buf)
{
	if (is_file_buff_all_used(file_buf))
	{
		file_buf->nbytes = fread(file_buf->buff, sizeof(char), MAX_BUF_LEN, fp);
		file_buf->n_used = 0;

		// if no bytes are read
		if (file_buf->nbytes == 0)
		{
			return EOF;
		}
	} 

	file_buf->n_used++;
	return file_buf->buff[file_buf->n_used-1];
}

static int process_log ()
{
	ros::NodeHandle nh;
	ros::Publisher pub_imu = nh.advertise<anello_ros_driver::APIMU>("APIMU",10);
	ros::Publisher pub_ins = nh.advertise<anello_ros_driver::APINS>("APINS",10);
	ros::Publisher pub_gps = nh.advertise<anello_ros_driver::APGPS>("APGPS",10);
	ros::Publisher pub_hdg = nh.advertise<anello_ros_driver::APHDG>("APHDG",10);


	file_read_buf_t serial_read_buf = { 0 };
	size_t bytes_read;

	int data = 0;
	char* val[MAXFIELD];
	bool checksum_passed;

	a1buff_t a1buff = { 0 };

	double gps[20] = { 0 };
	double gp2[20] = { 0 };
	double hdr[20] = { 0 };
	double imu[20] = { 0 };
	double ins[20] = { 0 };

	rtcm_apimu_t rtcm_apimu = { 0 };
	rtcm_old_apimu_t rtcm_old_apimu = { 0 };
	rtcm_apgps_t rtcm_apgps = { 0 };
	rtcm_aphdr_t rtcm_aphdr = { 0 };
	rtcm_apins_t rtcm_apins = { 0 };

	double last_imu_mcu_time = -1.0;
	double delta_imu_time = -1.0;

	serial_interface anello_device(default_serial_interface);
	while (true)
	{
		if (serial_read_buf.n_used >= serial_read_buf.nbytes)
		{
			serial_read_buf.nbytes = anello_device.get_data(serial_read_buf.buff, MAX_BUF_LEN);
			serial_read_buf.n_used = 0;
		} 
		else
		{
			int ret = input_a1_data(&a1buff, serial_read_buf.buff[serial_read_buf.n_used]);
			serial_read_buf.n_used++;
			if (ret)
			{
				int isOK = 0;
				int num = 0;
				if (ret == 1)
				{
					checksum_passed = checksum(a1buff.buf, a1buff.nbyte);

					if (checksum_passed)
					{
						num = parse_fields((char*)a1buff.buf, val);
					}
					else
					{
						ROS_WARN("Checksum Fail");
						num = 0;
					}
				}
				else if (ret == 5) /* rtcm */
				{
					if (a1buff.type == 4058 && !a1buff.crc)
					{
						if (a1buff.subtype == 1) /* IMU */
						{
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

							process_imu(imu, pub_imu);
						}
						else if (a1buff.subtype == 2) /* GPS PVT */
						{
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
								process_gps(gps, pub_gps);
							}
							else
							{
								process_gps(gps, pub_gps);
							}
						}
						else if (a1buff.subtype == 3) /* DUAL ANTENNA */
						{
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

							process_hdr(hdr, pub_hdg);
						}
						else if (a1buff.subtype == 4) /* INS */
						{
							memcpy((uint8_t*)&rtcm_apins, a1buff.buf + 5, sizeof(rtcm_apins_t));

							ins[0] = rtcm_apins.Time * 1e-6;
							ins[1] = rtcm_apins.GPS_Time;
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
							ins[12] = rtcm_apins.ZUPT; /* zupt */
							process_ins(ins, pub_ins);
						}
					}
					isOK = 1;
				}

				if (!isOK && num >= 17 && strstr(val[0], "APGPS") != NULL)
				{
					/* time [s], lat [deg], lon [deg], ht [m], speed [m/s], heading [deg], hor. accuracy [m], ver. accuracy [m], PDOP, fixType, sat num, gps second [s], pps [s] */
					/*
	#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,180.0000,0*65
					*/
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


					isOK = 1;
				}
				if (!isOK && num >= 12 && strstr(val[0], "APGP2") != NULL)
				{
					/*
	#APGP2,318213.258,1343773580499803648,37.3989018,-121.9791254,-27.2050,2.6840,0.0090,0.0000,0.2730,0.4510,1.1400,3,26,0.0600,180.0000,0*07
					*/
					gp2[0] = atof(val[1]); /* time MCU */
					gp2[1] = atof(val[2]); /* GPS ns */

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
					process_gps(gp2, pub_gps);
					isOK = 1;
				}
				if (!isOK && num >= 12 && strstr(val[0], "APHDG") != NULL)
				{
					/*
	#APHDG,31527.383,1362269876750000128,2.13,1.60,3.23,4.19,36.92845,0.2796,4.00156,303*59
					*/
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
					isOK = 1;
				}
				if (!isOK && num >= 12 && strstr(val[0], "APIMU") != NULL)
				{
					/*
					#APIMU,318214.937,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,0.01051,0.0000,318214.548,47.0547*55
					*/
					imu[0] = atof(val[1]);
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


					process_imu(imu, pub_imu);

					isOK = 1;
				}
				if (!isOK && num >= 14 && strstr(val[0], "APINS") != NULL)
				{
					/* time[s], lat[radian], lon[radian], ht[m], vn[m / s], ve[m / s], vd[m / s], roll[deg], pitch[deg], yaw[deg] */
					/*
					#APINS,318215,1343773580502990592,1,37.398875500000,-121.979132700000,-27.965002059937,,,,-0.166232,1.773182,0.250746,1*74
					*/
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

					isOK = 1;
				}
				if (!isOK)
				{
					printf("%s\n", a1buff.buf);
				}
				a1buff.nbyte = 0;
			}
		}
	}

	return 0;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"anello_ros_driver");
	process_log();


}