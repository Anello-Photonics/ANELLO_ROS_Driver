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
#include "decoder.h"
#include "rtcm_decoder.h"
#include "ascii_decoder.h"
#include "message_processing.h"

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



/*
* Parameters:
* a1buff_t *a1 : pointer to an a1buff object. This holds the current message candidate.
* uint8_t data : This holds the next byte to be added to the buffer. If the byte is correct it is added. If it isnt the buffer is reset.
*
* Return
* Outputs the status of the buffer
* 0 = not ready
* 1 = ASCII message ready
* 5 = RTCM message ready
*/
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



static int process_log ()
{
	//init ros publishers
	ros::NodeHandle nh;

	ros::Publisher pub_imu = nh.advertise<anello_ros_driver::APIMU>("APIMU",10);
	ros::Publisher pub_ins = nh.advertise<anello_ros_driver::APINS>("APINS",10);
	ros::Publisher pub_gps = nh.advertise<anello_ros_driver::APGPS>("APGPS",10);
	ros::Publisher pub_hdg = nh.advertise<anello_ros_driver::APHDG>("APHDG",10);

	ros_publishers_t pub_arr;
	pub_arr.imu = &pub_imu;
	pub_arr.ins = &pub_ins;
	pub_arr.gps = &pub_gps;
	pub_arr.hdg = &pub_hdg;


	//init buffer
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
		//if all data has been read... read more data into buff
		if (serial_read_buf.n_used >= serial_read_buf.nbytes)
		{
			serial_read_buf.nbytes = anello_device.get_data(serial_read_buf.buff, MAX_BUF_LEN);
			serial_read_buf.n_used = 0;
		} 
		else
		{

			int ret = input_a1_data(&a1buff, serial_read_buf.buff[serial_read_buf.n_used]);
			serial_read_buf.n_used++;
			
			//if message complete
			if (ret)
			{
				int isOK = 0;
				int num = 0;
				if (ret == 1)
				{
					// check that the checksum is correct
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
					isOK = decode_rtcm_message(a1buff, pub_arr);
				}

				if (!isOK && num >= 17 && strstr(val[0], "APGPS") != NULL)
				{
					isOK = decode_ascii_gps(val, *pub_arr.gps);
				}
				if (!isOK && num >= 12 && strstr(val[0], "APGP2") != NULL)
				{
					isOK = decode_ascii_gps(val, *pub_arr.gps);
				}
				if (!isOK && num >= 12 && strstr(val[0], "APHDG") != NULL)
				{
					isOK = decode_ascii_hdr(val, *pub_arr.hdg);
				}
				if (!isOK && num >= 12 && strstr(val[0], "APIMU") != NULL)
				{
					isOK = decode_ascii_imu(val, num, *pub_arr.imu);
				}
				if (!isOK && num >= 14 && strstr(val[0], "APINS") != NULL)
				{
					isOK = decode_ascii_ins(val, *pub_arr.ins);
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