/********************************************************************************
 * File Name:   main_anello_ros_driver.cpp
 * Description: This file contains the main loop for the anello_ros_driver node.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        Node entry point is ros_driver_main_loop().
 *
 * 				If a serial interface is not being used, the main loop can be modified
 * 				to read from a another interface with a new object that inherits from
 * 				base_interface.
 *
 * 				Serial interface is hardcoded to /dev/ttyUSB0. This can be changed in
 * 				serial interface constructor parameter.
 ********************************************************************************/

#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>
#include <csignal>
#include "main_anello_ros_driver.h"
#include "message_subscriber.h"
#include "health_message.h"

#if COMPILE_WITH_ROS
#include <ros/ros.h>
#include "anello_ros_driver/APIMU.h"
#include "anello_ros_driver/APIM1.h"
#include "anello_ros_driver/APINS.h"
#include "anello_ros_driver/APGPS.h"
#include "anello_ros_driver/APHDG.h"
#include "anello_ros_driver/APODO.h"
#include "anello_ros_driver/APHEALTH.h"
#include "nmea_msgs/Sentence.h"
#endif

#include "bit_tools.h"
#include "serial_interface.h"
#include "rtcm_decoder.h"
#include "ascii_decoder.h"
#include "message_publisher.h"
#include "ntrip_buffer.h"
// extern NTRIP_buffer global_ntrip_buffer;

// UPDATE THIS VARIABLE TO CHANGE SERIAL PORT
// DEFAULT_DATA_INTERFACE found in anello_ros_driver/include/anello_ros_driver/serial_interface.h
//
// Example:
// const char *serial_port_name = "/dev/ttyUSB0";
const char *serial_port_name = DEFAULT_DATA_INTERFACE;

#ifndef NO_GGA
#define NO_GGA
#endif

#ifndef NODE_NAME
#define NODE_NAME "anello_ros_driver"
#endif

#ifndef COM_TYPE_NAME 
#define COM_TYPE_NAME "/anello_com_type"
#endif

#ifndef DATA_PORT_NAME
#define DATA_PORT_NAME "/anello_uart_data_port"
#endif

#ifndef CONFIG_PORT_NAME
#define CONFIG_PORT_NAME "/anello_uart_config_port"
#endif

#ifndef REMOTE_IP_NAME
#define REMOTE_IP_NAME "/anello_remote_ip"
#endif

#ifndef LOCAL_DATA_PORT_NAME
#define LOCAL_DATA_PORT_NAME "/anello_local_data_port"
#endif

#ifndef LOCAL_CONFIG_PORT_NAME
#define LOCAL_CONFIG_PORT_NAME "/anello_local_config_port"
#endif

#ifndef LOCAL_ODOMETER_PORT_NAME
#define LOCAL_ODOMETER_PORT_NAME "/anello_local_odometer_port"
#endif

#ifndef COM_TYPE_PARAMETER_NAME
#define COM_TYPE_PARAMETER_NAME NODE_NAME COM_TYPE_NAME
#endif

#ifndef DATA_PORT_PARAMETER_NAME
#define DATA_PORT_PARAMETER_NAME NODE_NAME DATA_PORT_NAME
#endif

#ifndef CONFIG_PORT_PARAMETER_NAME
#define CONFIG_PORT_PARAMETER_NAME NODE_NAME CONFIG_PORT_NAME
#endif

#ifndef REMOTE_IP_PARAMETER_NAME
#define REMOTE_IP_PARAMETER_NAME NODE_NAME REMOTE_IP_NAME
#endif

#ifndef LOCAL_DATA_PORT_PARAMETER_NAME
#define LOCAL_DATA_PORT_PARAMETER_NAME NODE_NAME LOCAL_DATA_PORT_NAME
#endif

#ifndef LOCAL_CONFIG_PORT_PARAMETER_NAME
#define LOCAL_CONFIG_PORT_PARAMETER_NAME NODE_NAME LOCAL_CONFIG_PORT_NAME
#endif

#ifndef LOCAL_ODOMETER_PORT_PARAMETER_NAME
#define LOCAL_ODOMETER_PORT_PARAMETER_NAME NODE_NAME LOCAL_ODOMETER_PORT_NAME
#endif

#ifndef LOG_LATEST_SET
#define LOG_LATEST_SET 0
#endif

#ifndef LOG_FILE_NAME
#define LOG_FILE_NAME "latest_anello_log.txt"
#endif

volatile sig_atomic_t sigint_received = 0;

void sigint_handler(int sig)
{
	sigint_received = 1;
}

anello_config_port *gp_global_config_port;

using namespace std;

typedef struct
{
	int n_used;				// how many bytes have been put through the decoded
	int nbytes;				// how many bytes are stored in the buffer
	char buff[MAX_BUF_LEN]; // buffer from file
} file_read_buf_t;

static int input_a1_data(a1buff_t *a1, uint8_t data, FILE *log_file);
static void ros_driver_main_loop();

int main(int argc, char *argv[])
{
#if COMPILE_WITH_ROS
	ros::init(argc, argv, NODE_NAME);
	ROS_INFO("Anello ROS Driver Started\n");
#endif
	ros_driver_main_loop();
}

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
static int input_a1_data(a1buff_t *a1, uint8_t data, FILE *log_file)
{
	if (nullptr != log_file)
	{
		fprintf(log_file, "%c", data);
	}

	int ret = 0, i = 0;

	// if length is at maximum reset buffer
	if (a1->nbyte >= MAX_BUF_LEN)
		a1->nbyte = 0;

	// Detect correct start characters for message
	/* #AP, 0xD3 */
	if (a1->nbyte == 0 && !(data == '#' || data == 0xD3))
	{
		a1->nbyte = 0;
		return 0;
	}
	if (a1->nbyte == 1 && !((data == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3))
	{
		a1->nbyte = 0;
		return 0;
	}
	if (a1->nbyte == 2 && !((data == 'P' && a1->buf[1] == 'A' && a1->buf[0] == '#') || a1->buf[0] == 0xD3))
	{
		a1->nbyte = 0;
		return 0;
	}

	// zero buffer when correct start char is detected before adding
	if (a1->nbyte == 0)
	{
		memset(a1, 0, sizeof(a1buff_t));
	}

	// add start characters to buffer
	if (a1->nbyte < 3)
	{
		a1->buf[a1->nbyte++] = data;
		return 0;
	}

	// if not RTCM message
	if (a1->buf[0] != 0xD3)
	{
		// remember index of each delimiter to mark sections
		if (data == ',')
		{
			a1->loc[a1->nseg++] = a1->nbyte;
			if (a1->nseg == 2)
			{
				a1->nlen = 0;
			}
		}

		// add byte to buffer
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
					a1->loc[a1->nseg++] = a1->nbyte - 4; // mark checksum value as seperator
					ret = 1;							 // mark ready for read
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


#if COMPILE_WITH_ROS
static void get_interface_config(ros::NodeHandle *nh, interface_config_t *config)
{
#else
static void get_interface_config(interface_config_t *config)
{
#endif
#if COMPILE_WITH_ROS
	string config_type;
	
	if (!nh->getParam(COM_TYPE_PARAMETER_NAME, config_type))
	{
		ROS_ERROR("Failed to get anello_com_type from parameter server");
		exit(1);
	}

	// UART specific parameters
	if (!nh->getParam(DATA_PORT_PARAMETER_NAME, config->data_port_name))
	{
		ROS_ERROR("Failed to get uart data port name from parameter server");
		exit(1);
	}

	if (!nh->getParam(CONFIG_PORT_PARAMETER_NAME, config->config_port_name))
	{
		ROS_ERROR("Failed to get config port name from parameter server");
		exit(1);
	}


	// ETH specific parameters
	if (!nh->getParam(REMOTE_IP_PARAMETER_NAME, config->remote_ip))
	{
		ROS_ERROR("Failed to get remote ip from parameter server");
		exit(1);
	}

	if (!nh->getParam(LOCAL_DATA_PORT_PARAMETER_NAME, config->local_data_port))
	{
		ROS_ERROR("Failed to get local data port from parameter server");
		exit(1);
	}

	if (!nh->getParam(LOCAL_CONFIG_PORT_PARAMETER_NAME, config->local_config_port))
	{
		ROS_ERROR("Failed to get local config port from parameter server");
		exit(1);
	}

	if (!nh->getParam(LOCAL_ODOMETER_PORT_PARAMETER_NAME, config->local_odometer_port))
	{
		ROS_ERROR("Failed to get local odometer port from parameter server");
		exit(1);
	}

	if ("ETH" == config_type)
	{
		config->type = ETH;
	}
	else if ("UART" == config_type)
	{
		config->type = UART;
	}
	else
	{
		ROS_ERROR("Invalid interface type");
		config->type = UART;
	}

#else
	config->type = ETH;
	config->data_port_name = "/dev/ttyUSB0";
	config->config_port_name = "/dev/ttyUSB3";
	config->remote_ip = "192.168.1.111";
	config->local_data_port = 1111;
	config->local_config_port = 2222;
	config->local_odometer_port = 3333;
#endif
}



/*
 * Main loop for the driver
 * Handles calling all of the modules in the program.
 *
 * Includes
 * Calling the device interface
 * Maintaining a buffer of input bytes
 * Calling correct decoder for each message
 */
static void ros_driver_main_loop()
{
	// init signal handler
	signal(SIGINT, sigint_handler);

	// init ros publishers
#if COMPILE_WITH_ROS
	ros::NodeHandle nh;

	ros::Publisher pub_imu = nh.advertise<anello_ros_driver::APIMU>("APIMU", 10);
	ros::Publisher pub_im1 = nh.advertise<anello_ros_driver::APIM1>("APIM1", 10);
	ros::Publisher pub_ins = nh.advertise<anello_ros_driver::APINS>("APINS", 10);
	ros::Publisher pub_gps = nh.advertise<anello_ros_driver::APGPS>("APGPS", 10);
	ros::Publisher pub_gp2 = nh.advertise<anello_ros_driver::APGPS>("APGP2", 10);
	ros::Publisher pub_hdg = nh.advertise<anello_ros_driver::APHDG>("APHDG", 10);
	ros::Publisher pub_health = nh.advertise<anello_ros_driver::APHEALTH>("APHEALTH", 1);
	ros::Publisher pub_gga = nh.advertise<nmea_msgs::Sentence>("ntrip_client/nmea", 1);

	ros::Subscriber sub_rtcm = nh.subscribe("ntrip_client/rtcm", 1, ntrip_rtcm_callback);
	ros::Subscriber sub_odo = nh.subscribe("APODO", 1, apodo_callback);

#if APINI_UPD
	ros::ServiceServer srv_init_heading = nh.advertiseService("ini_heading", init_heading_callback);
	ros::ServiceServer srv_upd_heading = nh.advertiseService("upd_heading", upd_heading_callback);
#endif

	
	ROS_DEBUG("Anello ROS Driver Started\n");


	ros_publishers_t pub_arr;
	pub_arr.imu = &pub_imu;
	pub_arr.ins = &pub_ins;
	pub_arr.gps = &pub_gps;
	pub_arr.gp2 = &pub_gp2;
	pub_arr.hdg = &pub_hdg;
	
	const char *ntrip_data;
#endif

	// init health message class
	health_message health_msg;
	uint32_t INS_message_count = 0;

	FILE *log_file_fp = nullptr;
	if (LOG_LATEST_SET)
	{
		// open log file
		log_file_fp = fopen(LOG_FILE_NAME, "w");

		// check file was open correctly
		if (log_file_fp == NULL)
		{
#if COMPILE_WITH_ROS
			ROS_ERROR("Failed to open log file");
#else
			printf("Failed to open log file\n");
#endif
			exit(1);
		}
	}

	// init buffer
	file_read_buf_t serial_read_buf = {0};

	// MESSAGE PARSING VARIABLES
	char *val[MAXFIELD];
	double decoded_val[MAXFIELD];
	bool checksum_passed;
	a1buff_t a1buff = {0};


	// COMMUNICATION VARIABLES
	interface_config_t interface_config;

#if COMPILE_WITH_ROS
	get_interface_config(&nh, &interface_config);
#else
	get_interface_config(&interface_config);
#endif

	// DEBUG_PRINT("Interface Type: %d", interface_config.type);

	anello_config_port anello_device_config(&interface_config);
	// DEBUG_PRINT("Config post declared");
	anello_device_config.init();
	// DEBUG_PRINT("Config post initialized");
	gp_global_config_port = &anello_device_config;

	// DEBUG_PRINT("Config post set");
	anello_data_port anello_device_data(&interface_config);
	// DEBUG_PRINT("Data post declared");
	anello_device_data.init();
	// DEBUG_PRINT("Data post initialized");

	health_msg.set_baseline(anello_device_config.get_baseline());
#if DEBUG_MAIN
#if COMPILE_WITH_ROS
	ROS_INFO("Anello ROS Driver Started\n");
	ROS_INFO("Data Port: %s", anello_device_data.get_portname().c_str());
#else
	printf("Anello ROS Driver Started\n");
	printf("Data Port: %s", anello_device_data.get_portname().c_str());
#endif	

#if COMPILE_WITH_ROS
	ROS_INFO("Config Port: %s", anello_device_config.get_portname().c_str());
#else
	printf("Config Port: %s", anello_device_config.get_portname().c_str());
#endif

#endif

#if COMPILE_WITH_ROS
    while (ros::ok() && !sigint_received)
	{
		// allow ROS to process callbacks
		ros::spinOnce();
#else
	while (!sigint_received)
	{
#endif
		// when data is available write it to the serial port
		if (global_ntrip_buffer.is_read_ready())
		{
			anello_device_data.write_data((char *)global_ntrip_buffer.get_buffer(), global_ntrip_buffer.get_buffer_length());
			global_ntrip_buffer.set_read_ready_false();
		}

		if (global_config_buffer.is_read_ready())
		{
			anello_device_config.write_data((char *)global_config_buffer.get_buffer(), global_config_buffer.get_buffer_length());
			global_config_buffer.set_read_ready_false();
		}

		// if all data has been read... read more data into buff
		if (serial_read_buf.n_used >= serial_read_buf.nbytes)
		{
			serial_read_buf.nbytes = anello_device_data.get_data(serial_read_buf.buff, MAX_BUF_LEN);
			serial_read_buf.n_used = 0;
		}
		else
		{

			// add next byte to message buffer
			int ret = input_a1_data(&a1buff, serial_read_buf.buff[serial_read_buf.n_used], log_file_fp);
			serial_read_buf.n_used++;

			// if message complete
			if (ret)
			{
				int isOK = 0;
				int num = 0;

				// if ascii
				if (ret == 1)
				{
					// check that the checksum is correct
					checksum_passed = checksum(a1buff.buf, a1buff.nbyte);
					if (checksum_passed)
					{
						num = parse_fields((char *)a1buff.buf, val);
					}
					else
					{
#if COMPILE_WITH_ROS
						ROS_WARN("Checksum Fail: %s", a1buff.buf);
#else
						printf("Checksum Fail: %s", a1buff.buf);
#endif
						num = 0;
					}

					if (!isOK && num >= 17 && strstr(val[0], "APGPS") != NULL)
					{
						// ascii gps
						decode_ascii_gps(val, decoded_val);
						health_msg.add_gps_message(decoded_val);
#if COMPILE_WITH_ROS
						publish_gps(decoded_val, pub_gps);
						publish_gga(decoded_val, pub_gga);
#else
						printf("APGPSa\n");
#endif

						isOK = 1;
					}
					else if (!isOK && num >= 17 && strstr(val[0], "APGP2") != NULL)
					{
						// ascii gp2 (goes to the same place for now)
						decode_ascii_gps(val, decoded_val);
#if COMPILE_WITH_ROS
						publish_gp2(decoded_val, pub_gp2);
#else
						printf("APGP2a\n");
#endif

						isOK = 1;
					}
					else if (!isOK && num >= 12 && strstr(val[0], "APHDG") != NULL)
					{
						// ascii hdg
						decode_ascii_hdr(val, decoded_val);
						health_msg.add_hdg_message(decoded_val);
#if COMPILE_WITH_ROS
						publish_hdr(decoded_val, pub_hdg);
#else
						printf("APHDGa\n");
#endif

						isOK = 1;
					}
					else if (!isOK && num >= 12 && strstr(val[0], "APIMU") != NULL)
					{
						// ascii imu
						decode_ascii_imu(val, num, decoded_val);
						health_msg.add_imu_message(decoded_val);
#if COMPILE_WITH_ROS
						publish_imu(decoded_val, pub_imu);
#else
						printf("APIMUa\n");
#endif

						isOK = 1;
					}
					else if (!isOK && num >= 10 && strstr(val[0], "APIM1") != NULL)
					{
						// ascii imu
						decode_ascii_im1(val, num, decoded_val);
#if COMPILE_WITH_ROS
						publish_im1(decoded_val, pub_im1);
#else
						printf("APIM1a\n");
#endif

						isOK = 1;
					}
					else if (!isOK && num >= 14 && strstr(val[0], "APINS") != NULL)
					{
						// ascii ins
						decode_ascii_ins(val, decoded_val);
						health_msg.add_ins_message(decoded_val);
						INS_message_count++;
#if COMPILE_WITH_ROS
						publish_ins(decoded_val, pub_ins);
#else
						printf("APINSa\n");
#endif

						if (INS_message_count > 100)
						{
#if COMPILE_WITH_ROS
							publish_health(&health_msg, pub_health);
							INS_message_count = 0;
#endif
						}
						isOK = 1;
					}
					else if (!isOK && ((strstr(val[0], "APINI") != NULL ) || (strstr(val[0], "APUPD") != NULL)))
					{
						isOK = 1;
					}
				}
				// if rtcm
				else if (ret == 5) /* rtcm */
				{
					// isOK = decode_rtcm_message(a1buff, pub_arr);
					if (a1buff.type == 4058 && !a1buff.crc)
					{
						if (a1buff.subtype == 1) /* IMU */
						{
							decode_rtcm_imu_msg(decoded_val, a1buff);
							health_msg.add_imu_message(decoded_val);
#if COMPILE_WITH_ROS
							publish_imu(decoded_val, pub_imu);
#else
							printf("APIMUr\n");
#endif

							isOK = 1;
						}
						else if (a1buff.subtype == 2) /* GPS PVT */
						{
							int ant_id = decode_rtcm_gps_msg(decoded_val, a1buff);

							// only add gps1 to health message
							if (GPS1 == ant_id)
							{
								health_msg.add_gps_message(decoded_val);
#if COMPILE_WITH_ROS
								publish_gps(decoded_val, pub_gps);
								publish_gga(decoded_val, pub_gga);
#else
							printf("APGPSr\n");
#endif
							}
							else
							{
#if COMPILE_WITH_ROS
								publish_gp2(decoded_val, pub_gp2);
#else
								printf("APGP2r\n");
#endif
							}

							isOK = 1;
						}
						else if (a1buff.subtype == 3) /* DUAL ANTENNA */
						{
							decode_rtcm_hdg_msg(decoded_val, a1buff);
							health_msg.add_hdg_message(decoded_val);
#if COMPILE_WITH_ROS
							publish_hdr(decoded_val, pub_hdg);
#else
							printf("APHDGr\n");
#endif

							isOK = 1;
						}
						else if (a1buff.subtype == 4) /* INS */
						{
							decode_rtcm_ins_msg(decoded_val, a1buff);
							health_msg.add_ins_message(decoded_val);
							INS_message_count++;
#if COMPILE_WITH_ROS
							publish_ins(decoded_val, pub_ins);
#else
							printf("APINSr\n");
#endif

							if (INS_message_count > 100)
							{
#if COMPILE_WITH_ROS
								publish_health(&health_msg, pub_health);
								INS_message_count = 0;
#endif
							}

							isOK = 1;
						}
						else if (a1buff.subtype == 6) /* IM1 */
						{
							decode_rtcm_im1_msg(decoded_val, a1buff);
#if COMPILE_WITH_ROS
							publish_im1(decoded_val, pub_im1);
#else
							printf("APIM1r\n");
#endif

							isOK = 1;
						}
					}
				}

				// message does not fit into parameter
				if (!isOK)
				{
					printf("%s\n", a1buff.buf);
					anello_device_data.port_parse_fail();
				}
				else
				{
					memset(decoded_val, 0, MAXFIELD * sizeof(double));
					anello_device_data.port_confirm();
#if DEBUG_MAIN
					printf("%s\n", a1buff.buf);
#endif
				}
				a1buff.nbyte = 0;
			}
		}
	}

	anello_device_data.~anello_data_port();
	anello_device_config.~anello_config_port();

	if (LOG_LATEST_SET)
	{
		// close log file
		fclose(log_file_fp);
	}
}
