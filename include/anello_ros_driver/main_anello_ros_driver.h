/********************************************************************************
 * File Name:   main_anello_ros_driver.h
 * Description: header file for the main_anello_ros_driver.cpp.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        Contains type definitions for the anello_ros_driver including
 * 				rtcm message types.
 ********************************************************************************/

#ifndef MAIN_ANELLO_ROS_DRIVER_H
#define MAIN_ANELLO_ROS_DRIVER_H

#ifndef COMPILE_WITH_ROS
#define COMPILE_WITH_ROS 1
#endif

#include <stdint.h>
#if COMPILE_WITH_ROS
#include <ros/ros.h>
#endif

#include "bit_tools.h"
#include "serial_interface.h"
#include "ethernet_interface.h"
#include "anello_config_port.h"
#include "anello_data_port.h"

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

#ifndef DEBUG_MAIN
#define DEBUG_MAIN 0
#endif

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 0
#endif

#ifndef DEBUG_ETHERNET
#define DEBUG_ETHERNET 0
#endif

#ifndef DEBUG_PUBLISHERS
#define DEBUG_PUBLISHERS 0
#endif

#ifndef DEBUG_SUBSCRIBERS
#define DEBUG_SUBSCRIBERS 0
#endif

#ifndef OLD_MESSAGING
#define OLD_MESSAGING 1
#endif

#ifndef DEBUG_PRING
#if COMPILE_WITH_ROS
#define DEBUG_PRINT(...) ROS_INFO(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#endif
#endif

#ifndef APINI_UPD
#define APINI_UPD 0
#endif

extern anello_config_port *gp_global_config_port;

typedef struct
{
	uint8_t buf[MAX_BUF_LEN]; /* buffer where the raw data is held */
	int nseg;				  /* number of segments in the message */
	int nbyte;				  /* number of bytes in the message */
	int nlen;				  /* length of binary message */
	int type;
	int subtype;
	int crc;
	int loc[MAXFIELD]; /* location of the start of the segments */
} a1buff_t;

typedef struct
{
	uint64_t MCU_Time;	//	UInt64	ns	    Time since power on
	uint64_t Sync_Time; // UInt64   ns   Timestamp of external sync pulse
	int32_t AX;			// Int32	15 g	X-axis accel
	int32_t AY;			// Int32	15 g	Y-axis accel
	int32_t AZ;			// Int32	15 g	Z-axis accel
	int32_t WX;			// Int32	450 dps	X-axis angular rate (MEMS)
	int32_t WY;			// Int32	450 dps	Y-axis angular rate (MEMS)
	int32_t WZ;			// Int32	450 dps	Z-axis angular rate (MEMS)
	int32_t OG_WZ;		// Int32	450 dps	High precision z-axis angular rate
	int16_t Temp_C;		// Int16	�C
} rtcm_apim1_t;

typedef struct
{
	uint64_t	Time;			//UInt64	ns
	uint64_t	Sync_Time;		//Uint64	ns
	int32_t		Roll;			//Int32		1e-5 deg
	int32_t		Pitch;			//Int32		1e-5 deg
	int32_t		Yaw;			//Int32		1e-5 deg
	uint8_t		Zupt_Status;	//Uint8		1=stationary, 0-not_stationary
} rtcm_apahrs_t;

typedef struct
{
	uint64_t MCU_Time;	//	UInt64	ns	    Time since power on
	uint64_t Sync_Time; // UInt64   ns   Timestamp of external sync pulse
	uint64_t ODO_time;	//	Int64	ns	    Timestamp of ODometer reading
	int32_t AX;			// Int32	15 g	X-axis accel
	int32_t AY;			// Int32	15 g	Y-axis accel
	int32_t AZ;			// Int32	15 g	Z-axis accel
	int32_t WX;			// Int32	450 dps	X-axis angular rate (MEMS)
	int32_t WY;			// Int32	450 dps	Y-axis angular rate (MEMS)
	int32_t WZ;			// Int32	450 dps	Z-axis angular rate (MEMS)
	int32_t OG_WZ;		// Int32	450 dps	High precision z-axis angular rate
	int16_t ODO;		// Int16	m/s	    Scaled composite odometer value
	int16_t Temp_C;		// Int16	�C
} rtcm_apimu_t;

/* old format do not have Sync_Time */
typedef struct
{
	uint64_t MCU_Time; //	UInt64	ns	    Time since power on
	uint64_t ODO_time; //	Int64	ns	    Timestamp of ODometer reading
	int32_t AX;		   // Int32	15 g	X-axis accel
	int32_t AY;		   // Int32	15 g	Y-axis accel
	int32_t AZ;		   // Int32	15 g	Z-axis accel
	int32_t WX;		   // Int32	450 dps	X-axis angular rate (MEMS)
	int32_t WY;		   // Int32	450 dps	Y-axis angular rate (MEMS)
	int32_t WZ;		   // Int32	450 dps	Z-axis angular rate (MEMS)
	int32_t OG_WZ;	   // Int32	450 dps	High precision z-axis angular rate
	int16_t ODO;	   // Int16	m/s	    Scaled composite odometer value
	int16_t Temp_C;	   // Int16	�C
} rtcm_old_apimu_t;

typedef struct
{
	uint64_t Time;		   // UInt64	ns	    Time since power on
	uint64_t GPS_Time;	   // UInt64	ns	    GPS time (GTOW)
	int32_t Latitude;	   // Int32	    1e-7 deg
	int32_t Longitude;	   // Int32	    1e-7 deg
	int32_t Alt_ellipsoid; // Int32	    0.001 m
	int32_t Alt_msl;	   // Int32	    0.001 m
	int32_t Speed;		   // Int32	    0.001 mps
	int32_t Heading;	   // Int32	    0.001 deg
	uint32_t Hor_Acc;	   // UInt32	0.001 m
	uint32_t Ver_Acc;	   // UInt32	0.001 m
	uint32_t Hdg_Acc;	   // UInt32	1e-5 deg
	uint32_t Spd_Acc;	   // UInt32	0.001 mps
	uint16_t PDOP;		   // UInt16	0.01
	uint8_t FixType;	   // UInt8
	uint8_t SatNum;		   // UInt8
	uint8_t RTK_Status;	   // UInt8
	uint8_t Antenna_ID;	   // Uint8	    Primary antenna or 2nd antenna
} rtcm_apgps_t;

typedef struct
{
	uint64_t MCU_Time; // UInt64	ns
	uint64_t GPS_Time; // UInt64	    ns

	int32_t relPosN; // Int32	    0.001 m
	int32_t relPosE; // Int32	    0.001 m
	int32_t resPosD; // Int32	    0.001 m

	int32_t relPosLength;  // Int32	    0.001 m
	int32_t relPosHeading; // Int32	    1e-5 deg

	uint32_t relPosLength_Accuracy;	 // UInt32	1e-5 deg
	uint32_t relPosHeading_Accuracy; // UInt32	1e-5 deg

	uint16_t statusFlags;
} rtcm_aphdr_t;

typedef struct
{
	uint64_t Time;		   // UInt64	ns
	uint64_t GPS_Time;	   // UInt64	ns
	int32_t Latitude;	   // Int32	    1.0e-7 deg
	int32_t Longitude;	   // Int32	    1.0e-7 deg
	int32_t Alt_ellipsoid; // Int32	    0.001 m
	int32_t Vn;			   // Int32	    0.001 mps
	int32_t Ve;			   // Int32	    0.001 mps
	int32_t Vd;			   // Int32	    0.001 mps
	int32_t Roll;		   // Int32	    1e-5 deg
	int32_t Pitch;		   // Int32	    1e-5 deg
	int32_t Heading_Yaw;   // Int32	    1e-5 deg
	uint8_t ZUPT;		   // UInt8	    1 � stationary, 0 - moving
	uint8_t Status;		   // UInt8	    See ASCII packet
} rtcm_apins_t;

typedef struct
{
	uint64_t Time;		   // UInt64	ns
	float covLatLat;
	float covLonLon;
	float covAltAlt;
	float covLatLon;
	float covLatAlt;
	float covLonAlt;
	float covVnVn;
	float covVeVe;
	float covVdVd;
	float covVnVe;
	float covVnVd;
	float covVeVd;
	float covRollRoll;
	float covPitchPitch;
	float covYawYaw;
	float covRollPitch;
	float covRollYaw;
	float covPitchYaw;
} rtcm_apcov_t;

#if COMPILE_WITH_ROS
typedef struct
{
	ros::Publisher *imu;
	ros::Publisher *ins;
	ros::Publisher *gps;
	ros::Publisher *gp2;
	ros::Publisher *hdg;
} ros_publishers_t;
#endif

#endif
