/********************************************************************************
 * File Name:   message_publisher.cpp
 * Description: contains functions for publishing messages to ROS topics.
 *
 * Author:      Austin Johnson
 * Date:        7/1/23
 *
 * License:     MIT License
 *
 * Note:        The message_publisher.h file contains the definitions for the
 * 			   	message arrays.
 ********************************************************************************/

#include "message_publisher.h"
#include "main_anello_ros_driver.h"
#include "health_message.h"
#include "bit_tools.h"

#include <string.h>
#include <ctime>

#if COMPILE_WITH_ROS
#include <anello_ros_driver/APIMU.h>
#include <anello_ros_driver/APIM1.h>
#include <anello_ros_driver/APGPS.h>
#include <anello_ros_driver/APINS.h>
#include <anello_ros_driver/APHDG.h>
#include <anello_ros_driver/APHEALTH.h>

#include <nmea_msgs/Sentence.h>
#include <mavros_msgs/RTCM.h>
#include <std_msgs/Header.h>

#define SAMPLE_GGA_MESSAGE "$GNGGA,180921.50,3723.94984,N,12158.74997,W,5,13,2.64,13.00,M,,M,,*4A\r\n"

uint32_t gga_frame_id = 0;
void publish_gga(double *gps, ros::Publisher pub)
{
	std_msgs::Header msg_header;
	nmea_msgs::Sentence gga_message;

	msg_header.seq = gga_frame_id;
	msg_header.stamp = ros::Time::now();
	msg_header.frame_id = "anello gps data";
	gga_frame_id++;

	std::ostringstream gngga_message;

	gngga_message << "$GNGGA,";	//message header

	double gps_seconds = gps[1] * 1e-9;
	
	time_t utc_time_s = (gps_seconds) - 18.0; //subtract 18 leap seconds
	struct tm *utc_info = std::gmtime(&utc_time_s);

/*************************UTC Time*************************/
	int utc_hours = utc_info->tm_hour;
	int utc_minutes = utc_info->tm_min;
	int utc_seconds = utc_info->tm_sec;
	int utc_milliseconds = floor( (gps_seconds - floor(gps_seconds)) * 1e3);

	gngga_message << std::setw(2) << std::setfill('0') << utc_hours
					<< std::setw(2) << std::setfill('0') << utc_minutes
					<< std::setw(2) << std::setfill('0') << utc_seconds
					<< "."
					<< std::setw(3) << std::setfill('0') << utc_milliseconds << ",";


/*************************Lat Deg*************************/
	double lat_float = gps[2];
	char lat_hemisphere = (lat_float >= 0) ? 'N' : 'S';
	int lat_deg = floor(abs(lat_float));
	double lat_min = 60 * abs(abs(lat_float) - abs(lat_deg));
	int lat_min_int = floor(lat_min);
	int lat_min_dec = floor((lat_min - lat_min_int) * 1e7);
	
	gngga_message << std::setw(2) << std::setfill('0') << lat_deg
					<< std::setw(2) << std::setfill('0') << lat_min_int
					<< "."
					<< std::setw(7) << std::setfill('0') << lat_min_dec
					<< ",";
	
	gngga_message << lat_hemisphere << ",";


/*************************Lon Deg*************************/
	double lon_float = gps[3];
	char lon_hemisphere = (lon_float >= 0) ? 'E' : 'W';
	int lon_deg = floor(abs(lon_float));
	double lon_min = 60 * abs(abs(lon_float) - abs(lon_deg));
	int lon_min_int = floor(lon_min);
	int lon_min_dec = floor((lon_min - lon_min_int) * 1e7);

	gngga_message << std::setw(3) << std::setfill('0') << lon_deg
					<< std::setw(2) << std::setfill('0') << lon_min_int
					<< "."
					<< std::setw(7) << std::setfill('0') << lon_min_dec
					<< ",";

	gngga_message << lon_hemisphere << ",";

/*************************Fix Quality*************************/	
	int value_map[3] = {1, 5, 4}; // index is the anello fix type, value is the NMEA fix type
	int rtk_fix_quality = (int)gps[15];

	gngga_message << value_map[rtk_fix_quality] << ",";

/*************************Satellites*************************/
	int sat_num = (int)gps[12];
	gngga_message << sat_num << ",";

/*************************HDOP*************************/
	double pdop = gps[10];
	gngga_message << std::setw(4) << std::setfill('0') << pdop << ",";

/*************************Altitude*************************/
	double alt_msl = gps[5];
	
	gngga_message << alt_msl << ",M,";

/*************************Geoid Separation*************************/
	gngga_message << ",M,";

/*************************Age of Diff. Corr.*************************/
	gngga_message << ",";

/*************************Diff. Ref. Station ID*************************/
	// No comma for the final field
	gngga_message << "";

/*************************Checksum*************************/
	std::string ck = compute_checksum(gngga_message.str().c_str() + 1, gngga_message.str().length() - 1);
	gngga_message << "*" << ck << "\r\n";

#if DEBUG_PUBLISHERS
	ROS_INFO("GGA Message: %s",gngga_message.str().c_str());
#endif

	gga_message.header = msg_header;
	gga_message.sentence = gngga_message.str();

	pub.publish(gga_message);
}

void publish_gps(double *gps, ros::Publisher pub)
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
#if DEBUG_PUBLISHERS
	ROS_INFO("APGPS,%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", gps[0], gps[1], gps[2], gps[3], gps[4], gps[5], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15]);
#endif
}

void publish_gp2(double *gp2, ros::Publisher pub)
{
	/*
	 * gp2[0] = MCU_Time [ms]
	 * gp2[1] = GPS_Time [ns]
	 * gp2[2] = Latitude [deg]
	 * gp2[3] = Longitude [deg]
	 * gp2[4] = Alt_ellipsoid [m]
	 * gp2[5] = Alt_msl [m]
	 * gp2[6] = Speed [m/s]
	 * gp2[7] = heading [deg]
	 * gp2[8] = Hacc [m]
	 * gp2[9] = Vacc [m]
	 * gp2[10] = PDOP
	 * gp2[11] = FixType (0=No Fix, 2=2D Fix, 3=3D Fix, 5=Time only)
	 * gp2[12] = SatNum
	 * gp2[13] = Speed Accuracy [m/s]
	 * gp2[14] = Heading Accuracy [deg]
	 * gp2[15] = RTK Fix Status (0=SPP, 1=RTK Float, 2=RTK Fix)
	 *
	 */

	anello_ros_driver::APGPS msg;

	msg.mcu_time = gp2[0];
	msg.gps_time = gp2[1];

	msg.lat = gp2[2];
	msg.lon = gp2[3];
	msg.alt_ellipsoid = gp2[4];
	msg.alt_msl = gp2[5];

	msg.speed = gp2[6];
	msg.heading = gp2[7];

	msg.hacc = gp2[8];
	msg.vacc = gp2[9];

	msg.pdop = gp2[10];
	msg.fix_type = (uint8_t)gp2[11];
	msg.sat_num = (uint8_t)gp2[12];

	msg.speed_accuracy = gp2[13];
	msg.heading_accuracy = gp2[14];

	msg.rtk_fix_status = (uint8_t)gp2[15];

	pub.publish(msg);
#if DEBUG_PUBLISHERS
	ROS_INFO("APGP2,%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", gps[0], gps[1], gps[2], gps[3], gps[4], gps[5], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15]);
#endif
}

void publish_hdr(double *hdr, ros::Publisher pub)
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

#if DEBUG_PUBLISHERS
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
		(status & (1 << 9)) > 0);
#endif
}

double last_imu_mcu_time = -1.0;
void publish_imu(double *imu, ros::Publisher pub)
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
	 * imu[11] = T_Sync [ms]
	 */

	anello_ros_driver::APIMU msg;
	msg.mcu_time = imu[0];
	msg.ax = imu[1];
	msg.ay = imu[2];
	msg.az = imu[3];
	msg.wx = imu[4];
	msg.wy = imu[5];
	msg.wz = imu[6];
	msg.wz_fog = imu[7];
#if OLD_MESSAGING
	msg.odometer_speed = imu[8];
	msg.odometer_time = imu[9];
#else
	msg.odo_speed = imu[8];
	msg.odo_time = imu[9];
	msg.T_Sync = imu[11];
#endif
	msg.temp = imu[10];

	pub.publish(msg);

#if DEBUG_PUBLISHERS
	ROS_INFO("APIMU,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6], imu[7], imu[8], imu[9], imu[10], imu[11]);
#endif
}

void publish_im1(double *im1, ros::Publisher pub)
{
	/*
	 * im1[0] = MCU_Time [ms]
	 * im1[1] = ax [g]
	 * im1[2] = ay [g]
	 * im1[3] = az [g]
	 * im1[4] = wx [Deg/s]
	 * im1[5] = wy [Deg/s]
	 * im1[6] = wz [Deg/s]
	 * im1[7] = wz_fog [Deg/s]
	 * im1[8] = Temp [C]
	 * im1[9] = T_Sync [ms]
	 */

	anello_ros_driver::APIM1 msg;
	msg.mcu_time = im1[0];
	msg.ax = im1[1];
	msg.ay = im1[2];
	msg.az = im1[3];
	msg.wx = im1[4];
	msg.wy = im1[5];
	msg.wz = im1[6];
	msg.wz_fog = im1[7];
	msg.temp = im1[8];
	msg.T_Sync = im1[9];

	pub.publish(msg);

#if DEBUG_PUBLISHERS
	ROS_INFO("APIM1,%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", im1[0], im1[1], im1[2], im1[3], im1[4], im1[5], im1[6], im1[7], im1[8], im1[9]);
#endif
}

void publish_ins(double *ins, ros::Publisher pub)
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

#if DEBUG_PUBLISHERS
	ROS_INFO("APINS,%10.3f,%14.7f,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", ins[0], ins[1], ins[2], ins[3], ins[4], ins[5], ins[6], ins[7], ins[8], ins[9], ins[10], ins[11], ins[12]);
#endif
}

void publish_health(const health_message *health_msg, ros::Publisher pub)
{
	// Publish the health message
	anello_ros_driver::APHEALTH msg;
	msg.position_acc_flag = health_msg->get_position_status();
	msg.heading_health_flag = health_msg->get_heading_status();
	msg.gyro_health_flag = health_msg->get_gyro_status();

	pub.publish(msg);
}


#endif