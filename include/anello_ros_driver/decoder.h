#ifndef DECODER_H
#define DECODER_H

#include <stdint.h>
#include <ros/ros.h>

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

#ifndef MAXFIELD
#define MAXFIELD 20
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
	int loc[MAXFIELD];			/* location of the start of the segments */
}a1buff_t;

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

typedef struct {
    ros::Publisher *imu;
    ros::Publisher *ins;
    ros::Publisher *gps;
    ros::Publisher *hdg;
} ros_publishers_t;

#endif