#ifndef VERSION_H
#define VERSION_H

#ifndef MAJOR_VERSION
#define MAJOR_VERSION 1
#endif

#ifndef MINOR_VERSION
#define MINOR_VERSION 3
#endif

#ifndef PATCH_VERSION
#define PATCH_VERSION 9
#endif


/*
v1.3.9 : Added support for covariance output and commanding through services

v1.3.8 : Better AHRS set heading command

v1.3.7 : Added support for AHRS including ZUPT and heading setting

v1.3.6 : Removed extra comma from GNGGA NMEA message

v1.3.5 : Remove unnecessary debug messages

v1.3.4 : Add ethernet support along with NTRIP bug fixes

v1.3.3 : Add GP2 topic

v1.3.2 : Better version control of ntrip client

v1.3.1 : Gyro discrepancy fix

v1.3.0 : Update Version Number

v1.2.3 : Added health message topic

v1.2.2 : Added option for config port to be disabled if the parameter is set to "OFF"

v1.2.1 : Added support for dynamic gga message generation

v1.2.0 : Feature List:
            1. Added IMU+ support
            2. Update .msg files to include all available data
            3. Update default values to match IMU+ and GNSS INS defaults

v1.1.3 : Fixed serial port connection on boot

v1.1.0 : Feature List:
            1. Added odometer forwarding
            2. Added data-port and config port auto-connect
            3. Add launch file and parameter support

v1.0.0 : Initial basic feature set

v1.1.0 : Push changes to main

*/

#endif
