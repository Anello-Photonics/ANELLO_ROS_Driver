/*
* Sample file from previous project
*
*DELETE THIS FILE ONCE FINISHED
*
*/



#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "odo_evk.h"
#include <math.h>

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

void Calculate_Checksum(char *, char *, char *);
int setup_serial(char *);


int makeOdoMsg (char *odoMsg, int len, float linVelocity)
{
    memset(odoMsg, 0, len);

    char cka = '-', ckb = '-';
    char sign = (linVelocity >= 0) ? '+' : '-';
    float magnitude = fabs(linVelocity);    

    int numChar = sprintf(odoMsg, "#APODO,%c,%f*xx\r\n",sign,magnitude); 
    
    Calculate_Checksum(odoMsg, &cka, &ckb);
    
    odoMsg[numChar-4] = cka;
    odoMsg[numChar-3] = ckb;

    return numChar;
}

void chatterCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    int msgSize = 50;
    char odoMsg[msgSize] = { 0 };

    //asign the serial name 
    char portName[25];
    strcpy(portName, "/dev/ttyUSB3");
   
    //load the next odo message
    int numChar = makeOdoMsg(odoMsg, msgSize, msg->twist.twist.linear.x);

#if DEBUG
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
#endif
    //write to port

    int portNum = setup_serial(portName);
    if (portNum > 0){
        write(portNum, odoMsg, sizeof(odoMsg));
#if DEBUG
        ROS_INFO("portnum: %d", portNum);
#endif
    } else 
        ROS_INFO("Fail:%d",portNum);

    close(portNum);

    ROS_INFO("%s", odoMsg);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "odo_evk");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("odom", 4, chatterCallback);

    ros::spin();

    return 0;
}




void Calculate_Checksum(char *iBuf, char *iCka, char *iCkb)
{
    // The checksum is calculated over the message, starting and including the class field up
    // until, but excluding, the checksum fields (see the figure UBX frame structure).  The
    // checksum values and the input are 8-bit unsigned integers.
    char *q, sum;
    char ck[10] = { 0 };

    for (q = (char *)iBuf+1, sum = 0; *q != '*' && *q; q++){
        sum ^= *q;
    }

    sprintf(ck, "%02X", sum);

    *iCka = ck[0];
    *iCkb = ck[1];
}
