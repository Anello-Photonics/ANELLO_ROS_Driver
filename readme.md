# ANELLO ROS Driver

## Supported Firmware

Support FW Versions >= v1.1.1

## Installation

### Install ROS

Follow the instructions on the [ROS Wiki](http://wiki.ros.org/ROS/Installation) to install ROS on your system.

### Install ANELLO ROS Driver

Clone this repository into your catkin workspace and build it:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Anello-Photonics/ANELLO_ROS_Driver.git
```

### Install ANELLO ROS Driver Dependencies

Install the following dependencies:

#### ntrip_client Node

Follow the build instructions on the [ntrip_client github page](https://github.com/LORD-MicroStrain/ntrip_client)

### Configure ANELLO ROS Driver

#### Required Actions

Product Type:

GNSS/INS: None

IMU+: None

EVK: Update baud rate to 921600 in "anello_ros_driver/include/serial_interface.h"

```c++
#ifndef BAUDRATE
#define BAUDRATE B230400    //Default baudrate for anello GNSS/INS and IMU+
// #define BAUDRATE B921600    //Default baudrate for anello EVK
#endif
```

Update the serial port values in the launch file to match the ports in your system.
If the value is AUTO the program will automatically connect that port. The config port can be set to "OFF" if odometer correction are not being used.

```xml
    <!--Update data and config port values to the ports in your system-->
    <node name="anello_ros_driver" pkg="anello_ros_driver" type="anello_ros_driver">
        <param name="data_port" value="/dev/ttyUSB0"/>
        <param name="config_port" value="/dev/ttyUSB3"/>
    </node>
```

```xml
    <!--Update data and config port values to the ports in your system-->
    <node name="anello_ros_driver" pkg="anello_ros_driver" type="anello_ros_driver">
        <param name="data_port" value="AUTO"/>
        <param name="config_port" value="AUTO"/>
    </node>
```

Update the ntrip_client parameters in the launch file to match your system:

```xml
    <!--Launch ntrip client-->
    <include file="$(find ntrip_client)/launch/ntrip_client.launch">
        <arg name="host" value="127.0.0.1"/>
        <arg name="port" value="1111"/>
        <arg name="mountpoint" value="AUTO"/>
        <arg name="authenticate" value="true"/>
        <arg name="username" value=""/>
        <arg name="password" value=""/>
        <arg name="ntrip_version" value=""/>
        <arg name="ssl" value="false"/>
        <arg name="cert" value=""/>
        <arg name="key" value=""/>
        <arg name="ca_cert" value=""/>
        <arg name="debug" value="false"/>
        <arg name="rtcm_message_package" value="mavros_msgs"/>
    </include>
```

### Build the code

```bash
cd ~/catkin_ws
catkin_make
```

## Usage

### Launch ANELLO ROS Driver

```bash
roslaunch anello_ros_driver anello_ros_driver.launch
```

ANELLO messages will be published to topics (see below). For information on how to view these messages, see the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics).

### Topics

#### Published Topics

Topic definitions are defined in the [ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/) by the ASCII message format. Custom message definitions are used for Anello messages and can be found in the `/anello_ros_driver/msg` directory.

* `/APIMU`
* `/APIM1`
* `/APINS`
* `/APGPS`
* `/APHDG`
* `/APHEALTH`
  * Message Definitions:
    * `position_acc_flag`
      * 0 = CM accuracy
      * 1 = Submeter accuracy
      * 2 = Meter level accuracy
    * `heading_health_flag`
      * 0 = Confirmed good heading
      * 1 = Unable to confirm heading
    * `gyro_health_flag`
      * 0 = Gyro is operational
      * 1 = Gyro may have issue
* `/ntrip_client/nmea`

#### Subscribed Topics

Use the custom message and topic below to send odometer speed to the Anello Photonics unit.

* `/APODO`
  * Use the custom message definition found in `/anello_ros_driver/msg/APODO.msg`
* `/ntrip_client/rtcm_msg`

### Services

#### Provided Services

* None

#### Called Services

* None

## License

MIT License

```text
Copyright (c) 2023 ANELLO Photonics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

```

## Author Information

This driver was created in 2023 by [ANELLO Photonics](https://www.anellophotonics.com/).
