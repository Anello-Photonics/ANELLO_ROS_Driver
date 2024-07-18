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

Install rosdeps for the package

```bash
rosdep install --from-paths ~/catkin_ws/src --ignore-src -r -y
```

### Configure ANELLO ROS Driver

#### Required Actions for Communications

Product Type:

GNSS/INS: None

IMU+: None

EVK: Update baud rate to 921600 in "anello_ros_driver/include/serial_interface.h" if serial communication are being used.

```c++
#ifndef BAUDRATE
#define BAUDRATE B230400    //Default baudrate for anello GNSS/INS and IMU+
// #define BAUDRATE B921600    //Default baudrate for anello EVK
#endif
```

#### Setting up serial communication

1. Ensure that the ```anello_com_type``` arg is set to ```UART```.
2. Leave the UART port name configs on ```AUTO``` if you want to use the automatic connection procedure, or manually set the path to the virtual port (```/dev/ttyUSB0```)

Launch file sample:

```xml
    <!-- Set the COM_TYPE to UART or ETH -->
    <arg name="anello_com_type" default="UART"/>

    <!-- Uart port names -->
    <arg name="anello_uart_data_port" default="AUTO"/>
    <arg name="anello_uart_config_port" default="AUTO"/>
```

#### Setting up ethernet communication

1. Ensure that the ```anello_com_type``` arg is set to ```ETH```.
2. Set the IP address of the Anello device in the ```anello_remote_ip``` arg.
3. Ensure that the IP address of the system running the ROS driver is set in the Anello device corretly under the "Computer IP" configuration.
4. Set the ports for data, config, and odometer in the launch file. Make sure this matches the configuration in the Anello device.

Launch file sample:

```xml
    <!-- Set the COM_TYPE to UART or ETH -->
    <arg name="anello_com_type" default="ETH"/>
```

```xml
    <!-- Ethernet port names -->
    <arg name="anello_remote_ip" default="192.168.1.111"/>
    <arg name="anello_local_data_port" default="1111"/>
    <arg name="anello_local_config_port" default="2222"/>
    <arg name="anello_local_odometer_port" default="3333"/>
```

#### Setting up NTRIP client

Update the NTRIP client parametres in the launch file to point to your NTRIP caster:

```xml
    <!-- NTRIP CLIENT PARAMTERS -->
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
```

### Build the code

```bash
cd ~/catkin_ws
catkin_make
```

### Upgrading from older ros driver versions

If you are upgrading from an older version of the driver (<= v1.2.2), you may need to recompile the code to ensure that the new changes are applied. First, ensure that the instance of the ntrip_client package is deleted from the workspace. This node has been moved within the anello_ros_driver package.

```bash
rm -rf ~/catkin_ws/src/ntrip_client
```

update the driver code to the latest version:

```bash
cd ~/catkin_ws/src/ANELLO_ROS_Driver
git pull
```

Reset comilation:

```bash
cd ~/catkin_ws
rm -rf build/ devel/
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
* `/APGP2`
* `/APHDG`
* `/APHEALTH`
  * Message Definitions:
    * `position_acc_flag`
      * 0 = Centimeter-level accuracy
      * 1 = Meter-level accuracy
      * 2 = Poor GPS accuracy (< 1m)
    * `heading_health_flag`
      * 0 = Heading stability confirmed (GPS and INS headings match)
      * 1 = Unable to confirm heading stability (may be due to poor GPS signal or driving too slow to get reliable GPS heading)
    * `gyro_health_flag`
      * 0 = Rate sensor stable
      * 1 = Rate sensor unstable (large discrepancy between optical and MEMS gyros)
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
