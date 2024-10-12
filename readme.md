# ANELLO ROS Driver

## ANELLO Products Supported

The ANELLO ROS driver supports the ANELLO EVK, GNSS INS, and IMU+ running firmware versions >= v1.1.1.

## ANELLO ROS Driver Installation

### Install ROS

Follow the instructions on the [ROS2 Wiki](https://docs.ros.org/en/humble/Installation.html) to install ROS2 on your system.

### Install ANELLO ROS Driver

Clone this repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Anello-Photonics/ANELLO_ROS_Driver.git
```

## ANELLO ROS Driver Configuration

### Serial Communications

The ANELLO ROS driver defaults serial communications baud rate to 230400.
If you are using the ANELLO EVK or have changed the baud rate in the GNSS INS or IMU+, you will need to update the ```BAUDRATE``` in "anello_ros_driver/anello_node/src/anello_ros_driver/comm/serial_interface.h".

```c++
#ifndef BAUDRATE
#define BAUDRATE B230400    //Default baudrate for ANELLO GNSS INS and IMU+
// #define BAUDRATE B921600    //Default baudrate for ANELLO EVK
#endif
```

To set up serial communications:

1. Ensure that the ```anello_com_type``` arg is set to ```UART```.
2. Leave the UART port name configurations on ```AUTO``` to use the automatic connection procedure, or manually set the path (```/dev/ttyUSB0```).

Launch file sample:

```xml
    <!-- Set the COM_TYPE to UART or ETH -->
    <arg name="anello_com_type" default="UART"/>

    <!-- Uart port names -->
    <arg name="anello_uart_data_port" default="AUTO"/>
    <arg name="anello_uart_config_port" default="AUTO"/>
```

### Ethernet Communications

To set up ethernet communications:

1. Ensure that the ```anello_com_type``` arg is set to ```ETH```.
2. Set the IP address of the ANELLO device in the ```anello_remote_ip``` arg.
3. Ensure that the IP address of the system running the ROS driver is set in the ANELLO unit's "Computer IP" configuration.
4. Set the data port, config port, and odometer port in the launch file. Ensure these match the ANELLO unit configuration.

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

### Setting Up NTRIP client

Update the NTRIP client parameters in the launch file to point to your NTRIP caster:

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

### Build Code

```bash
cd ~/ros2_ws
colcon build
```

## ANELLO ROS Driver Usage

### Launch ANELLO ROS Driver

```bash
ros2 launch anello_ros_driver anello_ros_driver.launch
```

ANELLO messages will be published to topics (see below). For information on how to view these messages, see the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics).

### Topics

#### Published Topics

Topic definitions are defined in the [ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/) by the ASCII message format. Custom message definitions are used for ANELLO messages and can be found in the `/anello_ros_driver/anello_node/msg` directory.

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
      * 2 = Poor GPS accuracy (> 1m)
    * `heading_health_flag`
      * 0 = Heading stability confirmed (GPS and INS headings match)
      * 1 = Unable to confirm heading stability (may be due to poor GPS signal or driving too slow to get reliable GPS heading)
    * `gyro_health_flag`
      * 0 = Rate sensor stable
      * 1 = Rate sensor unstable (large discrepancy between optical and MEMS gyros)
* `/ntrip_client/nmea`

#### Subscribed Topics

Use the custom message and topic below to send odometer speed to the ANELLO unit.

* `/APODO`
  * Use the custom message definition found in `/anello_ros_driver/msg/APODO.msg`
* `/ntrip_client/rtcm_msg`

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