# ANELLO ROS Driver

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

Update the serial port values in the launch file to match the ports in your system:
if the value is AUTO the program will automatically connect that port.

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

Topic definitions are defined in the [ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/) by the ASCII message format.

* `/anello_ros_driver/APIMU.msg`
* `/anello_ros_driver/APINS.msg`
* `/anello_ros_driver/APGPS.msg`
* `/anello_ros_driver/APHDG.msg`

#### Subscribed Topics

* None

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
