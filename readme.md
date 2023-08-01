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

### Configure ANELLO ROS Driver

Update the code to use the correct serial port (default is `/dev/ttyUSB0`):

The serial port is defined in `anello_ros_driver/src/main_anello_ros_driver.cpp` as a global variable at the top of the file:

```c++
// UPDATE THIS VARIABLE TO CHANGE SERIAL PORT
// DEFAULT_DATA_INTERFACE found in anello_ros_driver/include/anello_ros_driver/serial_interface.h
//
// Example:
// const char *serial_port_name = "/dev/ttyUSB0";
const char *serial_port_name = DEFAULT_DATA_INTERFACE;
```

Example:

```c++
const char *serial_port_name = "/dev/ttyUSB1";
```

### Build the code

```bash
cd ~/catkin_ws
catkin_make
```

## Usage

### Launch ANELLO ROS Driver

```bash
rosrun anello_ros_driver anello_ros_driver
```

ANELLO messages will be published to topics (see below). For information on how to view these messages, see the [ROS Wiki](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics).

### Topics

#### Published Topics

Topic definitions are defined in the [ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/) in by the ASCII message format.

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
