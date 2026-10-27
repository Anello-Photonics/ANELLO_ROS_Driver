# ANELLO Ground Gen2 INS ROS Driver

This branch supports the **Ground Gen2 INS only**, on ROS 1 (Noetic/Python 3) or
ROS 2 (tested on Jazzy). It receives NMEA0183 over RS-232 or UDP and publishes
one decoded, typed topic for every output message in the firmware.
The package name remains `anello_ros_driver`. The original Ground INS driver remains on `main`. This branch is a naming adaptation
of `Maritime_INS`; message decoding and transport behavior are unchanged.

## Build

Use a separate workspace and terminal for each ROS version. Source the desired
ROS distribution before building: `ROS_VERSION` selects catkin or ament, including
conditional dependencies in `package.xml`. Do not mix their build/install spaces.
Clone this branch into your workspace's `src/` directory before running these commands.

ROS 1:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select anello_ros_driver
source install/setup.bash
```

## Launch

Serial (set the port and baud rate to match your INS):

```bash
# ROS 1
roslaunch anello_ros_driver ground_gen2.launch port:=/dev/ttyUSB0 baudrate:=921600

# ROS 2
ros2 launch anello_ros_driver ground_gen2.launch.py port:=/dev/ttyUSB0 baudrate:=921600
```

Configure the INS to output NMEA0183 on the connected RS-232 port using
`NM0183_CFG` (1 or 2), with matching `SER_TEL1_BAUD` or `SER_TEL2_BAUD`.
Set the desired `NM0183_ODR_*` output rates above zero. The launch baud rate
is configurable; older units or different port configurations may use 57600.

UDP unicast reception:

```bash
# ROS 1
roslaunch anello_ros_driver ground_gen2.launch transport:=udp udp_port:=19550

# ROS 2
ros2 launch anello_ros_driver ground_gen2.launch.py transport:=udp udp_port:=19550
```

For multicast, add `multicast_group:=239.1.2.3` (replace with the configured
INS group). If necessary, set `multicast_interface` to your computer's local
interface IPv4 address. Configure firmware `NMUDP_EN`, the desired
`NMUDP_ODR_*` rates, and its multicast (`NMUDP_MC_IP0` ... `3`) or unicast
(`NMUDP_UC_IP0` ... `3`) destination. These destination settings depend on the
firmware version; the local firmware supports both. The firmware sends output
to port 19550; port 19551 is its input port and is not this receiver's default.

The receiver does not configure the INS. See the
[GroundGen2 communication manual](https://docs-a1.readthedocs.io/en/ground_gen2/communication_messaging.html)
for device configuration, and use the actual firmware settings for your unit.

Both launch files accept:

| Argument | Default | Meaning |
| --- | --- | --- |
| `transport` | `serial` | `serial` or `udp` |
| `port` | `/dev/ttyUSB0` | Serial device |
| `baudrate` | `921600` | Serial baud rate, 8N1 |
| `bind_address` | `0.0.0.0` | Local UDP bind address |
| `udp_port` | `19550` | UDP receive port |
| `multicast_group` | empty | Optional IPv4 multicast group to join |
| `multicast_interface` | `0.0.0.0` | Local interface for multicast membership |
| `source_ip` | empty | Optional UDP sender IPv4 filter |
| `frame_id` | `ground_gen2` | Header frame label; does not transform data |

## Saved configuration files

Save device and NTRIP settings in a YAML text file. Copy the appropriate template
from `config/ground_gen2_ros1.yaml` or `config/ground_gen2_ros2.yaml`, edit it,
and reference your copy at launch:

```bash
# ROS 1
roslaunch anello_ros_driver ground_gen2.launch config_file:=/absolute/path/my_ins_ros1.yaml

# ROS 2
ros2 launch anello_ros_driver ground_gen2.launch.py config_file:=/absolute/path/my_ins_ros2.yaml
```

ROS 1 files contain a flat parameter mapping, for example:

```yaml
transport: serial
port: /dev/ttyUSB0
baudrate: 921600
ntrip_enabled: true
ntrip_host: caster.example.com
ntrip_mountpoint: YOUR_MOUNTPOINT
ntrip_username: YOUR_USERNAME
ntrip_password: YOUR_PASSWORD
```

ROS 2 requires the same values nested under the node name and `ros__parameters`:

```yaml
ground_gen2:
  ros__parameters:
    transport: serial
    port: /dev/ttyUSB0
    baudrate: 921600
    ntrip_enabled: true
    ntrip_host: caster.example.com
    ntrip_mountpoint: YOUR_MOUNTPOINT
    ntrip_username: YOUR_USERNAME
    ntrip_password: YOUR_PASSWORD
```

Files may contain only the settings you want to change. **File values override
matching launch arguments**; omitted values use the launch arguments/defaults.
Without `config_file`, the existing argument-based launch works as before.
Use YAML booleans/numbers for boolean/numeric parameters, and quote credentials
that YAML might otherwise interpret as numbers or booleans. The supplied templates
include all available device and NTRIP settings, with NTRIP disabled initially.

## NTRIP corrections

NTRIP is optional and disabled by default. Enable it in either launch file with
`ntrip_enabled:=true` and specify your caster and mountpoint. The client uses
NTRIP 2 requests and accepts HTTP and legacy ICY streams, including HTTP chunked
encoding. It forwards checksum-valid RTCM3 frames unchanged.

Serial example (ROS 2):

```bash
ros2 launch anello_ros_driver ground_gen2.launch.py \
  port:=/dev/ttyUSB0 baudrate:=921600 \
  ntrip_enabled:=true ntrip_host:=caster.example.com ntrip_port:=2101 \
  ntrip_mountpoint:=YOUR_MOUNTPOINT \
  ntrip_username:=YOUR_USERNAME ntrip_password:=YOUR_PASSWORD
```

RTCM is written through the **same serial connection** receiving NMEA0183; no
second serial port or ground INS configuration port is opened. Configure that
INS port for NMEA0183 input/output.

UDP example (ROS 1):

```bash
roslaunch anello_ros_driver ground_gen2.launch \
  transport:=udp udp_port:=19550 rtcm_host:=192.168.1.111 rtcm_port:=19552 \
  ntrip_enabled:=true ntrip_host:=caster.example.com ntrip_port:=2101 \
  ntrip_mountpoint:=YOUR_MOUNTPOINT \
  ntrip_username:=YOUR_USERNAME ntrip_password:=YOUR_PASSWORD
```

Replace `rtcm_host` with the INS address. The receive port remains 19550; RTCM
is sent to **19552**, not the NMEA input port 19551. Enable the firmware's UDP
bridge (`NMUDP_EN`) and configure its NMEA output destination/rates separately.
Each UDP datagram contains one complete RTCM3 frame. `source_ip` filters incoming
NMEA only; `rtcm_host` explicitly chooses the correction destination.

The examples work on either ROS version by changing the launch command/file.
No RTCM ROS topic or separate NTRIP node is required. Decoded navigation topics
remain unchanged. For casters requiring position (such as VRS services), enable
GGA output on the INS: the client privately sends its latest checksum-valid GGA
with a nonzero fix and finite position back to the caster. It does not publish
raw NMEA topics. GGA older than 30 seconds is not sent.

| Argument | Default | Meaning |
| --- | --- | --- |
| `ntrip_enabled` | `false` | Start the caster client |
| `ntrip_host` | empty | Caster hostname/IP (required when enabled) |
| `ntrip_port` | `2101` | Caster TCP port |
| `ntrip_mountpoint` | empty | Correction mountpoint (required when enabled) |
| `ntrip_username`, `ntrip_password` | empty | Optional HTTP Basic credentials |
| `ntrip_tls` | `false` | TLS with server certificate/hostname verification |
| `ntrip_ca_cert` | empty | Optional CA file; empty uses system trust |
| `ntrip_gga_interval` | `5.0` | GGA send interval in seconds; 0 disables feedback |
| `ntrip_connect_timeout` | `3.0` | TCP/TLS connection timeout in seconds |
| `ntrip_timeout` | `15.0` | Reconnect after this many seconds without valid RTCM |
| `ntrip_reconnect_delay` | `5.0` | Delay between connection attempts |
| `rtcm_host` | empty | INS destination IPv4 address/hostname; required for UDP NTRIP |
| `rtcm_port` | `19552` | INS UDP RTCM input port; unused for serial |

Caster connections run in a background worker. Device writes run in the receive
loop, with a bounded queue and a two-second correction age limit, so disconnections
do not accumulate old corrections. Failed caster connections retry automatically.
Credentials are never included in driver logs; as with other ROS parameters, they
are accessible to tools that can inspect the node's parameters.

## Topics

The topics and field definitions are identical in ROS 1 and ROS 2. ROS 1 type
names use `anello_ros_driver/GroundGen2GGA`; ROS 2 uses
`anello_ros_driver/msg/GroundGen2GGA`, and likewise for the other types.

| Topic | Message | Contents |
| --- | --- | --- |
| `/ground_gen2/GGA` | `GroundGen2GGA` | UTC, position, fix quality, satellites, HDOP, MSL altitude, geoid separation |
| `/ground_gen2/RMC` | `GroundGen2RMC` | UTC/date, status, position, speed and course over ground |
| `/ground_gen2/HDT` | `GroundGen2HDT` | True heading |
| `/ground_gen2/XDR` | `GroundGen2XDR` | Heading, roll, pitch |
| `/ground_gen2/ZDA` | `GroundGen2ZDA` | UTC date/time and optional zone fields |
| `/ground_gen2/PAPIMU` | `GroundGen2PAPIMU` | Accelerometer, MEMS and three SiPhOG rates, magnetometer, temperature, status |
| `/ground_gen2/PAPINS` | `GroundGen2PAPINS` | Navigation status, position, NED velocity, attitude |
| `/ground_gen2/PAPACC` | `GroundGen2PAPACC` | Position, velocity and attitude accuracy |

All publishers exist at startup, but publish only when the matching sentence
arrives. Standard sentences are routed by formatter (e.g. `GPGGA` and `GNGGA`
use `GGA`); `sentence_id` retains the original identifier. Topic names are
private to the node and follow its name/namespace if remapped.

```bash
rostopic echo /ground_gen2/PAPINS          # ROS 1
ros2 topic echo /ground_gen2/PAPINS       # ROS 2
```

Only decoded messages are published. Checksum failures, malformed fields,
unsupported sentences and unexpected field counts are rejected. Serial reads
can split sentences at arbitrary byte boundaries. UDP datagrams are independent;
multiple sentences per datagram are supported. Connections retry on I/O errors.

### Units and missing data

The `.msg` files document units. Latitude/longitude are decimal degrees; GGA/RMC
coordinates are converted from NMEA degrees/minutes. Other values preserve
firmware units: acceleration in g, angular rates in degrees/second, magnetic
field in gauss, speed in knots for RMC, and NED velocity in meters/second for INS.
There is no NED-to-ENU or body-frame conversion.

Empty floating-point fields become NaN, empty integer fields become -1, and empty
text fields stay empty. Navigation warnings and invalid fixes are still published
with their status intact. ZDA includes `local_zone_valid` because -1 can also be
a real time-zone hour offset. PAPINS currently leaves ZUPT empty.

`header.stamp` is host processing time in the selected ROS clock, not sensor
measurement time. `time_ms` and IMU `sync_time_ms` are device times in milliseconds
since boot. PAPINS/PAPACC `utc_time_s` is Unix UTC seconds (firmware sends zero if
unavailable); standard sentence UTC fields retain their NMEA text format.

Layouts follow `PX4-Autopilot/src/drivers/nmea0183/NMEAOutput.cpp`. In particular,
PAPACC emits position accuracy, then velocity accuracy, then attitude accuracy;
its source comment lists a different order. PAPINS has no extra height field
between ellipsoid altitude and north velocity. These distinctions are tested.

## Tests and scope

From the repository root, run the ROS-independent parser, schema, routing,
localhost UDP and pseudo-terminal serial tests:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_ground_gen2*.py' -v
```

After building and sourcing the ROS 2 install, test the installed launch and
all eight topics plus NTRIP correction routing and GGA feedback (starts and stops
its own node and test caster, using temporary local ports):

```bash
python3 tests/ros2_smoke.py
```

ROS 2 Jazzy interface generation, build/install, and launch/subscriber integration
have been tested. ROS 1 runtime/build and physical INS operation have not been
verified in this environment. NTRIP correction forwarding is supported over the shared serial port or the
dedicated UDP RTCM port. Configuration services and ground INS protocols are not included.

## License

MIT; see [LICENSE](LICENSE). Original driver by ANELLO Photonics.

## VBW speed input (ROS 1 and ROS 2)

Publish `anello_ros_driver/GroundGen2VBW` (ROS 2: `anello_ros_driver/msg/GroundGen2VBW`)
to `/ground_gen2/vbw_input`. Speeds are signed vessel-axis longitudinal and
transverse values in **knots**, matching NMEA VBW; no coordinate transform is
performed. Convert m/s to knots before publishing. Only mark a water or ground
pair valid when both measurements are available. Invalid pairs are encoded as
empty fields with status V. Valid pairs use status A. Nonfinite valid values and
magnitudes above 999.999 knots are rejected.

Serial sends VBW on the same open NMEA serial connection as RTCM. UDP sends to
`nmea_host:19551` (port configurable with `nmea_port`). An empty `nmea_host`
uses `rtcm_host`; if both are empty, input is rejected with a warning.
This destination is separate from the RTCM port 19552 and receive port 19550.
Publish at your sensor rate: the driver keeps only the newest pending sample,
drops samples pending for over one second, and does not repeat a sample.
ROS 1 and ROS 2 require rebuilding after adding this message type.

Bench example: 2 knots longitudinal ground speed, no water measurement (Ctrl+C to stop):

```bash
# ROS 1
rostopic pub -r 10 /ground_gen2/vbw_input anello_ros_driver/GroundGen2VBW \
  '{water_longitudinal_knots: 0.0, water_transverse_knots: 0.0, water_valid: false, ground_longitudinal_knots: 2.0, ground_transverse_knots: 0.0, ground_valid: true}'

# ROS 2
ros2 topic pub -r 10 /ground_gen2/vbw_input anello_ros_driver/msg/GroundGen2VBW \
  '{water_longitudinal_knots: 0.0, water_transverse_knots: 0.0, water_valid: false, ground_longitudinal_knots: 2.0, ground_transverse_knots: 0.0, ground_valid: true}'
```

Use `listener sensor_water_speed_generic -n 5` in the PX4 shell to verify reception;
this example should produce ground_speed_x about 1.029 m/s. Reception does not
by itself establish that the navigation filter is fusing the speed measurement.
