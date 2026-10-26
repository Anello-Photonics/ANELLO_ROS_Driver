

## VBW speed input (ROS 1 and ROS 2)

Publish `anello_ros_driver/MaritimeVBW` (ROS 2: `anello_ros_driver/msg/MaritimeVBW`)
to `/maritime_ins/vbw_input`. Speeds are signed vessel-axis longitudinal and
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
rostopic pub -r 10 /maritime_ins/vbw_input anello_ros_driver/MaritimeVBW \
  '{water_longitudinal_knots: 0.0, water_transverse_knots: 0.0, water_valid: false, ground_longitudinal_knots: 2.0, ground_transverse_knots: 0.0, ground_valid: true}'

# ROS 2
ros2 topic pub -r 10 /maritime_ins/vbw_input anello_ros_driver/msg/MaritimeVBW \
  '{water_longitudinal_knots: 0.0, water_transverse_knots: 0.0, water_valid: false, ground_longitudinal_knots: 2.0, ground_transverse_knots: 0.0, ground_valid: true}'
```

Use `listener sensor_water_speed_generic -n 5` in the PX4 shell to verify reception;
this example should produce ground_speed_x about 1.029 m/s. Reception does not
by itself establish that the navigation filter is fusing the speed measurement.
