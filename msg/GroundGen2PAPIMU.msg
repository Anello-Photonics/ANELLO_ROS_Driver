# Decoded Ground Gen2 INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
float64 time_ms # Device sample time since boot, milliseconds
float64 sync_time_ms # PPS capture time since boot, milliseconds
float64 accel_x_g
float64 accel_y_g
float64 accel_z_g
float64 mems_rate_x_deg_s
float64 mems_rate_y_deg_s
float64 mems_rate_z_deg_s
float64 siphog_rate_x_deg_s
float64 siphog_rate_y_deg_s
float64 siphog_rate_z_deg_s
float64 magnetic_x_gauss
float64 magnetic_y_gauss
float64 magnetic_z_gauss
float64 temperature_c
int16 status_x # -1 if unavailable; firmware SiPhOG status bit mask
int16 status_y
int16 status_z
