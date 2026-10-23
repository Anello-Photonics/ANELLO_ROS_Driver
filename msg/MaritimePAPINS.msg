# Decoded Maritime INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
float64 time_ms # Device sample time since boot, milliseconds
float64 utc_time_s # Unix UTC seconds; firmware sends 0 when unavailable
int16 status # Firmware estimator status; -1 if absent
float64 latitude_deg
float64 longitude_deg
float64 altitude_ellipsoid_m
float64 velocity_north_m_s
float64 velocity_east_m_s
float64 velocity_down_m_s
float64 roll_deg
float64 pitch_deg
float64 heading_deg # Firmware wraps to +/-180 degrees
int16 zupt # -1 if unavailable (currently always empty)
