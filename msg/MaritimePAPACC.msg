# Decoded Maritime INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
float64 time_ms # Device sample time since boot, milliseconds
float64 utc_time_s # Unix UTC seconds; firmware sends 0 when unavailable
float64 horizontal_position_accuracy_m
float64 vertical_position_accuracy_m
float64 velocity_north_accuracy_m_s
float64 velocity_east_accuracy_m_s
float64 velocity_down_accuracy_m_s
float64 roll_accuracy_deg
float64 pitch_accuracy_deg
float64 heading_accuracy_deg
