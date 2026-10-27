# Decoded Ground Gen2 INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
string utc_time # UTC hhmmss.ss
string status # A active, V navigation warning
float64 latitude_deg
float64 longitude_deg
float64 speed_knots
float64 course_deg # Course over ground, degrees true
string utc_date # ddmmyy
float64 magnetic_variation_deg
string magnetic_variation_direction # E or W
