# Decoded Maritime INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
float64 heading_deg # HDG transducer, degrees true
float64 roll_deg
float64 pitch_deg
