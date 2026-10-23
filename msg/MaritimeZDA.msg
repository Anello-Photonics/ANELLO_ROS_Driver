# Decoded Maritime INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
string utc_time # UTC hhmmss.ss
int16 day # -1 if unavailable
int16 month # -1 if unavailable
int16 year # -1 if unavailable
int16 local_zone_hours # -1 if unavailable; consult field presence below
int16 local_zone_minutes # -1 if unavailable
bool local_zone_valid
