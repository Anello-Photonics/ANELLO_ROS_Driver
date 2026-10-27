# Decoded Ground Gen2 INS NMEA0183. Empty floats are NaN; no axis/unit conversion.
std_msgs/Header header # Host receipt time; frame_id from launch configuration
string sentence_id # Original identifier without dollar sign
string utc_time # UTC hhmmss.ss; empty if unavailable
float64 latitude_deg
float64 longitude_deg
int16 fix_quality # -1 if empty; 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float, 6 dead reckoning
int16 satellites_used # -1 if empty
float64 hdop
float64 altitude_msl_m
float64 geoid_separation_m
float64 differential_age_s
string station_id
