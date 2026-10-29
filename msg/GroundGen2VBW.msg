# Vessel-axis speeds in knots, matching NMEA VBW. No frame transformation.
# NaN means unknown and is encoded as an empty NMEA field.
# A valid source with at least one finite component uses status A; otherwise V.
float64 water_longitudinal_knots
float64 water_transverse_knots
bool water_valid
float64 ground_longitudinal_knots
float64 ground_transverse_knots
bool ground_valid
