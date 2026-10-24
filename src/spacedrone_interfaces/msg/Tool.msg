# A tool that the registry has localized
int32 tag_id
string tool_name

# position in PX4 local NED frame (meters from home / takeoff spot)
geometry_msgs/Point position

# number of detections that went into the average
int32 num_sightings

# std dev of the sightings, if this is big something is off
float32 spread_m
