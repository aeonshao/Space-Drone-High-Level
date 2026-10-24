# One ArUco tag seen in one camera frame
int32 tag_id
string tool_name

# pose of the tag in the camera optical frame (x right, y down, z out of the lens), meters
geometry_msgs/Pose pose

# how many pixels the tag covers, kind of a confidence thing (bigger = closer/better)
float32 pixel_area
