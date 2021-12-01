# This contains the lanelet map in binary format.

# Timestamp and frame_id of the map
std_msgs/Header header

# version of map format. keep this as empty string if unnecessary
string format_version

# version of map version. keep this as empty string if unnecessary
string map_version

# binary data of lanelet2 map. This is meant to be filled using toBinMsg() in lanelet2_extension library
int8[] data
