# timestamp of measurement
std_msgs/Header header

int8 status -1

float64 longitude
float64 latitude
float64 altitude

# position of antenna in ENU (m)
geometry_msgs/Point antenna_position
bool antenna_position_valid false

# heading of dual antenna in ENU (rad)
float64 dual_antenna_heading
bool dual_antenna_heading_valid false
