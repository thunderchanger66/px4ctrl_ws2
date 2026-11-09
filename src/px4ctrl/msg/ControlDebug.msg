# All vectors are expressed in the PX4 local NED frame.
std_msgs/Header header
string state
geometry_msgs/Vector3 position_error
geometry_msgs/Vector3 velocity_error
geometry_msgs/Vector3 velocity_reference
geometry_msgs/Vector3 acceleration_requested
geometry_msgs/Vector3 acceleration_feasible
float32 thrust
float32 tilt_deg
float32 yaw_rate
float32 command_age_sec
uint8 saturation_flags
bool local_valid
bool status_fresh
bool armed
bool offboard
bool failsafe
