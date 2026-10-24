# stamp is the ROS-clock time of states[0], NOT PX4 boot time. frame_id must be "ned".
std_msgs/Header header
# Uniform sample period [s], must match ommpc_node's dt.
float64 dt
# M+1 states and M feedforward inputs; M must cover the controller horizon.
# Each transition must satisfy dynamic.hpp. No interpolation or terminal hold.
uav_ommpc_interfaces/ReferenceState[<=2001] states
uav_ommpc_interfaces/ThrustRates[<=2000] inputs
