# World NED position [m] and velocity [m/s], in the same local origin as /uav/state.
float64[3] position
float64[3] velocity
# Unit Hamilton quaternion [w, x, y, z], body FRD -> world NED (not ROS xyzw).
float64[4] q
