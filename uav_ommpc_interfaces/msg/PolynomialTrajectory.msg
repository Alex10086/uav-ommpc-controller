# ROS-clock start time of the FIRST segment; frame_id must be "ned".
# Renew the receipt watchdog by republishing the same plan with its original start time.
std_msgs/Header header
# Continue with stationary hover after the last segment only if terminal v/a/jerk/yaw_rate are zero.
# Receipt freshness still applies while holding. False means no extrapolation beyond the last segment.
bool hold_final
# Position must be C3 and unwrapped yaw C1 across segments. No per-message sample dt.
uav_ommpc_interfaces/PolynomialSegment[<=100] segments
