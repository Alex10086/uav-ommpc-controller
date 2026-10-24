# Segment duration [s], finite and positive.
float64 duration
# Ascending coefficients c0..cn of normalized local time s=(t-t_segment_start)/duration.
# Each axis supports degree 0..9; nonempty arrays may have different lengths.
# Position is world NED [m], yaw is unwrapped heading [rad]. Derivatives scale by duration^-order.
float64[<=10] x
float64[<=10] y
float64[<=10] z
float64[<=10] yaw
