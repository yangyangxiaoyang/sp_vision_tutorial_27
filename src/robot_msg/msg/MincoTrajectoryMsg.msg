std_msgs/Header header

# Absolute start time for evaluating trajectory time t=(now-start_time)
builtin_interfaces/Time start_time

# Monotonic trajectory sequence from planner
uint32 trajectory_id

# Polynomial order per piece (MINCO_S2NU uses cubic = 3)
uint8 poly_order

# Number of trajectory pieces
uint32 piece_num

# Total duration in seconds
float32 total_duration

# Per-piece duration in seconds, size=piece_num
float32[] durations

# Flattened polynomial coefficients in piece-major order.
# For each piece i, append coeffs [c0, c1, ..., cK] for x(t), y(t), z(t)
# where p(t)=c0+c1*t+...+cK*t^K, K=poly_order.
# Array size for each axis = piece_num * (poly_order + 1)
float32[] coeff_x
float32[] coeff_y
float32[] coeff_z
