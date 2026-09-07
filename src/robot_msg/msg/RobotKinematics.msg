# RobotKinematics.msg
# One robot state with uncertainty

uint32 id

# Position & orientation with 6x6 covariance (row-major, 36 elements)
geometry_msgs/PoseWithCovariance pose

# Linear & angular velocity with 6x6 covariance (row-major, 36 elements)
geometry_msgs/TwistWithCovariance twist

# Obstacle shape: equivalent ellipse in the map frame
# axis_a: semi-major axis (m), axis_b: semi-minor axis (m)
# angle_rad: orientation of the major axis (yaw, rad, map frame, right-hand Z)
float64 axis_a
float64 axis_b
float64 angle_rad
