# Trajectory type enums

# Undefined trajectory type
uint8 UNDEFINED=0

# General trajectory type that considers orientation from the pose and
# neglects heading values
uint8 GENERAL=1

# Trajectory types that compute orientation from acceleration and heading
# values and consider derivatives up to what is indicated by the name
uint8 ACCELERATION=2
uint8 JERK=3
uint8 SNAP=4

Header header

# Trajectory type as defined above
uint8 type

quadrotor_msgs/TrajectoryPoint[] points
