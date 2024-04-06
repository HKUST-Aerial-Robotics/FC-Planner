Header header

# the trajectory id, starts from "1".
uint32 trajectory_id

# the action command for trajectory server.
uint32 ACTION_ADD           =   1
uint32 ACTION_ABORT         =   2
uint32 ACTION_WARN_START           =   3
uint32 ACTION_WARN_FINAL           =   4
uint32 ACTION_WARN_IMPOSSIBLE      =   5
uint32 action

# the order of trajectory.
quadrotor_msgs/PolynomialMatrix[] trajectory 
quadrotor_msgs/PolynomialMatrix[] traj_yaw 
quadrotor_msgs/PolynomialMatrix[] traj_pitch 
