### Default parts of the message
std_msgs/Header header

# Current joint states
float64[7] joint_positions_curr
float64[7] joint_velocities_curr
float64[7] joint_velocities_curr_filtered

# Desired joint position
float64[7] joint_positions_desired

# Joint torque commands
float64[7] joint_torques_command