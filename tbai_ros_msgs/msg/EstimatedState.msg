## State as provided by the state estimator

float64 timestamp
float64[3] base_position
float64[4] base_orientation_xyzw
float64[3] base_lin_vel # expressed in base frame
float64[3] base_ang_vel # expressed in base frame
float64[12] joint_angles
float64[12] joint_velocities
bool[4] contact_flags