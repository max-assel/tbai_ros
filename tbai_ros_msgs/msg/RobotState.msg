# Robot state message
# The message contains the following values:
# Base orientation - quaternion xyzw - 4
# Base angular velocity (expressed in base frame) - 3
# Base linear acceleration (expressed in base frame) - 3
# Joint angles - 12
# Joint velocities - 12
#
# In total, there are (3+3+3+3+12+12) = 36 values in the message.
# The message also includes contact information for the feet.
# There are four feet, each contains a single contact flag
# The feet are ordered as follows: LF, RF, LH, RH

time stamp
float64[4] orientation_xyzw
float64[3] ang_vel
float64[3] lin_acc
float64[] joint_angles
float64[] joint_velocities
bool[4] contact_flags