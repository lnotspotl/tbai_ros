# Robot state message
# The message contains the following values:
# Base orientation - quaternion xyzw - 4
# Base angular velocity (expressed in base frame) - 3
# Base linear acceleration (expressed in base frame) - 3
# Joint angles - n_joints
# Joint velocities - n_joints
#
# The message also includes contact information for the feet.

time stamp
float64[4] orientation_xyzw
float64[3] ang_vel
float64[3] lin_acc
float64[] joint_angles
float64[] joint_velocities
bool[] contact_flags