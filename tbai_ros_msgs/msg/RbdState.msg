# Robot state message
# The message contains the following values:
# Base orientation - ZYX Euler angles - 3
# Base position in world frame - 3
# Base angular velocity in base frame - 3
# Base linear velocity in base frame - 3
# Joint angles - n_joints
# Joint velocities - n_joints
#
# In total, there are (3+3+3+3+n_joints+n_joints) values in the message.
# The message also includes contact information for the feet.

time stamp
float64[] rbd_state
bool[] contact_flags