std_msgs/Header header

# Sequence number of the serial feedback frame, useful to spot drops in bags.
uint8 seq

# Gimbal attitude reported by the lower board, in radians.
# Same convention as TrackedTarget: positive yaw points left, positive pitch
# points down, measured against the gimbal zero pose.
float32 yaw
float32 pitch
