std_msgs/Header header

# Transport-layer projection of TrackedTarget: exactly the fields that are
# written to the serial link. No control law is applied on the ROS side.
# Status: 0=LOST, 1=TRACKING, 2=HOLDING.
uint8 status

# Angular offset from the camera optical axis in radians.
# Positive yaw points left, positive pitch points down.
float32 yaw_error
float32 pitch_error

# Estimated target angular velocity in rad/s.
# Zero when the tracker has no valid observation.
float32 yaw_rate
float32 pitch_rate
