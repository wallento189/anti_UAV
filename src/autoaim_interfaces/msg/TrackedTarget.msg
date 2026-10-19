std_msgs/Header header

# Tracker status: 0=LOST (no target), 1=TRACKING (observed this cycle),
# 2=HOLDING (hold_frames consecutive frames without a valid observation).
# The state machine itself lives in the tracker; this message only carries the value.
uint8 status

# Filtered bounding box center and corners in original image pixel coordinates.
float32 x
float32 y
float32 x1
float32 y1
float32 x2
float32 y2

# Angular offset from the camera optical axis in radians.
# Positive yaw points left, positive pitch points down.
# Rotating the gimbal by these angles brings the target back to the image center.
# Zero when status is LOST.
float32 yaw_error
float32 pitch_error

# Estimated target angular velocity in rad/s, for downstream latency compensation.
# Zero when status is LOST.
float32 yaw_rate
float32 pitch_rate
