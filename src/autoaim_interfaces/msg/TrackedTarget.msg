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

# Absolute target direction in the gimbal frame, in radians.
# Positive yaw points left, positive pitch points down, measured against the
# gimbal zero pose. Computed as the gimbal attitude at the observation instant
# plus the angular offset of the target from the camera optical axis, so the
# lower board can drive straight to this angle instead of integrating an error.
# The value is NOT wrapped; it may exceed +-pi.
# Only present while status is TRACKING and a fresh gimbal attitude is
# available: when no attitude can be paired with the observation the tracker
# skips that frame entirely rather than publishing a bogus setpoint.
# Zero when status is HOLDING or LOST.
float32 yaw_target
float32 pitch_target

# Estimated target angular velocity in rad/s, for downstream latency compensation.
# Zero when status is not TRACKING.
float32 yaw_rate
float32 pitch_rate
