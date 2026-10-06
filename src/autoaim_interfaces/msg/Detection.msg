std_msgs/Header header

# Whether a drone was detected in this frame. When false all fields below are zero.
bool detected

# Detection confidence in [0, 1].
float32 score

# Center of the bounding box in original image pixel coordinates.
float32 x
float32 y

# Bounding box corners in original image pixel coordinates.
float32 x1
float32 y1
float32 x2
float32 y2
