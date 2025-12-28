# Single 2D detection
std_msgs/Header header

# Bounding box coordinates [x1, y1, x2, y2] in pixels
float32[4] bbox_xyxy

# Class information
string class_name
int32 class_id

# Detection confidence score
float32 score

# Pixel center coordinates [x, y]
float32[2] px_center
