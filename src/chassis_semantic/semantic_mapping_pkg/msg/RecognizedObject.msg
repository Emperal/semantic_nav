# RecognizedObject.msg

std_msgs/Header header
string label
float32 confidence

int32 bbox_x1
int32 bbox_y1
int32 bbox_x2
int32 bbox_y2

sensor_msgs/PointCloud2 cloud