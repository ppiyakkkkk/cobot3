# 입력 영상의 시간과 카메라 좌표계를 유지한다.
std_msgs/Header header

# 현재 프레임에서 조난자 후보가 검출됐는지 나타낸다.
bool detected
string class_name
float32 confidence

# Bounding Box가 계산된 원본 RGB 영상 크기다.
uint32 image_width
uint32 image_height

# 원본 RGB 영상 픽셀 좌표 기준 Bounding Box다.
int32 x_min
int32 y_min
int32 x_max
int32 y_max
