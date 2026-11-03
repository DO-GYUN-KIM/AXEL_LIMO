# 한 프레임의 검출 결과 목록입니다. header.stamp는 원본 이미지의 시각입니다.
std_msgs/Header header
uint32 image_width
uint32 image_height
Detection[] detections
