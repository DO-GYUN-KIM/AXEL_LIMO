# 차선 검출 결과입니다. 부호 규칙: 양수 = 로봇 기준 왼쪽.
std_msgs/Header header
bool lane_valid                # 차선 중심을 계산할 수 있었는지
bool left_detected
bool right_detected
float32 center_offset          # 차선 중심의 가로 위치 [m] (양수 = 중심이 로봇 왼쪽에 있음 -> 왼쪽으로 조향)
float32 heading_error          # 차선 방향과 로봇 방향의 차이 [rad] (양수 = 차선이 왼쪽으로 휨)
bool stop_line_detected
float32 stop_line_distance     # 앞바퀴에서 정지선까지 추정 거리 [m]
