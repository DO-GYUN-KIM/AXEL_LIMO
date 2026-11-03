# LiDAR 가공 결과입니다. 좌표계: x = 전방, y = 왼쪽 (로봇 기준).
std_msgs/Header header
float32 front_min_distance     # 전방 부채꼴 안 최소 거리 [m] (없으면 inf)

# 3차선 구간용: 로봇이 있는 차로를 center로 보고 좌/우 차로 점유 여부를 판단합니다.
bool left_lane_occupied
bool center_lane_occupied
bool right_lane_occupied
float32 left_lane_min_distance
float32 center_lane_min_distance
float32 right_lane_min_distance

# 라바콘 구간용
int32 num_clusters             # 전방에서 찾은 물체(라바콘 후보) 개수
float32 nearest_cluster_distance
bool gap_found
float32 gap_angle              # 통과할 지점 방향 [rad] (양수 = 왼쪽)
float32 gap_width              # 통과 폭 추정 [m]
