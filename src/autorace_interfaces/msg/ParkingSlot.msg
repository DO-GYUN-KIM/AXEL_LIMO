# 카메라(버드아이)로 찾은 주차칸입니다. 부호 규칙: 양수 = 로봇 기준 왼쪽.
std_msgs/Header header
bool slot_valid                # 칸의 좌/우 선 중 하나 이상을 찾았는지
bool left_line_found
bool right_line_found
float32 center_offset          # 칸 중심선의 가로 위치 [m] (양수 = 칸 중심이 왼쪽)
float32 heading_error          # 칸 방향과 로봇 방향 차이 [rad] (양수 = 칸이 왼쪽으로 기울어 보임)
float32 slot_width             # 좌/우 선 사이 폭 [m] (둘 다 찾았을 때만 의미 있음)
bool end_line_found            # 칸 안쪽 끝 선(가로선)을 찾았는지
float32 end_line_distance      # 앞바퀴에서 끝 선까지 거리 [m]
