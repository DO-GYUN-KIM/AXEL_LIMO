# 미션 매니저의 현재 상태입니다. 디버깅과 rosbag 분석용입니다.
std_msgs/Header header
string mission_name
uint8 mission_index
string phase                   # idle / approach / active / done
float32 elapsed                # 현재 단계에 들어온 뒤 지난 시간 [s]
string last_result             # 직전 미션 결과 (ok / approach_timeout / active_timeout / skipped)
string note
float32 cmd_linear
float32 cmd_angular
