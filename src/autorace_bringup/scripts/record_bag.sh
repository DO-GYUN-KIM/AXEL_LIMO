#!/usr/bin/env bash
# 주행 데이터를 rosbag2로 녹화합니다. 자세한 사용법: docs/rosbag_활용_안내.md
#
# 사용법: bash scripts/record_bag.sh <이름> [lap|debug|yolo] [추가 토픽...]
#   lap   (기본) 센서 + 차량 상태 + 주행 명령. 매일 첫 녹화, 라이다·주차 미션 녹화용
#   debug  lap + 우리 노드 출력(/autorace/*, 주최 YOLO 결과). 우리 코드를 켜고 달릴 때
#   yolo   카메라만. YOLO 학습 사진 수집용 (용량이 가장 작음)
#   예) bash scripts/record_bag.sh lap_slow
#   예) bash scripts/record_bag.sh m1_red debug
#   예) bash scripts/record_bag.sh signs_AB yolo
#   예) bash scripts/record_bag.sh m1_test /autorace/mission_state   # 예전 방식(프로필 생략 = lap)
#
# 환경 변수:
#   BAG_DIR      저장 위치 (기본 ~/bags). 저장소 안 bags/ 폴더는 .gitignore에 들어 있습니다.
#   BAG_STORAGE  sqlite3 (기본) 또는 mcap (ros-humble-rosbag2-storage-mcap 설치 필요)
set -euo pipefail

NAME="${1:-run}"
shift || true
PROFILE="lap"
if [[ $# -gt 0 && "$1" =~ ^(lap|debug|yolo)$ ]]; then
  PROFILE="$1"
  shift
fi

BAG_DIR="${BAG_DIR:-$HOME/bags}"
STORAGE="${BAG_STORAGE:-sqlite3}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${BAG_DIR}/${STAMP}_${NAME}_${PROFILE}"
mkdir -p "${BAG_DIR}"

CAMERA=(/camera/color/image_raw/compressed)   # 원본 image_raw는 용량이 약 20배라 녹화하지 않습니다.
VEHICLE=(
  /scan
  /odom
  /imu
  /limo_status
  /limo/ack_cmd      # 애커먼 모드에서 차에 실제로 들어간 명령
  /cmd_vel           # 변환 노드에 들어가기 전 우리 명령 (조종기로 몰 때는 비어 있을 수 있음)
  /tf
  /tf_static
)
OURS=(
  /autorace/mission_state
  /autorace/lane_info
  /autorace/lidar_info
  /autorace/detections
  /autorace/lane_cmd_vel
  /autorace/lane_debug/compressed   # lane_debug:=true 일 때만 나옵니다
  /yolo_bounding_boxes              # 주최 yolo_object_detect 를 쓸 때
)

case "${PROFILE}" in
  lap)   TOPICS=("${CAMERA[@]}" "${VEHICLE[@]}") ;;
  debug) TOPICS=("${CAMERA[@]}" "${VEHICLE[@]}" "${OURS[@]}") ;;
  yolo)  TOPICS=("${CAMERA[@]}") ;;
esac

# 디스크 여유 확인 (5 GB 미만이면 경고)
FREE_KB="$(df -Pk "${BAG_DIR}" | awk 'NR==2 {print $4}')"
if [[ -n "${FREE_KB}" && "${FREE_KB}" -lt 5242880 ]]; then
  echo "[record_bag] 경고: 남은 공간 $((FREE_KB / 1024)) MB. 오래된 bag을 노트북으로 옮긴 뒤 지우십시오."
fi

echo "[record_bag] 저장: ${OUT}  (프로필 ${PROFILE}, 형식 ${STORAGE})"
echo "[record_bag] 토픽: ${TOPICS[*]} $*"
echo "[record_bag] 아직 안 나오는 토픽은 나올 때부터 녹화됩니다. Ctrl+C로 끝냅니다."
echo "[record_bag] 끝나면 score_sheet.csv에 bag 이름과 배터리 %를 적어 두십시오."
exec ros2 bag record -s "${STORAGE}" -o "${OUT}" "${TOPICS[@]}" "$@"
