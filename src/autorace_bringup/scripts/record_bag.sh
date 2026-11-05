#!/usr/bin/env bash
# 주행 데이터를 rosbag2로 녹화합니다. 매일 차량을 쓰기 전에 먼저 한 바퀴 녹화하십시오.
#
# 사용법: bash scripts/record_bag.sh [이름] [추가 토픽...]
#   예) bash scripts/record_bag.sh lap_slow
#   예) bash scripts/record_bag.sh m1_light /autorace/mission_state /autorace/lane_info
# 저장 위치: $BAG_DIR (기본값 ~/bags). 저장소 안 bags/ 폴더는 .gitignore에 들어 있습니다.
set -euo pipefail

NAME="${1:-run}"
shift || true
BAG_DIR="${BAG_DIR:-$HOME/bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${BAG_DIR}/${STAMP}_${NAME}"
mkdir -p "${BAG_DIR}"

TOPICS=(
  /camera/color/image_raw/compressed
  /scan
  /odom          # TODO(확인): 로봇에서 실제 오도메트리 토픽 이름
  /cmd_vel
)

echo "[record_bag] 저장: ${OUT}"
echo "[record_bag] 토픽: ${TOPICS[*]} $*"
echo "[record_bag] Ctrl+C로 끝냅니다. 배터리 잔량을 score_sheet.csv에 적어 두십시오."
exec ros2 bag record -o "${OUT}" "${TOPICS[@]}" "$@"
