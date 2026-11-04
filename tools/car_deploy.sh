#!/usr/bin/env bash
# 차량(Jetson)에서 실행합니다. 지정한 브랜치·태그·커밋을 받아 빌드하고, 누가 무엇을 올렸는지 기록합니다.
# 차 1대를 3명이 함께 쓰므로 "차 사용 잠금"을 겸합니다.
#
# 사용법
#   bash tools/car_deploy.sh <이름> [브랜치|태그|커밋]   예) bash tools/car_deploy.sh 도균 feat/lane
#   bash tools/car_deploy.sh --status                    지금 누가 무엇을 올렸는지 확인
#   bash tools/car_deploy.sh --release <이름>            차 사용 끝(잠금 해제)
#   FORCE=1 bash tools/car_deploy.sh <이름> <ref>        남의 잠금을 무시(팀 합의 후에만)
#
# 동작 순서
#   1) 잠금 확인 → 2) 차량에서 고친 파일이 남아 있으면 중단 → 3) git fetch (실패하면 오프라인으로 보고 로컬 브랜치 사용)
#   4) 해당 커밋으로 detach checkout → 5) colcon build --symlink-install → 6) DEPLOYED 파일에 기록
set -euo pipefail

WS="${AXEL_WS:-$HOME/axel_ws}"
LOCK="$WS/.car_lock"
DEPLOYED="$WS/DEPLOYED"
cd "$WS"

case "${1:-}" in
  ""|-h|--help) sed -n '2,15p' "$0"; exit 0 ;;
  --status)
    echo "잠금: $(cat "$LOCK" 2>/dev/null || echo '없음')"
    echo "배포: $(tail -n 1 "$DEPLOYED" 2>/dev/null || echo '기록 없음')"
    git status --short
    exit 0 ;;
  --release)
    NAME="${2:?이름을 적으십시오}"
    if [ -f "$LOCK" ] && [ "$(cat "$LOCK")" != "$NAME" ] && [ "${FORCE:-0}" != "1" ]; then
      echo "잠금 주인은 $(cat "$LOCK") 입니다. 본인 이름으로 해제하십시오."; exit 1
    fi
    rm -f "$LOCK"; echo "잠금 해제됨"; exit 0 ;;
esac

NAME="$1"
REF="${2:-main}"

# 1) 잠금
if [ -f "$LOCK" ] && [ "$(cat "$LOCK")" != "$NAME" ] && [ "${FORCE:-0}" != "1" ]; then
  echo "차량 사용 중: $(cat "$LOCK"). 그 사람이 --release 하거나 팀 합의 후 FORCE=1 을 쓰십시오."; exit 1
fi
echo "$NAME" > "$LOCK"

# 2) 차량에서 직접 고친 파일 (파라미터 튜닝 등)
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "차량에서 고친 파일이 있습니다. 버리지 말고 먼저 브랜치로 남기십시오:"
  echo "  git switch -c tune/$(date +%m%d)-$NAME && git commit -am '차량 튜닝' && git push -u origin HEAD"
  git status --short
  exit 1
fi

# 3) 받기 (대회장처럼 인터넷이 없으면 실패하고, 노트북에서 'git push car' 한 로컬 브랜치를 씁니다)
TARGET=""
if timeout 15 git fetch --prune --tags origin 2>/dev/null; then
  if git rev-parse --verify -q "origin/$REF^{commit}" >/dev/null; then TARGET="origin/$REF"; fi
else
  echo "[오프라인] GitHub에 닿지 않습니다. 로컬 ref '$REF'를 씁니다."
fi
[ -z "$TARGET" ] && TARGET="$REF"
COMMIT="$(git rev-parse --verify "$TARGET^{commit}")"

# 4) 체크아웃 (detach: 노트북에서 'git push car <브랜치>'가 막히지 않게)
git checkout -q --detach "$COMMIT"

# 5) 빌드
set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/wego_ws/install/setup.bash" ] && source "$HOME/wego_ws/install/setup.bash"
set -u
colcon build --symlink-install

# 6) 기록
LINE="$(date '+%F %T') | $NAME | $REF | $(git log -1 --format='%h %s' "$COMMIT")"
echo "$LINE" >> "$DEPLOYED"
echo "배포 완료: $LINE"
echo "새 터미널에서: source $WS/install/setup.bash"
