#!/usr/bin/env bash
# 차량(Jetson)에서 실행하는 원격 개발 환경 설정 스크립트입니다. JetPack·WeGo 이미지 설치가 끝난 직후 씁니다.
# 여러 번 실행해도 안전합니다(이미 된 단계는 건너뜀).
#
# 사용법 (차량 터미널 또는 노트북에서 ssh 접속 후)
#   bash car_setup.sh check              읽기만 함: 버전·네트워크·ROS 설정 점검 결과를 출력
#   bash car_setup.sh install            SSH 서버, tmux, foxglove_bridge, mcap 녹화 플러그인 설치 (sudo 필요)
#   bash car_setup.sh bashrc <팀번호>     ~/.bashrc에 ROS 설정 블록 추가 (예: bashrc 30)
#   bash car_setup.sh github             차량 전용 GitHub 키 생성 → 출력된 키를 GitHub에 등록
#   bash car_setup.sh clone              저장소를 ~/axel_ws로 받고 첫 빌드
#   bash car_setup.sh claude             Claude Code 설치 (인터넷 필요)
#   bash car_setup.sh harden             비밀번호 로그인 끄기 (팀원 키 등록을 끝낸 뒤에만)
#   bash car_setup.sh all <팀번호>        check → install → bashrc → github (여기서 멈추고 키 등록 안내)
set -uo pipefail

REPO_SSH="git@github.com:DO-GYUN-KIM/AXEL_LIMO.git"
WS="${AXEL_WS:-$HOME/axel_ws}"
MARK_BEGIN="# >>> AXEL autorace >>>"
MARK_END="# <<< AXEL autorace <<<"

ok()   { echo "  [OK]   $*"; }
warn() { echo "  [주의] $*"; }
bad()  { echo "  [문제] $*"; }
hdr()  { echo; echo "== $* =="; }

cmd_check() {
  hdr "1. OS·JetPack"
  . /etc/os-release 2>/dev/null && { [ "${VERSION_ID:-}" = "22.04" ] && ok "Ubuntu $VERSION_ID" || bad "Ubuntu ${VERSION_ID:-?} (22.04가 아님)"; }
  if [ -f /etc/nv_tegra_release ]; then ok "L4T: $(head -n1 /etc/nv_tegra_release | cut -c1-60)"; else warn "/etc/nv_tegra_release 없음 (Jetson이 아니거나 L4T 정보 없음)"; fi
  ls -d /usr/local/cuda* >/dev/null 2>&1 && ok "CUDA: $(ls -d /usr/local/cuda-* 2>/dev/null | head -n1)" || warn "CUDA 폴더 없음"
  dpkg -l 2>/dev/null | grep -qi tensorrt && ok "TensorRT 설치됨" || warn "TensorRT 패키지 안 보임"

  hdr "2. ROS2·WeGo"
  if [ -f /opt/ros/humble/setup.bash ]; then ok "ROS2 Humble 있음"; else bad "/opt/ros/humble 없음 → WeGo 이미지(또는 ROS2 Humble) 설치가 먼저입니다. 여기서 멈추십시오."; fi
  if [ -d "$HOME/wego_ws" ]; then ok "~/wego_ws 있음"; else bad "~/wego_ws 없음 → 주최 측 드라이버 작업공간이 필요합니다(백업본 복원 또는 WeGo 이미지)"; fi
  [ -f "$HOME/wego_ws/install/setup.bash" ] && ok "~/wego_ws 빌드됨" || warn "~/wego_ws/install 없음 (빌드 필요할 수 있음)"
  echo "  ~/.bashrc의 ROS 관련 줄:"
  grep -n -E "ROS_LOCALHOST_ONLY|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|setup.bash" "$HOME/.bashrc" | sed 's/^/     /' || echo "     (없음)"

  hdr "3. 네트워크"
  ip -br addr | grep -v '^lo' | sed 's/^/     /'
  iw dev 2>/dev/null | grep -q Interface && { iw list 2>/dev/null | grep -qE "\b5[0-9]{3}(\.0)? MHz" && ok "무선이 5 GHz 대역 지원" || warn "5 GHz 지원 여부 확인 못 함"; }
  nmcli -t -f NAME,TYPE,AUTOCONNECT-PRIORITY con show 2>/dev/null | grep wireless | sed 's/^/     저장된 와이파이: /'
  ping -c1 -W2 github.com >/dev/null 2>&1 && ok "인터넷(github.com) 연결됨" || warn "인터넷 안 됨 (install·github·claude 단계는 인터넷 필요)"

  hdr "4. 원격 접속"
  systemctl is-active --quiet ssh && ok "SSH 서버 실행 중" || warn "SSH 서버 꺼짐 또는 미설치 → install 단계"
  [ -s "$HOME/.ssh/authorized_keys" ] && ok "등록된 노트북 키 $(grep -c . "$HOME/.ssh/authorized_keys")개" || warn "등록된 노트북 키 없음 (노트북 스크립트가 넣습니다)"
  grep -qE '^\s*PasswordAuthentication\s+no' /etc/ssh/sshd_config 2>/dev/null && ok "비밀번호 로그인 꺼짐" || warn "비밀번호 로그인 켜짐 (키 등록 후 harden 단계)"
  pgrep -f nxserver >/dev/null 2>&1 && ok "NoMachine 실행 중" || warn "NoMachine 프로세스 안 보임"

  hdr "5. 저장 공간·시간"
  df -h "$HOME" | tail -n1 | awk '{print "     남은 공간: "$4" (20GB 이상 권장)"}'
  timedatectl 2>/dev/null | grep -E "Local time|synchronized" | sed 's/^/    /'

  hdr "6. 작업공간"
  [ -d "$WS/.git" ] && ok "$WS 있음 ($(git -C "$WS" log -1 --format='%h %s' 2>/dev/null))" || warn "$WS 없음 → github, clone 단계"
  command -v claude >/dev/null && ok "Claude Code 설치됨" || warn "Claude Code 미설치 (claude 단계, 선택)"
  echo
  echo "이 출력 전체를 팀 채팅이나 Claude에 붙여 주십시오."
}

cmd_install() {
  hdr "패키지 설치 (sudo 비밀번호를 물으면 입력)"
  [ -f /opt/ros/humble/setup.bash ] || { bad "ROS2 Humble이 없어 중단합니다."; return 1; }
  sudo apt-get update
  sudo apt-get install -y openssh-server tmux git curl \
       ros-humble-foxglove-bridge ros-humble-rosbag2-storage-mcap || { bad "설치 실패. 출력의 오류를 확인하십시오."; return 1; }
  sudo systemctl enable --now ssh
  ok "설치 끝. 'apt upgrade'는 하지 마십시오(JetPack·드라이버가 바뀔 수 있음)."
}

cmd_bashrc() {
  local DOMAIN="${1:-}"
  [ -n "$DOMAIN" ] || { bad "팀 번호가 필요합니다. 예) bash car_setup.sh bashrc 30"; return 1; }
  hdr "~/.bashrc 설정 (ROS_DOMAIN_ID=$DOMAIN)"
  cp "$HOME/.bashrc" "$HOME/.bashrc.bak.$(date +%Y%m%d_%H%M%S)"
  # 기존 블록 제거 후 다시 씀
  sed -i "/$MARK_BEGIN/,/$MARK_END/d" "$HOME/.bashrc"
  if grep -nE '^\s*export\s+ROS_DOMAIN_ID' "$HOME/.bashrc" >/dev/null; then
    warn "기존 ROS_DOMAIN_ID 줄이 있어 주석 처리합니다(백업 있음)."
    sed -i -E 's/^(\s*export\s+ROS_DOMAIN_ID.*)$/# [AXEL 주석 처리] \1/' "$HOME/.bashrc"
  fi
  cat >> "$HOME/.bashrc" <<EOF
$MARK_BEGIN
# car_setup.sh가 넣은 블록입니다. 고칠 때는 이 블록만 고치십시오.
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f \$HOME/wego_ws/install/setup.bash ] && source \$HOME/wego_ws/install/setup.bash
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN
alias deploy='bash $WS/tools/car_deploy.sh'
alias bag='bash $WS/src/autorace_bringup/scripts/record_bag.sh'
alias viz='ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
$MARK_END
EOF
  ok "추가했습니다. ROS_LOCALHOST_ONLY 줄은 그대로 두었습니다(Foxglove 방식이면 남겨도 됨)."
  grep -n ROS_LOCALHOST_ONLY "$HOME/.bashrc" | sed 's/^/     현재: /'
  echo "  새 터미널을 열거나 'source ~/.bashrc' 하십시오."
}

cmd_github() {
  hdr "차량 전용 GitHub 키"
  mkdir -p "$HOME/.ssh"; chmod 700 "$HOME/.ssh"
  [ -f "$HOME/.ssh/id_github" ] || ssh-keygen -t ed25519 -C "axel-limo" -f "$HOME/.ssh/id_github" -N "" -q
  if ! grep -q "IdentityFile ~/.ssh/id_github" "$HOME/.ssh/config" 2>/dev/null; then
    printf '\nHost github.com\n    IdentityFile ~/.ssh/id_github\n    IdentitiesOnly yes\n' >> "$HOME/.ssh/config"
    chmod 600 "$HOME/.ssh/config"
  fi
  echo
  echo "  아래 한 줄을 복사해 GitHub에 등록하십시오:"
  echo "  저장소 AXEL_LIMO → Settings → Deploy keys → Add deploy key → 제목 'axel-limo' → 'Allow write access' 체크 → Add"
  echo
  cat "$HOME/.ssh/id_github.pub"
  echo
  echo "  등록 후: bash car_setup.sh clone"
}

cmd_clone() {
  hdr "저장소 받기·첫 빌드"
  ssh -o StrictHostKeyChecking=accept-new -T git@github.com 2>&1 | grep -q "successfully authenticated" \
    || { bad "GitHub 인증 실패. github 단계의 키를 Deploy keys에 등록했는지 확인하십시오."; return 1; }
  ok "GitHub 인증됨"
  if [ ! -d "$WS/.git" ]; then git clone "$REPO_SSH" "$WS" || return 1; fi
  git -C "$WS" config user.name "axel-limo"
  git -C "$WS" config user.email "axel-limo@users.noreply.github.com"
  [ -f "$WS/tools/car_deploy.sh" ] || { bad "main에 tools/car_deploy.sh가 없습니다. 원격 환경 PR을 main에 먼저 합치십시오."; return 1; }
  bash "$WS/tools/car_deploy.sh" "${DEPLOY_NAME:-setup}" main && bash "$WS/tools/car_deploy.sh" --release "${DEPLOY_NAME:-setup}"
}

cmd_claude() {
  hdr "Claude Code 설치"
  command -v claude >/dev/null && { ok "이미 설치됨: $(claude --version 2>/dev/null)"; return 0; }
  curl -fsSL https://claude.ai/install.sh | bash || { bad "설치 실패 (인터넷 확인)"; return 1; }
  ok "설치 끝. 로그인: cd $WS && tmux new -s claude && claude"
  echo "  휴대폰·노트북에서 이어서 지시하려면 tmux 안에서: claude remote-control"
}

cmd_harden() {
  hdr "비밀번호 로그인 끄기"
  local n; n=$(grep -c . "$HOME/.ssh/authorized_keys" 2>/dev/null || echo 0)
  echo "  등록된 노트북 키: ${n}개"
  sed 's/^.* \(.*\)$/     - \1/' "$HOME/.ssh/authorized_keys" 2>/dev/null
  [ "$n" -ge 1 ] || { bad "등록된 키가 없어 중단합니다(잠기면 모니터·키보드로만 복구 가능)."; return 1; }
  read -r -p "  3명 모두 'ssh limo'로 비밀번호 없이 접속되는 것을 확인했습니까? (yes 입력) " ans
  [ "$ans" = "yes" ] || { echo "  취소했습니다."; return 0; }
  echo "  먼저 wego 비밀번호를 바꿉니다(NoMachine 접속에 새 비밀번호 사용)."
  passwd
  echo "PasswordAuthentication no" | sudo tee /etc/ssh/sshd_config.d/90-axel.conf >/dev/null
  sudo systemctl restart ssh
  ok "끝. 새 노트북을 추가하려면 /etc/ssh/sshd_config.d/90-axel.conf를 잠시 지우고 ssh를 재시작하십시오."
}

case "${1:-}" in
  check)   cmd_check ;;
  install) cmd_install ;;
  bashrc)  cmd_bashrc "${2:-}" ;;
  github)  cmd_github ;;
  clone)   cmd_clone ;;
  claude)  cmd_claude ;;
  harden)  cmd_harden ;;
  all)     cmd_check; cmd_install && cmd_bashrc "${2:-}" && cmd_github ;;
  *)       sed -n '2,16p' "$0" ;;
esac
