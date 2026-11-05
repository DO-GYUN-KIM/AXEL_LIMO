# 팀 노트북(Windows)에서 한 번 실행합니다. 차량 SSH 키 등록, 'limo' 접속 별칭, 차량 설정 스크립트 전송, git 'car' 원격을 설정합니다.
# 여러 번 실행해도 안전합니다.
#
# 실행 (PowerShell, 저장소 폴더에서):
#   powershell -ExecutionPolicy Bypass -File tools\setup\laptop_setup.ps1 -CarIp 192.168.0.50 -Name 도균
# 처음 한 번은 차량 비밀번호(wego)를 1~2번 묻습니다.
param(
  [Parameter(Mandatory = $true)][string]$CarIp,
  [Parameter(Mandatory = $true)][string]$Name,
  [string]$CarUser = "wego"
)
$ErrorActionPreference = "Stop"
function Ok($m)   { Write-Host "  [OK]   $m" -ForegroundColor Green }
function Warn($m) { Write-Host "  [주의] $m" -ForegroundColor Yellow }
function Hdr($m)  { Write-Host ""; Write-Host "== $m ==" }

Hdr "0. 준비물 확인"
foreach ($c in @("ssh", "scp", "ssh-keygen", "git")) {
  if (-not (Get-Command $c -ErrorAction SilentlyContinue)) {
    throw "$c 이(가) 없습니다. Windows 설정 → 앱 → 선택적 기능 → 'OpenSSH 클라이언트' 추가, git은 git-scm.com에서 설치하십시오."
  }
}
Ok "ssh, scp, git 있음"
if (-not (Test-Connection -ComputerName $CarIp -Count 1 -Quiet)) { Warn "$CarIp 에 ping이 안 됩니다. 같은 공유기에 연결됐는지 확인하십시오." }

Hdr "1. 내 노트북 SSH 키"
$sshDir = Join-Path $env:USERPROFILE ".ssh"
New-Item -ItemType Directory -Force -Path $sshDir | Out-Null
$key = Join-Path $sshDir "id_ed25519"
if (-not (Test-Path $key)) {
  Write-Host "  키를 만듭니다. 암호(passphrase)를 묻으면 그냥 엔터 두 번 누르십시오."
  ssh-keygen -t ed25519 -C "$Name-laptop" -f $key
}
Ok "키: $key.pub"

Hdr "2. 'limo' 접속 별칭"
$cfg = Join-Path $sshDir "config"
$block = @"

Host limo
    HostName $CarIp
    User $CarUser
    ServerAliveInterval 15
    ServerAliveCountMax 4
"@
if ((Test-Path $cfg) -and (Select-String -Path $cfg -Pattern "^\s*Host\s+limo\s*$" -Quiet)) {
  Warn "이미 'Host limo'가 있습니다. IP가 바뀌었으면 $cfg 에서 HostName을 고치십시오."
} else {
  Add-Content -Path $cfg -Value $block -Encoding ascii
  Ok "추가함: ssh limo → $CarUser@$CarIp"
}

Hdr "3. 차량에 내 키 등록 (비밀번호를 물으면 wego 비밀번호 입력)"
$pub = (Get-Content "$key.pub" -Raw).Trim()
$remote = "mkdir -p ~/.ssh && chmod 700 ~/.ssh && touch ~/.ssh/authorized_keys && grep -qxF '$pub' ~/.ssh/authorized_keys || echo '$pub' >> ~/.ssh/authorized_keys; chmod 600 ~/.ssh/authorized_keys"
ssh -o StrictHostKeyChecking=accept-new limo $remote
$test = ssh -o BatchMode=yes limo "echo connected"
if ($test -eq "connected") { Ok "비밀번호 없이 'ssh limo' 접속됨" } else { Warn "아직 비밀번호를 묻습니다. 출력 오류를 Claude에 붙여 주십시오." }

Hdr "4. 차량 설정 스크립트 보내기"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
scp (Join-Path $here "car_setup.sh") limo:car_setup.sh
Ok "보냄 → 차량 ~/car_setup.sh  (실행: ssh limo 후 'bash ~/car_setup.sh check')"

Hdr "5. git 'car' 원격 (대회장 오프라인 배포용)"
$repo = Resolve-Path (Join-Path $here "..\..")
Push-Location $repo
try {
  $remotes = git remote
  if ($remotes -contains "car") { Ok "'car' 원격 이미 있음" }
  else { git remote add car "limo:axel_ws"; Ok "추가함: git push car <브랜치>" }
} finally { Pop-Location }

Write-Host ""
Write-Host "끝. 다음: ssh limo 접속 → bash ~/car_setup.sh check 결과를 팀 채팅에 붙이기"
