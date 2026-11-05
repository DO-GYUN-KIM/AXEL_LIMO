# Isaac Sim 준비 안내 (팀 데스크탑, 애커먼 주행)

작성 2026-11-04 · 대상: 팀 데스크탑(Ubuntu 22.04, RAM 64 GB, RTX 3070 8 GB, ROS2 Humble)

## 요약

- **목표**: 실차 없이 M7(3차선 정차 LIMO 회피), M8(라바콘) 같은 미션의 **상태 흐름**을 데스크탑에서 돌려 봅니다. 수치(조향 시간, 트리거 거리) 맞춤은 실차에서만 합니다.
- **방식**: 주최 측(WeGo)이 만든 Isaac Sim 4.5 LIMO 예제에, 저장소에 새로 넣은 **애커먼 예제 `limo_ros2_ackermann`** 을 얹습니다. 차가 받는 명령 경로를 실차와 같게 맞췄습니다: `/cmd_vel` → (변환 노드) → `/limo/ack_cmd` → (시뮬레이터 전용 중계) → Isaac Sim.
- **일정과 중단 기준**
  - 11/05: 설치, 주최 측 차동 예제로 센서 토픽 확인 (아래 1~3단계)
  - 11/06: 애커먼 예제 주행, 우리 노드 연결 (4·6단계). 대회 트랙·장애물·라바콘 배치는 11/05에 미리 만들어 둠(5단계)
  - **11/06 저녁까지 4단계(애커먼으로 좌우 회전 주행)가 안 되면 Gazebo로 넘어갑니다.** 기준은 아래 "합격 기준"에 있습니다.
- **주의**: 저장소에 넣은 애커먼 예제와 도구는 Isaac Sim이 없는 환경에서 작성했습니다. 코드 문법과 트랙 변환 도구(가짜 도면으로 시험)만 확인했고, **Isaac Sim 안에서 실행해 본 것은 아닙니다.** 4단계의 점검 순서는 이 때문에 있습니다.

## 0. 시작 전에 준비할 것

| 항목 | 상태 | 비고 |
|---|---|---|
| Ubuntu 22.04 | 설치됨 | |
| RAM 64 GB | 됨 | 최소 32 GB, 권장 64 GB (NVIDIA 기준) |
| RTX 3070 8 GB | 최소 사양 | 무거운 장면은 느릴 수 있음. 아래 7장 참고 |
| SSD 빈 공간 | **확인 필요** | 50 GB 이상 (Isaac Sim + 캐시) |
| NVIDIA 드라이버 | **확인 필요** | 535.129.03 이상 (Ubuntu 22.04.5에서는 535.216.01 이상 권장) |
| ROS2 Humble | **확인 필요** | 없으면 1-2단계에서 설치 |
| 인터넷 | 필요 | Isaac Sim 내려받기 약 수 GB |
| 대회 트랙 도면 DXF | 필요 없음 | 트랙 파일을 미리 만들어 저장소에 넣어 둠(5단계) |
| 저장소 | | `git clone https://github.com/DO-GYUN-KIM/AXEL_LIMO ~/AXEL_LIMO` |
| 조이스틱 | 없어도 됨 | 키보드 조종(`teleop_twist_keyboard`)으로 대신 |

**실차 오작동 주의**: 실차와 데스크탑이 같은 와이파이에 있고 `ROS_DOMAIN_ID`가 같으면, 시뮬레이터용 주행 명령(`/limo/ack_cmd`)을 **실차가 받아 움직일 수 있습니다.** 데스크탑은 `ROS_DOMAIN_ID=30`(WeGo 기본값)을 쓰고, 실차는 다른 번호인지 확인하십시오.

## 1. 기본 설치 (11/05 오전, 약 2시간)

### 1-1. 드라이버 확인

```bash
nvidia-smi        # 표 오른쪽 위 "Driver Version"이 535.129.03 이상이면 통과
```

낮거나 명령이 없으면: `소프트웨어 & 업데이트` → `추가 드라이버` 탭에서 `nvidia-driver-535`(또는 그 이상, "proprietary, tested") 선택 → 재부팅.

### 1-2. ROS2 Humble과 필요한 패키지

ROS2가 없으면 공식 안내(docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)대로 `ros-humble-desktop`을 먼저 설치합니다. 그다음:

```bash
sudo apt update
sudo apt install -y ros-humble-ackermann-msgs ros-humble-vision-msgs \
  ros-humble-rmw-cyclonedds-cpp ros-humble-image-transport-plugins \
  ros-humble-teleop-twist-keyboard python3-colcon-common-extensions
```

위키 23장의 명령은 `ackeramann`으로 철자가 틀려 있어 그대로 치면 실패합니다. 위 명령을 쓰십시오.

### 1-3. 터미널 설정 (`~/.bashrc` 맨 끝에 추가)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

저장 뒤 `source ~/.bashrc`. **Isaac Sim도 이 설정이 된 터미널에서 실행해야** ROS와 연결됩니다.

### 1-4. Isaac Sim 4.5 설치

1. NVIDIA 내려받기 페이지(docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)에서 **Linux용 4.5.0 standalone zip**을 받습니다.
2. 풀고 설치합니다.
   ```bash
   mkdir -p ~/isaacsim
   cd ~/Downloads
   unzip "isaac-sim-standalone@4.5.0-*.zip" -d ~/isaacsim
   cd ~/isaacsim
   ./post_install.sh
   ls *compat*            # 호환성 점검 스크립트 이름 확인
   ./isaac-sim.compatibility_check.sh   # (이름이 다르면 위에서 나온 이름으로) 빨간 항목이 없어야 함
   ```
3. 실행: `./isaac-sim.sh`
   - **첫 실행은 셰이더 준비 때문에 수 분~십수 분 멈춘 것처럼 보일 수 있습니다**(일반적인 현상, 정확한 시간은 PC마다 다름). 창이 "응답 없음"이어도 기다리십시오.

### 1-5. ROS2 연결 확인 (위키 23장)

1. Isaac Sim 메뉴 `Window → Extensions` → 검색창에 `ROS2` → **ROS2 Bridge**를 Enable, **Autoload** 체크.
2. Isaac Sim을 껐다 켭니다.
3. `Tools → Robotics → ROS2 OmniGraphs → Generic Publisher` → 기본값 그대로 OK → 화면 왼쪽 ▶(Play).
4. 새 터미널: `ros2 topic echo /topic` → 값이 나오면 통과.

## 2. 주최 측 LIMO 예제 설치 (11/05 오후, 약 1시간)

WeGo 저장소는 브랜치가 둘입니다. **Isaac Sim 확장은 `sim` 브랜치**, ROS 쪽 예제는 `ros2` 브랜치입니다(위키 24장은 브랜치를 안 적어서 기본 브랜치 `sim`이 받아집니다).

```bash
cd ~/Documents
git clone -b sim https://github.com/WeGo-Robotics/LimoIsaacSIM.git
# LIMO 모델·트랙 파일 (위키 첨부 WegoLimoUSD.zip, 65 MB. 프로젝트 폴더 autorace-prep/위키/첨부/ 에도 있음)
unzip WegoLimoUSD.zip -d ~/Documents/LimoIsaacSIM/USD/
ls ~/Documents/LimoIsaacSIM/USD/WegoLimo/Limo/     # limo_ackermann.usd, RPLidar_S2e.usd 등이 보여야 함
```

Isaac Sim에서:
1. `Window → Extensions` → 오른쪽 위 ☰ → **Settings** → Extension Search Paths의 **+** → `/home/<사용자>/Documents/LimoIsaacSIM` 입력.
2. 왼쪽 목록 **THIRD PARTY**에서 `limo_ros2_differential`, `limo_ros2_differential_obs`를 Enable.
3. `Window → Examples → Robotics Examples` → **Wego → Limo ROS2 Differential** → **Load** → ■(정지) 후 ▶(Play).
4. 터미널에서 `ros2 topic list` → `/sim/scan`, `/sim/odom`, `/sim/camera/color/image_raw`, `/clock`이 보이면 **11/05 목표 달성**.
5. (선택) 차동 예제를 키보드로 움직여 보기: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/sim/cmd_vel`

## 3. 애커먼 예제 넣기 (11/06 오전)

우리 저장소 `sim/isaac_sim/`에 다음을 넣어 두었습니다.

| 파일 | 하는 일 |
|---|---|
| `limo_ros2_ackermann/` | Isaac Sim 확장. 주최 측 차동 예제를 복사해 **애커먼 모델(`limo_ackermann.usd`)과 애커먼 제어 그래프**로 바꾼 것 |
| `limo_ros2_ackermann/.../scenario.json` | 트랙 파일, 출발 위치, 정차 LIMO(상자)·라바콘 위치. **코드 수정 없이 이 파일만 고쳐 배치를 바꿉니다** |
| `tools/ack_stamp_relay.py` | `/limo/ack_cmd`(실차 형식) → `/sim/ackermann_cmd`(Isaac 형식) 중계 |
| `tools/dxf_to_usd.py` | 대회 도면 DXF → Isaac용 바닥 트랙 USD (이미 만들어 둔 결과가 `track/`에 있음) |
| `launch/isaac_sim.launch.py` | 중계 + 카메라 압축 + 변환 노드 + 우리 노드를 한 번에 실행 |

### 애커먼 예제가 주최 측 차동 예제와 다른 점 (모델 파일을 직접 열어 확인한 값)

- 축간거리 0.20 m, 좌우 바퀴 간격 0.14 m, 바퀴 반지름 0.045 m. 조향 한계는 실차 값 ±0.42 rad로 넣었습니다(모델 조인트 한계는 ±30°).
- 애커먼 모델에는 **관절 묶음의 기준점(ArticulationRoot)이 두 곳**에 잘못 들어 있습니다. 주최 측이 고친 차동 모델은 `base_link` 한 곳이라, 예제가 불러올 때 `base_footprint` 쪽을 지웁니다.
- 애커먼 모델의 라이다는 구형 방식이라 차동 예제와 같은 RTX 라이다(`RPLidar_S2e.usd`)를 붙였습니다. 이 라이다는 360°이고 실차는 앞쪽 약 220°입니다. 우리 `lidar_processor`는 앞쪽 부채꼴만 보므로 영향이 작을 것으로 봅니다(추정).
- 토픽 이름을 실차와 같게 했습니다: `/scan`, `/odom`, `/camera/color/image_raw`(→ launch가 `/camera/color/image_raw/compressed`로 압축). 그래서 우리 노드 설정을 바꿀 필요가 없습니다.
- Isaac의 애커먼 구독 노드는 헤더가 있는 `AckermannDriveStamped`만 받습니다. 실차 드라이버는 헤더 없는 `AckermannDrive`를 받으므로 중계 노드를 하나 끼웠습니다(NVIDIA 문서 확인).
- IMU 그래프는 뺐습니다(애커먼 모델에 IMU 센서가 없고, 우리 코드는 IMU를 안 씀).

### 설치

```bash
cd ~/AXEL_LIMO && git pull
# 확장은 WeGo 폴더 안에 있어야 모델 경로(../../USD/WegoLimo)가 맞습니다
ln -s ~/AXEL_LIMO/sim/isaac_sim/limo_ros2_ackermann ~/Documents/LimoIsaacSIM/limo_ros2_ackermann
```

- 복사(`cp -r`) 대신 링크(`ln -s`)를 쓰면 `git pull`만으로 최신이 됩니다.
- **변환 노드 `cmd_vel_to_ackermann`은 저장소 PR #1에 들어 있습니다.** PR #1이 main에 합쳐진 뒤 `colcon build` 하십시오. 그 전에는 해당 브랜치를 받아 빌드합니다.
  ```bash
  cd ~/AXEL_LIMO && colcon build --symlink-install && source install/setup.bash
  ```

Isaac Sim을 다시 켜고 `Window → Extensions → THIRD PARTY`에서 `limo_ros2_ackermann` Enable → `Window → Examples → Robotics Examples → Wego → Limo ROS2 Ackermann (AXEL)` → Load.

## 4. 첫 실행 점검 (11/06 오전, 이 단계가 합격 기준)

순서대로 확인하고, 막히면 9장 "문제 해결"을 보십시오.

1. **Load 후 오류 확인**: Isaac Sim 아래 `Console` 탭에 빨간 글자(특히 `articulation`, `OmniGraph 구성 실패`)가 없어야 합니다.
2. **▶ Play 후 토픽 확인**
   ```bash
   ros2 topic list                    # /scan /odom /camera/color/image_raw /clock /sim/ackermann_cmd 가 보여야 함
   ros2 topic hz /scan                # 숫자가 계속 나오면 라이다 정상
   ```
3. **중계·변환만 띄우고 키보드로 조종**
   ```bash
   # 터미널 1
   source ~/AXEL_LIMO/install/setup.bash
   ros2 launch ~/AXEL_LIMO/sim/isaac_sim/launch/isaac_sim.launch.py use_nodes:=false
   # 터미널 2
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   `i` 앞으로, `j`/`l` 회전, `u`/`o` 앞으로 가며 좌우, `k` 정지. 처음엔 `z`를 몇 번 눌러 속도를 0.2~0.3 m/s로 낮춥니다.
4. **방향 확인 (두 가지)**
   - `i`를 눌렀는데 **뒤로 가면**: launch에 `speed_sign:=-1.0`을 붙여 다시 실행합니다.
   - `u`(왼쪽 앞)를 눌렀는데 **오른쪽으로 꺾이면**: `scenario.json`의 `"invert_steering": true`로 바꾸고 예제를 다시 Load 합니다.
5. **회전 반경 확인**: 최대 조향(0.42 rad)일 때 계산상 회전 반경은 0.2 / tan(0.42) ≈ **0.45 m**입니다(위키의 실차 최소 회전 반경은 0.4 m). `ros2 topic echo /odom`으로 한 바퀴 돌며 대략 맞는지 봅니다.

**합격 기준(11/06 저녁)**: 좌·우 모두 회전하며 앞으로 달리고, `/scan`에 장애물이 보이면 통과. 이게 안 되면 Isaac Sim은 접고 `LIMO_시뮬레이터_검토.md`의 Gazebo 방법으로 넘어갑니다.

## 5. 대회 트랙과 장애물 배치 (11/05에 미리 만들어 둠)

대회 트랙과 배치 파일은 이미 저장소에 들어 있습니다(2026-11-05, 사용자 노트북의 도면으로 작성). **11/06에는 `git pull`만 하면 됩니다.**

| 파일 | 내용 |
|---|---|
| `limo_ros2_ackermann_python/track/autorace2026.usda` | 대회 트랙 바닥 (10 x 6 m, 충돌 바닥 포함) |
| `limo_ros2_ackermann_python/track/autorace2026_floor.png` | 바닥 그림 (1픽셀 = 2 mm). 테이프 색은 주최 측 그림(2026Track.png)에서 가져옴 |
| `limo_ros2_ackermann_python/scenario.json` | 출발 위치 4곳, 3차선 정차 LIMO 7대, 라바콘 8개, 주차 칸 양옆 LIMO 2대 |

### 도면에서 확인한 사실

- 도면(DXF)은 **1:30 축척**입니다(도면 1단위 = 30 mm). 보드 전체 333.33 x 200 단위 = 10 x 6 m로 확인했습니다.
- 도면은 테이프를 윤곽선 두 줄로 그렸습니다. 변환 도구가 테이프 안쪽을 칠해 실제 노면처럼 만듭니다. 테이프 폭은 도면 기준 약 20 mm입니다.
- 좌표: 보드 왼쪽 아래 모서리 = (0, 0), x 오른쪽, y 위쪽, 단위 m. 진행은 시계 방향(위쪽 띠에서 오른쪽으로 출발).
- 3차선 구간: x 4.30~8.36 m, 차로 중심 y = 0.42(1차로, A쪽) / 0.94(2차로, 가운데) / 1.46(3차로, B쪽), 차로 폭 0.52 m. 진행 방향 −x. 2차로로 들어옵니다.
- 라바콘 구간: 왼쪽 직선, x 중심 0.72 m(폭 0.52 m), y 1.84~3.94 m(흰 선 사이), 진행 방향 +y.
- 주차 칸: 3 x 2칸, 칸 하나 약 0.53 x 0.63 m. 위쪽 줄 칸 중심 y 4.27, x 6.36 / 6.88 / 7.40.

### 배치는 추정입니다

규정집과 미션 상세 문서에는 **정차 LIMO 7대·라바콘의 정확한 위치가 없습니다**("주최 측이 배치", "수량·간격은 확정 필요"). 그래서 `scenario.json`의 배치는 연습용 예시입니다.
- M7: 2차로 1대 → 1·3차로 2대 → 2차로 1대 → 1·3차로 2대 → 2차로 1대 (x 7.8 / 7.0 / 6.2 / 5.4 / 4.7 m). 차가 지그재그로 한 차로씩 비켜 가야 합니다.
- M8: 콘 두 개로 문을 만들고(문 폭 0.4 m) 좌우로 엇갈리게 4개 문, 8개.
- M4: 위쪽 줄 가운데 칸을 목표로 가정하고 양옆 칸에 LIMO.
- 라바콘 크기(밑면 반지름 5 cm, 높이 20 cm)도 추정입니다. 실물을 재서 `radius`, `height`를 고치십시오.

### 출발 위치 바꾸기

`scenario.json`의 `"robot_start"`에 이름을 넣습니다: `"start"`(출발선), `"m7"`(3차선 진입 직전), `"m8"`(라바콘 구간 직전), `"parking"`(주차 구역 앞). 고친 뒤 예제를 다시 Load 합니다.

### 도면이 바뀌면 다시 만들기

주최 측이 도면을 새로 주면, 도면 zip 안의 DXF와 2026Track.png로 다시 만듭니다.

```bash
python3 -m pip install --user ezdxf usd-core numpy scipy pillow
cd ~/AXEL_LIMO/sim/isaac_sim/tools
python3 dxf_to_usd.py "2026Track 도면 시트1.dxf" --unit 0.03 --ref-png 2026Track.png \
        -o ../limo_ros2_ackermann/limo_ros2_ackermann_python/track/autorace2026.usda
```

- 끝나면 보드 크기가 출력됩니다. **10 x 6 m가 아니면 `--unit`이 틀린 것**입니다.
- 원본 도면은 저장소에 넣지 않았습니다(사용자 노트북 `D:\도균\AutoRace`에 있음).

## 6. 우리 노드 연결 (11/06 오후)

```bash
source ~/AXEL_LIMO/install/setup.bash
ros2 launch ~/AXEL_LIMO/sim/isaac_sim/launch/isaac_sim.launch.py start_mission:=m7_obstacles
# 라바콘: start_mission:=m8_cones
# 상태 보기: ros2 topic echo /autorace/mission_state
```

- `use_sim_time`은 launch가 켭니다. Isaac Sim에서 ▶가 눌려 있어야 시간(`/clock`)이 흐릅니다.
- YOLO는 기본으로 껐습니다(`use_yolo:=false`). 그래픽 메모리 8 GB를 Isaac Sim과 나눠 쓰면 둘 다 느려집니다.
- 시뮬레이터 카메라 영상은 실제 트랙의 조명·바닥과 다릅니다. **차선 인식 값(HLS 문턱 등)은 시뮬레이터에서 맞추지 마십시오.** 차선 인식이 불안하면 M7/M8 구간만 따로 시험합니다.

## 7. 그래픽 메모리 나눠 쓰기 (YOLO 학습과)

- RTX 3070의 8 GB는 Isaac Sim 혼자서도 상당 부분을 씁니다(정확한 양은 장면마다 다름, 추정). **YOLO 학습과 Isaac Sim은 동시에 돌리지 않습니다.**
- 시작 전 `nvidia-smi`로 다른 프로그램이 그래픽 메모리를 쓰고 있지 않은지 봅니다.
- 권장 배분: 낮(사람이 보는 시간) Isaac Sim, 밤에는 YOLO 학습을 걸어 둡니다. 지금은 YOLO를 미뤘으므로 11/05~06은 Isaac Sim이 데스크탑을 씁니다.
- 느릴 때: 카메라가 필요 없는 시험(M7/M8)에서는 Isaac Sim 창의 렌더 모드를 낮추거나, `scenario.json`의 `camera_width`/`camera_height`를 320×240으로 줄입니다.

## 8. 시뮬레이터로 할 것과 하지 않을 것

| 시뮬레이터에서 | 실차에서만 |
|---|---|
| 미션 순서·전환 조건, 차로 선택 논리, 타임아웃 | 조향 시간·각속도·트리거 거리 같은 수치 맞춤 |
| 장애물 7대 배치를 여러 가지로 바꿔 보기 | 실제 라이다 잡음·빠진 점, 콘 반사 |
| 변환 노드(`/cmd_vel`→`/limo/ack_cmd`) 동작 | 실제 서보 지연, 미끄러짐 |
| 노드가 죽거나 명령이 끊겼을 때 차가 서는지 | 차선 인식 문턱값, YOLO 인식률 |

## 9. 문제 해결

| 증상 | 확인할 것 |
|---|---|
| `ros2 topic list`에 Isaac 토픽이 없음 | Isaac Sim을 1-3단계 설정이 된 터미널에서 켰는지, 두 터미널의 `ROS_DOMAIN_ID`가 같은지, ▶가 눌렸는지 |
| Examples에 Wego가 안 보임 | Extension Search Path가 `LimoIsaacSIM` 폴더(그 안의 하위 폴더 아님)인지, THIRD PARTY에서 Enable 했는지 |
| 차가 전혀 안 움직임 | `ros2 topic echo /sim/ackermann_cmd`에 값이 오는지 → 안 오면 중계 노드, 오면 Console 오류 확인 |
| 차가 뒤로 감 / 반대로 꺾임 | 4단계 4번 (`speed_sign`, `invert_steering`) |
| 바퀴가 미끄러지거나 차가 튐 | 조향 관절 힘이 약할 수 있음. Stage 창에서 `left/right_steering_hinge_wheel` 조인트의 Drive stiffness를 올려 봄(추정 원인) |
| Load 시 articulation 관련 오류 | 모델의 기준점 정리가 안 된 것. Stage에서 `/World/Limo/base_footprint`의 Physics → Articulation Root를 직접 지움 |
| 화면이 매우 느림 | 7장. 다른 GPU 프로그램 종료, 트랙을 구간만 사용 |
| 첫 실행이 오래 멈춤 | 정상. 셰이더 준비 중 |

## 10. 모르는 것·추정

- 애커먼 제어 노드의 바퀴 순서(뒤 왼쪽·뒤 오른쪽·앞 왼쪽·앞 오른쪽)는 NVIDIA 튜토리얼의 연결 순서를 따랐습니다. 문서에 순서가 명시돼 있지 않아 4단계에서 눈으로 확인해야 합니다.
- 최대 바퀴 속도 30 rad/s, 조향 속도 2 rad/s는 추정값입니다. 실차 영상과 비교해 고칩니다.
- 라바콘 크기, 테이프 폭, 3차선 장애물 정확한 배치는 실측·규정집으로 채워야 합니다.
- 실차와 데스크탑의 `ROS_DOMAIN_ID`(실차 쪽 값은 아직 확인 안 됨).

## 근거

- Limo 위키 23~26장 (`autorace-prep/위키/00_Limo_Wiki/23~26`)
- WeGo `LimoIsaacSIM` sim 브랜치 `limo_ros2_diff.py` (그래프 구성 방식)
- `WegoLimoUSD.zip`의 `limo_ackermann.usd`, `limo_diff_thin.usd`를 usd-core로 직접 열어 확인한 조인트·치수·센서
- NVIDIA Isaac Sim 4.5 문서: 설치 요구 사양, ROS2 Ackermann Controller 튜토리얼, `AckermannController`·`ROS2SubscribeAckermannDrive` 노드 설명
- 조사 메모 `autorace-prep/위키/IsaacSim_애커먼_조사메모.md`
