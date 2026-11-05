# Autorace 2026 LIMO 자율주행 코드 뼈대

WeGo-Robotics × 국민대학교 Autorace 2026(본선 2026-11-19, 11-20) 출전용 ROS2 Humble 작업 공간입니다.
완성된 해답이 아니라 **바로 빌드되고 실행되는 출발점**입니다. 각 노드는 동작하는 최소 구현과
`TODO(확인)` 표시가 붙은 임시값으로 되어 있으니, rosbag과 실차로 확인하면서 채워 나가십시오.

- 차량: LIMO Pro 1대, 배터리 1개 → 개발은 대부분 **rosbag 재생으로 오프라인**에서 합니다.
- 트랙: 10 m × 6 m, 차선 폭 550 mm, 시계방향 1바퀴, 1회 10분, 2회 주행
- 속도: 기본 0.3~0.4 m/s, 서행 구간 0.2 m/s 이하
- 감점: 차로 이탈 1회 -1점, 시설물 충돌 1회 -1점
- 미션마다 15초 안에 진입/재출발하지 못하면 그 미션은 실패 → **차가 멈춰 서 있으면 안 됩니다.**
  그래서 모든 미션 핸들러에 타임아웃이 있고, 시간이 지나면 포기하고 다음 미션으로 넘어갑니다.
- 판정 기준점: 앞바퀴 (정지선 ±5 cm 판정도 앞바퀴 기준)

## 1. 패키지 구성

```
autorace_ws/
├── README.md
├── .gitignore
├── models/README.md                  YOLO 가중치 두는 곳 (*.pt는 Git 제외)
├── tools/
│   ├── extract_frames.py             rosbag -> JPEG (YOLO 라벨링용)
│   └── score_sheet.csv               리허설 점수 기록표
└── src/
    ├── autorace_interfaces/          (ament_cmake) 공용 메시지
    │   └── msg/ Detection, DetectionArray, LaneInfo, LidarInfo, MissionState
    ├── autorace_perception/          (ament_python) 인식
    │   ├── lane_detector.py          버드아이 + HSV + 슬라이딩 윈도우 -> LaneInfo, 정지선 스텁
    │   ├── yolo_detector.py          ultralytics YOLO -> DetectionArray
    │   └── lidar_processor.py        전방 최소거리, 3차선 점유, 라바콘 사이 방향 -> LidarInfo
    ├── autorace_control/             (ament_python) 제어
    │   ├── lane_follower.py          LaneInfo -> 조향 (PID / Stanley)
    │   ├── mission_manager.py        상태 머신, 최종 /cmd_vel 출력
    │   └── missions/                 미션 핸들러 (미션당 클래스 1개)
    │       ├── base.py               공통 클래스, StopHold, StopLineApproach
    │       ├── simple.py             START, M5 서행, M6 지그재그, FINISH
    │       ├── stop_line.py          M3 보행자, M1 신호등
    │       ├── lidar_missions.py     M7 3차선 장애물, M8 라바콘
    │       ├── fork.py               M2 갈림길
    │       └── parking.py            M4 T자 주차
    └── autorace_bringup/             (ament_python) 실행/설정
        ├── launch/ robot.launch.py, offline.launch.py, nodes.launch.py
        ├── config/ params.yaml (인식/제어), missions.yaml (미션 순서/미션별 값)
        └── scripts/record_bag.sh
```

### 토픽 흐름

```
/camera/color/image_raw/compressed ─┬─> lane_detector ──> /autorace/lane_info ──┬─> lane_follower ─> /autorace/lane_cmd_vel ─┐
                                    └─> yolo_detector ──> /autorace/detections ─┤                                           │
/scan ─────────────────────────────────> lidar_processor > /autorace/lidar_info ┤                                           │
/odom ──────────────────────────────────────────────────────────────────────────┴──────────> mission_manager <──────────────┘
                                                                                                  ├─> /cmd_vel
                                                                                                  └─> /autorace/mission_state
```

- `/cmd_vel`은 **mission_manager만** 내보냅니다. lane_follower의 출력은 조향(angular) 참고값이고,
  속도는 미션이 정합니다(`base_speed`, 미션별 `speed`, `speed_cap`, `max_linear`).
- 차선 정보가 0.5초 넘게 끊기면(노드가 죽는 등) 안전을 위해 정지합니다.
- 디버깅용 토픽: `/autorace/estop` (Bool, True = 정지), `/autorace/jump_to` (String, 미션 이름으로 이동)

### 미션 순서 (상태 머신 = 트랙 진행 순서)

| 순서 | 이름 (missions.yaml) | 내용 | 배점 | 시작 신호 | 동작 |
|---|---|---|---|---|---|
| 1 | `start` | 출발 | - | 즉시 | `start_delay_s` 대기 |
| 2 | `m5_slow_zone` | 방지턱 서행 | 10 | YOLO `slow` | `section_length_m` 동안 0.15 m/s |
| 3 | `m6_zigzag` | 지그재그 서행 유도선 | 15 | 진입 후 거리 | 직전 헤딩(odom yaw) 유지, 서행 |
| 4 | `m3_crosswalk` | 보행자 표지판 | 10 | YOLO `pedestrian` | 정지선 정지, 3.5 s 대기, 출발 |
| 5 | `m7_obstacles` | 3차선 정적 장애물 | 10 | LiDAR 현재 차로 막힘 | 빈 차로로 S자 차선 변경 |
| 6 | `m2_fork` | 갈림길 A/B | 10 | YOLO `left_A`/`right_B` 투표 | 조향 bias로 갈래 선택 (못 보면 `default_branch`) |
| 7 | `m8_cones` | 라바콘 | 15 | LiDAR 클러스터 | 좌/우 콘 중점으로 조향 |
| 8 | `m1_traffic_light` | 신호등 | 15 | YOLO `red`/`yellow`/`green` | 빨강/노랑 정지선 정지, 초록 확인 후 출발 |
| 9 | `m4_parking` | T자 주차 | 15 | YOLO `parking` (임시) | 전면 진입: 지나치기 → odom으로 90° 회전 → `parking_detector`(칸 선)와 라이다 양옆 거리로 맞추며 진입 → 끝 선 앞 정지 |
| 10 | `finish` | 도착 | - | - | 정지 |

모든 미션은 `approach`(차선 추종하며 시작 신호 대기) → `active`(동작) 두 단계이며,
단계마다 `approach_timeout_s`, `active_timeout_s`가 있습니다. 새 미션을 추가하려면
`missions/`에 `MissionHandler`를 상속한 클래스를 만들고 `missions/__init__.py`의 `MISSION_REGISTRY`에 등록하십시오.

## 2. 빌드

로봇의 `~/wego_ws`(주최 측 드라이버)를 먼저 source 한 다음 이 작업 공간을 덧씌워(overlay) 빌드합니다.

```bash
cd ~
git clone <저장소 주소> autorace_ws
cd ~/autorace_ws
source /opt/ros/humble/setup.bash
source ~/wego_ws/install/setup.bash      # 로봇에서만. 노트북에는 없어도 됩니다.
colcon build --symlink-install
source install/setup.bash
```

- `--symlink-install`이므로 Python 파일과 YAML을 고치면 다시 빌드하지 않아도 반영됩니다.
  (새 파일 추가, `setup.py`나 `.msg` 변경 시에는 다시 빌드합니다.)
- YOLO를 쓰려면 `pip3 install ultralytics`가 필요합니다. 없으면 yolo_detector는 경고만 남기고
  아무것도 내보내지 않으며, 나머지 노드는 그대로 동작합니다.

## 3. 실행

### 실차

```bash
ros2 launch autorace_bringup robot.launch.py                         # 드라이버 + 전체 노드
ros2 launch autorace_bringup robot.launch.py start_mission:=m7_obstacles   # 특정 미션부터
ros2 launch autorace_bringup robot.launch.py use_camera:=false use_lidar:=false  # 드라이버는 따로 띄운 경우
ros2 topic pub --once /autorace/estop std_msgs/Bool "data: true"     # 비상 정지
ros2 topic pub --once /autorace/jump_to std_msgs/String "data: m1_traffic_light"
```

카메라: `ros2 launch orbbec_camera astra_stereo_u3.launch.py`, LiDAR: `ros2 launch ydlidar_ros2_driver ydlidar.launch.py`
를 robot.launch.py가 대신 실행합니다.

### 오프라인 (rosbag 재생)

```bash
# 터미널 1
ros2 launch autorace_bringup offline.launch.py lane_debug:=true start_mission:=start
# 터미널 2
ros2 bag play ~/bags/20261103_101500_lap1 --clock
# 터미널 3 (확인)
ros2 topic echo /autorace/mission_state
ros2 run rqt_image_view rqt_image_view /autorace/lane_debug/compressed
```

- `use_sim_time=true`로 실행되므로 `ros2 bag play`에 반드시 `--clock`을 붙이십시오.
- bag 안에 녹화 당시의 `/cmd_vel`이 있으므로 오프라인에서는 미션 매니저 출력이 `/autorace/cmd_vel_offline`으로 나갑니다.
  두 토픽을 rqt_plot으로 겹쳐 보면 "내 코드라면 어떻게 움직였을지"를 비교할 수 있습니다.
- 오프라인에서는 차가 실제로 움직이지 않으므로 odom(주행거리)은 녹화 당시 값입니다.
  정지 위치 같은 폐루프 동작은 결국 실차에서 확인해야 합니다.

### rosbag 녹화

```bash
bash src/autorace_bringup/scripts/record_bag.sh lap1            # lap: 센서 + 차량 상태 + /limo/ack_cmd
bash src/autorace_bringup/scripts/record_bag.sh m1_test debug   # debug: lap + 우리 노드 출력 /autorace/*
bash src/autorace_bringup/scripts/record_bag.sh signs yolo      # yolo: 카메라만 (학습 사진용)
```

저장 위치는 `~/bags`(`BAG_DIR`로 변경), 형식은 sqlite3(`BAG_STORAGE=mcap`으로 변경)입니다.
미션별 녹화 계획·재생 안전 설정(`ROS_DOMAIN_ID`)·용량 주의는 `docs/rosbag_활용_안내.md`를 보십시오.

### YOLO 라벨링용 프레임 뽑기

```bash
python3 tools/extract_frames.py ~/bags/20261103_101500_lap1 --every 10 --out frames/lap1
```

뽑은 사진을 주최 측 라벨링 도구로 라벨링 → Colab에서 YOLO11n 학습 → `best.pt`를 로봇에 복사 →
`params.yaml`의 `yolo_detector.weights_path`를 지정합니다. 클래스 이름은 아래 임시 목록을 기준으로 맞추십시오.

`red, yellow, green, pedestrian, left_A, right_B, slow, parking` (임시, 라벨링 data.yaml 순서와 같아야 합니다)

## 4. 역할 분담

| 담당 | 범위 | 주로 고치는 파일 |
|---|---|---|
| A | 차선 인식, 차선 추종 제어, 상태 머신, 통합, 정지선 정지 | `lane_detector.py`, `lane_follower.py`, `mission_manager.py`, `missions/base.py`, `simple.py`, `stop_line.py` |
| B | YOLO 데이터 수집/라벨링/학습, 표지판·신호등 검출, M2 갈래 결정, M1 광고 모니터 거르기 | `yolo_detector.py`, `fork.py`, `tools/extract_frames.py`, `models/` |
| C | LiDAR 가공, M7 장애물 회피, M8 라바콘, M4 주차 | `lidar_processor.py`, `lidar_missions.py`, `parking_detector.py`, `parking.py` |

미션 파라미터(`missions.yaml`)는 각 미션 담당자가 고치고, 통합 리허설 전에 A가 한 번에 확인합니다.

## 5. 차량 1대 작업 방식

1. **매일 차량을 쓰기 시작할 때 먼저 bag을 녹화합니다.** (트랙 1바퀴 수동/저속 주행 + 미션별 구간)
   배터리를 아끼기 위해, 녹화가 끝나면 차량은 검증용으로만 씁니다.
2. bag을 팀 공유 드라이브에 올리고, 세 사람 모두 자기 노트북에서 `offline.launch.py`로 개발합니다.
3. 차량 시간은 미리 정해 둔 순서로 나눕니다 (예: A 차선 튜닝 → C LiDAR 미션 → B 신호등/표지판 → 통합 1바퀴).
   차량에서는 "확인하고 숫자만 바꾸기"를 하고, 코드 수정은 노트북에서 합니다.
4. 리허설은 `tools/score_sheet.csv`에 한 줄씩 기록합니다 (배터리 %, git commit, bag 이름 포함).
   배터리 잔량에 따라 속도/조향이 달라질 수 있으니 같이 적어 두면 원인 찾기가 쉽습니다.

## 6. Git 규칙

- **코드는 `git pull`로만 로봇에 들어갑니다. 로봇에서 직접 코드를 고치지 않습니다.**
  로봇에서 숫자를 바꿔 봤다면, 그 값을 노트북에서 YAML에 반영해 커밋한 뒤 로봇에서 다시 pull 합니다.
  (로봇에서 `git status`가 깨끗하지 않으면 `git stash` 하고 pull 하십시오.)
- `main` 브랜치는 항상 실차에서 돌아가는 상태로 둡니다. 작업은 `a/lane-tuning`, `b/yolo-v2`, `c/parking`처럼 브랜치에서 하고,
  오프라인 bag으로 확인한 뒤 `main`에 합칩니다.
- 대회 전날에는 쓸 커밋에 태그를 붙입니다 (`git tag race-day1`). 당일에는 태그만 checkout 합니다.
- bag, `*.pt`, `build/ install/ log/`는 Git에 올리지 않습니다(`.gitignore`). 대용량 파일은 공유 드라이브를 씁니다.
- 파라미터 YAML의 실수 값에는 소수점을 꼭 붙이십시오 (`1` 이 아니라 `1.0`). ROS2 Humble은 타입이 다르면 노드가 시작되지 않습니다.

## 7. 첫날 로봇에서 확인할 가정 목록

코드의 `TODO(확인)` 표시와 같은 내용입니다. `grep -rn "TODO(확인)" src` 로 전체 목록을 볼 수 있습니다.

| # | 확인할 것 | 확인 방법 | 관련 파일 |
|---|---|---|---|
| 1 | 카메라 토픽 `/camera/color/image_raw/compressed`가 나오는지, 해상도와 주기 | `ros2 topic hz`, `ros2 topic echo --once ... --field format` | params.yaml |
| 2 | `/scan` 이름, QoS(Best Effort), 0 rad가 정면인지, 각도가 왼쪽으로 증가하는지 | 차 앞에 물체를 두고 `ranges` 확인 | lidar_processor.py |
| 3 | **`/odom`이 나오는지(가정입니다)**, 이름, 1 m 주행 시 오차 | 줄자로 1 m 주행 후 비교 | mission_manager.py |
| 4 | LIMO 베이스 드라이버를 따로 실행해야 하는지 (`/cmd_vel` 수신 확인) | `ros2 topic info /cmd_vel` | robot.launch.py |
| 5 | 젯슨에서 `import ultralytics`가 되는지, YOLO11n 추론 속도(ms) | `python3 -c "import ultralytics"` | yolo_detector.py |
| 6 | 버드아이 변환 점, 차선 폭 픽셀 수, 세로 m/픽셀, 버드아이 아래 끝 ~ 앞바퀴 거리 | 트랙 위 사진 + 줄자 | params.yaml |
| 7 | 차선 색 (중앙선 노란색 여부), 대회장 조명에서 HSV 값 | `lane_debug:=true` | params.yaml |
| 8 | 정지선 정지 오차 (±5 cm 안인지) → `stop_margin_m` 보정 | 10회 반복 측정 | missions.yaml |
| 9 | YOLO 클래스 이름/순서 (임시 목록과 data.yaml 일치) | 학습 결과 확인 | params.yaml |
| 10 | 3차선 구간 진입 차로, 차선 변경 시간/각속도 | 실차 | missions.yaml |
| 11 | 각 구간 길이 (서행, 3차선, 라바콘), 갈림길 분기점까지 거리 | 트랙 도면 | missions.yaml |
| 12 | 신호등 옆 광고 모니터 위치, 신호 주기, 초록일 때 정지 필요 여부 | 주최 측 문의 | missions.yaml |
| 13 | T자 주차: 지정 칸(6칸 중 어디), 칸 크기(DXF), 회전 방향(도면상 오른쪽 추정), 애커먼으로 한 번에 들어가는지, 정지 유지 시간 | 트랙 도면 + 실차 + 주최 측 | parking.py, parking_detector.py |
| 14 | 출발 신호 방식 | 주최 측 문의 | simple.py |
| 15 | LIMO Pro 축거 (Stanley 모드 사용 시) | 실측 | params.yaml |
