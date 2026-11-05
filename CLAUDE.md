# AXEL_LIMO — Claude 작업 안내

국민대 AutoRace 2026(본선 11/19~20, 코드 동결 11/17 저녁) 출전용 ROS2 Humble 작업 공간입니다. 팀 3명, 차량 WeGo LIMO Pro 1대(Jetson, Ubuntu 22.04).
사용자 응답은 한국어 합니다체로 합니다.

## 차량 사실 (확인된 것 / 확인 필요)
- 접속: `ssh limo` (계정 wego, 팀 공유기에서 IP 고정). GUI는 NoMachine.
- 작업 공간: 저장소 = `~/axel_ws`. 주최 측 드라이버는 `~/wego_ws`(건드리지 않음, 그 위에 overlay).
- 기동: `ros2 launch wego teleop_launch.py` (위키 기준, 실차 확인 필요).
- **주행은 애커먼 모드만.** 애커먼 드라이버는 `/limo/ack_cmd`만 받습니다. 현재 우리 코드는 `/cmd_vel`을 내보내므로 변환이 필요합니다(미구현).
- 센서: 카메라 `/camera/color/image_raw/compressed`, 라이다 `/scan`(전방 약 ±110°), 주최 YOLO `/yolo_bounding_boxes`.
- 노트북에서 토픽 보기는 foxglove_bridge(`ws://<차량IP>:8765`). 그래서 `ROS_LOCALHOST_ONLY=1`은 남겨도 됩니다.

## 규칙
- 차량 배포는 `bash tools/car_deploy.sh <이름> <브랜치|태그>`로만 합니다. 차량에서 직접 `git checkout`/`git pull` 하지 않습니다.
- 차량에서는 코드를 고치지 않고 파라미터(`src/autorace_bringup/config/*.yaml`)만 바꿉니다. 바꾼 값은 `tune/<월일>-<이름>` 브랜치로 커밋합니다.
- `main`은 차에서 검증된 코드만. 기능 브랜치 → PR.
- **차를 움직이는 명령(`ros2 topic pub /limo/ack_cmd`, `/cmd_vel`, 주행 launch)은 사람에게 확인받고, 조종기를 든 사람이 있을 때만 실행합니다.** 바퀴를 띄운 받침대 시험이 기본입니다.
- `sudo apt upgrade`, JetPack·드라이버 변경, `~/wego_ws` 수정은 하지 않습니다.
- 노트북에서 차량 명령은 `ssh limo 'bash -lic "<명령>"'` 형태로 실행합니다(ROS 환경을 불러오기 위해).

## 자주 쓰는 것
- 점검: `bash tools/setup/car_setup.sh check`
- 녹화: `bash src/autorace_bringup/scripts/record_bag.sh <이름>` (저장 `~/bags`)
- 현재 배포 상태: `bash tools/car_deploy.sh --status`
- 문서: `docs/원격개발_환경.md`, `docs/젯팩직후_원격설정.md`, `docs/로드맵_정석.md`
