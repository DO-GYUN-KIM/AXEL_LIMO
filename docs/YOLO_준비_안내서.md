# YOLO 준비 안내서 (처음부터 대회 투입까지)

작성: 2026-11-04 · 대상: B(비전) 담당, 팀 전원 참고 · 기존 `YOLO_학습_3070.md`를 보강해 대체합니다.
근거: 주최 측 GitHub `wego_deep_learning`(위키에서 받아 둠), Limo 위키, 교육 자료 정리본, 우리 저장소 코드. 확인하지 못한 내용은 **(추정)** 또는 **(확인 필요)**로 표시했습니다.

---

## 0. 결론 먼저

1. **처음부터 만들지 않습니다.** 주최 측이 학습된 모델 `best.pt`(7클래스)와 그 라벨 데이터를 공개했습니다. 우리는 여기에 **갈림길 A/B 표지판을 추가**하고, 데이터가 적은 **보행자(19개)·황색등(35개)을 보강**해서 다시 학습합니다.
2. **실행은 주최 측 노드를 그대로 씁니다.** Jetson에서 `yolo_to_trt`로 TensorRT 변환 → `yolo_object_detect`가 `/yolo_bounding_boxes`를 냅니다. B가 넘기는 결과물은 **모델 파일 하나와 클래스 이름 목록**입니다.
3. **가장 먼저 할 일(11/5)**: 주최 `best.pt`를 우리 차에 올려 대회 소품(신호등·보행자 표지판·서행 표지판·A/B 표지판·광고 모니터)을 비춰 봅니다. 결과에 따라 일이 크게 줄 수도, 늘 수도 있습니다.
4. 학습은 팀 데스크탑(RTX 3070)에서 **밤에** 돌립니다. 낮에는 Isaac Sim이 GPU를 씁니다.
5. 마지막 학습은 **11/15 밤**까지입니다. 11/16은 검증만, 11/17 코드 동결.

전체 흐름:

```
[차량] 촬영 ─▶ [데스크탑] 라벨링 ─▶ 데이터 합치기 ─▶ 밤에 학습 ─▶ 결과 판독
                                                                    │
[차량] 실차 확인 ◀─ yolo_object_detect ◀─ TensorRT 변환(Jetson) ◀─ best.pt 복사
   │
   └─ 틀린 장면 다시 촬영 ─▶ (반복)
```

---

## 1. 용어 1분 정리

| 용어 | 뜻 |
|---|---|
| 클래스 | 구분할 대상의 종류. 예: `red_light`, `pedestrian` |
| 라벨 | 사진 속 대상에 그린 상자(박스)와 클래스 번호. 사진 1장당 `.txt` 1개 |
| 학습 / epoch | 사진 전체를 모델에 한 번 다 보여주는 것이 1 epoch. 보통 100~200번 반복 |
| `.pt` | 학습 결과 모델 파일(PyTorch). 어느 컴퓨터에서나 열림 |
| `.engine` | Jetson 전용으로 바꾼 빠른 모델 파일(TensorRT). **만든 그 Jetson에서만** 동작 |
| mAP50 | 정확도 점수(0~1). 1에 가까울수록 좋음. 클래스별로 따로 나옴 |
| conf(신뢰도) | 모델이 "이게 맞다"고 확신하는 정도. 주최 노드는 0.5 미만을 버림(코드 고정) |
| 배경 사진 | 대상이 없는 장면. 빈 라벨로 저장하면 "여기엔 아무것도 없다"를 배움 |

---

## 2. 0단계: 주최 모델로 먼저 시험 (11/5, 차량 30분)

주최 `best.pt`가 대회 소품을 이미 잘 잡으면 신호등·보행자·서행은 거의 끝난 셈입니다. 대회가 WeGo 주관이라 소품이 같을 가능성이 높지만 **(추정)** 입니다.

**준비물**: 대회 소품 실물, 광고 모니터(또는 노트북에 신호등 영상 띄우기)

**차량에서** (NoMachine 접속, 계정 wego/wego):
```bash
# 터미널 1: 차체·카메라 기동
ros2 launch wego teleop_launch.py

# 터미널 2: 주최 모델로 인식 노드 실행
conda activate dl_env
cd ~/wego_ws && source install/setup.bash
pip show ultralytics | grep Version          # ← 버전을 적어 둡니다 (3장에서 씀)
ros2 run dl_ros2_application yolo_object_detect --ros-args \
  -p model_weight_path:=$HOME/wego_ws/src/wego_deep_learning/dl_ros2_application/weight/best.pt
#  경로는 차량마다 다를 수 있습니다. 못 찾으면: find ~ -name best.pt

# 터미널 3: 결과 보기
ros2 topic echo /yolo_bounding_boxes          # 글자로
rqt_image_view /yolo                          # 박스 그린 화면으로
```

**확인표** (각 소품을 30cm, 70cm, 1.5m에서 비춰 봅니다. 결과는 `score_sheet.csv` 옆에 적어 두십시오)

| 소품 | 잡히는가 | 이름이 맞는가 | 몇 m부터 잡히는가 | 비고 |
|---|---|---|---|---|
| 신호등 빨강/노랑/초록 | | | | |
| 보행자 표지판 | | | | |
| 서행 표지판 | | | | 주최 이름은 `slow_down` |
| 갈림길 A 표지판 | | | | 주최 모델엔 클래스 없음. 무엇으로 오인되는지만 봄 |
| 갈림길 B 표지판 | | | | 〃 |
| 광고 모니터 속 신호등 | | | | **잡히면 안 됨** |

**판단**
- 신호등·보행자가 잘 잡힘 → 이 데이터에 A/B만 추가하면 됩니다(가장 가벼운 경우).
- 잘 안 잡힘(모양이 다름) → 그 클래스도 우리 사진을 클래스당 300장 이상 찍어야 합니다. 일정이 하루 이상 늘어납니다.
- `.engine`으로 변환해 보고 **초당 처리 수(FPS)** 도 함께 봅니다(7장 참고). 이것이 모델 크기(n/s) 선택 기준이 됩니다.

> 주의: 차량에 있는 `best.pt`가 위키 GitHub 것(2026-03-05 학습, 7클래스)과 같은지 확인하십시오. 차량 소프트웨어가 구버전이면 클래스가 3개인 옛 모델일 수 있습니다. 갱신하기 전에는 `~/wego_ws/src`를 반드시 백업합니다.

---

## 3. 클래스 목록 확정 (11/5, 0단계 직후)

**한 번 정하면 바꾸지 않습니다.** 번호 순서가 바뀌면 지금까지 만든 라벨이 전부 틀어집니다. 주최 라벨러는 숫자키 0~9를 쓰므로 **최대 10개**입니다.

권장안: 주최 7개 순서를 그대로 두고 뒤에 붙입니다.

| 번호 | 이름 | 미션 | 출처 |
|---|---|---|---|
| 0 | `speed_up` | (속행 표지판, 쓰일지 미정) | 주최 데이터 |
| 1 | `slow_down` | M5 서행 | 주최 데이터 |
| 2 | `turn_right` | (우회전 표지판, 쓰일지 미정) | 주최 데이터 |
| 3 | `red_light` | M1 신호등 | 주최 데이터 |
| 4 | `yellow_light` | M1 신호등 | 주최 데이터 + **보강** |
| 5 | `green_light` | M1 신호등 | 주최 데이터 |
| 6 | `pedestrian` | M3 보행자 | 주최 데이터 + **보강** |
| 7 | `sign_A` | M2 갈림길 | **신규** |
| 8 | `sign_B` | M2 갈림길 | **신규** |
| 9 | (예비) | 주차 표지판이 실제로 있으면 `parking` | 실물 확인 후 |

- 대회에 안 나오는 클래스(0, 2)도 **지우지 않습니다.** 지우면 주최 라벨 925개의 번호를 모두 고쳐야 합니다. 남겨 두어도 손해가 없습니다.
- A/B 표지판 실물이 화살표 모양이라면 이름을 `sign_left`, `sign_right`처럼 실제 뜻에 맞게 바꿔도 됩니다. 단, **오늘 정한 이름을 끝까지** 씁니다.
- 정한 목록은 저장소 `docs/YOLO_준비_안내서.md`의 이 표를 고쳐서 팀에 알립니다.

### 3-1. 코드 쪽 이름도 같은 날 맞춥니다 (필수)
모델이 내보내는 `class_name`과 주행 코드 설정의 이름이 한 글자라도 다르면 **미션이 시작되지 않습니다.** 지금 저장소 설정은 임시 이름(`slow`, `red`, `green` …)이라, 주최 `best.pt`를 올리면 **M5 서행과 M1 신호등이 아예 동작하지 않습니다.** 클래스 목록을 확정하는 날, 아래 표대로 코드 설정도 함께 고칩니다(A와 B가 같이 확인).

| 파일 · 항목 | 지금 값 | 바꿀 값 (권장안 기준) |
|---|---|---|
| `missions.yaml` M5 `trigger_classes` | `slow` | `slow_down` |
| `missions.yaml` M1 `stop_classes` | `red,yellow` | `red_light,yellow_light` |
| `missions.yaml` M1 `go_classes` | `green` | `green_light` |
| `missions.yaml` M3 `trigger_classes` | `pedestrian` | `pedestrian` (그대로) |
| `missions.yaml` M2 `left_class` / `right_class` | `left_A` / `right_B` | `sign_A` / `sign_B` (A/B 중 어느 쪽이 왼쪽인지 실물로 확인) |
| `missions.yaml` M4 `trigger_classes` | `parking` | 주차 표지판이 있으면 `parking`(9번), 없으면 비움 |
| `params.yaml` `yolo_detector.class_names` | `[red, yellow, green, pedestrian, left_A, right_B, slow, parking]` | 3장 표 순서 그대로 0~8(9) |
| 각 미션 `.py`의 기본값(`stop_line.py`, `simple.py`, `fork.py`) | 위와 같은 임시 이름 | 같은 값으로 (yaml이 없을 때 쓰는 값이므로 맞춰 둠) |

**확인 방법**: 차에서 `ros2 topic echo /yolo_bounding_boxes`로 실제 나오는 `class_name`을 보고, 그 글자를 그대로 복사해 위 설정에 넣습니다. 모델을 바꿀 때마다(v1, v2 …) 한 번씩 다시 확인합니다.

---

## 4. 데스크탑 환경 만들기 (11/5, 1시간)

팀 데스크탑: Ubuntu 22.04, RTX 3070(8GB), RAM 64GB.

```bash
# 1) 드라이버 확인 — 표가 나오면 됩니다
nvidia-smi

# 2) Miniconda가 없으면 설치 후, 학습 전용 환경 생성
conda create -n yolo python=3.10 -y
conda activate yolo

# 3) GPU용 PyTorch: pytorch.org "Get Started"에서 Linux / Pip / CUDA 12.x 를 골라 나온 명령을 그대로 실행

# 4) Ultralytics — 버전을 차량 dl_env와 맞춥니다
pip install ultralytics==8.3.107

# 5) GPU 인식 확인 → "True NVIDIA GeForce RTX 3070"
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

- **8.3.107인 이유**: 주최 `best.pt` 파일 안에 "Ultralytics 8.3.107로 학습"이라고 기록돼 있습니다. 2장에서 차량의 `pip show ultralytics` 값이 다르면 **차량 값**을 따르십시오. 학습한 버전과 Jetson 버전이 크게 다르면 모델을 불러오다 실패할 수 있습니다 **(추정)**.
- 주최 라벨러는 ROS 없이 데스크탑에서 그대로 돕니다(파이썬 + OpenCV + Tkinter). 차량에서 파일 하나만 복사해 옵니다.
  ```bash
  pip install opencv-python numpy
  sudo apt install -y python3-tk
  # 저장소 위치: wego_deep_learning/dl_ros2_application/dl_ros2_application/yolo_labeler.py
  python yolo_labeler.py
  ```
- 주최 데이터셋: 프로젝트 폴더 `autorace-prep/위키/첨부/dataset_yolo.zip`(165MB)을 데스크탑 `~/yolo/wego_dataset/`에 풉니다.

권장 폴더 구조:
```
~/yolo/
  wego_dataset/        주최 데이터 (손대지 않음)
  ours/1105_A_B/       우리 촬영분, 날짜_내용으로 폴더를 나눔
       images/ labels/train/
  ours/1106_monitor/
  dataset_v1/          prepare_dataset.py가 만드는 학습용 (매번 새로 만듦)
  runs/                학습 결과
```

---

## 5. 촬영 (11/5~11/7 집중, 이후 오검출 장면만)

### 방법 두 가지
1. **주최 촬영 노드** — 한 장씩 원하는 순간에 찍기
   ```bash
   ros2 run dl_ros2_application take_a_picture
   ros2 service call /photoshoot dl_ros2_msgs/srv/PhotoShoot "{chalkak: false}"   # false = YOLO용 640×480
   ```
2. **녹화 후 뽑기(권장)** — 차를 천천히 굴리며 녹화하고, 데스크탑에서 몇 장마다 1장씩 뽑습니다. 차량 시간을 적게 씁니다.
   ```bash
   # 차량
   ros2 bag record /camera/color/image_raw/compressed -o bags/1105_signs
   # 데스크탑 (ROS2 Humble 환경)
   python3 tools/extract_frames.py bags/1105_signs --every 10 --out ~/yolo/ours/1105_signs/images
   ```
   `--every`는 30fps 기준 10이면 초당 3장입니다. 거의 같은 사진이 많으면 15~20으로 늘립니다.

### 무엇을 얼마나 찍나
| 대상 | 목표(라벨된 박스 수) | 요령 |
|---|---|---|
| `sign_A`, `sign_B` | 각 **300 이상** | 매 주행 무작위이므로 두 표지판을 **번갈아** 세워 같은 조건에서 찍습니다. 갈림길 진입 위치에서 차가 실제로 보게 될 각도 위주 |
| `pedestrian` | **+200** | 주최 19개뿐 |
| `yellow_light` | **+200** | 주최 35개뿐. 신호등을 황색으로 고정해 두고 찍습니다 |
| `red_light`, `green_light` | 각 +100 | 우리 트랙 조명에서 |
| `slow_down` | +100 | 0단계에서 잘 잡히면 생략 가능 |
| 광고 모니터 화면 | **배경 100장** | 모니터에 신호등 영상이 나오는 장면. **박스를 그리지 않고 빈 라벨로 저장** |
| 트랙 일반 장면 | 배경 50장 | 아무 표지판도 없는 구간 |

- 거리 30cm~1.5m, 정면·좌우 비스듬히, 형광등 켜고/끄고, 창문 빛 있을 때/없을 때를 섞습니다.
- 대회장 조명은 우리 연습장과 다릅니다. 밝기를 다양하게 찍어 두는 것이 가장 싼 보험입니다.
- 한 장면을 수십 장 찍는 것보다, **조건이 다른 장면을 여러 개** 찍는 편이 훨씬 낫습니다.

---

## 6. 라벨링 (가장 시간이 많이 드는 일)

```bash
conda activate yolo
python yolo_labeler.py
# ① 이미지가 '직접' 들어 있는 폴더 선택 (예: ~/yolo/ours/1105_signs/images)
# ② 라벨 폴더를 만들 '상위' 폴더 선택 (예: ~/yolo/ours/1105_signs) → labels/train/ 이 생김
# ③ 터미널에 클래스 이름 입력 — 반드시 3장 순서 그대로, 띄어쓰기로 구분:
#    speed_up slow_down turn_right red_light yellow_light green_light pedestrian sign_A sign_B
```
단축키: 왼쪽 드래그 = 박스, 숫자 0~9 = 클래스 선택, **S = 저장(다음 사진 전에 꼭)**, D = 마지막 박스 삭제, N/P = 다음/이전, Q = 종료.

### 라벨 규칙 (팀 공통, 어기면 모델이 헷갈립니다)
1. 박스는 **대상 테두리에 딱 맞게**. 신호등은 **켜진 등 하나만**이 아니라 **주최 데이터와 같은 기준**으로 그립니다. 시작 전에 주최 라벨 몇 장을 라벨러로 열어 보고 박스를 어디까지 그렸는지 확인하십시오(주최 폴더를 ①②로 지정하면 기존 박스가 보입니다).
2. 사진 속 해당 대상은 **전부** 그립니다. 하나라도 빼면 "이건 아니다"라고 가르치는 셈입니다.
3. 너무 작거나(대략 15픽셀 미만) 반 이상 잘린 것은 그리지 않습니다. 대신 그 사진은 쓰지 않는 편이 낫습니다(N으로 넘기고 저장하지 않음).
4. **광고 모니터 속 신호등은 그리지 않습니다.** 대상이 하나도 없으면 박스 없이 S를 눌러 **빈 라벨**로 저장합니다.
5. 저장하지 않고 넘긴 사진은 "아직 안 함"으로 보고 학습에서 빠집니다(7장 스크립트 동작).

### 시간 줄이기
- 1인 기준 시간당 150~250장 정도로 봅니다 **(추정)**. 600장이면 3~4시간입니다.
- 1차 모델이 나온 뒤부터는 **모델이 먼저 그리고 사람은 고치기만** 합니다(10장).
- 라벨링은 차량이 필요 없으므로 A·C가 차를 쓰는 동안 B가 진행합니다. 막히면 팀원이 나눠 들 수 있도록 폴더를 날짜별로 나눕니다.

---

## 7. 데이터 합치기 → 밤에 학습

### 7-1. 합치기 (저장소 `tools/prepare_dataset.py`, 이번에 추가함)
```bash
cd ~/AXEL_LIMO
python3 tools/prepare_dataset.py \
  --src ~/yolo/wego_dataset ~/yolo/ours/1105_signs ~/yolo/ours/1106_monitor \
  --out ~/yolo/dataset_v1 \
  --names speed_up slow_down turn_right red_light yellow_light green_light pedestrian sign_A sign_B
```
이 스크립트가 해 주는 일:
- 여러 폴더를 합치고 `data.yaml`을 만듭니다.
- **연속 사진 20장을 한 묶음**으로 train/val을 나눕니다. 거의 같은 사진이 양쪽에 섞이면 점수가 실제보다 좋게 나오기 때문입니다. 주최 사진도 0.3초 간격 연속 촬영입니다.
- 라벨 파일이 없는 사진은 뺍니다. 주최 데이터는 약 2,950장 중 925장만 라벨이 있어서 나머지는 자동으로 빠집니다. 주최 val 폴더에도 라벨이 없어서 그대로 쓰면 검증 점수가 0이 됩니다. 이 스크립트가 train에서 val을 새로 떼어 해결합니다.
- 빈 라벨(배경) 사진은 라벨 있는 사진 수의 10%까지만 넣습니다(`--max-bg-ratio`).
- 마지막에 **클래스별 박스 수 표**를 보여 주고, 150개 미만이면 "부족"이라고 표시합니다.

주최 데이터만 넣어 시험한 결과(오늘 확인): train 829장 / val 96장, `yellow_light` 27개·`pedestrian` 15개로 "부족" 표시, `sign_A`·`sign_B` 0개. 즉 **보강 대상이 위 5장 표와 일치**합니다.

### 7-2. 학습 (밤, Isaac Sim 종료 후)
```bash
conda activate yolo
nvidia-smi        # 다른 프로그램이 GPU를 쓰고 있지 않은지 확인 (Isaac Sim 꺼짐)
cd ~/yolo
yolo detect train model=yolo11n.pt data=$HOME/yolo/dataset_v1/data.yaml \
  imgsz=640 epochs=150 patience=40 batch=16 workers=4 \
  hsv_h=0.005 fliplr=0.0 \
  project=runs name=v1
```
| 설정 | 이유 |
|---|---|
| `yolo11n.pt` | 주최와 같은 가장 작은 모델. Jetson에서 빠름 |
| `batch=16` | 3070 8GB에서 안전한 값. 메모리 부족 오류가 나면 8 |
| `hsv_h=0.005` | 색을 크게 바꾸는 변형을 줄임. 신호등은 색 자체가 정답이라 기본값이면 빨강·노랑을 헷갈릴 수 있음 **(추정)** |
| `fliplr=0.0` | **좌우 뒤집기 끔.** 켜 두면 A/B 표지판이 화살표·글자일 때 좌우가 뒤바뀐 사진을 같은 클래스로 배워 버립니다. 표지판이 좌우 대칭 모양이면 켜도 됩니다 |
| `patience=40` | 40 epoch 동안 나아지지 않으면 자동 종료 |
| `imgsz=640` | 카메라 사진 640×480 그대로 |

- 걸리는 시간: 1,500~2,500장, 150 epoch면 1~2시간으로 봅니다 **(추정)**. 첫날 밤에 실제 시간을 적어 두십시오.
- 처음부터(`yolo11n.pt`) 학습하는 것을 기본으로 합니다. 주최 `best.pt`에서 이어 학습할 수도 있지만 클래스 수가 7→9로 바뀌어 이득이 크지 않습니다 **(추정)**.
- 자다가 끊기지 않도록 `tmux`나 `nohup`으로 실행합니다: `nohup yolo detect train ... > train_v1.log 2>&1 &`
- 결과: `~/yolo/runs/v1/weights/best.pt`

### 7-3. 결과 판독 (아침, 10분)
`runs/v1/` 폴더에서:
1. `results.png`: 위쪽 loss 곡선이 내려가다 평평해지고, 아래쪽 mAP50 곡선이 올라가다 평평하면 정상입니다.
2. 학습 로그 마지막의 **클래스별 mAP50**: 미션 클래스(1, 3~8)가 모두 **0.9 이상**이면 실차로 넘어갑니다(목표치, 경험상 이런 실내 소품 문제는 도달 가능한 수준 **(추정)**). 한 클래스만 낮으면 그 클래스 사진을 더 찍습니다.
3. `confusion_matrix.png`: 어떤 클래스끼리 헷갈리는지 보입니다. `red_light`↔`yellow_light`, `sign_A`↔`sign_B`가 섞이면 그 둘을 집중 보강합니다.
4. `val_batch0_pred.jpg`: 실제 사진 위에 예측 박스를 그린 그림. 눈으로 한 번 훑습니다.

**주의**: 점수가 높아도 실제 차에서는 다를 수 있습니다. 최종 판단은 항상 실차입니다.

---

## 8. Jetson으로 옮겨 변환·실행

```bash
# 데스크탑 → 차량 (같은 공유기, IP는 차량에서 ifconfig)
# SSH가 안 되면 USB나 NoMachine 파일 전송을 씁니다. 파일 이름에 버전을 붙입니다.
ssh wego@<차량IP> mkdir -p ~/models
scp ~/yolo/runs/v1/weights/best.pt wego@<차량IP>:~/models/yolo_v1.pt

# 차량 (반드시 차량에서. .engine은 만든 장치에서만 동작)
conda activate dl_env
cd ~/wego_ws && source install/setup.bash
ros2 run dl_ros2_application yolo_to_trt ~/models/yolo_v1.pt
#   → ~/models/yolo_v1.engine 생성 (FP16, 몇 분 걸림)

ros2 run dl_ros2_application yolo_object_detect --ros-args \
  -p model_weight_path:=$HOME/models/yolo_v1.engine
```

**FPS 확인**: `ros2 topic hz /yolo_bounding_boxes`. 주최 노드는 최대 20Hz로 돌도록 짜여 있습니다. **10Hz 이상**이면 충분합니다(0.3m/s에서 한 번 판단하는 사이 3cm 이동). 10Hz 미만이면 차선 인식과 CPU/GPU를 나눠 쓰는 문제인지 먼저 봅니다.

**실차 확인**: 2장 확인표를 새 모델로 다시 채웁니다. 특히
- A/B를 무작위로 바꿔 세우고 **20번 중 몇 번 맞히는지** 셉니다(목표 20/20).
- 광고 모니터 앞을 지나갈 때 `red_light`/`green_light`가 뜨는지 봅니다.

**모델 관리 규칙**: 차량 `~/models/`에 `yolo_v1.pt`, `yolo_v1.engine`, `yolo_v2...`를 지우지 않고 쌓습니다. 실행 중인 모델 이름은 팀 채팅에 적습니다. 대회 당일 쓸 모델은 USB에도 한 벌 둡니다.

---

## 9. 주행 코드와 연결 (A와 합의할 것)

B → A 전달 형식은 주최 메시지를 그대로 씁니다.
```
/yolo_bounding_boxes  (dl_ros2_msgs/BoundingBoxes)
  bounding_boxes[]: class_name, probability, xmin, ymin, xmax, ymax   (conf 0.5 미만은 이미 버려짐)
```
- **우리 저장소 현재 상태와 다른 점** (A가 고칠 일, 11/7까지):
  - 뼈대의 `yolo_detector` 노드는 `.pt`를 직접 돌리고 `/autorace/detections`를 냅니다. 주최 노드(TensorRT)가 더 빠르므로 `mission_manager`가 `/yolo_bounding_boxes`를 받도록 바꾸고, 뼈대 노드는 예비로 둡니다.
  - 클래스 이름 맞추기는 **3-1의 표**대로 합니다. 이것을 안 하면 M5·M1이 시작되지 않습니다.
- **얼마나 가까워야 반응할지(박스 면적 기준)** 는 B가 정해 A에게 넘깁니다. 주최 기본값은 표지판 2,700 / 적·녹색등 7,000 / 보행자 4,000 px²입니다. 정지선 위치에 차를 세워 두고 그때의 박스 면적을 재서 정합니다.
- **한 장면만 보고 결정하지 않습니다.** 같은 클래스가 연속 3~5회 나올 때 인정하도록 A와 합의합니다. 오검출 한 번에 차가 서는 것을 막습니다.
- **신호등 판단**: 광고 모니터 대책으로 ① 모니터 장면을 배경으로 학습 ② 박스 위치(모니터는 화면 어느 쪽에 나오는지)와 크기로 거르기를 함께 씁니다. ②의 기준은 실제 대회 소품 배치를 보고 정합니다 **(확인 필요: 모니터 위치)**.

---

## 10. 재학습 반복 (11/8~11/15)

1. 실차에서 틀린 장면을 녹화합니다(오검출·미검출 모두).
2. 뽑은 사진에 **직전 모델로 박스를 미리 그립니다.**
   ```bash
   yolo detect predict model=~/yolo/runs/v1/weights/best.pt source=~/yolo/ours/1109_miss/images \
     save_txt=True conf=0.3 project=~/yolo/pre name=1109
   mkdir -p ~/yolo/ours/1109_miss/labels/train
   cp ~/yolo/pre/1109/labels/*.txt ~/yolo/ours/1109_miss/labels/train/
   ```
3. 라벨러를 같은 폴더로 열면 미리 그린 박스가 보입니다. **틀린 것만 고치고 S.** (고치지 않고 넘긴 사진도 미리 그린 라벨 파일이 있어 학습에 들어갑니다. 대충 넘기지 마십시오.)
4. `prepare_dataset.py`로 `dataset_v2`를 새로 만들고 `name=v2`로 학습합니다.
5. **v2가 v1보다 실차에서 나을 때만** 바꿉니다. 점수만 보고 바꾸지 않습니다.

---

## 11. 일정 (B 기준)

| 날짜 | 할 일 | 끝났다는 기준 |
|---|---|---|
| 11/5 | 0단계 시험, 클래스 확정, **코드 쪽 이름 맞추기(3-1)**, 데스크탑 환경 | 확인표 채움, 클래스 표 확정, 설정 이름 일치, GPU True |
| 11/5~11/7 | A/B·보행자·황색등·모니터 촬영과 라벨링 | `prepare_dataset.py` 표에서 "부족" 없음 |
| 11/7 밤 | **1차 학습 (v1)** | 미션 클래스 mAP50 0.9 이상 |
| 11/8 | Jetson 변환·실차 확인, FPS, 박스 면적 기준 측정 | 10Hz 이상, 면적 기준 A에게 전달 |
| 11/8~11/11 | 오검출 장면 수집, 11/11 밤 **2차 학습 (v2)** | 광고 모니터 앞 오검출 0회 |
| 11/12~11/14 | 통합 주행 중 문제 장면 보강, 11/14 밤 **3차 (v3)** | A/B 무작위 20/20 |
| 11/15 밤 | **마지막 학습** (필요할 때만) | — |
| 11/16 | 최종 모델 고정, 차량·USB에 보관 | 대회용 모델 이름 확정 |
| 11/17 | 코드 동결 | — |

밀리면 줄이는 순서: `slow_down` 보강 → 3차 학습 → 2차 학습. **A/B와 신호등은 줄이지 않습니다**(15+10점).

---

## 12. 자주 막히는 곳

| 증상 | 원인과 해결 |
|---|---|
| `torch.cuda.is_available()`가 False | PyTorch를 CPU판으로 깔았음. 4장 3)을 다시 |
| `CUDA out of memory` | `batch=8`. Isaac Sim이 켜져 있는지 `nvidia-smi`로 확인 |
| 학습 중 mAP가 계속 0 | val에 라벨이 없음(주최 val 폴더를 그대로 쓴 경우). `prepare_dataset.py`로 다시 만듦 |
| 클래스 이름이 뒤죽박죽 | 라벨러에 입력한 순서와 `--names` 순서가 다름. 3장 순서로 통일 |
| Jetson에서 `.engine`이 안 열림 | 데스크탑에서 변환한 파일. 반드시 차량에서 `yolo_to_trt` |
| Jetson에서 `.pt` 로딩 오류 | Ultralytics 버전 차이 **(추정)**. 데스크탑 버전을 차량 `dl_env` 버전에 맞추고 다시 학습 |
| `yolo_object_detect`가 바로 종료 | `model_weight_path` 경로 오타. 노드가 파일을 못 찾으면 바로 끝남 |
| 박스는 나오는데 차가 반응 안 함 | `class_name`과 `missions.yaml`의 이름 불일치, 또는 박스 면적 기준이 너무 큼 |

---

## 13. 확인이 필요한 것 (B가 11/5에 실물로 확인)

- 대회 A/B 표지판의 실제 모양(글자인지 화살표인지, 좌우 대칭인지) → 클래스 이름과 `fliplr` 설정이 달라짐
- 주차 표지판이 실제로 있는지 → 9번 클래스 사용 여부
- 대회 소품이 주최 데이터 속 소품과 같은지 → 0단계 결과
- 차량 `dl_env`의 Ultralytics 버전, 차량 `best.pt`가 최신(7클래스)인지
- 광고 모니터의 위치와 화면 내용(신호등 영상인지, 일반 광고인지)
- 대회 전날(11/18) 대회장에서 촬영이 허용되는지 → 허용되면 그 조명에서 50장만 추가해도 효과가 큼 **(추정)**. 단 11/17 동결 이후라 모델 교체는 팀 판단
