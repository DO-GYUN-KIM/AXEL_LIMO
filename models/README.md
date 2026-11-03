# models/

YOLO 가중치(`*.pt`)를 두는 폴더입니다. 용량 때문에 `.gitignore`로 Git에서 제외했습니다.

- Colab에서 학습한 `best.pt`는 팀 공유 드라이브에 날짜를 붙여 올립니다 (예: `best_20261105.pt`).
- 로봇에는 직접 복사(`scp`)한 뒤 `config/params.yaml`의 `yolo_detector.weights_path`를 그 경로로 바꿉니다.
- 어떤 가중치로 달렸는지 `tools/score_sheet.csv`의 notes에 적어 두십시오.
