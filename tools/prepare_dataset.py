#!/usr/bin/env python3
"""여러 곳의 YOLO 라벨 데이터를 하나로 합치고 train/val로 나눈 뒤 data.yaml을 만듭니다.

- 주최 측 dataset_yolo.zip(압축 푼 폴더)과 우리가 라벨링한 폴더를 함께 넣을 수 있습니다.
- 라벨 파일(.txt)이 없는 사진은 '아직 라벨링 안 함'으로 보고 뺍니다.
- 라벨 파일이 있지만 비어 있는 사진은 '아무것도 없는 장면(배경)'으로 넣습니다.
  광고 모니터처럼 오검출을 막고 싶은 장면은 빈 라벨로 저장해 두면 됩니다.
- 연속 촬영 사진이 train과 val에 섞이지 않도록, 파일 이름 순서대로 묶음(block) 단위로 나눕니다.

사용 예 (Ubuntu, 팀 데스크탑):
  python3 tools/prepare_dataset.py \
      --src ~/yolo/wego_dataset ~/yolo/ours_1104 \
      --out ~/yolo/dataset_v1 \
      --names speed_up slow_down turn_right red_light yellow_light green_light pedestrian sign_A sign_B
"""
import argparse
import os
import random
import shutil
import sys
from collections import Counter

IMG_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def parse_args():
    ap = argparse.ArgumentParser(description='YOLO 데이터 합치기 + train/val 나누기')
    ap.add_argument('--src', nargs='+', required=True, help='원본 폴더들 (안에 이미지와 라벨 .txt가 있으면 됨)')
    ap.add_argument('--out', required=True, help='만들 데이터셋 폴더 (비어 있거나 없어야 함)')
    ap.add_argument('--names', nargs='+', required=True, help='클래스 이름, 0번부터 순서대로')
    ap.add_argument('--val-ratio', type=float, default=0.15, help='val 비율 (기본 0.15)')
    ap.add_argument('--block', type=int, default=20, help='연속 사진 몇 장을 한 묶음으로 볼지 (기본 20)')
    ap.add_argument('--max-bg-ratio', type=float, default=0.10,
                    help='배경(빈 라벨) 사진을 라벨 있는 사진 수의 몇 배까지 넣을지 (기본 0.10)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--link', action='store_true', help='복사 대신 심볼릭 링크 (디스크 절약)')
    return ap.parse_args()


def find_pairs(src):
    """src 아래 모든 이미지를 찾아 같은 이름의 .txt 라벨과 짝짓습니다."""
    labels = {}
    images = []
    for root, _, files in os.walk(src):
        for f in files:
            stem, ext = os.path.splitext(f)
            p = os.path.join(root, f)
            if ext.lower() == '.txt':
                labels[stem] = p
            elif ext.lower() in IMG_EXT:
                images.append((stem, p))
    images.sort()
    return [(img, labels.get(stem)) for stem, img in images]


def read_label(path, n_classes):
    """라벨 파일을 읽어 클래스 id 목록을 돌려줍니다. 형식이 틀리면 None."""
    ids = []
    with open(path) as fh:
        for line in fh:
            parts = line.split()
            if not parts:
                continue
            if len(parts) != 5:
                return None
            try:
                cid = int(float(parts[0]))
                vals = [float(v) for v in parts[1:]]
            except ValueError:
                return None
            if not (0 <= cid < n_classes) or any(v < 0 or v > 1 for v in vals):
                return None
            ids.append(cid)
    return ids


def main():
    a = parse_args()
    rnd = random.Random(a.seed)
    n = len(a.names)
    if os.path.exists(a.out) and os.listdir(a.out):
        sys.exit('출력 폴더가 비어 있지 않습니다: %s (덮어쓰기 방지)' % a.out)

    labeled, background = [], []   # 각 항목: (블록키, 이미지, 라벨, 클래스id목록)
    skipped_nolabel = 0
    bad = []
    for si, src in enumerate(a.src):
        pairs = find_pairs(os.path.expanduser(src))
        if not pairs:
            print('경고: 이미지를 찾지 못했습니다: %s' % src)
        for k, (img, lab) in enumerate(pairs):
            if lab is None:
                skipped_nolabel += 1
                continue
            ids = read_label(lab, n)
            if ids is None:
                bad.append(lab)
                continue
            key = (si, k // a.block)
            (labeled if ids else background).append((key, img, lab, ids))

    if bad:
        print('형식이 틀리거나 클래스 번호가 범위를 넘는 라벨 %d개는 뺐습니다. 예: %s' % (len(bad), bad[:3]))
    max_bg = int(len(labeled) * a.max_bg_ratio)
    if len(background) > max_bg:
        rnd.shuffle(background)
        print('배경 사진 %d장 중 %d장만 넣습니다 (--max-bg-ratio).' % (len(background), max_bg))
        background = background[:max_bg]
    items = labeled + background
    if not items:
        sys.exit('라벨 있는 사진이 없습니다.')

    # 묶음 단위로 val 배정
    keys = sorted({it[0] for it in items})
    rnd.shuffle(keys)
    n_val = max(1, int(round(len(keys) * a.val_ratio)))
    val_keys = set(keys[:n_val])

    counts = {'train': Counter(), 'val': Counter()}
    n_img = Counter()
    used = set()
    for key, img, lab, ids in items:
        split = 'val' if key in val_keys else 'train'
        stem = os.path.splitext(os.path.basename(img))[0]
        name = 's%d_%s' % (key[0], stem)   # 원본 폴더가 달라도 이름이 겹치지 않게
        if name in used:
            continue
        used.add(name)
        for sub in ('images', 'labels'):
            os.makedirs(os.path.join(a.out, sub, split), exist_ok=True)
        dst_img = os.path.join(a.out, 'images', split, name + os.path.splitext(img)[1].lower())
        dst_lab = os.path.join(a.out, 'labels', split, name + '.txt')
        if a.link:
            os.symlink(os.path.abspath(img), dst_img)
        else:
            shutil.copy2(img, dst_img)
        shutil.copy2(lab, dst_lab)
        counts[split].update(ids)
        n_img[split] += 1

    out_abs = os.path.abspath(os.path.expanduser(a.out))
    with open(os.path.join(out_abs, 'data.yaml'), 'w') as fh:
        fh.write('path: %s\ntrain: images/train\nval: images/val\n\nnc: %d\nnames:\n' % (out_abs, n))
        for nm in a.names:
            fh.write('  - %s\n' % nm)

    print('\n완료: %s' % os.path.join(out_abs, 'data.yaml'))
    print('라벨 없는 사진(제외): %d장 / 배경 사진(포함): %d장' % (skipped_nolabel, len(background)))
    print('사진 수: train %d, val %d\n' % (n_img['train'], n_img['val']))
    print('%-3s %-16s %7s %7s' % ('id', 'class', 'train', 'val'))
    for i, nm in enumerate(a.names):
        t, v = counts['train'][i], counts['val'][i]
        warn = '  <- 부족' if t < 150 else ''
        warn += '  <- val 없음' if v == 0 else ''
        print('%-3d %-16s %7d %7d%s' % (i, nm, t, v, warn))


if __name__ == '__main__':
    main()
