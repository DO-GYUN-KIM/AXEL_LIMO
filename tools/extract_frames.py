#!/usr/bin/env python3
"""rosbag2에서 카메라 압축 이미지를 N프레임마다 JPEG로 뽑아 YOLO 라벨링용 폴더에 저장합니다.

ROS2 Humble 환경(source /opt/ros/humble/setup.bash)에서 실행하십시오. rosbag2_py를 사용합니다.

사용 예:
  python3 tools/extract_frames.py bags/20261103_101500_lap1 --every 10 --out frames/lap1
  python3 tools/extract_frames.py bags/xxx --start 30 --end 60   # bag 시작 후 30~60초 구간만
"""
import argparse
import os
import sys


def parse_args():
    ap = argparse.ArgumentParser(description='rosbag2 -> JPEG frames')
    ap.add_argument('bag', help='rosbag2 폴더 경로')
    ap.add_argument('--topic', default='/camera/color/image_raw/compressed')
    ap.add_argument('--every', type=int, default=10, help='N프레임마다 1장 저장')
    ap.add_argument('--out', default='frames', help='저장 폴더')
    ap.add_argument('--prefix', default='', help='파일 이름 앞에 붙일 글자 (기본: bag 폴더 이름)')
    ap.add_argument('--storage', default='sqlite3', help='sqlite3 (Humble 기본) 또는 mcap')
    ap.add_argument('--start', type=float, default=0.0, help='bag 시작 후 몇 초부터')
    ap.add_argument('--end', type=float, default=-1.0, help='bag 시작 후 몇 초까지 (-1 = 끝까지)')
    return ap.parse_args()


def main():
    args = parse_args()
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import CompressedImage
    except ImportError as exc:
        sys.exit('ROS2 환경을 먼저 source 하십시오: %s' % exc)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage),
                rosbag2_py.ConverterOptions('', ''))
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in topics:
        sys.exit('토픽 %s 이 bag에 없습니다. 있는 토픽: %s' % (args.topic, list(topics)))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    os.makedirs(args.out, exist_ok=True)
    prefix = args.prefix or os.path.basename(os.path.normpath(args.bag))
    count = saved = 0
    t_first = None
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        t_first = t_ns if t_first is None else t_first
        rel = (t_ns - t_first) * 1e-9
        if rel < args.start or (args.end >= 0 and rel > args.end):
            continue
        count += 1
        if (count - 1) % max(args.every, 1) != 0:
            continue
        msg = deserialize_message(data, CompressedImage)
        path = os.path.join(args.out, '%s_%06d.jpg' % (prefix, count))
        if 'jpeg' in msg.format.lower() or 'jpg' in msg.format.lower():
            with open(path, 'wb') as f:
                f.write(bytes(msg.data))
        else:
            # png 등 다른 형식이면 OpenCV로 다시 JPEG로 저장합니다.
            import cv2
            import numpy as np
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            cv2.imwrite(path, img)
        saved += 1
    print('읽은 프레임 %d장 중 %d장 저장 -> %s' % (count, saved, args.out))


if __name__ == '__main__':
    main()
