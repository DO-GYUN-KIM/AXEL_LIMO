"""YOLO 표지판/신호등 검출 노드입니다.

주최 측 Colab(Ultralytics)에서 학습한 YOLO11n 가중치(.pt)를 불러와 압축 이미지에서 물체를 찾고
DetectionArray로 내보냅니다. ultralytics가 설치되어 있지 않거나 가중치 파일이 없으면
경고만 남기고 아무것도 내보내지 않습니다(다른 노드는 계속 동작합니다).

젯슨 부하를 줄이기 위해 이미지가 올 때마다 추론하지 않고, 타이머(max_rate_hz)마다
가장 최근 이미지 1장만 추론합니다.

담당: B (YOLO 비전)
"""
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from autorace_interfaces.msg import Detection, DetectionArray

try:
    from ultralytics import YOLO
    _IMPORT_ERROR = None
except Exception as exc:  # ImportError 외에 torch 관련 오류도 여기서 잡습니다.
    YOLO = None
    _IMPORT_ERROR = exc


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        d = self.declare_parameter
        d('image_topic', '/camera/color/image_raw/compressed')
        d('detections_topic', '/autorace/detections')
        d('weights_path', '')  # 예: /home/wego/autorace_ws/models/best.pt
        # TODO(확인): 클래스 이름과 순서는 라벨링 결과(data.yaml)와 똑같이 맞춰야 합니다. 임시 목록입니다.
        d('class_names', ['red', 'yellow', 'green', 'pedestrian', 'left_A', 'right_B', 'slow', 'parking'])
        d('conf_threshold', 0.5)
        d('imgsz', 640)  # TODO(확인): 젯슨에서 속도가 부족하면 320~416으로 낮춥니다.
        d('device', '')  # '' = ultralytics가 자동 선택, 'cuda:0' 또는 'cpu'
        d('max_rate_hz', 10.0)

        self.class_names = list(self.gp('class_names'))
        self.model = self.load_model()
        self.latest = None
        self.pub = self.create_publisher(DetectionArray, self.gp('detections_topic'), 10)
        self.create_subscription(CompressedImage, self.gp('image_topic'), self.on_image,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / max(self.gp('max_rate_hz'), 0.1), self.on_timer)

    def gp(self, name):
        return self.get_parameter(name).value

    def load_model(self):
        if YOLO is None:
            self.get_logger().warn('ultralytics를 불러오지 못했습니다. 검출 없이 실행합니다: %s' % _IMPORT_ERROR)
            return None
        path = os.path.expanduser(self.gp('weights_path'))
        if not path or not os.path.isfile(path):
            self.get_logger().warn('가중치 파일이 없습니다(weights_path="%s"). 검출 없이 실행합니다.' % path)
            return None
        try:
            model = YOLO(path)
        except Exception as exc:
            self.get_logger().error('YOLO 모델 로드 실패: %s' % exc)
            return None
        names = [model.names[i] for i in sorted(model.names)]
        if names != self.class_names:
            self.get_logger().warn('모델 클래스 %s 와 파라미터 class_names %s 가 다릅니다. 모델 이름을 사용합니다.'
                                   % (names, self.class_names))
        self.class_names = names
        self.get_logger().info('YOLO 모델 로드 완료: %s (%d classes)' % (path, len(names)))
        return model

    def on_image(self, msg):
        self.latest = msg

    def on_timer(self):
        if self.model is None or self.latest is None:
            return
        msg, self.latest = self.latest, None
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        kwargs = dict(conf=float(self.gp('conf_threshold')), imgsz=int(self.gp('imgsz')), verbose=False)
        if self.gp('device'):
            kwargs['device'] = self.gp('device')
        t0 = time.monotonic()
        try:
            results = self.model.predict(img, **kwargs)
        except Exception as exc:
            self.get_logger().error('추론 실패: %s' % exc, throttle_duration_sec=2.0)
            return
        dt_ms = (time.monotonic() - t0) * 1000.0

        out = DetectionArray()
        out.header = msg.header
        out.image_height, out.image_width = img.shape[:2]
        if results:
            boxes = results[0].boxes
            for xyxy, conf, cls in zip(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist()):
                det = Detection()
                det.class_id = int(cls)
                det.class_name = self.class_names[det.class_id] if det.class_id < len(self.class_names) else str(det.class_id)
                det.confidence = float(conf)
                det.x_min, det.y_min, det.x_max, det.y_max = [float(v) for v in xyxy]
                out.detections.append(det)
        self.pub.publish(out)
        self.get_logger().debug('검출 %d개, 추론 %.1f ms' % (len(out.detections), dt_ms))


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
