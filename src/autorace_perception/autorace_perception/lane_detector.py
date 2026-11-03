"""차선 검출 노드입니다.

압축 이미지를 받아 버드아이(위에서 본 시점)로 변환하고, 흰색/노란색 차선을 HSV로 골라낸 뒤
슬라이딩 윈도우로 좌/우 차선을 찾습니다. 결과는 LaneInfo로 내보냅니다.
정지선은 버드아이 이미지에서 "가로로 길게 흰색인 행"을 찾는 단순한 방식입니다(스텁).

담당: A (차선/제어/통합)
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from autorace_interfaces.msg import LaneInfo


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        d = self.declare_parameter
        d('image_topic', '/camera/color/image_raw/compressed')
        d('lane_info_topic', '/autorace/lane_info')
        d('publish_debug', False)
        d('debug_topic', '/autorace/lane_debug/compressed')
        # 버드아이 변환 점: 원본 이미지 크기 대비 비율 [x, y] x 4 (좌상, 우상, 우하, 좌하)
        # TODO(확인): 실제 카메라 장착 각도에서 트랙 사진을 찍어 다시 맞춰야 합니다.
        d('warp_src', [0.30, 0.60, 0.70, 0.60, 0.98, 0.95, 0.02, 0.95])
        d('warp_dst', [0.25, 0.00, 0.75, 0.00, 0.75, 1.00, 0.25, 1.00])
        d('warp_size', [320, 240])  # 버드아이 이미지 [폭, 높이] 픽셀
        d('white_hsv_low', [0, 0, 180])
        d('white_hsv_high', [180, 50, 255])
        d('yellow_hsv_low', [15, 80, 100])
        d('yellow_hsv_high', [40, 255, 255])
        d('lane_width_m', 0.55)
        d('lane_width_px', 160.0)  # TODO(확인): 버드아이 이미지에서 차선 폭이 몇 픽셀인지 측정
        d('meters_per_px_y', 0.004)  # TODO(확인): 버드아이 세로 1픽셀이 몇 m인지 측정
        d('bottom_to_front_wheel_m', 0.15)  # TODO(확인): 버드아이 맨 아래 행이 앞바퀴에서 몇 m 앞인지
        d('n_windows', 8)
        d('window_margin_px', 30)
        d('min_pixels', 40)
        d('stop_line_row_ratio', 0.55)  # 한 행의 55% 이상이 흰색이면 정지선 후보
        d('stop_line_min_rows', 4)

        self.pub = self.create_publisher(LaneInfo, self.gp('lane_info_topic'), 10)
        self.debug_pub = None
        if self.gp('publish_debug'):
            self.debug_pub = self.create_publisher(CompressedImage, self.gp('debug_topic'), 1)
        self.create_subscription(CompressedImage, self.gp('image_topic'), self.on_image,
                                 qos_profile_sensor_data)
        self._warp_cache = {}
        self.get_logger().info('lane_detector 시작: ' + self.gp('image_topic'))

    def gp(self, name):
        return self.get_parameter(name).value

    def warp_matrix(self, w, h):
        key = (w, h)
        if key not in self._warp_cache:
            ww, wh = self.gp('warp_size')
            src = np.float32(self.gp('warp_src')).reshape(4, 2) * [w, h]
            dst = np.float32(self.gp('warp_dst')).reshape(4, 2) * [ww, wh]
            self._warp_cache[key] = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
        return self._warp_cache[key]

    def make_masks(self, bev):
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array(self.gp('white_hsv_low')), np.array(self.gp('white_hsv_high')))
        yellow = cv2.inRange(hsv, np.array(self.gp('yellow_hsv_low')), np.array(self.gp('yellow_hsv_high')))
        return white, yellow

    def sliding_window(self, mask, base_x):
        """한쪽 차선을 아래에서 위로 따라가며 픽셀을 모아 2차 다항식 x = f(y)로 맞춥니다."""
        h, w = mask.shape
        n = self.gp('n_windows')
        margin = self.gp('window_margin_px')
        min_px = self.gp('min_pixels')
        ys, xs = mask.nonzero()
        win_h = h // n
        x_cur = base_x
        keep = []
        for i in range(n):
            y_lo, y_hi = h - (i + 1) * win_h, h - i * win_h
            idx = ((ys >= y_lo) & (ys < y_hi) & (xs >= x_cur - margin) & (xs < x_cur + margin)).nonzero()[0]
            keep.append(idx)
            if len(idx) > min_px // 2:
                x_cur = int(np.mean(xs[idx]))
        idx = np.concatenate(keep) if keep else np.array([], dtype=int)
        if len(idx) < min_px:
            return None
        return np.polyfit(ys[idx], xs[idx], 2)

    def find_stop_line(self, white):
        h, w = white.shape
        ratio = (white > 0).sum(axis=1) / float(w)
        rows = np.where(ratio >= self.gp('stop_line_row_ratio'))[0]
        if len(rows) < self.gp('stop_line_min_rows'):
            return False, 0.0
        row = int(rows.max())  # 가장 가까운(아래쪽) 행
        dist = self.gp('bottom_to_front_wheel_m') + (h - 1 - row) * self.gp('meters_per_px_y')
        return True, float(dist)

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn('이미지 디코딩 실패', throttle_duration_sec=2.0)
            return
        h, w = img.shape[:2]
        ww, wh = self.gp('warp_size')
        bev = cv2.warpPerspective(img, self.warp_matrix(w, h), (ww, wh))
        white, yellow = self.make_masks(bev)
        # TODO(확인): 트랙의 중앙선이 노란색인지, 양쪽 모두 흰색인지 확인 후 좌/우 마스크를 분리할지 결정
        mask = cv2.bitwise_or(white, yellow)

        out = LaneInfo()
        out.header = msg.header
        hist = (mask[wh // 2:, :] > 0).sum(axis=0)
        mid = ww // 2
        left_fit = right_fit = None
        if hist[:mid].max() > 0:
            left_fit = self.sliding_window(mask, int(np.argmax(hist[:mid])))
        if hist[mid:].max() > 0:
            right_fit = self.sliding_window(mask, int(np.argmax(hist[mid:])) + mid)

        half_px = self.gp('lane_width_px') / 2.0
        y_eval = wh - 1
        center_fit = None
        if left_fit is not None and right_fit is not None:
            center_fit = (left_fit + right_fit) / 2.0
        elif left_fit is not None:
            center_fit = left_fit + np.array([0.0, 0.0, half_px])
        elif right_fit is not None:
            center_fit = right_fit - np.array([0.0, 0.0, half_px])

        out.left_detected = left_fit is not None
        out.right_detected = right_fit is not None
        if center_fit is not None:
            mx = self.gp('lane_width_m') / self.gp('lane_width_px')
            my = self.gp('meters_per_px_y')
            cx = np.polyval(center_fit, y_eval)
            dxdy = 2 * center_fit[0] * y_eval + center_fit[1]
            out.lane_valid = True
            out.center_offset = float(-(cx - ww / 2.0) * mx)
            out.heading_error = float(math.atan2(dxdy * mx, my))

        out.stop_line_detected, out.stop_line_distance = self.find_stop_line(white)
        self.pub.publish(out)

        if self.debug_pub is not None:
            self.publish_debug(msg, bev, mask, center_fit, out)

    def publish_debug(self, msg, bev, mask, center_fit, info):
        dbg = bev.copy()
        dbg[mask > 0] = (0, 255, 0)
        if center_fit is not None:
            ys = np.arange(0, dbg.shape[0], 4)
            pts = np.stack([np.polyval(center_fit, ys), ys], axis=1).astype(np.int32)
            cv2.polylines(dbg, [pts], False, (0, 0, 255), 2)
        txt = 'off=%.3f hd=%.2f stop=%s' % (info.center_offset, info.heading_error,
                                           'Y %.2f' % info.stop_line_distance if info.stop_line_detected else 'N')
        cv2.putText(dbg, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        ok, buf = cv2.imencode('.jpg', dbg)
        if ok:
            out = CompressedImage()
            out.header = msg.header
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
