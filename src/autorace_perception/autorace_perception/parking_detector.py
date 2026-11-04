"""주차칸 검출 노드입니다 (M4 T자 주차용).

카메라 이미지를 버드아이로 바꾼 뒤 흰색 선을 골라,
  - 세로선 두 개(칸의 좌/우 선) -> 칸 중심 가로 위치, 칸 방향
  - 칸 안쪽의 가로선(끝 선)     -> 앞바퀴에서 끝 선까지 거리
를 ParkingSlot으로 내보냅니다. 앞으로 들어가는(전면 진입) 주차를 전제로 합니다.
카메라 시야가 앞쪽 약 71°뿐이라 후진하면 칸이 보이지 않기 때문입니다.

버드아이 변환 값은 lane_detector와 같은 값을 params.yaml에서 함께 맞추십시오.
담당: C (주차). 정지 거리 계산 방식은 A의 정지선 정지와 같습니다.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from autorace_interfaces.msg import ParkingSlot


class ParkingDetector(Node):
    def __init__(self):
        super().__init__('parking_detector')
        d = self.declare_parameter
        d('image_topic', '/camera/color/image_raw/compressed')
        d('slot_topic', '/autorace/parking_slot')
        d('publish_debug', False)
        d('debug_topic', '/autorace/parking_debug/compressed')
        # TODO(확인): lane_detector와 같은 값으로 맞춥니다.
        d('warp_src', [0.30, 0.60, 0.70, 0.60, 0.98, 0.95, 0.02, 0.95])
        d('warp_dst', [0.25, 0.00, 0.75, 0.00, 0.75, 1.00, 0.25, 1.00])
        d('warp_size', [320, 240])
        d('white_hsv_low', [0, 0, 180])
        d('white_hsv_high', [180, 50, 255])
        d('meters_per_px_x', 0.0034)  # TODO(확인): 0.55 m / lane_width_px(160) 기준 추정값
        d('meters_per_px_y', 0.004)   # TODO(확인)
        d('bottom_to_front_wheel_m', 0.15)  # TODO(확인)
        d('min_line_pixels', 60)        # 세로선 하나로 인정할 최소 흰 픽셀 수
        d('line_band_px', 12)           # 세로선 주변으로 픽셀을 모을 폭
        d('expected_slot_width_m', 0.50)  # TODO(확인): DXF에서 칸 폭을 재서 넣습니다
        d('end_line_row_ratio', 0.6)    # 좌/우 선 사이 폭의 60% 이상이 흰색인 행 = 끝 선 후보
        d('end_line_min_rows', 3)

        self.pub = self.create_publisher(ParkingSlot, self.gp('slot_topic'), 10)
        self.debug_pub = None
        if self.gp('publish_debug'):
            self.debug_pub = self.create_publisher(CompressedImage, self.gp('debug_topic'), 1)
        self.create_subscription(CompressedImage, self.gp('image_topic'), self.on_image,
                                 qos_profile_sensor_data)
        self._warp = {}
        self.get_logger().info('parking_detector 시작')

    def gp(self, name):
        return self.get_parameter(name).value

    def warp_matrix(self, w, h):
        if (w, h) not in self._warp:
            ww, wh = self.gp('warp_size')
            src = np.float32(self.gp('warp_src')).reshape(4, 2) * [w, h]
            dst = np.float32(self.gp('warp_dst')).reshape(4, 2) * [ww, wh]
            self._warp[(w, h)] = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
        return self._warp[(w, h)]

    def fit_vertical(self, mask, x0):
        """x0 근처의 흰 픽셀로 직선 x = a*y + b를 맞춥니다. 실패하면 None."""
        band = self.gp('line_band_px')
        ys, xs = mask.nonzero()
        sel = np.abs(xs - x0) < band
        if sel.sum() < self.gp('min_line_pixels'):
            return None
        return np.polyfit(ys[sel], xs[sel], 1)

    def find_lines(self, mask):
        """가로 히스토그램의 봉우리 두 개를 좌/우 선 후보로 씁니다."""
        h, w = mask.shape
        hist = (mask > 0).sum(axis=0).astype(np.float32)
        mid = w // 2
        mx = self.gp('meters_per_px_x')
        exp_px = self.gp('expected_slot_width_m') / mx
        left = right = None
        if hist[:mid].max() >= self.gp('min_line_pixels') / 4:
            left = self.fit_vertical(mask, int(np.argmax(hist[:mid])))
        if hist[mid:].max() >= self.gp('min_line_pixels') / 4:
            right = self.fit_vertical(mask, int(np.argmax(hist[mid:])) + mid)
        # 두 선 간격이 예상 칸 폭과 너무 다르면, 더 강한 선 하나만 믿습니다.
        if left is not None and right is not None:
            gap = np.polyval(right, h - 1) - np.polyval(left, h - 1)
            if not (0.6 * exp_px < gap < 1.5 * exp_px):
                if hist[:mid].max() >= hist[mid:].max():
                    right = None
                else:
                    left = None
        return left, right

    def find_end_line(self, mask, xl, xr):
        h, _ = mask.shape
        lo, hi = int(max(0, min(xl, xr))), int(min(mask.shape[1], max(xl, xr)))
        if hi - lo < 10:
            return False, 0.0
        ratio = (mask[:, lo:hi] > 0).sum(axis=1) / float(hi - lo)
        rows = np.where(ratio >= self.gp('end_line_row_ratio'))[0]
        if len(rows) < self.gp('end_line_min_rows'):
            return False, 0.0
        row = int(rows.max())
        return True, float(self.gp('bottom_to_front_wheel_m') + (h - 1 - row) * self.gp('meters_per_px_y'))

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        ww, wh = self.gp('warp_size')
        bev = cv2.warpPerspective(img, self.warp_matrix(w, h), (ww, wh))
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.gp('white_hsv_low')), np.array(self.gp('white_hsv_high')))

        out = ParkingSlot()
        out.header = msg.header
        left, right = self.find_lines(mask)
        out.left_line_found = left is not None
        out.right_line_found = right is not None
        mx, my = self.gp('meters_per_px_x'), self.gp('meters_per_px_y')
        half = self.gp('expected_slot_width_m') / mx / 2.0
        y = wh - 1
        if left is not None and right is not None:
            center = (left + right) / 2.0
            out.slot_width = float((np.polyval(right, y) - np.polyval(left, y)) * mx)
        elif left is not None:
            center = left + np.array([0.0, half])
        elif right is not None:
            center = right - np.array([0.0, half])
        else:
            center = None
        if center is not None:
            out.slot_valid = True
            cx = np.polyval(center, y)
            out.center_offset = float(-(cx - ww / 2.0) * mx)
            # x = a*y + b : 위로 갈수록(y 감소) x가 줄면 칸이 왼쪽으로 기울어 보입니다.
            out.heading_error = float(math.atan2(center[0] * mx, my))
            xl = np.polyval(left, y) if left is not None else cx - half
            xr = np.polyval(right, y) if right is not None else cx + half
            out.end_line_found, out.end_line_distance = self.find_end_line(mask, xl, xr)
        self.pub.publish(out)

        if self.debug_pub is not None:
            dbg = bev.copy()
            dbg[mask > 0] = (0, 255, 0)
            for fit, col in ((left, (255, 0, 0)), (right, (0, 0, 255))):
                if fit is not None:
                    ys = np.array([0, wh - 1])
                    pts = np.stack([np.polyval(fit, ys), ys], axis=1).astype(np.int32)
                    cv2.polylines(dbg, [pts], False, col, 2)
            txt = 'off=%.3f hd=%.2f end=%s' % (out.center_offset, out.heading_error,
                                              '%.2f' % out.end_line_distance if out.end_line_found else '-')
            cv2.putText(dbg, txt, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            ok, buf = cv2.imencode('.jpg', dbg)
            if ok:
                m = CompressedImage()
                m.header, m.format, m.data = msg.header, 'jpeg', buf.tobytes()
                self.debug_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
