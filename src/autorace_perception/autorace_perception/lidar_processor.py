"""LiDAR 가공 노드입니다.

/scan(LaserScan)을 받아 미션에서 바로 쓸 수 있는 값으로 줄여 LidarInfo로 내보냅니다.
  1) 전방 부채꼴 최소 거리 (충돌 방지)
  2) 3차선 구간: 왼쪽/가운데(현재)/오른쪽 차로 점유 여부와 최소 거리 (M7)
  3) 라바콘 구간: 점들을 묶어(클러스터) 가장 가까운 왼쪽/오른쪽 콘 사이 중점 방향 (M8)
     양쪽 콘을 다 못 찾으면 전방에서 가장 넓게 비어 있는 방향을 대신 씁니다.

좌표계: x = 전방, y = 왼쪽, 각도 양수 = 왼쪽.
담당: C (LiDAR 미션 + 주차)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from autorace_interfaces.msg import LidarInfo


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        d = self.declare_parameter
        d('scan_topic', '/scan')
        d('lidar_info_topic', '/autorace/lidar_info')
        # TODO(확인): YDLIDAR의 0 rad가 로봇 정면인지, 각도가 반시계(왼쪽) 방향으로 증가하는지 확인
        d('angle_offset_rad', 0.0)
        d('flip_direction', False)  # 각도 방향이 반대로 나오면 True
        d('range_min_m', 0.08)  # 차체에 맞은 점 제거
        d('range_max_m', 6.0)
        d('front_half_angle_deg', 15.0)
        # 3차선 (M7)
        d('lane_width_m', 0.55)
        d('occupancy_lookahead_m', 1.2)
        d('occupancy_min_points', 3)
        # 라바콘 (M8)
        d('cone_max_range_m', 1.2)
        d('cone_half_angle_deg', 80.0)
        d('cluster_gap_m', 0.10)
        d('min_cluster_points', 2)
        d('side_window_m', 0.20)  # 주차용: 차체 중심 기준 전후 이 범위 안의 옆 물체만 봅니다
        d('side_max_m', 0.60)

        self.pub = self.create_publisher(LidarInfo, self.gp('lidar_info_topic'), 10)
        # LiDAR는 Best Effort QoS로 나오므로 sensor_data 프로파일로 구독합니다.
        self.create_subscription(LaserScan, self.gp('scan_topic'), self.on_scan, qos_profile_sensor_data)

    def gp(self, name):
        return self.get_parameter(name).value

    def on_scan(self, scan):
        r = np.asarray(scan.ranges, dtype=np.float32)
        a = scan.angle_min + np.arange(len(r), dtype=np.float32) * scan.angle_increment
        if self.gp('flip_direction'):
            a = -a
        a = a + self.gp('angle_offset_rad')
        a = np.arctan2(np.sin(a), np.cos(a))  # [-pi, pi]로 정리
        valid = np.isfinite(r) & (r > self.gp('range_min_m')) & (r < self.gp('range_max_m'))
        r, a = r[valid], a[valid]
        order = np.argsort(a)
        r, a = r[order], a[order]
        x, y = r * np.cos(a), r * np.sin(a)

        out = LidarInfo()
        out.header = scan.header
        front = np.abs(a) < math.radians(self.gp('front_half_angle_deg'))
        out.front_min_distance = float(r[front].min()) if front.any() else float('inf')
        self.fill_lanes(out, x, y)
        self.fill_cones(out, r, a, x, y)
        self.fill_sides(out, x, y)
        self.pub.publish(out)

    def fill_lanes(self, out, x, y):
        w = self.gp('lane_width_m')
        ahead = (x > 0.0) & (x < self.gp('occupancy_lookahead_m'))
        bands = {
            'left': (y > w / 2) & (y < 1.5 * w),
            'center': np.abs(y) <= w / 2,
            'right': (y < -w / 2) & (y > -1.5 * w),
        }
        n_min = self.gp('occupancy_min_points')
        for name, band in bands.items():
            sel = ahead & band
            occupied = int(sel.sum()) >= n_min
            dist = float(x[sel].min()) if sel.any() else float('inf')
            setattr(out, name + '_lane_occupied', occupied)
            setattr(out, name + '_lane_min_distance', dist)

    def fill_sides(self, out, x, y):
        """주차 때 양옆 정차 LIMO까지 거리. 시야가 ±110°라 바로 옆(±90°)까지는 보입니다."""
        win, mx = self.gp('side_window_m'), self.gp('side_max_m')
        near = np.abs(x) < win
        left = near & (y > 0) & (y < mx)
        right = near & (y < 0) & (y > -mx)
        out.side_left_distance = float(y[left].min()) if left.any() else float('inf')
        out.side_right_distance = float(-y[right].max()) if right.any() else float('inf')

    def clusters(self, x, y):
        """각도 순서로 이웃한 점끼리 거리가 가까우면 한 물체로 묶고, 각 물체의 중심을 돌려줍니다."""
        if len(x) == 0:
            return []
        gap = self.gp('cluster_gap_m')
        groups, cur = [], [0]
        for i in range(1, len(x)):
            if math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) < gap:
                cur.append(i)
            else:
                groups.append(cur)
                cur = [i]
        groups.append(cur)
        n_min = self.gp('min_cluster_points')
        return [(float(np.mean(x[g])), float(np.mean(y[g]))) for g in groups if len(g) >= n_min]

    def fill_cones(self, out, r, a, x, y):
        max_r = self.gp('cone_max_range_m')
        half = math.radians(self.gp('cone_half_angle_deg'))
        sel = (r < max_r) & (np.abs(a) < half)
        cents = self.clusters(x[sel], y[sel])
        out.num_clusters = len(cents)
        out.nearest_cluster_distance = min((math.hypot(cx, cy) for cx, cy in cents), default=float('inf'))
        # TODO(확인): 콘 크기/간격을 보고 너무 큰 클러스터(정차 LIMO, 벽)를 거르는 조건을 추가
        left = [c for c in cents if c[1] > 0 and c[0] > -0.05]
        right = [c for c in cents if c[1] <= 0 and c[0] > -0.05]
        if left and right:
            lc = min(left, key=lambda c: math.hypot(*c))
            rc = min(right, key=lambda c: math.hypot(*c))
            mx, my = (lc[0] + rc[0]) / 2.0, (lc[1] + rc[1]) / 2.0
            out.gap_found = True
            out.gap_angle = float(math.atan2(my, max(mx, 0.05)))
            out.gap_width = float(math.hypot(lc[0] - rc[0], lc[1] - rc[1]))
            return
        # 대안: 전방 부채꼴에서 max_r 안에 아무것도 없는 각도 구간 중 가장 넓은 곳의 가운데
        in_fov = np.abs(a) < half
        if not in_fov.any():
            return
        fa, fr = a[in_fov], r[in_fov]
        free = fr >= max_r
        best_len, best_mid, start = 0, None, None
        for i, f in enumerate(np.append(free, False)):
            if f and start is None:
                start = i
            elif not f and start is not None:
                if i - start > best_len:
                    best_len, best_mid = i - start, (fa[start] + fa[i - 1]) / 2.0
                    width_rad = fa[i - 1] - fa[start]
                start = None
        if best_mid is not None and best_len >= 3:
            out.gap_found = True
            out.gap_angle = float(best_mid)
            out.gap_width = float(width_rad * max_r)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
