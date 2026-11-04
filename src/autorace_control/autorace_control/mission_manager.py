"""미션 매니저(상태 머신) 노드입니다. 최종 /cmd_vel은 이 노드만 내보냅니다.

- mission_order(YAML) 순서대로 미션 핸들러를 하나씩 실행합니다.
- start_mission 파라미터로 원하는 미션부터 시작할 수 있습니다(연습, 재출발 규정 대비).
- 주행 중에도 /autorace/jump_to (std_msgs/String, 미션 이름)로 미션을 바꿀 수 있습니다.
- /autorace/estop (std_msgs/Bool) True면 정지합니다.
- 속도 제한: max_linear, max_angular, 미션별 speed_cap.

담당: A (차선/제어/통합)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from autorace_interfaces.msg import DetectionArray, LaneInfo, LidarInfo, MissionState, ParkingSlot

from .missions import DEFAULT_ORDER, MISSION_REGISTRY, Cmd, MissionContext
from .ackermann_math import max_angular as ackermann_max_angular
from .missions.base import clamp


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        d = self.declare_parameter
        d('mission_order', DEFAULT_ORDER)
        d('start_mission', 'start')
        d('control_rate_hz', 20.0)
        d('base_speed', 0.35)  # 규정 권고 0.3~0.4 m/s
        d('slow_speed', 0.18)  # 서행 구간 0.2 m/s 이하
        d('max_linear', 0.4)
        d('max_angular', 1.5)
        # 애커먼 모드만 씀(2026-11-04 결정): 속도별로 낼 수 있는 각속도까지만 내보냅니다.
        d('ackermann_limit', True)
        d('wheelbase_m', 0.2)  # 위키 스펙 축거 200 mm
        d('max_steering_rad', 0.42)  # 위키: 조향 ±0.42 rad
        d('lane_timeout_s', 0.5)
        d('detection_timeout_s', 0.5)
        d('lidar_timeout_s', 0.5)
        d('use_odom', True)  # TODO(확인): /odom이 실제로 나오는지, 주행거리가 정확한지
        d('cmd_vel_topic', '/cmd_vel')
        d('lane_info_topic', '/autorace/lane_info')
        d('lane_cmd_topic', '/autorace/lane_cmd_vel')
        d('detections_topic', '/autorace/detections')
        d('lidar_info_topic', '/autorace/lidar_info')
        d('parking_slot_topic', '/autorace/parking_slot')
        d('parking_timeout_s', 0.5)
        d('odom_topic', '/odom')

        self.order = list(self.gp('mission_order'))
        self.handlers = []
        for name in self.order:
            if name not in MISSION_REGISTRY:
                raise ValueError('알 수 없는 미션 이름: %s (가능: %s)' % (name, list(MISSION_REGISTRY)))
            self.handlers.append(MISSION_REGISTRY[name](self, name))
        start = self.gp('start_mission') or self.order[0]
        if start not in self.order:
            self.get_logger().warn('start_mission=%s 가 mission_order에 없어 처음부터 시작합니다' % start)
            start = self.order[0]
        self.index = self.order.index(start)
        self.started = False
        self.last_result = ''

        self.lane = self.lane_t = None
        self.lane_angular = 0.0
        self.dets = None
        self.dets_t = None
        self.lidar = self.lidar_t = None
        self.parking = self.parking_t = None
        self.pose = None
        self.odom_dist = 0.0
        self.cmd_dist = 0.0
        self.estop = False
        self.last_cmd = Cmd()
        self.last_tick = None

        self.cmd_pub = self.create_publisher(Twist, self.gp('cmd_vel_topic'), 10)
        self.state_pub = self.create_publisher(MissionState, '/autorace/mission_state', 10)
        self.create_subscription(LaneInfo, self.gp('lane_info_topic'), self.on_lane, 10)
        self.create_subscription(Twist, self.gp('lane_cmd_topic'), self.on_lane_cmd, 10)
        self.create_subscription(DetectionArray, self.gp('detections_topic'), self.on_dets, 10)
        self.create_subscription(LidarInfo, self.gp('lidar_info_topic'), self.on_lidar, 10)
        self.create_subscription(ParkingSlot, self.gp('parking_slot_topic'), self.on_parking, 10)
        self.create_subscription(Odometry, self.gp('odom_topic'), self.on_odom, 10)
        self.create_subscription(Bool, '/autorace/estop', self.on_estop, 10)
        self.create_subscription(String, '/autorace/jump_to', self.on_jump, 10)
        self.create_timer(1.0 / self.gp('control_rate_hz'), self.on_timer)
        self.get_logger().info('미션 순서: %s / 시작: %s' % (self.order, start))

    def gp(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- 입력 ----
    def on_lane(self, msg):
        self.lane, self.lane_t = msg, self.now()

    def on_lane_cmd(self, msg):
        self.lane_angular = msg.angular.z

    def on_dets(self, msg):
        self.dets, self.dets_t = msg, self.now()

    def on_lidar(self, msg):
        self.lidar, self.lidar_t = msg, self.now()

    def on_parking(self, msg):
        self.parking, self.parking_t = msg, self.now()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.pose is not None:
            self.odom_dist += math.hypot(p.x - self.pose[0], p.y - self.pose[1])
        self.pose = (p.x, p.y, yaw)

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        self.get_logger().warn('estop = %s' % self.estop)

    def on_jump(self, msg):
        if msg.data not in self.order:
            self.get_logger().warn('jump_to: 알 수 없는 미션 %s' % msg.data)
            return
        self.index = self.order.index(msg.data)
        self.started = False
        self.last_result = 'jump'
        self.get_logger().warn('미션 이동 -> %s' % msg.data)

    # ---- 제어 주기 ----
    def fresh(self, t, timeout, now):
        return t is not None and (now - t) <= timeout

    def build_ctx(self, now):
        lane_ok = self.fresh(self.lane_t, self.gp('lane_timeout_s'), now)
        det_ok = self.fresh(self.dets_t, self.gp('detection_timeout_s'), now)
        lidar_ok = self.fresh(self.lidar_t, self.gp('lidar_timeout_s'), now)
        parking_ok = self.fresh(self.parking_t, self.gp('parking_timeout_s'), now)
        use_odom = self.gp('use_odom') and self.pose is not None
        return MissionContext(
            now=now,
            base_speed=self.gp('base_speed'),
            slow_speed=self.gp('slow_speed'),
            lane=self.lane if lane_ok else None,
            lane_angular=self.lane_angular if lane_ok else 0.0,
            detections=list(self.dets.detections) if det_ok else [],
            image_width=self.dets.image_width if det_ok else 0,
            image_height=self.dets.image_height if det_ok else 0,
            lidar=self.lidar if lidar_ok else None,
            parking=self.parking if parking_ok else None,
            pose=self.pose if use_odom else None,
            traveled=self.odom_dist if use_odom else self.cmd_dist,
        )

    def on_timer(self):
        now = self.now()
        if self.last_tick is not None:
            # odom이 없을 때를 위한 대체 주행거리: 명령 속도를 적분합니다.
            self.cmd_dist += abs(self.last_cmd.linear) * max(0.0, now - self.last_tick)
        self.last_tick = now
        ctx = self.build_ctx(now)
        h = self.handlers[self.index]
        if not self.started:
            h.start(ctx)
            self.started = True
            self.get_logger().info('=== 미션 시작: %s (%d/%d) ===' % (h.name, self.index + 1, len(self.order)))
        cmd = h.tick(ctx)
        if h.is_done():
            self.last_result = '%s:%s' % (h.name, h.result)
            if self.index < len(self.handlers) - 1:
                self.index += 1
                self.started = False
        cmd = self.limit(cmd, h, ctx)
        self.publish(cmd, h, ctx)

    def limit(self, cmd, h, ctx):
        if self.estop:
            return Cmd(0.0, 0.0)
        if cmd.source == 'lane' and ctx.lane is None:
            # 차선 정보가 끊기면(노드 죽음, 카메라 문제) 안전을 위해 정지합니다.
            self.get_logger().warn('차선 정보 없음 -> 정지', throttle_duration_sec=1.0)
            return Cmd(0.0, 0.0)
        vmax = self.gp('max_linear')
        if h.speed_cap is not None:
            vmax = min(vmax, h.speed_cap)
        wmax = self.gp('max_angular')
        v = clamp(cmd.linear, -vmax, vmax)
        if self.gp('ackermann_limit') and abs(v) >= 0.01:
            # 최소 회전 반경(약 0.45 m) 때문에 느릴수록 낼 수 있는 각속도가 작습니다.
            wmax = min(wmax, ackermann_max_angular(v, self.gp('wheelbase_m'), self.gp('max_steering_rad')))
        return Cmd(v, clamp(cmd.angular, -wmax, wmax), cmd.source)

    def publish(self, cmd, h, ctx):
        self.last_cmd = cmd
        t = Twist()
        t.linear.x = float(cmd.linear)
        t.angular.z = float(cmd.angular)
        self.cmd_pub.publish(t)
        s = MissionState()
        s.header.stamp = self.get_clock().now().to_msg()
        s.mission_name = h.name
        s.mission_index = self.order.index(h.name)
        s.phase = h.phase
        s.elapsed = float(h.elapsed(ctx))
        s.last_result = self.last_result
        s.note = h.note
        s.cmd_linear, s.cmd_angular = t.linear.x, t.angular.z
        self.state_pub.publish(s)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
