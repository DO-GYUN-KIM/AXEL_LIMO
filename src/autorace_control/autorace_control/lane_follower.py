"""차선 추종 노드입니다.

LaneInfo(center_offset, heading_error)를 받아 조향 각속도를 계산하고
/autorace/lane_cmd_vel(Twist)로 내보냅니다. 이 값은 바로 로봇으로 가지 않고
mission_manager가 받아서 미션에 맞게 속도를 바꾸거나 무시한 뒤 /cmd_vel로 내보냅니다.

제어 방식(mode 파라미터):
  - 'pid'     : angular = kp*offset + ki*∫offset + kd*d(offset)/dt + k_heading*heading
  - 'stanley' : 조향각 = heading + atan(k_stanley*offset / (v + eps)), angular = v*tan(조향각)/wheelbase

담당: A (차선/제어/통합)
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from autorace_interfaces.msg import LaneInfo


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        d = self.declare_parameter
        d('lane_info_topic', '/autorace/lane_info')
        d('cmd_topic', '/autorace/lane_cmd_vel')
        d('mode', 'pid')
        d('speed', 0.35)  # 기본 속도 [m/s] (미션 매니저가 다시 덮어쓸 수 있습니다)
        d('kp', 2.0)
        d('ki', 0.0)
        d('kd', 0.2)
        d('k_heading', 1.0)
        d('k_stanley', 1.5)
        d('wheelbase_m', 0.2)  # TODO(확인): LIMO Pro 축거 실측
        d('max_angular', 1.5)
        d('integral_limit', 0.5)
        d('lost_decay', 0.9)  # 차선을 잃었을 때 직전 각속도를 매 프레임 이 비율로 줄입니다.

        self.pub = self.create_publisher(Twist, self.gp('cmd_topic'), 10)
        self.create_subscription(LaneInfo, self.gp('lane_info_topic'), self.on_lane, 10)
        self.integral = 0.0
        self.prev_offset = None
        self.prev_t = None
        self.last_angular = 0.0

    def gp(self, name):
        return self.get_parameter(name).value

    def on_lane(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self.prev_t) if self.prev_t is not None else 0.0
        self.prev_t = now
        v = float(self.gp('speed'))

        if not msg.lane_valid:
            # 차선을 잃으면 직전 조향을 조금씩 줄이며 유지합니다(멈추지 않음).
            self.last_angular *= self.gp('lost_decay')
            self.prev_offset = None
            self.publish(v, self.last_angular)
            return

        off, head = msg.center_offset, msg.heading_error
        if self.gp('mode') == 'stanley':
            steer = head + math.atan2(self.gp('k_stanley') * off, v + 0.1)
            angular = v * math.tan(clamp(steer, -1.2, 1.2)) / self.gp('wheelbase_m')
        else:
            if dt > 0.0:
                lim = self.gp('integral_limit')
                self.integral = clamp(self.integral + off * dt, -lim, lim)
            deriv = 0.0
            if self.prev_offset is not None and dt > 1e-3:
                deriv = (off - self.prev_offset) / dt
            angular = (self.gp('kp') * off + self.gp('ki') * self.integral
                       + self.gp('kd') * deriv + self.gp('k_heading') * head)
        self.prev_offset = off
        m = self.gp('max_angular')
        self.last_angular = clamp(angular, -m, m)
        self.publish(v, self.last_angular)

    def publish(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
