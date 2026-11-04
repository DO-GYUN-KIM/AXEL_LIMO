"""/cmd_vel(Twist)을 LIMO 애커먼 드라이버 명령 /limo/ack_cmd(AckermannDrive)로 바꾸는 노드입니다.

LIMO Pro의 ROS2 애커먼 드라이버(limo_base_ackerman.launch.py)는 /cmd_vel을 받지 않고
/limo/ack_cmd만 받습니다(위키 'ROS2 Limo Driver' 화면 기준). 우리 mission_manager는 /cmd_vel을 내므로
이 노드가 중간에서 변환합니다.

- 조향각 = atan(축거 * w / v), ±max_steering_rad로 자름
- 명령이 timeout_sec 동안 안 오면 정지 명령을 냅니다(노드가 죽어도 차가 계속 가지 않도록).
"""
import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .ackermann_math import twist_to_ackermann


class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        d = self.declare_parameter
        d('input_topic', '/cmd_vel')
        d('output_topic', '/limo/ack_cmd')
        d('wheelbase_m', 0.2)            # 위키 스펙: 축거 200 mm
        d('max_steering_rad', 0.42)      # 위키: steering angle -0.42 ~ 0.42
        d('steering_sign', 1.0)          # TODO(확인): 실차에서 +조향이 왼쪽인지 확인, 반대면 -1.0
        d('timeout_sec', 0.3)
        d('rate_hz', 20.0)
        self.gp = lambda n: self.get_parameter(n).value

        self.pub = self.create_publisher(AckermannDrive, self.gp('output_topic'), 10)
        self.create_subscription(Twist, self.gp('input_topic'), self.on_twist, 10)
        self.last = None
        self.last_time = None
        self.create_timer(1.0 / self.gp('rate_hz'), self.tick)
        self.get_logger().info('%s → %s (축거 %.3f m, 조향 한계 ±%.2f rad)' % (
            self.gp('input_topic'), self.gp('output_topic'), self.gp('wheelbase_m'),
            self.gp('max_steering_rad')))

    def on_twist(self, msg):
        self.last = (msg.linear.x, msg.angular.z)
        self.last_time = self.get_clock().now()

    def tick(self):
        out = AckermannDrive()
        fresh = (self.last_time is not None and
                 (self.get_clock().now() - self.last_time).nanoseconds * 1e-9 < self.gp('timeout_sec'))
        if fresh:
            v, w = self.last
            speed, steer, sat = twist_to_ackermann(v, w, self.gp('wheelbase_m'), self.gp('max_steering_rad'))
            if sat:
                self.get_logger().warn('요청 각속도 %.2f rad/s는 속도 %.2f m/s에서 낼 수 없어 조향을 한계로 자릅니다'
                                       % (w, v), throttle_duration_sec=2.0)
            out.speed = float(speed)
            out.steering_angle = float(self.gp('steering_sign') * steer)
        self.pub.publish(out)  # fresh가 아니면 속도 0, 조향 0


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(AckermannDrive())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
