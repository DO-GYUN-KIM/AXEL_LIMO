#!/usr/bin/env python3
"""/limo/ack_cmd(AckermannDrive) → /sim/ackermann_cmd(AckermannDriveStamped) 중계 노드입니다.

실차 드라이버는 헤더 없는 AckermannDrive를 받지만, Isaac Sim 4.5의 'ROS2 Subscribe AckermannDrive' 노드는
AckermannDriveStamped를 받습니다. 시뮬레이터에서만 이 노드를 띄워 우리 코드(cmd_vel_to_ackermann)를 그대로 씁니다.

실행: python3 ack_stamp_relay.py --ros-args -p use_sim_time:=true [-p speed_sign:=-1.0]
"""
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from rclpy.node import Node


class AckStampRelay(Node):
    def __init__(self):
        super().__init__('ack_stamp_relay')
        self.declare_parameter('input_topic', '/limo/ack_cmd')
        self.declare_parameter('output_topic', '/sim/ackermann_cmd')
        self.declare_parameter('frame_id', 'base_link')
        # 시뮬레이터에서 앞으로 명령했는데 뒤로 가면 -1.0 (모델 바퀴 축 방향 문제일 때만, 실차와 무관)
        self.declare_parameter('speed_sign', 1.0)
        gp = lambda n: self.get_parameter(n).value
        self.frame_id = gp('frame_id')
        self.speed_sign = float(gp('speed_sign'))
        self.pub = self.create_publisher(AckermannDriveStamped, gp('output_topic'), 10)
        self.create_subscription(AckermannDrive, gp('input_topic'), self.on_cmd, 10)
        self.get_logger().info('%s → %s' % (gp('input_topic'), gp('output_topic')))

    def on_cmd(self, msg):
        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.drive = msg
        out.drive.speed = self.speed_sign * msg.speed
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AckStampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(AckermannDriveStamped())  # 끝날 때 정지 명령
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
