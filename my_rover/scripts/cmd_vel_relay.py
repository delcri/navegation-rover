#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)

    def callback(self, msg):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
