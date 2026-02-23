#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class MecanumDrive(Node):
    def __init__(self):
        super().__init__('mecanum_drive')
        self.pub = self.create_publisher(Float64MultiArray, '/velocity_cont/commands', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # distancias del centro a las ruedas
        self.lx = 0.131  # mitad largo
        self.ly = 0.108  # mitad ancho

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        l = self.lx + self.ly

        # Cinemática Mecanum
        # orden: [mleft, mright, left, right]
        fl = (vx - vy - l * wz)  # delantera izq
        fr = (vx + vy + l * wz)  # delantera der
        rl = (vx + vy - l * wz)  # trasera izq
        rr = (vx - vy + l * wz)  # trasera der

        out = Float64MultiArray()
        out.data = [rl, rr, fl, fr]  # mleft, mright, left, right
        self.pub.publish(out)

def main():
    rclpy.init()
    node = MecanumDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
