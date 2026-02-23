#!/usr/bin/env python3
import sys, tty, termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

MSG = """
Teleop Rover
  w: adelante   x: atras
  a: curva izq  d: curva der
  s: stop       q: salir
"""

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop')
        self.pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        self.linear = 0.5
        self.angular = 1.0

    def send(self, lx, az):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(lx)
        msg.twist.angular.z = float(az)
        self.pub.publish(msg)

    def run(self):
        print(MSG)
        try:
            while True:
                key = get_key()
                if   key == 'w': self.send( self.linear,  0.0);         print("adelante")
                elif key == 'x': self.send(-self.linear,  0.0);         print("atras")
                elif key == 'a': self.send( self.linear,  self.angular); print("curva izq")
                elif key == 'd': self.send( self.linear, -self.angular); print("curva der")
                elif key == 's': self.send( 0.0,  0.0);                 print("stop")
                elif key == 'q' or key == '\x03': break
        finally:
            self.send(0.0, 0.0)

def main():
    rclpy.init()
    node = RoverTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
