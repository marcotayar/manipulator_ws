#!/usr/bin/env python3
"""
base_gripper_teleop
===================

Keyboard control for the continuous-rotation base servo and the gripper.

Controls:
  j / l  -> base rotate left / right (velocity)
  k      -> base stop
  o      -> gripper open
  p      -> gripper close
  x      -> quit

Publishes:
  /base_cmd    (std_msgs/Float32)  velocity -1..1
  /gripper_cmd (std_msgs/Float32)  0 open .. 1 closed
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

BASE_SPEED = 0.5  # default rotation speed magnitude

HELP = """
╔══════════════════════════════════════╗
║   Base + Gripper Teleop               ║
╠══════════════════════════════════════╣
║  j / l  →  base rotate left / right   ║
║  k      →  base stop                  ║
║  o / p  →  gripper open / close       ║
║  x      →  quit                       ║
╚══════════════════════════════════════╝
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


class BaseGripperTeleop(Node):
    def __init__(self):
        super().__init__('base_gripper_teleop')
        self.base_pub = self.create_publisher(Float32, '/base_cmd', 10)
        self.grip_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        self.get_logger().info(HELP)

    def send_base(self, v):
        m = Float32()
        m.data = float(v)
        self.base_pub.publish(m)

    def send_grip(self, g):
        m = Float32()
        m.data = float(g)
        self.grip_pub.publish(m)

    def run(self):
        while rclpy.ok():
            key = get_key()
            if key == 'x':
                self.send_base(0.0)
                break
            elif key == 'j':
                self.send_base(BASE_SPEED)
                self.get_logger().info('base: rotate left')
            elif key == 'l':
                self.send_base(-BASE_SPEED)
                self.get_logger().info('base: rotate right')
            elif key == 'k':
                self.send_base(0.0)
                self.get_logger().info('base: stop')
            elif key == 'o':
                self.send_grip(0.0)
                self.get_logger().info('gripper: open')
            elif key == 'p':
                self.send_grip(1.0)
                self.get_logger().info('gripper: close')


def main():
    rclpy.init()
    node = BaseGripperTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
