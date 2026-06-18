#!/usr/bin/env python3
"""
Keyboard teleop for RRRR manipulator.

Controls:
  q/a    -> J1 base: spin left / right (velocity)
  space  -> J1 base: stop
  w/s    -> J2 (shoulder)    +/-
  e/d    -> J3 (elbow)       +/-
  r/f    -> J4 (wrist)       +/-
  t/g    -> gripper close/open
  z      -> home (all zeros, base stops)
  x      -> quit
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from math import pi

DT = 0.02          # timer period (s) — 50 Hz

BASE_VEL = 1.5     # rad/s for base spin
STEP_ROT = 0.05    # ~3 deg per keypress for position joints
STEP_LIN = 0.002   # 2 mm per keypress for gripper

# Position limits for the non-continuous joints
JOINT_LIMITS = {
    'joint2': (-pi/2, pi/2),
    'joint3': (-pi/2, pi/2),
    'joint4': (-pi/2, pi/2),
    'finger_left_joint':  (-0.03, 0.0),
    'finger_right_joint': (-0.03, 0.0),
}

HELP = """
╔════════════════════════════════════════╗
║   RRRR Manipulator — Keyboard Teleop   ║
╠════════════════════════════════════════╣
║  q / a    →  base spin left / right    ║
║  space    →  base stop                 ║
║  w / s    →  J2 shoulder       +/-     ║
║  e / d    →  J3 elbow          +/-     ║
║  r / f    →  J4 wrist          +/-     ║
║  t / g    →  gripper close / open      ║
║  z        →  home (all zeros)          ║
║  x        →  quit                      ║
╚════════════════════════════════════════╝
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


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'finger_left_joint', 'finger_right_joint',
        ]
        self.positions = {name: 0.0 for name in self.joint_names}
        self.base_vel = 0.0  # rad/s — integrated each timer tick

        self.create_subscription(Float32, '/base_cmd', self._base_cmd_cb, 10)
        self.timer = self.create_timer(DT, self.timer_cb)
        self.get_logger().info(HELP)

    def _base_cmd_cb(self, msg: Float32):
        self.base_vel = float(msg.data)

    def timer_cb(self):
        # Integrate base velocity into its accumulated angle
        self.positions['joint1'] += self.base_vel * DT

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.positions[n] for n in self.joint_names]
        self.pub.publish(msg)

    def run(self):
        try:
            while rclpy.ok():
                key = get_key()

                if key == 'x':
                    self.get_logger().info('Exiting...')
                    break

                elif key == 'z':
                    for name in self.joint_names:
                        self.positions[name] = 0.0
                    self.base_vel = 0.0
                    self.get_logger().info('Home position')

                elif key == 'q':
                    self.base_vel = BASE_VEL
                    self.get_logger().info('Base: spin left')

                elif key == 'a':
                    self.base_vel = -BASE_VEL
                    self.get_logger().info('Base: spin right')

                elif key == ' ':
                    self.base_vel = 0.0
                    self.get_logger().info('Base: stop')

                elif key in ('w', 's', 'e', 'd', 'r', 'f', 't', 'g'):
                    key_map = {
                        'w': ('joint2',  STEP_ROT),
                        's': ('joint2', -STEP_ROT),
                        'e': ('joint3',  STEP_ROT),
                        'd': ('joint3', -STEP_ROT),
                        'r': ('joint4',  STEP_ROT),
                        'f': ('joint4', -STEP_ROT),
                        't': ('gripper', -STEP_LIN),
                        'g': ('gripper',  STEP_LIN),
                    }
                    joint, delta = key_map[key]
                    if joint == 'gripper':
                        for fn in ('finger_left_joint', 'finger_right_joint'):
                            lo, hi = JOINT_LIMITS[fn]
                            self.positions[fn] = clamp(self.positions[fn] + delta, lo, hi)
                    else:
                        lo, hi = JOINT_LIMITS[joint]
                        self.positions[joint] = clamp(self.positions[joint] + delta, lo, hi)

        except KeyboardInterrupt:
            pass


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
