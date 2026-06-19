#!/usr/bin/env python3
"""
Translate /joy messages to arm control topics.

Connect any USB or Bluetooth gamepad to the PC, then run:
    sudo apt install ros-humble-joy
    ros2 run joy joy_node
    ros2 run manipulator_control joy_to_arm

Button mapping — PS3/PS4 connected via USB/BT on Linux.
If your controller maps differently, adjust the index constants below.

    L1  (buttons[4])         base rotate left   (-0.5 rad/s)
    R1  (buttons[5])         base rotate right  (+0.5 rad/s)
    D-pad up   (axes[7]+)    EE height +5 mm
    D-pad down (axes[7]-)    EE height -5 mm
    D-pad right (axes[6]-)   EE reach  +5 mm
    D-pad left  (axes[6]+)   EE reach  -5 mm
    Triangle / Y (buttons[3]) gripper open   (hold to keep spinning)
    Square   / X (buttons[2]) gripper close  (hold to keep pressing)

Publishes:
    /base_cmd     std_msgs/Float32          base velocity -1..1
    /target_pose  geometry_msgs/Point       x=reach, y=0, z=height
    /gripper_cmd  std_msgs/Float32          -1=open, 0=stop, +1=close
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

# ── Button / axis indices (PS3/PS4 on Linux) ──────────────
# Run `ros2 topic echo /joy` to find your controller's mapping.
BTN_L1       = 4
BTN_R1       = 5
BTN_OPEN     = 3   # Triangle / Y
BTN_CLOSE    = 2   # Square   / X
AXIS_DPAD_H  = 6   # +1 = left,  -1 = right
AXIS_DPAD_V  = 7   # +1 = up,    -1 = down

STEP       = 0.005   # 5 mm per d-pad press
BASE_SPEED = 0.5     # rad/s when L1/R1 held
REACH_MIN  = 0.11
REACH_MAX  = 0.33    # out to near-full extension; IK rejects anything unreachable
HEIGHT_MIN = 0.00
HEIGHT_MAX = 0.15


class JoyToArm(Node):
    def __init__(self):
        super().__init__('joy_to_arm')

        self.sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.pub_base  = self.create_publisher(Float32, '/base_cmd',    10)
        self.pub_pose  = self.create_publisher(Point,   '/target_pose', 10)
        self.pub_grip  = self.create_publisher(Float32, '/gripper_cmd', 10)

        self.reach   = 0.140
        self.height  = 0.000
        self.gripper = 0.0

        self._prev_buttons = []
        self._prev_axes    = []

        # No heartbeat: arm_commander holds state and republishes /arm_command
        # at 50 Hz on its own. We only publish a target when the D-pad actually
        # moves it, so the arm keeps its horizontal-extended startup pose until
        # the operator commands a reachable target.
        self.get_logger().info(
            'joy_to_arm ready — connect a gamepad and run: ros2 run joy joy_node')

    # ── /joy callback ─────────────────────────────────────
    def _joy_cb(self, msg: Joy):
        btns = msg.buttons
        axes = msg.axes

        if len(self._prev_buttons) != len(btns):
            self._prev_buttons = list(btns)
        if len(self._prev_axes) != len(axes):
            self._prev_axes = list(axes)

        # ── Base (held) ──────────────────────────────────
        r1 = btns[BTN_R1] if len(btns) > BTN_R1 else 0
        l1 = btns[BTN_L1] if len(btns) > BTN_L1 else 0
        base_v = BASE_SPEED if (r1 and not l1) else \
                -BASE_SPEED if (l1 and not r1) else 0.0

        base_msg = Float32()
        base_msg.data = float(base_v)
        self.pub_base.publish(base_msg)

        # ── D-pad (edge-triggered steps) ─────────────────
        changed = False

        if len(axes) > AXIS_DPAD_V:
            dv = axes[AXIS_DPAD_V]
            pv = self._prev_axes[AXIS_DPAD_V]
            if dv > 0.5 and pv <= 0.5:
                self.height = min(self.height + STEP, HEIGHT_MAX)
                changed = True
            elif dv < -0.5 and pv >= -0.5:
                self.height = max(self.height - STEP, HEIGHT_MIN)
                changed = True

        if len(axes) > AXIS_DPAD_H:
            dh = axes[AXIS_DPAD_H]
            ph = self._prev_axes[AXIS_DPAD_H]
            if dh < -0.5 and ph >= -0.5:   # right on d-pad = axis goes negative
                self.reach = min(self.reach + STEP, REACH_MAX)
                changed = True
            elif dh > 0.5 and ph <= 0.5:
                self.reach = max(self.reach - STEP, REACH_MIN)
                changed = True

        if changed:
            pose = Point()
            pose.x = self.reach
            pose.z = self.height
            self.pub_pose.publish(pose)
            self.get_logger().info(
                f'EE target  reach={self.reach:.3f} m  height={self.height:.3f} m')

        # ── Gripper (held: Y=open, X=close, neither=stop) ───
        y = btns[BTN_OPEN]  if len(btns) > BTN_OPEN  else 0
        x = btns[BTN_CLOSE] if len(btns) > BTN_CLOSE else 0
        gripper_v = -1.0 if (y and not x) else \
                     1.0 if (x and not y) else 0.0

        if gripper_v != self.gripper:
            self.gripper = gripper_v
            grip = Float32()
            grip.data = float(gripper_v)
            self.pub_grip.publish(grip)
            if gripper_v < 0:   self.get_logger().info('Gripper: opening')
            elif gripper_v > 0: self.get_logger().info('Gripper: closing')
            else:                self.get_logger().info('Gripper: stop')

        self._prev_buttons = list(btns)
        self._prev_axes    = list(axes)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
