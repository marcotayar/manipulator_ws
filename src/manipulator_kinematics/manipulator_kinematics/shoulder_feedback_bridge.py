#!/usr/bin/env python3
"""
shoulder_feedback_bridge
========================

Publishes /joint_states so RViz mirrors the real arm on hardware.

  shoulder  measured: /shoulder_feedback (potentiometer, rad, "angle from
            horizontal" as in arm_ik_2d). Falls back to the commanded value
            when the pot has been silent for POT_TIMEOUT seconds.
  elbow, wrist, gripper
            commanded (/arm_command), open loop: no sensor is wired for them.
  base      not shown: J1 is a continuous servo with no encoder.

Before the first /arm_command the arm sits at the safe-start pose.
"""
import time
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray

from manipulator_control import arm_ik_2d as ik
from manipulator_control.arm_telemetry import ESTIMATE_FRAME

POT_TIMEOUT = 1.0     # s before the commanded shoulder replaces a dead pot
GRIP_TRAVEL = 0.03    # finger joint: -0.03 open .. 0.0 closed (see ik_node)


class ShoulderFeedbackBridge(Node):
    def __init__(self):
        super().__init__('shoulder_feedback_bridge')

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'finger_left_joint', 'finger_right_joint',
        ]
        # arm_ik_2d (angle from horizontal) -> URDF (angle from +Z).
        self.shoulder_cmd = ik.START_SHOULDER
        self.elbow_cmd = ik.START_ELBOW
        self.wrist_cmd = ik.START_WRIST
        self.gripper_cmd = 0.0
        self.pot = None
        self.pot_time = None

        self.create_subscription(
            Float32, '/shoulder_feedback', self._feedback_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/arm_command', self._command_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self._publish_state)

        self.get_logger().info(
            'shoulder_feedback_bridge ready: shoulder from pot, '
            'elbow/wrist/gripper from /arm_command -> /joint_states')

    def _feedback_cb(self, msg: Float32):
        self.pot = max(ik.SH_MIN, min(ik.SH_MAX, msg.data))
        self.pot_time = time.monotonic()

    def _command_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            _, self.shoulder_cmd, self.elbow_cmd, self.wrist_cmd, \
                self.gripper_cmd = msg.data[:5]

    def _publish_state(self):
        pot_fresh = (self.pot_time is not None
                     and time.monotonic() - self.pot_time < POT_TIMEOUT)
        shoulder = self.pot if pot_fresh else self.shoulder_cmd
        finger = -GRIP_TRAVEL * (1.0 - self.gripper_cmd)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        # Tells the GUIs this is an estimate, not an encoder reading.
        out.header.frame_id = ESTIMATE_FRAME
        out.name = self.joint_names
        out.position = [
            0.0, pi / 2 - shoulder, -self.elbow_cmd, -self.wrist_cmd,
            finger, finger,
        ]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ShoulderFeedbackBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
