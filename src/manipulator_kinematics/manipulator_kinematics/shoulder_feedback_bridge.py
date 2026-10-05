#!/usr/bin/env python3
"""
shoulder_feedback_bridge
========================

Bridges the ESP32 potentiometer reading (/shoulder_feedback, std_msgs/Float32,
radians in arm_ik_2d's "angle from horizontal" convention) into /joint_states,
so RViz mirrors the real shoulder servo position in real time.

Elbow, wrist and gripper are held at the safe-start pose since only the
shoulder has a feedback sensor wired up for now.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from math import pi

from manipulator_control import arm_ik_2d as ik


class ShoulderFeedbackBridge(Node):
    def __init__(self):
        super().__init__('shoulder_feedback_bridge')

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'finger_left_joint', 'finger_right_joint',
        ]
        # angle-from-horizontal (arm_ik_2d) -> angle-from-+Z (URDF), same
        # conversion ik_node.py uses for shoulder.
        self.q2 = pi / 2 - ik.START_SHOULDER
        self.q3 = -ik.START_ELBOW
        self.q4 = -ik.START_WRIST

        self.create_subscription(
            Float32, '/shoulder_feedback', self._feedback_cb, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self._publish_state)

        self.get_logger().info(
            'shoulder_feedback_bridge ready: /shoulder_feedback -> /joint_states')

    def _feedback_cb(self, msg: Float32):
        shoulder_rad = max(ik.SH_MIN, min(ik.SH_MAX, msg.data))
        self.q2 = pi / 2 - shoulder_rad

    def _publish_state(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = self.joint_names
        out.position = [0.0, self.q2, self.q3, self.q4, 0.0, 0.0]
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
