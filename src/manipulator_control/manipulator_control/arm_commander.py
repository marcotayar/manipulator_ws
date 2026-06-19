#!/usr/bin/env python3
"""
arm_commander
=============

Central node that builds the /arm_command message the ESP32 (micro-ROS)
subscribes to.

Outputs (std_msgs/Float32MultiArray, 5 floats):
  [0] base_velocity   -1.0 .. +1.0
  [1] shoulder_angle  rad
  [2] elbow_angle     rad
  [3] wrist_angle     rad
  [4] gripper         -1.0 open .. 0.0 stop .. +1.0 close  (velocity servo)

Inputs:
  /target_pose  (geometry_msgs/Point)   -> 2D IK for shoulder/elbow/wrist
                                            (uses x,y; the planar reach is
                                             sqrt(x^2+y^2), height = z)
  /base_cmd     (std_msgs/Float32)      -> base rotation velocity
  /gripper_cmd  (std_msgs/Float32)      -> gripper velocity -1..1

The base servo is a 360-deg continuous-rotation servo, so it is NOT solved
by IK — its velocity is passed straight through. Point the base manually
to aim the arm plane at the target, then click to set the arm pose.

Publishes /arm_command at 50 Hz so the ESP32 failsafe never trips.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Point
from math import sqrt, degrees, radians

from manipulator_control import arm_ik_2d as ik


class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')

        # Current command state
        self.base_vel = 0.0
        # Safe start: gripper hovers ABOVE ground, low shoulder load.
        self.shoulder = radians(90)    # tucked up
        self.elbow = radians(-60)      # folded
        self.wrist = radians(-80)      # gripper down-ish
        self.gripper = 0.0             # stopped

        # Publisher to ESP32
        self.pub = self.create_publisher(Float32MultiArray, '/arm_command', 10)

        # Inputs
        self.create_subscription(Point, '/target_pose', self.target_cb, 10)
        self.create_subscription(Float32, '/base_cmd', self.base_cb, 10)
        self.create_subscription(Float32, '/gripper_cmd', self.gripper_cb, 10)

        # Publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_cmd)

        self.get_logger().info(
            'arm_commander ready.\n'
            '  /target_pose -> 2D IK (shoulder/elbow/wrist)\n'
            '  /base_cmd    -> base velocity (-1..1)\n'
            '  /gripper_cmd -> gripper velocity (-1 open .. 0 stop .. 1 close)'
        )

    def target_cb(self, msg: Point):
        # Planar reach: horizontal distance in the arm plane, vertical = z
        reach = sqrt(msg.x * msg.x + msg.y * msg.y)
        height = msg.z

        prev = {'shoulder': self.shoulder,
                'elbow': self.elbow,
                'wrist': self.wrist}
        sol = ik.solve(reach, height, prev)
        if sol is None:
            self.get_logger().warn(
                f'IK unreachable: reach={reach:.3f} m, height={height:.3f} m'
            )
            return

        self.shoulder = sol['shoulder']
        self.elbow = sol['elbow']
        self.wrist = sol['wrist']

        self.get_logger().info(
            f'IK ok: reach={reach:.3f} h={height:.3f} | '
            f'shoulder={degrees(self.shoulder):.0f} '
            f'elbow={degrees(self.elbow):.0f} '
            f'wrist={degrees(self.wrist):.0f} '
            f'(phi={degrees(sol["phi"]):.0f})'
        )

    def base_cb(self, msg: Float32):
        self.base_vel = max(-1.0, min(1.0, msg.data))

    def gripper_cb(self, msg: Float32):
        self.gripper = max(-1.0, min(1.0, msg.data))

    def publish_cmd(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.base_vel),
            float(self.shoulder),
            float(self.elbow),
            float(self.wrist),
            float(self.gripper),
        ]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
