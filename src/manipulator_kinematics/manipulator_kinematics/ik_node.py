#!/usr/bin/env python3
"""
Geometric inverse kinematics for RRRR manipulator.

Uses tf2 to read actual EE position from the URDF TF tree (ground truth),
so the trajectory marker always matches the real robot model.

IK is solved geometrically in the vertical plane.
Gripper is constrained to point straight DOWN.
Returns None (no motion) for any target outside the reachable workspace
or joint limits.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration
from math import atan2, sqrt, acos, pi, cos, sin

import tf2_ros


# Robot dimensions — must match URDF (manipulator.urdf.xacro)
BASE_HEIGHT = 0.09   # base_link(0.07) + turntable(0.02)
L1 = 0.10            # shoulder → elbow
L2 = 0.09            # elbow → wrist
L_GRIP = 0.16        # wrist → EE tip: gripper_base(0.03) + fingers(0.13)

# Joint limits (rad) — must match URDF
J2_LIMIT = pi / 2
J3_LIMIT = pi / 2
J4_LIMIT = pi / 2

# Trajectory interpolation
N_STEPS = 50
STEP_DELAY = 0.03  # seconds between steps


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def solve_ik(x, y, z, logger=None):
    """
    Solve IK for target (x, y, z) in world frame.
    Gripper points straight DOWN.

    URDF convention: at q=[0,0,0,0] all links point UP (+Z).
    J2/J3/J4 rotate about the local Y axis; positive rotation takes +Z toward +X.
    So for a link of length L at angle q from vertical:
      horizontal component = L * sin(q)
      vertical component   = L * cos(q)

    Returns (theta1, q2, q3, q4) or None if unreachable.
    """
    if z < 0:
        if logger:
            logger.warn(f'Target below ground: z={z:.3f}')
        return None

    # J1: base yaw (continuous, no position limit)
    theta1 = atan2(y, x)

    # Horizontal distance from base axis to target
    r_target = sqrt(x * x + y * y)

    # Wrist must be directly above target (gripper hangs straight down)
    r_w = r_target
    h_w = (z + L_GRIP) - BASE_HEIGHT   # wrist height relative to shoulder

    d_sq = r_w * r_w + h_w * h_w
    d = sqrt(d_sq)

    if d >= L1 + L2:
        if logger:
            logger.warn(f'Out of reach: d={d:.3f} m, max={L1+L2:.3f} m')
        return None
    if d <= abs(L1 - L2):
        if logger:
            logger.warn(f'Too close to shoulder: d={d:.3f} m, min={abs(L1-L2):.3f} m')
        return None

    cos_q3 = clamp((d_sq - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1.0, 1.0)
    alpha = atan2(r_w, h_w)   # angle from +Z toward +X of shoulder→wrist vector

    best = None
    best_cost = float('inf')

    for q3_val in [acos(cos_q3), -acos(cos_q3)]:
        beta = atan2(L2 * sin(q3_val), L1 + L2 * cos(q3_val))
        q2_val = alpha - beta
        q4_val = pi - q2_val - q3_val   # gripper points down constraint

        if abs(q2_val) > J2_LIMIT:
            continue
        if abs(q3_val) > J3_LIMIT:
            continue
        if abs(q4_val) > J4_LIMIT:
            continue

        cost = abs(q2_val) + abs(q3_val) + abs(q4_val)
        if cost < best_cost:
            best_cost = cost
            best = (q2_val, q3_val, q4_val)

    if best is None:
        if logger:
            logger.warn(
                f'No configuration within joint limits for '
                f'({x:.3f}, {y:.3f}, {z:.3f})'
            )
        return None

    q2, q3, q4 = best
    return clamp(theta1, -pi, pi), q2, q3, q4


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.sub = self.create_subscription(
            Point, '/target_pose', self.target_cb, 10
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/ee_trajectory', 10
        )

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'finger_left_joint', 'finger_right_joint',
        ]
        self.current_q = [0.0, 0.0, 0.0, 0.0]
        self.target_q = [0.0, 0.0, 0.0, 0.0]
        self.move_start_q = [0.0, 0.0, 0.0, 0.0]
        self.is_moving = False
        self.move_progress = 0.0
        self.gripper_pos = 0.0
        self.trajectory_points = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.02, self.timer_cb)

        self.get_logger().info(
            'IK node ready. Publish a Point to /target_pose.\n'
            '  Gripper points DOWN at target.\n'
            f'  Reachable annulus: {abs(L1-L2):.3f} m < r < {L1+L2:.3f} m '
            f'(horizontal distance from base axis, at ground level).'
        )

    def get_ee_position_from_tf(self):
        """Compute EE tip position from the TF tree (ground truth)."""
        try:
            t = self.tf_buffer.lookup_transform(
                'world', 'gripper_base', rclpy.time.Time()
            )
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            # Z-column of rotation matrix: direction of gripper_base +Z axis in world
            zx = 2 * (qx * qz + qw * qy)
            zy = 2 * (qy * qz - qw * qx)
            zz = 1 - 2 * (qx * qx + qy * qy)

            # EE tip is L_GRIP along gripper_base +Z
            p = Point()
            p.x = t.transform.translation.x + L_GRIP * zx
            p.y = t.transform.translation.y + L_GRIP * zy
            p.z = t.transform.translation.z + L_GRIP * zz
            return p
        except Exception:
            return None

    def target_cb(self, msg: Point):
        target = solve_ik(msg.x, msg.y, msg.z, self.get_logger())
        if target is None:
            return  # unreachable — don't move

        self.get_logger().info(
            f'Target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) → '
            f'J1={target[0]:.2f} J2={target[1]:.2f} '
            f'J3={target[2]:.2f} J4={target[3]:.2f}'
        )

        self.target_q = list(target)
        self.move_start_q = list(self.current_q)
        self.move_progress = 0.0
        self.is_moving = True
        self.target_msg = msg
        self.check_pending = False

    def timer_cb(self):
        if self.is_moving:
            duration = N_STEPS * STEP_DELAY
            step = 0.02 / duration
            self.move_progress += step

            if self.move_progress >= 1.0:
                self.move_progress = 1.0
                self.is_moving = False
                self.check_pending = True
                self.check_delay_ticks = 10

            t = self.move_progress
            self.current_q = [
                s + (e - s) * t
                for s, e in zip(self.move_start_q, self.target_q)
            ]

            ee_pos = self.get_ee_position_from_tf()
            if ee_pos is not None:
                prev = self.trajectory_points
                if not prev or sqrt(
                    (ee_pos.x - prev[-1].x) ** 2 +
                    (ee_pos.y - prev[-1].y) ** 2 +
                    (ee_pos.z - prev[-1].z) ** 2
                ) > 0.001:
                    self.trajectory_points.append(ee_pos)
                    self.publish_trajectory()

        elif getattr(self, 'check_pending', False):
            self.check_delay_ticks -= 1
            if self.check_delay_ticks <= 0:
                self.check_pending = False
                final_pos = self.get_ee_position_from_tf()
                if final_pos and hasattr(self, 'target_msg'):
                    err = sqrt(
                        (final_pos.x - self.target_msg.x) ** 2 +
                        (final_pos.y - self.target_msg.y) ** 2 +
                        (final_pos.z - self.target_msg.z) ** 2
                    )
                    self.get_logger().info(
                        f'  EE actual: ({final_pos.x:.3f}, {final_pos.y:.3f}, {final_pos.z:.3f})\n'
                        f'  Position error: {err * 100:.1f} cm'
                    )

        self.publish_state()

    def publish_trajectory(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ee_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.color = ColorRGBA(r=1.0, g=0.3, b=0.1, a=0.9)
        marker.lifetime = Duration(sec=30)

        if len(self.trajectory_points) > 500:
            self.trajectory_points = self.trajectory_points[-500:]

        marker.points = self.trajectory_points
        self.marker_pub.publish(marker)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.current_q) + [self.gripper_pos, self.gripper_pos]
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
