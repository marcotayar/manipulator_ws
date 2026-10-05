#!/usr/bin/env python3
"""
Geometric inverse kinematics for RRRR manipulator.

Uses tf2 to read actual EE position from the URDF TF tree (ground truth),
so the trajectory marker always matches the real robot model.

IK is delegated to manipulator_control.arm_ik_2d, the single source of truth
for arm geometry, joint limits, and the EE approach-angle search (straight
down preferred, not enforced), so the simulation and the hardware commander
accept exactly the same targets.
Returns None (no motion) for any target outside the reachable workspace
or joint limits.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool, ColorRGBA, Float32
from builtin_interfaces.msg import Duration
from math import sqrt, pi

from manipulator_control import arm_ik_2d as ik2
from manipulator_control.estop import ESTOP_QOS, ESTOP_TOPIC

import tf2_ros


# Only the gripper offset is needed locally (TF-based EE tip reconstruction).
# All arm geometry and joint limits live in arm_ik_2d — single source of truth.
L_GRIP = 0.16        # wrist → EE tip: gripper_base(0.03) + fingers(0.13)


# Simulation loop
DT = 0.02          # timer period (s) — 50 Hz
N_STEPS = 50
STEP_DELAY = 0.03  # seconds between interpolation steps

# /base_cmd is normalized (-1..1); this is the rad/s it maps to at full
# command. Matches BASE_VEL in manipulator_control.keyboard_teleop so both
# sim drivers spin the turntable at the same rate.
BASE_MAX_RADPS = 1.5

# Finger travel. Measured from the URDF TF tree: joint value 0.0 puts the
# fingers at their 0.030 m nominal gap (CLOSED) and -grip_upper swings both
# outward to 0.090 m (OPEN). The mirrored `axis` signs keep the pair
# symmetric, so both joints take the same value.
GRIP_TRAVEL = 0.03


def solve_ik(x, y, z, logger=None):
    """Solve IK for target (x, y, z) in world frame.

    Delegates to ``manipulator_control.arm_ik_2d`` so the simulation and the
    hardware commander share one workspace and one set of joint limits. That
    solver works in the "angle from horizontal" convention with a searched EE
    approach angle; the URDF measures every joint from +Z (all links up at
    q=0, positive rotation taking +Z toward +X), hence the conversion below.

    Returns (theta1, q2, q3, q4) or None if unreachable.
    """
    solution = ik2.solve_cartesian(x, y, z)
    if solution is None:
        if logger:
            logger.warn(
                f'Unreachable target ({x:.3f}, {y:.3f}, {z:.3f}) — '
                'outside the shared arm_ik_2d workspace.'
            )
        return None

    # angle-from-horizontal (arm_ik_2d) -> angle-from-+Z (URDF)
    q2 = pi / 2 - solution['shoulder']
    q3 = -solution['elbow']
    q4 = -solution['wrist']
    return solution['base_yaw'], q2, q3, q4


class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')

        self.sub = self.create_subscription(
            Point, '/target_pose', self.target_cb, 10
        )
        self.create_subscription(Float32, '/base_cmd', self.base_cmd_cb, 10)
        self.create_subscription(
            Float32, '/gripper_cmd', self.gripper_cmd_cb, 10)
        self.create_subscription(Bool, ESTOP_TOPIC, self.estop_cb, ESTOP_QOS)
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
        # Start open, matching arm_commander's initial gripper command of 0.0.
        self.gripper_pos = -GRIP_TRAVEL
        self.estop = False
        self.base_cmd = 0.0      # normalized -1..1, integrated into joint1
        self.trajectory_points = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(DT, self.timer_cb)

        self.get_logger().info(
            'IK node ready. Publish a Point to /target_pose.\n'
            '  EE approach angle is searched, straight down preferred.\n'
            '  Shared workspace/limits with arm_commander (arm_ik_2d): '
            f'reach <= {ik2.L1 + ik2.L2 + ik2.L3:.3f} m, tip height >= '
            f'{ik2.GROUND_CLEAR:.3f} m.'
        )

    def estop_cb(self, msg: Bool):
        """Latched stop: freeze in place, ignore commands until released."""
        self.estop = bool(msg.data)
        if self.estop:
            self.base_cmd = 0.0
            self.is_moving = False

    def base_cmd_cb(self, msg: Float32):
        """Normalized base velocity (-1..1), as published by the Qt panel."""
        if not self.estop:
            self.base_cmd = max(-1.0, min(1.0, float(msg.data)))

    def gripper_cmd_cb(self, msg: Float32):
        """Gripper position: 0.0 open .. 1.0 closed."""
        if self.estop:
            return
        closure = max(0.0, min(1.0, float(msg.data)))
        self.gripper_pos = -GRIP_TRAVEL * (1.0 - closure)

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
        if self.estop:
            return
        target = solve_ik(msg.x, msg.y, msg.z, self.get_logger())
        if target is None:
            return  # unreachable — don't move

        self.get_logger().info(
            f'Cylindrical target: yaw={target[0]:.2f} rad, '
            f'radius={sqrt(msg.x * msg.x + msg.y * msg.y):.3f} m, '
            f'height={msg.z:.3f} m → J1={target[0]:.2f} J2={target[1]:.2f} '
            f'J3={target[2]:.2f} J4={target[3]:.2f}'
        )

        self.target_q = list(target)
        self.move_start_q = list(self.current_q)
        self.move_progress = 0.0
        self.is_moving = True
        self.target_msg = msg
        self.check_pending = False

    def timer_cb(self):
        # Base is a continuous-rotation servo: velocity integrates into the
        # joint1 angle. Shift the interpolation endpoints by the same delta so
        # an in-flight IK move doesn't undo the operator's jog.
        if self.base_cmd != 0.0:
            delta = self.base_cmd * BASE_MAX_RADPS * DT
            self.current_q[0] += delta
            self.target_q[0] += delta
            self.move_start_q[0] += delta

        if self.is_moving:
            duration = N_STEPS * STEP_DELAY
            step = DT / duration
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
