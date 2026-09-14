#!/usr/bin/env python3
"""Closed-loop numerical plant for the manipulator.

The node consumes the same /arm_command vector as the ESP32. Unlike the RViz
kinematic driver, it keeps desired and measured state separate: PID outputs
actuator effort, a numerical rigid-link approximation advances the plant, and
only the simulated encoder state is published on /joint_states.
"""
from math import pi, sin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from manipulator_control import arm_ik_2d as ik

DT = 0.005                 # plant/controller integration, 200 Hz
PUBLISH_DIVISOR = 4        # ROS state/diagnostics, 50 Hz
BASE_MAX_RADPS = 1.5
GRIP_TRAVEL = 0.03         # each finger: -0.03 open, 0.0 closed
GRAVITY = 9.80665

# Diagonal inertia approximation (kg m^2). The plant includes distal masses in
# each upstream joint. These are nominal URDF-derived values, not identified
# hardware parameters; replace them after measuring the real arm.
INERTIA = (0.0012, 0.0100, 0.0038, 0.0008)
DAMPING = (0.003, 0.025, 0.012, 0.005)
EFFORT_LIMIT = (2.2, 4.8, 2.2, 2.2)

# Position PID gains [J2, J3, J4]. Derivative is on measurement, avoiding
# derivative kick when IK changes the setpoint. Conditional integration is
# used for saturation anti-windup.
POSITION_GAINS = (
    (0.90, 0.22, 0.15),
    (0.42, 0.12, 0.075),
    (0.16, 0.045, 0.030),
)
BASE_GAINS = (0.035, 0.080, 0.0015)       # velocity PID -> base torque
GRIPPER_GAINS = (18.0, 5.0, 0.8)          # position PID -> normalized effort
BASE_DERIVATIVE_ALPHA = 0.95               # low-pass measured acceleration
JOINT_LOWER = (-float('inf'), -pi / 2, -pi / 2, -pi / 2)
JOINT_UPPER = (float('inf'), pi / 2, pi / 2, pi / 2)


class PID:
    """PID with derivative-on-measurement and conditional anti-windup."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0

    def update(self, setpoint, measurement, measurement_rate, dt):
        error = setpoint - measurement
        candidate_i = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * dt),
        )
        raw = self.kp * error + self.ki * candidate_i - self.kd * measurement_rate
        output = max(-self.output_limit, min(self.output_limit, raw))

        # Integrate only when unsaturated, or when the error would drive the
        # saturated output back toward the controllable range.
        if raw == output or (raw > output and error < 0.0) or (raw < output and error > 0.0):
            self.integral = candidate_i
        return output, error


class PIDSimulation(Node):
    def __init__(self):
        super().__init__('pid_sim')

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'finger_left_joint', 'finger_right_joint',
        ]
        self.arm_command = [
            0.0, ik.START_SHOULDER, ik.START_ELBOW, ik.START_WRIST, 0.0,
        ]
        self.desired_q = self._arm_command_to_urdf(self.arm_command)
        self.q = list(self.desired_q)
        self.qd = [0.0] * 4
        self.gripper = 0.0                # normalized closure: 0 open, 1 closed
        self.gripper_velocity = 0.0
        self.previous_base_velocity = 0.0
        self.filtered_base_acceleration = 0.0
        self.desired_gripper = 0.0
        self.disturbance = [0.0] * 5
        self.tick = 0

        self.position_pid = [
            PID(kp, ki, kd, limit, integral_limit=2.0)
            for (kp, ki, kd), limit in zip(POSITION_GAINS, EFFORT_LIMIT[1:])
        ]
        self.base_pid = PID(*BASE_GAINS, EFFORT_LIMIT[0], integral_limit=5.0)
        self.gripper_pid = PID(*GRIPPER_GAINS, 1.0, integral_limit=0.5)

        self.create_subscription(
            Float32MultiArray, '/arm_command', self.arm_command_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/sim_disturbance', self.disturbance_cb, 10)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.setpoint_pub = self.create_publisher(JointState, '/joint_setpoint', 10)
        self.diag_pub = self.create_publisher(
            Float32MultiArray, '/pid_diagnostics', 10)
        self.timer = self.create_timer(DT, self.timer_cb)

        self.last_effort = [0.0] * 5
        self.last_error = [0.0] * 5
        self.get_logger().info(
            'PID numerical simulation ready.\n'
            '  /arm_command      -> controller setpoints\n'
            '  /joint_states     -> simulated encoder measurements\n'
            '  /joint_setpoint   -> desired joint positions/velocity\n'
            '  /pid_diagnostics  -> [errors(5), efforts(5)]\n'
            '  /sim_disturbance  -> additive [J1,J2,J3,J4,gripper] effort'
        )

    @staticmethod
    def _arm_command_to_urdf(command):
        # /arm_command uses the physical arm convention (angle from horizontal).
        return [
            0.0,
            pi / 2 - command[1],
            -command[2],
            -command[3],
        ]

    def arm_command_cb(self, msg):
        if len(msg.data) < 5:
            self.get_logger().warn('/arm_command requires 5 values')
            return
        self.arm_command = [float(v) for v in msg.data[:5]]
        converted = self._arm_command_to_urdf(self.arm_command)
        self.desired_q[1:] = converted[1:]
        self.desired_gripper = max(0.0, min(1.0, self.arm_command[4]))

    def disturbance_cb(self, msg):
        values = list(msg.data[:5])
        self.disturbance = values + [0.0] * (5 - len(values))

    def gravity_torque(self):
        """Approximate planar gravity vector in URDF joint coordinates."""
        q2, q3, q4 = self.q[1], self.q[2], self.q[3]
        s2 = sin(q2)
        s23 = sin(q2 + q3)
        s234 = sin(q2 + q3 + q4)
        m1, m2 = 0.095, 0.095
        mg, mf = 0.080, 0.010
        distal = mg + 2.0 * mf
        g2 = -GRAVITY * (
            m1 * ik.L1 * 0.5 * s2
            + m2 * (ik.L1 * s2 + ik.L2 * 0.5 * s23)
            + distal * (ik.L1 * s2 + ik.L2 * s23 + ik.L3 * 0.5 * s234)
        )
        g3 = -GRAVITY * (
            m2 * ik.L2 * 0.5 * s23
            + distal * (ik.L2 * s23 + ik.L3 * 0.5 * s234)
        )
        g4 = -GRAVITY * distal * ik.L3 * 0.5 * s234
        return 0.0, g2, g3, g4

    def timer_cb(self):
        base_velocity_sp = max(-1.0, min(1.0, self.arm_command[0])) * BASE_MAX_RADPS
        raw_base_acceleration = (
            self.qd[0] - self.previous_base_velocity) / DT
        self.previous_base_velocity = self.qd[0]
        self.filtered_base_acceleration = (
            BASE_DERIVATIVE_ALPHA * self.filtered_base_acceleration
            + (1.0 - BASE_DERIVATIVE_ALPHA) * raw_base_acceleration
        )
        base_effort, base_error = self.base_pid.update(
            base_velocity_sp, self.qd[0],
            self.filtered_base_acceleration, DT)

        gravity = self.gravity_torque()
        efforts = [base_effort]
        errors = [base_error]
        for i, pid in enumerate(self.position_pid, start=1):
            feedback, error = pid.update(
                self.desired_q[i], self.q[i], self.qd[i], DT)
            # Feed forward the torque predicted to hold the current pose;
            # PID then rejects model error and external disturbance.
            effort = max(
                -EFFORT_LIMIT[i],
                min(EFFORT_LIMIT[i], feedback + gravity[i]),
            )
            efforts.append(effort)
            errors.append(error)

        # Diagonal rigid-link approximation: I*qdd = tau - b*qd - gravity.
        for i in range(4):
            applied = efforts[i] + self.disturbance[i]
            qdd = (applied - DAMPING[i] * self.qd[i] - gravity[i]) / INERTIA[i]
            self.qd[i] += qdd * DT
            self.q[i] += self.qd[i] * DT

            # Ideal hard stops at the URDF limits. Cancel only velocity that
            # points farther into a stop; motion back into range remains free.
            if self.q[i] <= JOINT_LOWER[i]:
                self.q[i] = JOINT_LOWER[i]
                self.qd[i] = max(0.0, self.qd[i])
            elif self.q[i] >= JOINT_UPPER[i]:
                self.q[i] = JOINT_UPPER[i]
                self.qd[i] = min(0.0, self.qd[i])

        grip_effort, grip_error = self.gripper_pid.update(
            self.desired_gripper, self.gripper, self.gripper_velocity, DT)
        grip_accel = grip_effort + self.disturbance[4] - 2.0 * self.gripper_velocity
        self.gripper_velocity += grip_accel * DT
        self.gripper += self.gripper_velocity * DT
        if self.gripper <= 0.0:
            self.gripper = 0.0
            self.gripper_velocity = max(0.0, self.gripper_velocity)
        elif self.gripper >= 1.0:
            self.gripper = 1.0
            self.gripper_velocity = min(0.0, self.gripper_velocity)

        self.last_effort = efforts + [grip_effort]
        self.last_error = errors + [grip_error]
        self.tick += 1
        if self.tick % PUBLISH_DIVISOR == 0:
            self.publish_state(base_velocity_sp)

    def publish_state(self, base_velocity_sp):
        now = self.get_clock().now().to_msg()
        finger = -GRIP_TRAVEL * (1.0 - self.gripper)

        state = JointState()
        state.header.stamp = now
        state.name = self.joint_names
        state.position = list(self.q) + [finger, finger]
        state.velocity = list(self.qd) + [
            GRIP_TRAVEL * self.gripper_velocity,
            GRIP_TRAVEL * self.gripper_velocity,
        ]
        state.effort = list(self.last_effort[:4]) + [
            self.last_effort[4], self.last_effort[4],
        ]
        self.state_pub.publish(state)

        desired = JointState()
        desired.header.stamp = now
        desired.name = self.joint_names
        desired.position = list(self.desired_q) + [
            -GRIP_TRAVEL * (1.0 - self.desired_gripper),
            -GRIP_TRAVEL * (1.0 - self.desired_gripper),
        ]
        desired.velocity = [base_velocity_sp, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.setpoint_pub.publish(desired)

        diag = Float32MultiArray()
        diag.data = [float(v) for v in self.last_error + self.last_effort]
        self.diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = PIDSimulation()
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
