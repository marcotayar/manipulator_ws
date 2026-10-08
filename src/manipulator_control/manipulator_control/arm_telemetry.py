"""Shared ROS subscriptions for the Qt GUIs (latest command and measurements)."""

import time
from math import pi

from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray

FRESH_S = 1.0        # a reading older than this is treated as missing
ESTIMATE_FRAME = 'estimate'
GRIP_TRAVEL = 0.03   # finger joint: -0.03 open .. 0.0 closed (see ik_node)


class ArmTelemetry(Node):
    """Latest /arm_command, /shoulder_feedback and /joint_states.

    Angles are in the /arm_command convention (rad, from horizontal).
    /joint_states messages with header.frame_id == ESTIMATE_FRAME come from
    shoulder_feedback_bridge (RViz estimate on hardware, partly commanded
    values) and are not treated as measurements.
    """

    def __init__(self, node_name):
        super().__init__(node_name)
        self.create_subscription(
            Float32MultiArray, '/arm_command', self._command_cb, 10)
        self.create_subscription(
            Float32, '/shoulder_feedback', self._feedback_cb, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self.last_command = None        # (base, shoulder, elbow, wrist, gripper)
        self.last_command_time = None
        self.last_feedback = None       # shoulder pot, rad
        self.last_feedback_time = None
        self.last_joints = None         # encoder (shoulder, elbow, wrist), rad
        self.last_gripper = None        # encoder closure 0 open .. 1 closed
        self.last_joints_time = None

    def _command_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self.last_command = tuple(msg.data[:5])
            self.last_command_time = time.monotonic()

    def _feedback_cb(self, msg: Float32):
        self.last_feedback = msg.data
        self.last_feedback_time = time.monotonic()

    def _joint_state_cb(self, msg: JointState):
        if msg.header.frame_id == ESTIMATE_FRAME:
            return
        pos = dict(zip(msg.name, msg.position))
        try:
            self.last_joints = (
                pi / 2 - pos['joint2'], -pos['joint3'], -pos['joint4'])
        except KeyError:
            return
        finger = pos.get('finger_left_joint')
        self.last_gripper = (
            None if finger is None else 1.0 + finger / GRIP_TRAVEL)
        self.last_joints_time = time.monotonic()

    @staticmethod
    def is_fresh(stamp, now=None):
        if stamp is None:
            return False
        return (time.monotonic() if now is None else now) - stamp < FRESH_S

    def measured_joints(self):
        """(shoulder, elbow, wrist) in rad, None per joint without a sensor.

        The shoulder prefers the potentiometer over the encoder.
        """
        shoulder = elbow = wrist = None
        if self.is_fresh(self.last_joints_time):
            shoulder, elbow, wrist = self.last_joints
        if self.is_fresh(self.last_feedback_time):
            shoulder = self.last_feedback
        return shoulder, elbow, wrist
