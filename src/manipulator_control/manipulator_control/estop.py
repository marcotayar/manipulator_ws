"""Emergency-stop topic shared by the GUI and the nodes that move the arm.

/estop is a latched std_msgs/Bool: True stops the base and freezes the arm at
its last commanded pose (new targets, base and gripper commands are ignored);
False releases it. Transient-local QoS means a node that starts while the
stop is engaged still sees it.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

ESTOP_TOPIC = '/estop'

ESTOP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
