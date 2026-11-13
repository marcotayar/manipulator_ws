"""
Real hardware: tuning + IK panel for full_arm_controller.

  1. Flash full_arm_controller/full_arm_controller.ino and start the agent:
       docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
  2. ros2 launch manipulator_control hardware.launch.py

The panel talks to the firmware directly over /arm_joint_targets_deg,
/arm_pid_enable, /arm_pid_gains, /arm_joint_telemetry and /estop.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_control',
            executable='arm_control_gui',
            name='arm_control_gui',
            output='screen',
        ),
    ])
