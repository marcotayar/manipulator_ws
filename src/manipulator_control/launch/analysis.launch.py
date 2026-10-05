"""Analysis window (tracking error, gripper/base, pause, CSV export).

Read-only: run it next to sim.launch.py or hardware.launch.py.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_control',
            executable='arm_analysis_gui',
            name='arm_analysis_gui',
            output='screen',
        ),
    ])
