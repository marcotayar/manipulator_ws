"""Launch the hardware command bridge and Qt control panel."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_control',
            executable='arm_commander',
            name='arm_commander',
            output='screen',
        ),
        Node(
            package='manipulator_control',
            executable='arm_control_gui',
            name='arm_control_gui',
            output='screen',
        ),
    ])
