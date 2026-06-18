"""
Launch teleop mode: visualize robot + keyboard control.

Usage:
  ros2 launch manipulator_control teleop.launch.py

NOTE: run keyboard_teleop in a separate terminal for key input:
  ros2 run manipulator_control keyboard_teleop
"""
import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_dir = get_package_share_directory('manipulator_description')
    xacro_file = os.path.join(desc_dir, 'urdf', 'manipulator.urdf.xacro')
    rviz_config = os.path.join(desc_dir, 'rviz', 'display.rviz')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        Node(
            package='manipulator_control',
            executable='base_vel_gui',
            name='base_vel_gui',
            output='screen',
        ),
    ])
