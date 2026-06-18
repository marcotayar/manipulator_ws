"""
Launch file: visualize RRRR manipulator in RViz2.

Nodes:
  - robot_state_publisher: publishes /robot_description and /tf
  - joint_state_publisher_gui: slider GUI to move joints interactively
  - rviz2: 3D visualization
"""
import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('manipulator_description')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'manipulator.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'display.rviz')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        # Publish robot_description and TF tree
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # GUI sliders to control joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
