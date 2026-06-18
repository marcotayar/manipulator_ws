"""
Launch interactive click-to-move mode.

Starts:
  - robot_state_publisher (URDF → TF)
  - ik_node (IK solver, listens to /target_pose)
  - click_to_target (bridges RViz clicks → /target_pose)
  - rviz2 (with Publish Point tool preconfigured)

Usage:
  ros2 launch manipulator_control click_move.launch.py

Then select 'Publish Point' in the RViz toolbar and click on the grid.
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
            package='manipulator_kinematics',
            executable='ik_node',
            name='ik_node',
            output='screen',
        ),

        Node(
            package='manipulator_control',
            executable='click_to_target',
            name='click_to_target',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
