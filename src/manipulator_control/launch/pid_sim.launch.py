"""Launch the full Qt/RViz stack with a closed-loop PID numerical plant.

Unlike click_move.launch.py, no kinematic ik_node publishes /joint_states.
arm_commander supplies the deployment-format /arm_command setpoint and
pid_sim_node publishes only simulated encoder measurements.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


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
            package='manipulator_control',
            executable='arm_commander',
            name='arm_commander',
            output='screen',
        ),
        Node(
            package='manipulator_kinematics',
            executable='pid_sim_node',
            name='pid_sim',
            output='screen',
        ),
        Node(
            package='manipulator_control',
            executable='click_to_target',
            name='click_to_target',
            output='screen',
        ),
        Node(
            package='manipulator_control',
            executable='arm_control_gui',
            name='arm_control_gui',
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
