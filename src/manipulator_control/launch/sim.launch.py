"""
Simulation: test the IK + control stack without hardware.

  ros2 launch manipulator_control sim.launch.py              # kinematic plant
  ros2 launch manipulator_control sim.launch.py plant:=pid   # closed-loop PID

Starts robot_state_publisher, arm_commander (IK, publishes /arm_command),
click_to_target, the Qt control panel and RViz, plus one plant that publishes
/joint_states:
  plant:=kinematic  ik_node: interpolates to each IK solution (default)
  plant:=pid        pid_sim_node: 200 Hz PID plant driven by /arm_command

Plots in the panel then show the commanded angles against the simulated
encoders. In RViz select 'Publish Point' and click the grid to set a target.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, EqualsSubstitution,
                                  LaunchConfiguration)
from launch_ros.actions import Node


def generate_launch_description():
    desc_dir = get_package_share_directory('manipulator_description')
    xacro_file = os.path.join(desc_dir, 'urdf', 'manipulator.urdf.xacro')
    rviz_config = os.path.join(desc_dir, 'rviz', 'display.rviz')
    plant = LaunchConfiguration('plant')

    return LaunchDescription([
        DeclareLaunchArgument(
            'plant', default_value='kinematic',
            choices=['kinematic', 'pid'],
            description='Simulated arm: kinematic (ik_node) or pid (pid_sim_node)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}],
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
            executable='ik_node',
            name='ik_node',
            output='screen',
            condition=IfCondition(EqualsSubstitution(plant, 'kinematic')),
        ),
        Node(
            package='manipulator_kinematics',
            executable='pid_sim_node',
            name='pid_sim',
            output='screen',
            condition=IfCondition(EqualsSubstitution(plant, 'pid')),
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
