"""
Real hardware: Qt panel + RViz mirroring the arm.

  1. Flash esp32_microros.ino and start the micro-ROS agent:
       docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
  2. ros2 launch manipulator_control hardware.launch.py
  3. In RViz select 'Publish Point' and click the grid to move the arm, or use
     the Qt panel.

Starts robot_state_publisher, arm_commander (IK + /arm_command -> ESP32),
shoulder_feedback_bridge, click_to_target, the Qt panel and RViz.

RViz shows the shoulder from the potentiometer (/shoulder_feedback) and the
elbow, wrist and gripper as commanded (no sensors on those joints); the base
is not shown (no encoder). The panel plots only real measurements.
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

    return LaunchDescription([
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
            executable='shoulder_feedback_bridge',
            name='shoulder_feedback_bridge',
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
