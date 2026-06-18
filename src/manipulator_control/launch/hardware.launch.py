"""
Launch full hardware control stack.

Starts:
  - robot_state_publisher  (URDF → TF, for RViz)
  - arm_commander          (IK + hardware bridge → /arm_command → ESP32)
  - click_to_target        (RViz clicks → /target_pose → arm_commander)
  - base_vel_gui           (floating buttons for base velocity)
  - rviz2

Workflow:
  1. Flash esp32_microros.ino and start the micro-ROS agent:
       docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
  2. ros2 launch manipulator_control hardware.launch.py
  3. In RViz select 'Publish Point', click the grid to move the arm.
     Use the base velocity buttons to rotate the base first.

Note: RViz shows the URDF at its home pose — it does not mirror the
real hardware position (arm_commander does not publish /joint_states).
Use it for click targeting only.
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
            package='manipulator_control',
            executable='arm_commander',
            name='arm_commander',
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
            executable='base_vel_gui',
            name='base_vel_gui',
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
