from setuptools import find_packages, setup

package_name = 'manipulator_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/teleop.launch.py',
            'launch/click_move.launch.py',
            'launch/hardware.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'keyboard_teleop    = manipulator_control.keyboard_teleop:main',
            'click_to_target    = manipulator_control.click_to_target:main',
            'base_vel_gui       = manipulator_control.base_vel_gui:main',
            'arm_commander      = manipulator_control.arm_commander:main',
            'base_gripper_teleop = manipulator_control.base_gripper_teleop:main',
        ],
    },
)
