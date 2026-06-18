from setuptools import find_packages, setup

package_name = 'manipulator_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'ik_node = manipulator_kinematics.ik_node:main',
        ],
    },
)
