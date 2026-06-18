#!/usr/bin/env python3
"""
DH parameter validation for RRRR manipulator using Peter Corke's
Robotics Toolbox for Python.

Install: pip install roboticstoolbox-python

This script:
  1. Defines the DH table for the 4-DOF manipulator
  2. Computes forward kinematics for a few test configurations
  3. Prints the DH table and workspace analysis
  4. Plots the robot in the test configurations
"""
import numpy as np

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
except ImportError:
    print("Install roboticstoolbox-python:")
    print("  pip install roboticstoolbox-python")
    exit(1)

# ============================================
#  DH Parameters (standard convention)
# ============================================
#
#  Joint |  theta  |   d     |   a     | alpha
#  ------+---------+---------+---------+--------
#    J1  |  theta1 |  0.09   |   0     |  pi/2
#    J2  |  theta2 |   0     |  0.10   |   0
#    J3  |  theta3 |   0     |  0.09   |   0
#    J4  |  theta4 |   0     |  0.16   |   0
#
#  d1 = base_height + turntable = 0.07 + 0.02 = 0.09 m
#  a2 = link1_length = 0.10 m
#  a3 = link2_length = 0.09 m
#  a4 = wrist_to_tip  = 0.16 m  (gripper_base 0.03 + fingers 0.13)

L1 = 0.10  # link1
L2 = 0.09  # link2
LG = 0.16  # wrist to EE tip
D1 = 0.09  # shoulder height (base + turntable)

robot = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=D1, a=0,  alpha=np.pi/2),  # J1: base yaw
        rtb.RevoluteDH(d=0,  a=L1, alpha=0),          # J2: shoulder
        rtb.RevoluteDH(d=0,  a=L2, alpha=0),          # J3: elbow
        rtb.RevoluteDH(d=0,  a=LG, alpha=0),          # J4: wrist
    ],
    name="RRRR_Manipulator",
)

# Joint limits
robot.qlim = np.array([
    [-np.pi,     np.pi],       # J1
    [-np.pi/2,   np.pi/2],     # J2
    [-np.pi/2,   np.pi/2],     # J3
    [-np.pi/2,   np.pi/2],     # J4
])


def main():
    print("=" * 50)
    print("  RRRR Manipulator — DH Validation")
    print("=" * 50)
    print()
    print(robot)
    print()

    # ---- Test configurations ----
    configs = {
        "Home (all zeros)":       [0, 0, 0, 0],
        "Arm forward horizontal": [0, 0, 0, 0],
        "Arm up vertical":        [0, np.pi/2, 0, 0],
        "Elbow bent 90°":         [0, np.pi/4, -np.pi/2, np.pi/4],
        "Rotated 90° yaw":        [np.pi/2, np.pi/4, -np.pi/4, 0],
    }

    for name, q in configs.items():
        T = robot.fkine(q)
        pos = T.t
        print(f"{name}:")
        print(f"  q = [{', '.join(f'{v:.2f}' for v in q)}] rad")
        print(f"  End-effector position: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f} m")
        print()

    # ---- Workspace analysis ----
    print("Workspace analysis:")
    max_reach = L1 + L2 + LG
    min_reach = abs(L1 - L2 - LG)
    print(f"  Max horizontal reach: {max_reach:.3f} m ({max_reach*100:.1f} cm)")
    print(f"  Min reach:            {min_reach:.3f} m ({min_reach*100:.1f} cm)")
    print(f"  Max height:           {D1 + max_reach:.3f} m ({(D1+max_reach)*100:.1f} cm)")
    print()

    # ---- Plot ----
    try:
        print("Plotting robot... (close window to continue)")
        robot.plot(configs["Elbow bent 90°"], block=True)
    except Exception as e:
        print(f"  Could not plot (no display?): {e}")
        print("  Run this script on a machine with a display.")


if __name__ == '__main__':
    main()
