#!/usr/bin/env python3
"""Render the manipulator command ranges and sampled reachable workspace."""

from pathlib import Path
import sys
import warnings

import matplotlib

warnings.filterwarnings('ignore', message='Unable to import Axes3D.*')
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PACKAGE_ROOT))

from manipulator_control import arm_ik_2d as ik  # noqa: E402


def sample_workspace(samples_per_joint=35):
    shoulder = np.linspace(ik.SH_MIN, ik.SH_MAX, samples_per_joint)
    elbow = np.linspace(ik.EL_MIN, ik.EL_MAX, samples_per_joint)
    wrist = np.linspace(ik.WR_MIN, ik.WR_MAX, samples_per_joint)
    q1, q2, q3 = np.meshgrid(shoulder, elbow, wrist, indexing='ij')

    radius = (
        ik.L1 * np.cos(q1)
        + ik.L2 * np.cos(q1 + q2)
        + ik.L3 * np.cos(q1 + q2 + q3)
    )
    height = (
        ik.BASE_Y
        + ik.L1 * np.sin(q1)
        + ik.L2 * np.sin(q1 + q2)
        + ik.L3 * np.sin(q1 + q2 + q3)
    )

    valid = (radius >= 0.0) & (height >= ik.GROUND_CLEAR)
    return radius[valid], height[valid]


def render(output_path):
    radius, height = sample_workspace()

    plt.style.use('dark_background')
    figure = plt.figure(figsize=(18, 7.5), constrained_layout=True)
    grid = figure.add_gridspec(1, 3, width_ratios=(0.82, 1.0, 1.35))

    action_panel = figure.add_subplot(grid[0, 0])
    planar = figure.add_subplot(grid[0, 1])
    planar.hexbin(
        radius,
        height,
        gridsize=65,
        mincnt=1,
        cmap='viridis',
        linewidths=0,
    )
    planar.scatter([0.0], [ik.BASE_Y], marker='o', s=65, color='#ff9f43')
    planar.axhline(ik.GROUND_CLEAR, color='#ff6b6b', linewidth=1.2, alpha=0.8)
    planar.set_title('Planar workspace cross-section', fontsize=15, pad=12)
    planar.set_xlabel('Radial reach r (m)')
    planar.set_ylabel('Height z (m)')
    planar.set_aspect('equal', adjustable='box')
    planar.grid(alpha=0.16)

    workspace_3d = figure.add_subplot(grid[0, 2])
    stride = max(1, len(radius) // 900)
    sampled_r = radius[::stride]
    sampled_z = height[::stride]
    yaw = np.linspace(-np.pi, np.pi, 40, endpoint=False)
    rr, tt = np.meshgrid(sampled_r, yaw)
    zz = np.broadcast_to(sampled_z, rr.shape)
    xx = rr * np.cos(tt)
    yy = rr * np.sin(tt)
    # Orthographic isometric projection, kept explicit so rendering does not
    # depend on Matplotlib's optional mplot3d installation.
    azimuth = np.deg2rad(38.0)
    elevation = np.deg2rad(24.0)
    horizontal = xx * np.cos(azimuth) - yy * np.sin(azimuth)
    depth = xx * np.sin(azimuth) + yy * np.cos(azimuth)
    vertical = zz * np.cos(elevation) - depth * np.sin(elevation)
    workspace_3d.scatter(
        horizontal.ravel(),
        vertical.ravel(),
        c=zz.ravel(),
        cmap='viridis',
        s=1.0,
        alpha=0.22,
        rasterized=True,
    )
    workspace_3d.set_title(
        'Cylindrical workspace with base yaw (isometric view)',
        fontsize=15,
        pad=12,
    )
    workspace_3d.set_aspect('equal', adjustable='box')
    workspace_3d.set_axis_off()

    action_text = (
        'Hardware action vector\n'
        'a = [base velocity, shoulder, elbow, wrist, gripper]\n'
        'base velocity:  −1.00 … +1.00 normalized\n'
        'shoulder:       +0.17 … +1.57 rad   (10° … 90°)\n'
        'elbow:          −1.40 … +1.40 rad   (−80° … 80°)\n'
        'wrist:          −1.40 … +1.40 rad   (−80° … 80°)\n'
        'gripper:         0.00 … 1.00         (open … closed)'
    )
    action_panel.text(
        0.04,
        0.55,
        action_text,
        va='center',
        fontsize=10.5,
        family='monospace',
        color='#e8edf2',
        bbox={
            'boxstyle': 'round,pad=0.65',
            'facecolor': '#171b22',
            'edgecolor': '#465262',
            'alpha': 0.96,
        },
    )
    action_panel.set_title('Command action space', fontsize=15, pad=12)
    action_panel.set_axis_off()
    figure.suptitle(
        'RRRR Manipulator Action and Reachable Workspace',
        fontsize=20,
        fontweight='bold',
    )
    figure.text(
        0.99,
        0.015,
        'Sampled from joint limits; self-collision and payload are not modeled.',
        ha='right',
        fontsize=9,
        color='#aab4bf',
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=150, facecolor='#10141a')
    plt.close(figure)


def main():
    output_path = REPOSITORY_ROOT / 'docs' / 'images' / 'action_space.png'
    render(output_path)
    print(f'Wrote {output_path}')


if __name__ == '__main__':
    main()
